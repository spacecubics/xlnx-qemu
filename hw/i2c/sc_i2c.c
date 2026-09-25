/*
 * Space Cubics I2C Master Controller
 *
 * The bus runs in virtual time: each byte on the wire takes nine SCL
 * periods as programmed in the timing registers. Filter, sampling and
 * SCL timeout settings are only stored. There is no other bus master,
 * so BIT error, arbitration lost and SCL timeout are never raised.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/fifo8.h"
#include "qemu/fifo32.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TYPE_SC_I2C "sc.i2c"
OBJECT_DECLARE_SIMPLE_TYPE(SCI2CState, SC_I2C)

REG32(ENR, 0x0000)
    FIELD(ENR, EN, 0, 1)
REG32(TXFIFOR, 0x0004)
    FIELD(TXFIFOR, TXDATA, 0, 8)
    FIELD(TXFIFOR, STOP, 8, 1)
    FIELD(TXFIFOR, RESTART, 9, 1)
REG32(RXFIFOR, 0x0008)
REG32(BSR, 0x000C)
    FIELD(BSR, SELFBUSY, 0, 1)
    FIELD(BSR, OTHERBUSY, 1, 1)
REG32(ISR, 0x0010)
    FIELD(ISR, COMP, 0, 1)
    FIELD(ISR, ARBLST, 1, 1)
    FIELD(ISR, TXFIFOUTH, 4, 1)
    FIELD(ISR, RXFIFOOTH, 5, 1)
    FIELD(ISR, ACKER, 8, 1)
    FIELD(ISR, BITER, 9, 1)
    FIELD(ISR, TXFIFOOVF, 10, 1)
    FIELD(ISR, RXFIFOUDF, 11, 1)
    FIELD(ISR, SCLTO, 12, 1)
REG32(IER, 0x0014)
REG32(FIFOSR, 0x0018)
    FIELD(FIFOSR, TXFIFOCAP, 0, 5)
    FIELD(FIFOSR, RXFIFOCAP, 16, 5)
REG32(FIFORR, 0x001C)
    FIELD(FIFORR, TXFIFORST, 0, 1)
    FIELD(FIFORR, RXFIFORST, 16, 1)
REG32(FTLSR, 0x0020)
    FIELD(FTLSR, TXFIFOTHL, 0, 5)
    FIELD(FTLSR, RXFIFOTHL, 16, 5)
REG32(SCLTSR, 0x0024)
    FIELD(SCLTSR, SCLTOPROD, 0, 16)
REG32(THDSTAR, 0x0030)
REG32(TSUSTOR, 0x0034)
REG32(TSUSTAR, 0x0038)
REG32(THIGHR, 0x003C)
REG32(THDDATR, 0x0040)
REG32(TSUDATR, 0x0044)
REG32(TBUFR, 0x0048)
REG32(TBSMPLR, 0x004C)
REG32(BFSR, 0x0050)
    FIELD(BFSR, FLTCYC, 0, 8)
REG32(VER, 0xF000)

#define SC_I2C_MMIO_SIZE    0x10000
#define SC_I2C_FIFO_DEPTH   16

#define SC_I2C_ISR_MASK \
    (R_ISR_COMP_MASK | R_ISR_ARBLST_MASK | R_ISR_TXFIFOUTH_MASK | \
     R_ISR_RXFIFOOTH_MASK | R_ISR_ACKER_MASK | R_ISR_BITER_MASK | \
     R_ISR_TXFIFOOVF_MASK | R_ISR_RXFIFOUDF_MASK | R_ISR_SCLTO_MASK)

#define SC_I2C_TXFIFO_MASK \
    (R_TXFIFOR_TXDATA_MASK | R_TXFIFOR_STOP_MASK | R_TXFIFOR_RESTART_MASK)

/* Bus timing registers THDSTAR..TBSMPLR, only writable while disabled */
#define SC_I2C_NUM_TIMING   ((A_TBSMPLR - A_THDSTAR) / 4 + 1)

static const uint32_t sc_i2c_timing_reset[SC_I2C_NUM_TIMING] = {
    0xEF,   /* THDSTAR */
    0xEF,   /* TSUSTOR */
    0x117,  /* TSUSTAR */
    0xE5,   /* THIGHR */
    0x13,   /* THDDATR */
    0xE5,   /* TSUDATR */
    0x117,  /* TBUFR */
    0x0,    /* TBSMPLR */
};

#define SC_I2C_BFSR_RESET   0x2F

#define SC_I2C_TIMING(s, reg) ((s)->timing[(A_##reg - A_THDSTAR) / 4])

typedef enum SCI2CPhase {
    SC_I2C_PHASE_ADDR,      /* Next TX FIFO entry is an address byte */
    SC_I2C_PHASE_WRITE,     /* Next TX FIFO entry is data to send */
    SC_I2C_PHASE_READ,      /* Next TX FIFO entry is a receive count */
} SCI2CPhase;

struct SCI2CState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    I2CBus *bus;
    QEMUTimer *timer;

    Fifo32 tx_fifo;
    Fifo8 rx_fifo;

    uint32_t enr;
    uint32_t isr;
    uint32_t ier;
    uint32_t ftlsr;
    uint32_t scltsr;
    uint32_t timing[SC_I2C_NUM_TIMING];
    uint32_t bfsr;
    uint32_t ip_version;
    uint32_t clock_frequency;

    /* Transfer state */
    uint32_t phase;
    bool bus_active;
    uint8_t address;
    uint32_t rx_remaining;
    uint32_t rx_end_flags;
};

static void sc_i2c_update_irq(SCI2CState *s)
{
    qemu_set_irq(s->irq, !!(s->isr & s->ier));
}

static bool sc_i2c_threshold_enabled(uint32_t level)
{
    /* A level of zero or the FIFO maximum disables the interrupt */
    return level != 0 && level < SC_I2C_FIFO_DEPTH;
}

static void sc_i2c_update_thresholds(SCI2CState *s)
{
    uint32_t tx_level = FIELD_EX32(s->ftlsr, FTLSR, TXFIFOTHL);
    uint32_t rx_level = FIELD_EX32(s->ftlsr, FTLSR, RXFIFOTHL);

    if (sc_i2c_threshold_enabled(tx_level) &&
        fifo32_num_used(&s->tx_fifo) < tx_level) {
        s->isr |= R_ISR_TXFIFOUTH_MASK;
    }
    if (sc_i2c_threshold_enabled(rx_level) &&
        fifo8_num_used(&s->rx_fifo) > rx_level) {
        s->isr |= R_ISR_RXFIFOOTH_MASK;
    }
}

static void sc_i2c_stop(SCI2CState *s)
{
    i2c_end_transfer(s->bus);
    s->bus_active = false;
    s->phase = SC_I2C_PHASE_ADDR;
}

/* Handle the STOP / Repeated START flags of the last byte of a message */
static void sc_i2c_end_message(SCI2CState *s, uint32_t flags)
{
    if (flags & R_TXFIFOR_STOP_MASK) {
        trace_sc_i2c_stop(s->address);
        sc_i2c_stop(s);
        s->isr |= R_ISR_COMP_MASK;
    } else if (flags & R_TXFIFOR_RESTART_MASK) {
        /* Keep the bus, the next entry is the address after Sr */
        s->phase = SC_I2C_PHASE_ADDR;
    }
}

/*
 * The target did not acknowledge: stop, send STOP and disable the
 * controller, as the hardware does.
 */
static void sc_i2c_ack_error(SCI2CState *s)
{
    trace_sc_i2c_ack_error(s->address);
    sc_i2c_stop(s);
    fifo32_reset(&s->tx_fifo);
    s->rx_remaining = 0;
    s->enr = 0;
    s->isr |= R_ISR_ACKER_MASK;
}

static void sc_i2c_start(SCI2CState *s, uint8_t byte)
{
    uint8_t address = byte >> 1;
    bool is_recv = byte & 1;

    /* The I2C core keeps the current target on a repeated START */
    if (s->bus_active && address != s->address) {
        i2c_end_transfer(s->bus);
    }

    s->address = address;
    trace_sc_i2c_start(address, is_recv, s->bus_active);

    if (i2c_start_transfer(s->bus, address, is_recv)) {
        sc_i2c_ack_error(s);
        return;
    }

    s->bus_active = true;
    s->phase = is_recv ? SC_I2C_PHASE_READ : SC_I2C_PHASE_WRITE;
}

/* Time for one byte plus ACK on the wire */
static int64_t sc_i2c_byte_time_ns(SCI2CState *s)
{
    uint64_t scl_cycles = (SC_I2C_TIMING(s, THIGHR) + 1) +
                          (SC_I2C_TIMING(s, THDDATR) + 1) +
                          (SC_I2C_TIMING(s, TSUDATR) + 1);

    return muldiv64(9 * scl_cycles, NANOSECONDS_PER_SECOND,
                    s->clock_frequency);
}

static bool sc_i2c_has_work(SCI2CState *s)
{
    if (s->rx_remaining) {
        return !fifo8_is_full(&s->rx_fifo);
    }
    return !fifo32_is_empty(&s->tx_fifo);
}

/* Start the next byte on the bus, if possible */
static void sc_i2c_kick(SCI2CState *s)
{
    if (FIELD_EX32(s->enr, ENR, EN) && !timer_pending(s->timer) &&
        sc_i2c_has_work(s)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            sc_i2c_byte_time_ns(s));
    }
}

/* Perform one byte transfer on the bus */
static void sc_i2c_step(SCI2CState *s)
{
    uint32_t entry;
    uint8_t data;

    if (!s->rx_remaining) {
        entry = fifo32_pop(&s->tx_fifo);
        data = FIELD_EX32(entry, TXFIFOR, TXDATA);

        switch (s->phase) {
        case SC_I2C_PHASE_ADDR:
            sc_i2c_start(s, data);
            if (FIELD_EX32(s->enr, ENR, EN)) {
                sc_i2c_end_message(s, entry);
            }
            return;
        case SC_I2C_PHASE_WRITE:
            trace_sc_i2c_send(s->address, data);
            if (i2c_send(s->bus, data)) {
                sc_i2c_ack_error(s);
                return;
            }
            sc_i2c_end_message(s, entry);
            return;
        case SC_I2C_PHASE_READ:
            /* The receive count itself is not sent on the bus */
            s->rx_remaining = data + 1;
            s->rx_end_flags = entry & (R_TXFIFOR_STOP_MASK |
                                       R_TXFIFOR_RESTART_MASK);
            if (fifo8_is_full(&s->rx_fifo)) {
                return;
            }
            break;
        default:
            g_assert_not_reached();
        }
    }

    data = i2c_recv(s->bus);
    trace_sc_i2c_recv(s->address, data);
    fifo8_push(&s->rx_fifo, data);

    /* The last byte of a message is followed by a NACK */
    if (--s->rx_remaining == 0 && s->rx_end_flags) {
        i2c_nack(s->bus);
        sc_i2c_end_message(s, s->rx_end_flags);
    }
}

static void sc_i2c_update(SCI2CState *s)
{
    sc_i2c_update_thresholds(s);
    sc_i2c_update_irq(s);
    sc_i2c_kick(s);
}

static void sc_i2c_timer_cb(void *opaque)
{
    SCI2CState *s = opaque;

    if (FIELD_EX32(s->enr, ENR, EN) && sc_i2c_has_work(s)) {
        sc_i2c_step(s);
    }
    sc_i2c_update(s);
}

static uint64_t sc_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    SCI2CState *s = opaque;
    uint32_t val = 0;

    switch (addr) {
    case A_ENR:
        val = s->enr;
        break;
    case A_RXFIFOR:
        if (fifo8_is_empty(&s->rx_fifo)) {
            s->isr |= R_ISR_RXFIFOUDF_MASK;
            sc_i2c_update_irq(s);
            break;
        }
        val = fifo8_pop(&s->rx_fifo);
        /* A read waiting for FIFO space continues */
        sc_i2c_update(s);
        break;
    case A_BSR:
        val = FIELD_DP32(0, BSR, SELFBUSY,
                         s->bus_active || timer_pending(s->timer));
        break;
    case A_ISR:
        val = s->isr;
        break;
    case A_IER:
        val = s->ier;
        break;
    case A_FIFOSR:
        val = FIELD_DP32(val, FIFOSR, TXFIFOCAP, fifo32_num_used(&s->tx_fifo));
        val = FIELD_DP32(val, FIFOSR, RXFIFOCAP, fifo8_num_used(&s->rx_fifo));
        break;
    case A_FTLSR:
        val = s->ftlsr;
        break;
    case A_SCLTSR:
        val = s->scltsr;
        break;
    case A_THDSTAR ... A_TBSMPLR:
        val = s->timing[(addr - A_THDSTAR) / 4];
        break;
    case A_BFSR:
        val = s->bfsr;
        break;
    case A_VER:
        val = s->ip_version;
        break;
    case A_TXFIFOR:
    case A_FIFORR:
        /* Write-only */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%" HWADDR_PRIx
                      "\n", __func__, addr);
        break;
    }

    trace_sc_i2c_reg_read(addr, val);
    return val;
}

static void sc_i2c_write(void *opaque, hwaddr addr, uint64_t val64,
                         unsigned size)
{
    SCI2CState *s = opaque;
    uint32_t val = val64;

    trace_sc_i2c_reg_write(addr, val);

    switch (addr) {
    case A_ENR:
        s->enr = val & R_ENR_EN_MASK;
        if (!s->enr) {
            /* Disabling aborts the transfer and releases the bus */
            timer_del(s->timer);
            if (s->bus_active) {
                sc_i2c_stop(s);
            }
            s->rx_remaining = 0;
        }
        break;
    case A_TXFIFOR:
        if (fifo32_is_full(&s->tx_fifo)) {
            s->isr |= R_ISR_TXFIFOOVF_MASK;
            break;
        }
        fifo32_push(&s->tx_fifo, val & SC_I2C_TXFIFO_MASK);
        break;
    case A_ISR:
        s->isr &= ~(val & SC_I2C_ISR_MASK);
        break;
    case A_IER:
        s->ier = val & SC_I2C_ISR_MASK;
        break;
    case A_FIFORR:
        if (FIELD_EX32(val, FIFORR, TXFIFORST)) {
            fifo32_reset(&s->tx_fifo);
        }
        if (FIELD_EX32(val, FIFORR, RXFIFORST)) {
            fifo8_reset(&s->rx_fifo);
        }
        break;
    case A_FTLSR:
        s->ftlsr = val & (R_FTLSR_TXFIFOTHL_MASK | R_FTLSR_RXFIFOTHL_MASK);
        break;
    case A_SCLTSR:
        s->scltsr = val & R_SCLTSR_SCLTOPROD_MASK;
        break;
    case A_THDSTAR ... A_TBSMPLR:
        if (FIELD_EX32(s->enr, ENR, EN)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: timing register 0x%"
                          HWADDR_PRIx " written while enabled\n",
                          __func__, addr);
            break;
        }
        s->timing[(addr - A_THDSTAR) / 4] = val & 0xFFFF;
        break;
    case A_BFSR:
        s->bfsr = val & R_BFSR_FLTCYC_MASK;
        break;
    case A_RXFIFOR:
    case A_BSR:
    case A_FIFOSR:
    case A_VER:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write offset 0x%" HWADDR_PRIx
                      "\n", __func__, addr);
        break;
    }

    sc_i2c_update(s);
}

static const MemoryRegionOps sc_i2c_ops = {
    .read = sc_i2c_read,
    .write = sc_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sc_i2c_reset(DeviceState *dev)
{
    SCI2CState *s = SC_I2C(dev);

    timer_del(s->timer);
    if (s->bus_active) {
        i2c_end_transfer(s->bus);
    }

    fifo32_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
    s->enr = 0;
    s->isr = 0;
    s->ier = 0;
    s->ftlsr = 0;
    s->scltsr = 0;
    memcpy(s->timing, sc_i2c_timing_reset, sizeof(s->timing));
    s->bfsr = SC_I2C_BFSR_RESET;
    s->phase = SC_I2C_PHASE_ADDR;
    s->bus_active = false;
    s->address = 0;
    s->rx_remaining = 0;
    s->rx_end_flags = 0;
    sc_i2c_update_irq(s);
}

static void sc_i2c_realize(DeviceState *dev, Error **errp)
{
    SCI2CState *s = SC_I2C(dev);

    if (!s->clock_frequency) {
        error_setg(errp, "clock-frequency must be non-zero");
        return;
    }

    fifo32_create(&s->tx_fifo, SC_I2C_FIFO_DEPTH);
    fifo8_create(&s->rx_fifo, SC_I2C_FIFO_DEPTH);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sc_i2c_timer_cb, s);
}

static void sc_i2c_init(Object *obj)
{
    SCI2CState *s = SC_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &sc_i2c_ops, s, TYPE_SC_I2C,
                          SC_I2C_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);

    /* Named "i2c" so fdt-generic attaches child nodes to it */
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const VMStateDescription vmstate_sc_i2c = {
    .name = TYPE_SC_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_FIFO32(tx_fifo, SCI2CState),
        VMSTATE_FIFO8(rx_fifo, SCI2CState),
        VMSTATE_UINT32(enr, SCI2CState),
        VMSTATE_UINT32(isr, SCI2CState),
        VMSTATE_UINT32(ier, SCI2CState),
        VMSTATE_UINT32(ftlsr, SCI2CState),
        VMSTATE_UINT32(scltsr, SCI2CState),
        VMSTATE_UINT32_ARRAY(timing, SCI2CState, SC_I2C_NUM_TIMING),
        VMSTATE_UINT32(bfsr, SCI2CState),
        VMSTATE_UINT32(phase, SCI2CState),
        VMSTATE_BOOL(bus_active, SCI2CState),
        VMSTATE_UINT8(address, SCI2CState),
        VMSTATE_UINT32(rx_remaining, SCI2CState),
        VMSTATE_UINT32(rx_end_flags, SCI2CState),
        VMSTATE_TIMER_PTR(timer, SCI2CState),
        VMSTATE_END_OF_LIST()
    }
};

static Property sc_i2c_props[] = {
    DEFINE_PROP_UINT32("ip-version", SCI2CState, ip_version, 0),
    /* The reset timing values give 100 kHz at 48 MHz */
    DEFINE_PROP_UINT32("clock-frequency", SCI2CState, clock_frequency,
                       48000000),
    DEFINE_PROP_END_OF_LIST(),
};

static void sc_i2c_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sc_i2c_realize;
    dc->reset = sc_i2c_reset;
    dc->vmsd = &vmstate_sc_i2c;
    device_class_set_props(dc, sc_i2c_props);
}

static const TypeInfo sc_i2c_info = {
    .name = TYPE_SC_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SCI2CState),
    .instance_init = sc_i2c_init,
    .class_init = sc_i2c_class_init,
};

static void sc_i2c_register_types(void)
{
    type_register_static(&sc_i2c_info);
}

type_init(sc_i2c_register_types)
