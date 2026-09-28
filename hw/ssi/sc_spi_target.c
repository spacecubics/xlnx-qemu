/*
 * Space Cubics Single SPI Target (SC-SPI-TARGET)
 *
 * Double buffered SPI target: the SPI controller exchanges a frame with
 * the SPI access buffer while the CPU works on the CPU access buffer.
 * Only SPI modes 1 and 3 are supported by the hardware; CPOL does not
 * matter at byte level. Frames are always whole bytes, so the Invalid
 * Frame interrupt is never raised.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/remote-port.h"
#include "hw/remote-port-device.h"
#include "hw/remote-port-proto.h"
#include "hw/ssi/sc_spi_target.h"
#include "migration/vmstate.h"
#include "trace.h"

REG32(IPVER, 0x000)
REG32(IPCONF, 0x004)
    FIELD(IPCONF, MAX_FRAME_SIZE, 0, 6)
REG32(RSTCTRL, 0x010)
    FIELD(RSTCTRL, IP_RESET, 0, 1)
REG32(SCRPAD, 0x01C)
REG32(INTSTATS, 0x020)
    FIELD(INTSTATS, FRAME_COMP, 0, 1)
    FIELD(INTSTATS, FRAME_THR, 1, 1)
    FIELD(INTSTATS, BUF_OVERRUN, 16, 1)
    FIELD(INTSTATS, INVALID_FRAME, 17, 1)
REG32(INTENB, 0x024)
REG32(CTRL, 0x030)
    FIELD(CTRL, IP_EN, 0, 1)
    FIELD(CTRL, CPHA, 8, 1)
    FIELD(CTRL, CPOL, 9, 1)
    FIELD(CTRL, TX_BUF_CLR_DIS, 12, 1)
    FIELD(CTRL, SPI_BUF_TOGGLE_DIS, 14, 1)
REG32(FRMTHR, 0x034)
    FIELD(FRMTHR, SIZE, 0, 6)
REG32(BUFCTRL, 0x038)
    FIELD(BUFCTRL, CPU_BUF_CHANGE, 0, 1)
    FIELD(BUFCTRL, SPI_BUF_CHANGE, 16, 1)
REG32(BUFSTATS, 0x03C)
    FIELD(BUFSTATS, FRAME_SIZE, 0, 6)
    FIELD(BUFSTATS, CS, 8, 4)
    FIELD(BUFSTATS, CPU_BUF, 16, 1)
    FIELD(BUFSTATS, SPI_BUF, 17, 1)
REG32(GPO0, 0x080)
REG32(GPO1, 0x084)
REG32(TXBUF, 0x100)
REG32(RXBUF, 0x200)

#define SC_SPI_TARGET_MMIO_SIZE     0x1000
#define SC_SPI_TARGET_BUF_WINDOW    0x100

#define SC_SPI_TARGET_INT_MASK \
    (R_INTSTATS_FRAME_COMP_MASK | R_INTSTATS_FRAME_THR_MASK | \
     R_INTSTATS_BUF_OVERRUN_MASK | R_INTSTATS_INVALID_FRAME_MASK)

#define SC_SPI_TARGET_CTRL_MASK \
    (R_CTRL_IP_EN_MASK | R_CTRL_CPHA_MASK | R_CTRL_CPOL_MASK | \
     R_CTRL_TX_BUF_CLR_DIS_MASK | R_CTRL_SPI_BUF_TOGGLE_DIS_MASK)

#define SC_SPI_TARGET_CTRL_RESET    R_CTRL_CPHA_MASK
#define SC_SPI_TARGET_GPO_MASK      MAKE_64BIT_MASK(0, SC_SPI_TARGET_GPO_WIDTH)

/* The receive bit pointer saturates at 0x1FF, so the byte count at 63 */
#define SC_SPI_TARGET_MAX_BYTE_COUNT    63

static bool sc_spi_target_in_reset(SCSPITargetState *s)
{
    return FIELD_EX32(s->rstctrl, RSTCTRL, IP_RESET);
}

static bool sc_spi_target_enabled(SCSPITargetState *s)
{
    return FIELD_EX32(s->ctrl, CTRL, IP_EN);
}

/* Frame events are only generated in the supported modes (CPHA = 1) */
static bool sc_spi_target_ip_valid(SCSPITargetState *s)
{
    return sc_spi_target_enabled(s) && FIELD_EX32(s->ctrl, CTRL, CPHA);
}

static void sc_spi_target_update_gpo(SCSPITargetState *s)
{
    for (int i = 0; i < ARRAY_SIZE(s->gpo_out); i++) {
        qemu_set_irq(s->gpo_out[i],
                     extract32(s->gpo[i / SC_SPI_TARGET_GPO_WIDTH],
                               i % SC_SPI_TARGET_GPO_WIDTH, 1));
    }
}

/*
 * The threshold interrupt fires on the rising edge of the SPI access
 * buffer frame size matching the threshold. As in the hardware, this
 * includes a threshold of zero matching an empty buffer.
 */
static void sc_spi_target_update(SCSPITargetState *s)
{
    bool match = sc_spi_target_ip_valid(s) &&
                 s->frame_size[s->spi_buf] ==
                 FIELD_EX32(s->frmthr, FRMTHR, SIZE);

    if (match && !s->match_thr) {
        s->intstats |= R_INTSTATS_FRAME_THR_MASK;
    }
    s->match_thr = match;

    qemu_set_irq(s->irq, !!(s->intstats & s->intenb));
}

/* Everything except the reset control and the RX buffers */
static void sc_spi_target_ip_reset(SCSPITargetState *s)
{
    s->scratch_pad = 0;
    s->intstats = 0;
    s->intenb = 0;
    s->ctrl = SC_SPI_TARGET_CTRL_RESET;
    s->frmthr = 0;
    for (int i = 0; i < SC_SPI_TARGET_NUM_GPO_PORTS; i++) {
        s->gpo[i] = s->gpo_init[i];
    }

    memset(s->tx_buf, 0, sizeof(s->tx_buf));
    memset(s->frame_size, 0, sizeof(s->frame_size));
    memset(s->cs_det, 0, sizeof(s->cs_det));
    s->cpu_buf = 0;
    s->spi_buf = 0;

    s->frame_active = false;
    s->frame_bytes = 0;
    s->match_thr = false;

    sc_spi_target_update_gpo(s);
    sc_spi_target_update(s);
}

static void sc_spi_target_end_frame(SCSPITargetState *s)
{
    if (!s->frame_active) {
        return;
    }

    s->frame_active = false;
    trace_sc_spi_target_frame_end(s->spi_buf, s->frame_bytes);

    /* A frame aborted by disabling the IP does not complete */
    if (!sc_spi_target_ip_valid(s)) {
        return;
    }

    s->intstats |= R_INTSTATS_FRAME_COMP_MASK;
    if (s->frame_bytes > s->max_frame_size) {
        s->intstats |= R_INTSTATS_BUF_OVERRUN_MASK;
    }
    if (!FIELD_EX32(s->ctrl, CTRL, TX_BUF_CLR_DIS)) {
        memset(s->tx_buf[s->spi_buf], 0, sizeof(s->tx_buf[s->spi_buf]));
    }
    if (!FIELD_EX32(s->ctrl, CTRL, SPI_BUF_TOGGLE_DIS)) {
        s->spi_buf ^= 1;
    }
}

void sc_spi_target_set_cs(SCSPITargetState *s, uint8_t cs)
{
    s->cs = cs & MAKE_64BIT_MASK(0, SC_SPI_TARGET_NUM_CS);
    trace_sc_spi_target_set_cs(s->cs);

    if (!s->cs) {
        sc_spi_target_end_frame(s);
        sc_spi_target_update(s);
    }
}

/* The shift logic runs while a chip select is asserted and the IP enabled */
static bool sc_spi_target_shifting(SCSPITargetState *s)
{
    return s->cs && sc_spi_target_enabled(s);
}

static void sc_spi_target_start_frame(SCSPITargetState *s)
{
    if (s->frame_active) {
        return;
    }

    s->frame_active = true;
    s->frame_bytes = 0;
    if (sc_spi_target_ip_valid(s)) {
        s->cs_det[s->spi_buf] = s->cs;
    }
}

/* First SCK edge of a byte: the MSB of the MISO byte is driven */
static uint8_t sc_spi_target_shift_out(SCSPITargetState *s)
{
    uint8_t *tx = s->tx_buf[s->spi_buf];
    uint8_t miso;

    if (!sc_spi_target_shifting(s)) {
        /*
         * The shift logic is held at the first bit: every SCK edge
         * shifts out the MSB of the first TX byte.
         */
        miso = (tx[0] & 0x80) ? 0xFF : 0x00;
        trace_sc_spi_target_shift_out(s->spi_buf, 0, miso);
        return miso;
    }

    sc_spi_target_start_frame(s);

    if (s->frame_bytes < s->max_frame_size) {
        miso = tx[s->frame_bytes];
    } else {
        /* MISO keeps the last bit shifted out */
        miso = (tx[s->max_frame_size - 1] & 1) ? 0xFF : 0x00;
    }
    trace_sc_spi_target_shift_out(s->spi_buf, s->frame_bytes, miso);
    return miso;
}

/* Last SCK edge of a byte: the MOSI byte is complete */
static void sc_spi_target_shift_in(SCSPITargetState *s, uint8_t mosi)
{
    uint8_t *rx = s->rx_buf[s->spi_buf];

    if (!sc_spi_target_shifting(s)) {
        /* Every SCK edge samples MOSI into the MSB of the first RX byte */
        rx[0] = deposit32(rx[0], 7, 1, mosi & 1);
        trace_sc_spi_target_shift_in(s->spi_buf, 0, mosi);
        return;
    }

    sc_spi_target_start_frame(s);

    if (s->frame_bytes < s->max_frame_size) {
        rx[s->frame_bytes] = mosi;
    }
    trace_sc_spi_target_shift_in(s->spi_buf, s->frame_bytes, mosi);

    if (s->frame_bytes < SC_SPI_TARGET_MAX_BYTE_COUNT) {
        s->frame_bytes++;
        s->frame_size[s->spi_buf] = s->frame_bytes;
    }

    sc_spi_target_update(s);
}

uint8_t sc_spi_target_transfer(SCSPITargetState *s, uint8_t mosi)
{
    uint8_t miso = sc_spi_target_shift_out(s);

    sc_spi_target_shift_in(s, mosi);
    return miso;
}

/*
 * Remote-port SSI transfers are shifted at the SCK frequency in virtual
 * time and answered once the last byte is complete. The controller
 * waits for the answer, so the CPU can react to interrupts in the
 * middle of a frame, as on the real bus.
 */
typedef struct SCSPITargetXfer {
    uint32_t id;
    uint32_t dev;
    uint32_t flags;
    uint32_t cs;
    uint32_t len;
    uint32_t pos;
    uint8_t *mosi;
    uint8_t *miso;
    uint8_t data[];
} SCSPITargetXfer;

static void sc_spi_target_xfer_respond(SCSPITargetState *s,
                                       SCSPITargetXfer *xfer)
{
    size_t size = sizeof(struct rp_pkt_ssi) + xfer->len;
    g_autofree struct rp_pkt_ssi *pkt = g_malloc0(size);

    trace_sc_spi_target_xfer_done(xfer->id, xfer->len);

    if (xfer->flags & RP_PKT_FLAGS_posted) {
        return;
    }

    rp_encode_ssi(xfer->id, xfer->dev, pkt, rp_normalized_vmclk(s->rp),
                  xfer->cs, xfer->len, xfer->flags | RP_PKT_FLAGS_response);
    memcpy(rp_ssi_dataptr(pkt), xfer->miso, xfer->len);
    rp_write(s->rp, pkt, size);
}

static int64_t sc_spi_target_byte_time_ns(SCSPITargetState *s)
{
    return muldiv64(8, NANOSECONDS_PER_SECOND, s->sck_frequency);
}

/* Start the transfers at the head of the queue until one takes time */
static void sc_spi_target_xfer_start(SCSPITargetState *s)
{
    SCSPITargetXfer *xfer;

    while ((xfer = g_queue_peek_head(&s->xfers))) {
        trace_sc_spi_target_xfer_start(xfer->id, xfer->cs, xfer->len);
        sc_spi_target_set_cs(s, xfer->cs);

        if (xfer->len && s->sck_frequency) {
            xfer->miso[0] = sc_spi_target_shift_out(s);
            timer_mod(s->xfer_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                     sc_spi_target_byte_time_ns(s));
            return;
        }

        for (xfer->pos = 0; xfer->pos < xfer->len; xfer->pos++) {
            xfer->miso[xfer->pos] =
                sc_spi_target_transfer(s, xfer->mosi[xfer->pos]);
        }
        sc_spi_target_xfer_respond(s, xfer);
        g_free(g_queue_pop_head(&s->xfers));
    }
}

static void sc_spi_target_xfer_timer_cb(void *opaque)
{
    SCSPITargetState *s = opaque;
    SCSPITargetXfer *xfer = g_queue_peek_head(&s->xfers);

    sc_spi_target_shift_in(s, xfer->mosi[xfer->pos++]);

    if (xfer->pos < xfer->len) {
        xfer->miso[xfer->pos] = sc_spi_target_shift_out(s);
        timer_mod(s->xfer_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                 sc_spi_target_byte_time_ns(s));
        return;
    }

    sc_spi_target_xfer_respond(s, xfer);
    g_free(g_queue_pop_head(&s->xfers));
    sc_spi_target_xfer_start(s);
}

static void sc_spi_target_rp_ssi(RemotePortDevice *rpdev, struct rp_pkt *pkt)
{
    SCSPITargetState *s = SC_SPI_TARGET(rpdev);
    uint32_t avail = pkt->hdr.len - (sizeof(pkt->ssi) - sizeof(pkt->hdr));
    uint32_t len = pkt->ssi.len;
    SCSPITargetXfer *xfer;

    if (len > avail) {
        error_report("%s: SSI packet with %" PRIu32 " of %" PRIu32 " bytes",
                     object_get_canonical_path(OBJECT(s)), avail, len);
        len = avail;
    }

    xfer = g_malloc0(sizeof(*xfer) + 2 * len);
    xfer->id = pkt->hdr.id;
    xfer->dev = pkt->hdr.dev;
    xfer->flags = pkt->hdr.flags;
    xfer->cs = pkt->ssi.cs;
    xfer->len = len;
    xfer->mosi = xfer->data;
    xfer->miso = xfer->data + len;
    memcpy(xfer->mosi, rp_ssi_dataptr(&pkt->ssi), len);

    g_queue_push_tail(&s->xfers, xfer);
    if (g_queue_get_length(&s->xfers) == 1) {
        sc_spi_target_xfer_start(s);
    }
}

static bool sc_spi_target_buf_index(SCSPITargetState *s, hwaddr addr,
                                    hwaddr base, unsigned *index)
{
    if (addr < base || addr >= base + SC_SPI_TARGET_BUF_WINDOW) {
        return false;
    }
    *index = (addr - base) / 4;
    return *index < s->max_frame_size;
}

static uint64_t sc_spi_target_read(void *opaque, hwaddr addr, unsigned size)
{
    SCSPITargetState *s = opaque;
    uint32_t val = 0;
    unsigned index;

    switch (addr) {
    case A_IPVER:
        val = s->ip_version;
        break;
    case A_IPCONF:
        val = FIELD_DP32(0, IPCONF, MAX_FRAME_SIZE, s->max_frame_size);
        break;
    case A_RSTCTRL:
        val = s->rstctrl;
        break;
    case A_SCRPAD:
        val = s->scratch_pad;
        break;
    case A_INTSTATS:
        val = s->intstats;
        break;
    case A_INTENB:
        val = s->intenb;
        break;
    case A_CTRL:
        val = s->ctrl;
        break;
    case A_FRMTHR:
        val = s->frmthr;
        break;
    case A_BUFCTRL:
        /* Self clearing */
        break;
    case A_BUFSTATS:
        val = FIELD_DP32(val, BUFSTATS, FRAME_SIZE,
                         s->frame_size[s->cpu_buf]);
        val = FIELD_DP32(val, BUFSTATS, CS, s->cs_det[s->cpu_buf]);
        val = FIELD_DP32(val, BUFSTATS, CPU_BUF, s->cpu_buf);
        val = FIELD_DP32(val, BUFSTATS, SPI_BUF, s->spi_buf);
        break;
    case A_GPO0:
    case A_GPO1:
        val = s->gpo[(addr - A_GPO0) / 4];
        break;
    default:
        if (sc_spi_target_buf_index(s, addr, A_TXBUF, &index)) {
            val = s->tx_buf[s->cpu_buf][index];
        } else if (sc_spi_target_buf_index(s, addr, A_RXBUF, &index)) {
            val = s->rx_buf[s->cpu_buf][index];
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%"
                          HWADDR_PRIx "\n", __func__, addr);
        }
        break;
    }

    trace_sc_spi_target_reg_read(addr, val);
    return val;
}

static void sc_spi_target_write(void *opaque, hwaddr addr, uint64_t val64,
                                unsigned size)
{
    SCSPITargetState *s = opaque;
    uint32_t val = val64;
    unsigned index;

    trace_sc_spi_target_reg_write(addr, val);

    if (addr == A_RSTCTRL) {
        s->rstctrl = val & R_RSTCTRL_IP_RESET_MASK;
        if (sc_spi_target_in_reset(s)) {
            sc_spi_target_ip_reset(s);
        }
        return;
    }

    /* All other registers are held in reset */
    if (sc_spi_target_in_reset(s)) {
        return;
    }

    switch (addr) {
    case A_SCRPAD:
        s->scratch_pad = val;
        break;
    case A_INTSTATS:
        s->intstats &= ~(val & SC_SPI_TARGET_INT_MASK);
        break;
    case A_INTENB:
        s->intenb = val & SC_SPI_TARGET_INT_MASK;
        break;
    case A_CTRL:
        s->ctrl = val & SC_SPI_TARGET_CTRL_MASK;
        if (!sc_spi_target_enabled(s)) {
            /* Disabling holds the shift logic in reset, aborting a frame */
            sc_spi_target_end_frame(s);
        }
        break;
    case A_FRMTHR:
        s->frmthr = val & R_FRMTHR_SIZE_MASK;
        break;
    case A_BUFCTRL:
        if (FIELD_EX32(val, BUFCTRL, CPU_BUF_CHANGE)) {
            /* The status of the buffer handed back is cleared */
            s->frame_size[s->cpu_buf] = 0;
            s->cs_det[s->cpu_buf] = 0;
            s->cpu_buf ^= 1;
        }
        if (FIELD_EX32(val, BUFCTRL, SPI_BUF_CHANGE)) {
            s->spi_buf ^= 1;
        }
        break;
    case A_GPO0:
    case A_GPO1:
        s->gpo[(addr - A_GPO0) / 4] = val & SC_SPI_TARGET_GPO_MASK;
        sc_spi_target_update_gpo(s);
        break;
    case A_IPVER:
    case A_IPCONF:
    case A_BUFSTATS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    default:
        if (sc_spi_target_buf_index(s, addr, A_TXBUF, &index)) {
            s->tx_buf[s->cpu_buf][index] = val;
        } else if (sc_spi_target_buf_index(s, addr, A_RXBUF, &index)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only RX buffer"
                          " 0x%" HWADDR_PRIx "\n", __func__, addr);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write offset 0x%"
                          HWADDR_PRIx "\n", __func__, addr);
        }
        break;
    }

    sc_spi_target_update(s);
}

static const MemoryRegionOps sc_spi_target_ops = {
    .read = sc_spi_target_read,
    .write = sc_spi_target_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sc_spi_target_reset(DeviceState *dev)
{
    SCSPITargetState *s = SC_SPI_TARGET(dev);

    s->rstctrl = R_RSTCTRL_IP_RESET_MASK;
    s->cs = 0;
    memset(s->rx_buf, 0, sizeof(s->rx_buf));
    sc_spi_target_ip_reset(s);
}

static void sc_spi_target_realize(DeviceState *dev, Error **errp)
{
    SCSPITargetState *s = SC_SPI_TARGET(dev);

    s->xfer_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 sc_spi_target_xfer_timer_cb, s);
    g_queue_init(&s->xfers);

    if (s->max_frame_size < 1 ||
        s->max_frame_size > SC_SPI_TARGET_MAX_FRAME_SIZE) {
        error_setg(errp, "max-frame-size must be between 1 and %d",
                   SC_SPI_TARGET_MAX_FRAME_SIZE);
        return;
    }

    for (int i = 0; i < SC_SPI_TARGET_NUM_GPO_PORTS; i++) {
        if (s->gpo_init[i] & ~SC_SPI_TARGET_GPO_MASK) {
            error_setg(errp, "gpo%d-init must fit in %d bits", i,
                       SC_SPI_TARGET_GPO_WIDTH);
            return;
        }
    }
}

static void sc_spi_target_init(Object *obj)
{
    SCSPITargetState *s = SC_SPI_TARGET(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &sc_spi_target_ops, s,
                          TYPE_SC_SPI_TARGET, SC_SPI_TARGET_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);

    /* Port N bit M is GPO line N * 4 + M */
    qdev_init_gpio_out_named(DEVICE(obj), s->gpo_out, "gpo",
                             ARRAY_SIZE(s->gpo_out));

    /* SPI controller on the other side of a remote-port link */
    object_property_add_link(obj, "rp-adaptor0", "remote-port",
                             (Object **)&s->rp,
                             qdev_prop_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
}

static const VMStateDescription vmstate_sc_spi_target = {
    .name = TYPE_SC_SPI_TARGET,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(rstctrl, SCSPITargetState),
        VMSTATE_UINT32(scratch_pad, SCSPITargetState),
        VMSTATE_UINT32(intstats, SCSPITargetState),
        VMSTATE_UINT32(intenb, SCSPITargetState),
        VMSTATE_UINT32(ctrl, SCSPITargetState),
        VMSTATE_UINT32(frmthr, SCSPITargetState),
        VMSTATE_UINT32_ARRAY(gpo, SCSPITargetState,
                             SC_SPI_TARGET_NUM_GPO_PORTS),
        VMSTATE_UINT8_2DARRAY(tx_buf, SCSPITargetState, 2,
                              SC_SPI_TARGET_MAX_FRAME_SIZE),
        VMSTATE_UINT8_2DARRAY(rx_buf, SCSPITargetState, 2,
                              SC_SPI_TARGET_MAX_FRAME_SIZE),
        VMSTATE_UINT8_ARRAY(frame_size, SCSPITargetState, 2),
        VMSTATE_UINT8_ARRAY(cs_det, SCSPITargetState, 2),
        VMSTATE_UINT8(cpu_buf, SCSPITargetState),
        VMSTATE_UINT8(spi_buf, SCSPITargetState),
        VMSTATE_UINT8(cs, SCSPITargetState),
        VMSTATE_BOOL(frame_active, SCSPITargetState),
        VMSTATE_UINT8(frame_bytes, SCSPITargetState),
        VMSTATE_BOOL(match_thr, SCSPITargetState),
        VMSTATE_END_OF_LIST()
    }
};

static Property sc_spi_target_props[] = {
    /* IP v1.0.0 */
    DEFINE_PROP_UINT32("ip-version", SCSPITargetState, ip_version,
                       0x01000000),
    DEFINE_PROP_UINT32("max-frame-size", SCSPITargetState, max_frame_size,
                       SC_SPI_TARGET_MAX_FRAME_SIZE),
    DEFINE_PROP_UINT32("gpo0-init", SCSPITargetState, gpo_init[0], 0),
    DEFINE_PROP_UINT32("gpo1-init", SCSPITargetState, gpo_init[1], 0),
    /* Remote-port transfers are shifted at this SCK rate, 0 is immediate */
    DEFINE_PROP_UINT32("sck-frequency", SCSPITargetState, sck_frequency,
                       100000),
    DEFINE_PROP_END_OF_LIST(),
};

static void sc_spi_target_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    RemotePortDeviceClass *rpdc = REMOTE_PORT_DEVICE_CLASS(oc);

    rpdc->ops[RP_CMD_ssi] = sc_spi_target_rp_ssi;
    dc->realize = sc_spi_target_realize;
    dc->reset = sc_spi_target_reset;
    dc->vmsd = &vmstate_sc_spi_target;
    device_class_set_props(dc, sc_spi_target_props);
}

static const TypeInfo sc_spi_target_info = {
    .name = TYPE_SC_SPI_TARGET,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SCSPITargetState),
    .instance_init = sc_spi_target_init,
    .class_init = sc_spi_target_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_REMOTE_PORT_DEVICE },
        { },
    },
};

static void sc_spi_target_register_types(void)
{
    type_register_static(&sc_spi_target_info);
}

type_init(sc_spi_target_register_types)
