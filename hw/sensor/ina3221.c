/*
 * Texas Instruments INA3221 triple-channel shunt and bus voltage monitor
 *
 * The measured voltages come from the QOM properties "chN-bus-mv" (bus
 * voltage in mV) and "chN-shunt-uv" (shunt voltage in uV) for channel
 * N = 1..3, which can be changed at runtime with qom-set.
 *
 * Conversions complete immediately, and in continuous mode every read
 * returns the current inputs. Alert limits are stored but not evaluated,
 * and the alert pins are not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/registerfields.h"
#include "migration/vmstate.h"

#define TYPE_INA3221 "ina3221"
OBJECT_DECLARE_SIMPLE_TYPE(INA3221State, INA3221)

#define INA3221_NUM_CHANNELS    3
#define INA3221_NUM_REGS        0x12

REG16(CONFIG, 0x00)
    FIELD(CONFIG, MODE_SHUNT, 0, 1)
    FIELD(CONFIG, MODE_BUS, 1, 1)
    FIELD(CONFIG, MODE_CONT, 2, 1)
    FIELD(CONFIG, RST, 15, 1)
REG16(SHUNT_V1, 0x01)
REG16(BUS_V1, 0x02)
REG16(CRIT_LIMIT1, 0x07)
REG16(SHUNT_SUM, 0x0D)
REG16(SHUNT_SUM_LIMIT, 0x0E)
REG16(MASK_ENABLE, 0x0F)
    FIELD(MASK_ENABLE, CVRF, 0, 1)
REG16(PV_UPPER_LIMIT, 0x10)
REG16(PV_LOWER_LIMIT, 0x11)
REG16(MANUF_ID, 0xFE)
REG16(DIE_ID, 0xFF)

#define INA3221_MANUF_ID_VALUE  0x5449
#define INA3221_DIE_ID_VALUE    0x3220

#define INA3221_BUS_LSB_MV      8
#define INA3221_SHUNT_LSB_UV    40

static const uint16_t ina3221_reset_values[INA3221_NUM_REGS] = {
    [A_CONFIG] = 0x7127,
    [A_CRIT_LIMIT1 ... A_CRIT_LIMIT1 + 5] = 0x7FF8,
    [A_SHUNT_SUM_LIMIT] = 0x7FFE,
    [A_MASK_ENABLE] = 0x0002,
    [A_PV_UPPER_LIMIT] = 0x2710,
    [A_PV_LOWER_LIMIT] = 0x2328,
};

struct INA3221State {
    I2CSlave parent_obj;

    /* I2C transaction state */
    uint8_t pointer;
    uint8_t len;
    uint8_t buf[2];

    uint16_t regs[INA3221_NUM_REGS];

    /* Analog inputs */
    int32_t bus_mv[INA3221_NUM_CHANNELS];
    int32_t shunt_uv[INA3221_NUM_CHANNELS];
};

/* Voltage registers hold a 13-bit signed value in bits 15:3 */
static uint16_t ina3221_encode(int32_t lsbs)
{
    return (uint16_t)(MIN(MAX(lsbs, -4096), 4095) << 3);
}

/* Latch the inputs of all enabled channels, as a finished conversion */
static void ina3221_convert(INA3221State *s)
{
    uint16_t config = s->regs[A_CONFIG];
    int ch;

    if (!FIELD_EX16(config, CONFIG, MODE_SHUNT) &&
        !FIELD_EX16(config, CONFIG, MODE_BUS)) {
        return;     /* Power-down */
    }

    for (ch = 0; ch < INA3221_NUM_CHANNELS; ch++) {
        /* Channel enable bits are CH1 = 14, CH2 = 13, CH3 = 12 */
        if (!(config & BIT(14 - ch))) {
            continue;
        }
        if (FIELD_EX16(config, CONFIG, MODE_SHUNT)) {
            s->regs[A_SHUNT_V1 + 2 * ch] =
                ina3221_encode(s->shunt_uv[ch] / INA3221_SHUNT_LSB_UV);
        }
        if (FIELD_EX16(config, CONFIG, MODE_BUS)) {
            s->regs[A_BUS_V1 + 2 * ch] =
                ina3221_encode(s->bus_mv[ch] / INA3221_BUS_LSB_MV);
        }
    }

    s->regs[A_MASK_ENABLE] |= R_MASK_ENABLE_CVRF_MASK;
}

static void ina3221_reset_regs(INA3221State *s)
{
    memcpy(s->regs, ina3221_reset_values, sizeof(s->regs));
    ina3221_convert(s);
}

static uint16_t ina3221_read_reg(INA3221State *s, uint8_t reg)
{
    uint16_t val;

    switch (reg) {
    case A_MANUF_ID:
        return INA3221_MANUF_ID_VALUE;
    case A_DIE_ID:
        return INA3221_DIE_ID_VALUE;
    case A_CONFIG ... A_PV_LOWER_LIMIT:
        if (FIELD_EX16(s->regs[A_CONFIG], CONFIG, MODE_CONT)) {
            ina3221_convert(s);
        }
        val = s->regs[reg];
        if (reg == A_MASK_ENABLE) {
            /* The next conversion is ready right away */
            ina3221_convert(s);
        }
        return val;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown register 0x%02x\n",
                      __func__, reg);
        return 0;
    }
}

static void ina3221_write_reg(INA3221State *s, uint8_t reg, uint16_t val)
{
    switch (reg) {
    case A_CONFIG:
        if (FIELD_EX16(val, CONFIG, RST)) {
            ina3221_reset_regs(s);
            return;
        }
        s->regs[reg] = val;
        ina3221_convert(s);
        return;
    case A_CRIT_LIMIT1 ... A_CRIT_LIMIT1 + 5:
    case A_SHUNT_SUM_LIMIT:
    case A_MASK_ENABLE:
    case A_PV_UPPER_LIMIT:
    case A_PV_LOWER_LIMIT:
        s->regs[reg] = val;
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only or unknown "
                      "register 0x%02x\n", __func__, reg);
        return;
    }
}

static uint8_t ina3221_recv(I2CSlave *i2c)
{
    INA3221State *s = INA3221(i2c);

    /* Registers are read MSB first; the pointer does not auto-increment */
    if (s->len < sizeof(s->buf)) {
        return s->buf[s->len++];
    }
    return 0xff;
}

static int ina3221_send(I2CSlave *i2c, uint8_t data)
{
    INA3221State *s = INA3221(i2c);

    if (s->len == 0) {
        s->pointer = data;
    } else if (s->len <= sizeof(s->buf)) {
        s->buf[s->len - 1] = data;
        if (s->len == sizeof(s->buf)) {
            ina3221_write_reg(s, s->pointer, (s->buf[0] << 8) | s->buf[1]);
        }
    }
    s->len++;

    return 0;
}

static int ina3221_event(I2CSlave *i2c, enum i2c_event event)
{
    INA3221State *s = INA3221(i2c);

    if (event == I2C_START_RECV) {
        uint16_t val = ina3221_read_reg(s, s->pointer);

        s->buf[0] = val >> 8;
        s->buf[1] = val;
    }
    s->len = 0;

    return 0;
}

static void ina3221_get_input(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    int32_t *input = opaque;

    visit_type_int32(v, name, input, errp);
}

static void ina3221_set_input(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    int32_t *input = opaque;
    int64_t val;

    /* int64 so that 32-bit device tree cells can express negative values */
    if (!visit_type_int64(v, name, &val, errp)) {
        return;
    }
    *input = (int32_t)val;
}

static void ina3221_reset(DeviceState *dev)
{
    INA3221State *s = INA3221(dev);

    s->pointer = 0;
    s->len = 0;
    ina3221_reset_regs(s);
}

static void ina3221_init(Object *obj)
{
    INA3221State *s = INA3221(obj);
    int ch;

    for (ch = 0; ch < INA3221_NUM_CHANNELS; ch++) {
        g_autofree char *bus = g_strdup_printf("ch%d-bus-mv", ch + 1);
        g_autofree char *shunt = g_strdup_printf("ch%d-shunt-uv", ch + 1);

        object_property_add(obj, bus, "int32", ina3221_get_input,
                            ina3221_set_input, NULL, &s->bus_mv[ch]);
        object_property_add(obj, shunt, "int32", ina3221_get_input,
                            ina3221_set_input, NULL, &s->shunt_uv[ch]);
    }
}

static const VMStateDescription vmstate_ina3221 = {
    .name = TYPE_INA3221,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(pointer, INA3221State),
        VMSTATE_UINT8(len, INA3221State),
        VMSTATE_UINT8_ARRAY(buf, INA3221State, 2),
        VMSTATE_UINT16_ARRAY(regs, INA3221State, INA3221_NUM_REGS),
        VMSTATE_INT32_ARRAY(bus_mv, INA3221State, INA3221_NUM_CHANNELS),
        VMSTATE_INT32_ARRAY(shunt_uv, INA3221State, INA3221_NUM_CHANNELS),
        VMSTATE_I2C_SLAVE(parent_obj, INA3221State),
        VMSTATE_END_OF_LIST()
    }
};

static void ina3221_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->reset = ina3221_reset;
    dc->vmsd = &vmstate_ina3221;
    k->event = ina3221_event;
    k->recv = ina3221_recv;
    k->send = ina3221_send;
}

static const TypeInfo ina3221_info = {
    .name = TYPE_INA3221,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(INA3221State),
    .instance_init = ina3221_init,
    .class_init = ina3221_class_init,
};

static void ina3221_register_types(void)
{
    type_register_static(&ina3221_info);
}

type_init(ina3221_register_types)
