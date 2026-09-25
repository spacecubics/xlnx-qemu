/*
 * Space Cubics SC-OBC Module V1 Safety Processor System Register
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TYPE_SC_SYSTEM "sc.system"
OBJECT_DECLARE_SIMPLE_TYPE(SCSystemState, SC_SYSTEM)

REG32(IP_VERSION, 0x000)
    FIELD(IP_VERSION, PATCH, 0, 16)
    FIELD(IP_VERSION, MINOR, 16, 8)
    FIELD(IP_VERSION, MAJOR, 24, 8)
REG32(GIT_HASH, 0x004)
    FIELD(GIT_HASH, REPO_STATUS, 0, 8)
    FIELD(GIT_HASH, HASH, 8, 24)
REG32(SCRATCH_PAD, 0x008)
REG32(BOARD_ID, 0x010)
    FIELD(BOARD_ID, GRADE, 0, 1)
    FIELD(BOARD_ID, REVISION, 1, 1)
REG32(LED_CTRL, 0x020)
    FIELD(LED_CTRL, GREEN, 0, 1)
    FIELD(LED_CTRL, RED, 1, 1)
REG32(KEEP_ALIVE, 0x024)
    FIELD(KEEP_ALIVE, CTRL, 0, 1)
REG32(BOOT_MEM_SEL, 0x028)
    FIELD(BOOT_MEM_SEL, SEL, 0, 1)

#define SC_SYSTEM_MMIO_SIZE 0x1000

struct SCSystemState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq led[2];
    qemu_irq keep_alive;
    qemu_irq boot_mem_sel;

    /* Hardwired values, configured per board */
    uint32_t ip_version;
    uint32_t git_hash;
    uint32_t board_id;

    uint32_t scratch_pad;
    uint32_t led_ctrl;
    uint32_t keep_alive_ctrl;
    uint32_t boot_mem_sel_ctrl;
};

static void sc_system_update_outputs(SCSystemState *s)
{
    qemu_set_irq(s->led[0], FIELD_EX32(s->led_ctrl, LED_CTRL, GREEN));
    qemu_set_irq(s->led[1], FIELD_EX32(s->led_ctrl, LED_CTRL, RED));
    qemu_set_irq(s->keep_alive,
                 FIELD_EX32(s->keep_alive_ctrl, KEEP_ALIVE, CTRL));
    qemu_set_irq(s->boot_mem_sel,
                 FIELD_EX32(s->boot_mem_sel_ctrl, BOOT_MEM_SEL, SEL));
}

static uint64_t sc_system_read(void *opaque, hwaddr addr, unsigned size)
{
    SCSystemState *s = opaque;
    uint32_t val;

    switch (addr) {
    case A_IP_VERSION:
        val = s->ip_version;
        break;
    case A_GIT_HASH:
        val = s->git_hash;
        break;
    case A_SCRATCH_PAD:
        val = s->scratch_pad;
        break;
    case A_BOARD_ID:
        val = s->board_id & (R_BOARD_ID_GRADE_MASK | R_BOARD_ID_REVISION_MASK);
        break;
    case A_LED_CTRL:
        val = s->led_ctrl;
        break;
    case A_KEEP_ALIVE:
        val = s->keep_alive_ctrl;
        break;
    case A_BOOT_MEM_SEL:
        val = s->boot_mem_sel_ctrl;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%" HWADDR_PRIx
                      "\n", __func__, addr);
        val = 0;
        break;
    }

    trace_sc_system_read(addr, val);
    return val;
}

static void sc_system_write(void *opaque, hwaddr addr, uint64_t val64,
                            unsigned size)
{
    SCSystemState *s = opaque;
    uint32_t val = val64;

    trace_sc_system_write(addr, val);

    switch (addr) {
    case A_SCRATCH_PAD:
        s->scratch_pad = val;
        break;
    case A_LED_CTRL:
        s->led_ctrl = val & (R_LED_CTRL_GREEN_MASK | R_LED_CTRL_RED_MASK);
        trace_sc_system_led(FIELD_EX32(s->led_ctrl, LED_CTRL, GREEN),
                            FIELD_EX32(s->led_ctrl, LED_CTRL, RED));
        break;
    case A_KEEP_ALIVE:
        s->keep_alive_ctrl = val & R_KEEP_ALIVE_CTRL_MASK;
        trace_sc_system_keep_alive(s->keep_alive_ctrl);
        break;
    case A_BOOT_MEM_SEL:
        s->boot_mem_sel_ctrl = val & R_BOOT_MEM_SEL_SEL_MASK;
        trace_sc_system_boot_mem_sel(s->boot_mem_sel_ctrl);
        break;
    case A_IP_VERSION:
    case A_GIT_HASH:
    case A_BOARD_ID:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write offset 0x%" HWADDR_PRIx
                      "\n", __func__, addr);
        return;
    }

    sc_system_update_outputs(s);
}

static const MemoryRegionOps sc_system_ops = {
    .read = sc_system_read,
    .write = sc_system_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sc_system_reset(DeviceState *dev)
{
    SCSystemState *s = SC_SYSTEM(dev);

    s->scratch_pad = 0;
    s->led_ctrl = 0;
    s->keep_alive_ctrl = 0;
    s->boot_mem_sel_ctrl = 0;
    sc_system_update_outputs(s);
}

static void sc_system_init(Object *obj)
{
    SCSystemState *s = SC_SYSTEM(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &sc_system_ops, s, TYPE_SC_SYSTEM,
                          SC_SYSTEM_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    qdev_init_gpio_out_named(dev, s->led, "led", ARRAY_SIZE(s->led));
    qdev_init_gpio_out_named(dev, &s->keep_alive, "keep-alive", 1);
    qdev_init_gpio_out_named(dev, &s->boot_mem_sel, "boot-mem-sel", 1);
}

static const VMStateDescription vmstate_sc_system = {
    .name = TYPE_SC_SYSTEM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(scratch_pad, SCSystemState),
        VMSTATE_UINT32(led_ctrl, SCSystemState),
        VMSTATE_UINT32(keep_alive_ctrl, SCSystemState),
        VMSTATE_UINT32(boot_mem_sel_ctrl, SCSystemState),
        VMSTATE_END_OF_LIST()
    }
};

static Property sc_system_props[] = {
    DEFINE_PROP_UINT32("ip-version", SCSystemState, ip_version, 0),
    DEFINE_PROP_UINT32("git-hash", SCSystemState, git_hash, 0),
    DEFINE_PROP_UINT32("board-id", SCSystemState, board_id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void sc_system_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->reset = sc_system_reset;
    dc->vmsd = &vmstate_sc_system;
    device_class_set_props(dc, sc_system_props);
}

static const TypeInfo sc_system_info = {
    .name = TYPE_SC_SYSTEM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SCSystemState),
    .instance_init = sc_system_init,
    .class_init = sc_system_class_init,
};

static void sc_system_register_types(void)
{
    type_register_static(&sc_system_info);
}

type_init(sc_system_register_types)
