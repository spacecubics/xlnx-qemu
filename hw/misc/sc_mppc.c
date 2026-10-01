/*
 * Space Cubics SC-OBC Module V1 Main Processor Power Controller (MPPC)
 *
 * The safety processor uses the MPPC to sequence the power domains of the
 * main processor and to control its reset. QEMU has no power supplies, so
 * a rail becomes Power Good as soon as it is enabled. Supply faults can be
 * injected at runtime through the "power-fault" property, a bit mask in
 * the layout of the Power Status Register, e.g.
 *
 *   qom-set /machine/.../mppc power-fault 0x8     (FPD rail fails)
 *
 * The reset is also available as "main-por-b", the level of the Versal
 * POR_B pin it drives on the board. In the device tree the outputs are
 * GPIOs 0-4 (power enables), 5 (main-reset) and 6 (main-por-b), and GPIO
 * input 0 is the Power Cycle Request from PMC MIO28 of the main processor.
 *
 * While the reset is asserted the Versal MIO are not driven and the pull
 * down on the safety processor side holds Power Cycle Request low. QEMU
 * does not reset the GPIO pins of the main processor on POR, so drop the
 * request here and ignore it until the reset is released. It is raised
 * again once software on the main processor drives MIO28.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/fdt_generic_util.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TYPE_SC_MPPC "sc.mppc"
OBJECT_DECLARE_SIMPLE_TYPE(SCMPPCState, SC_MPPC)

REG32(IP_VERSION, 0x000)
    FIELD(IP_VERSION, PATCH, 0, 16)
    FIELD(IP_VERSION, MINOR, 16, 8)
    FIELD(IP_VERSION, MAJOR, 24, 8)
REG32(INTSTS, 0x010)
REG32(INTENB, 0x014)
    /* Error bits 6:0 share the layout of POWER_STATUS */
    FIELD(INT, ERR, 0, 7)
    FIELD(INT, POWER_CYCLE_REQ, 16, 1)
REG32(POWER_STATUS, 0x020)
    FIELD(POWER_STATUS, VDD_OUT, 0, 1)
    FIELD(POWER_STATUS, PMC_LPD, 1, 1)
    FIELD(POWER_STATUS, PMC_MIO, 2, 1)
    FIELD(POWER_STATUS, FPD, 3, 1)
    FIELD(POWER_STATUS, SPD, 4, 1)
    FIELD(POWER_STATUS, PL, 5, 1)
    FIELD(POWER_STATUS, GTYP, 6, 1)
REG32(MAIN_RESET, 0x024)
    FIELD(MAIN_RESET, ASSERT, 0, 1)
REG32(POWER_CYCLE_REQ, 0x028)
    FIELD(POWER_CYCLE_REQ, REQ, 0, 1)
REG32(PMC_LPD_CTRL, 0x040)
REG32(FPD_CTRL, 0x044)
REG32(SPD_CTRL, 0x048)
REG32(PL_CTRL, 0x04c)
REG32(GTYP_CTRL, 0x050)
    /* Common layout of the power control/status registers */
    FIELD(CTRL, ENABLE, 0, 1)
    FIELD(CTRL, STATUS, 8, 2)
    FIELD(CTRL, KEY, 16, 16)

#define SC_MPPC_WRITE_KEY       0x5a5a
#define SC_MPPC_MMIO_SIZE       0x1000
#define SC_MPPC_INT_MASK        (R_INT_ERR_MASK | R_INT_POWER_CYCLE_REQ_MASK)
#define SC_MPPC_RAIL_MASK       R_INT_ERR_MASK

enum {
    SC_MPPC_PMC_LPD,
    SC_MPPC_FPD,
    SC_MPPC_SPD,
    SC_MPPC_PL,
    SC_MPPC_GTYP,
    SC_MPPC_NUM_DOMAINS,
};

typedef struct SCMPPCDomain {
    const char *name;
    /* Rails switched by the enable bit, in POWER_STATUS layout */
    uint32_t rails;
    /* Rails that must be Power Good before the enable bit can be set */
    uint32_t on_requires;
    /* Rails that must be off before the enable bit can be cleared */
    uint32_t off_requires;
} SCMPPCDomain;

#define PS(x) R_POWER_STATUS_##x##_MASK

static const SCMPPCDomain sc_mppc_domains[SC_MPPC_NUM_DOMAINS] = {
    [SC_MPPC_PMC_LPD] = {
        .name = "pmc-lpd",
        .rails = PS(PMC_LPD) | PS(PMC_MIO),
        .on_requires = PS(VDD_OUT),
        .off_requires = PS(FPD) | PS(SPD) | PS(PL) | PS(GTYP),
    },
    [SC_MPPC_FPD] = {
        .name = "fpd",
        .rails = PS(FPD),
        .on_requires = PS(VDD_OUT) | PS(PMC_LPD) | PS(PMC_MIO),
    },
    [SC_MPPC_SPD] = {
        .name = "spd",
        .rails = PS(SPD),
        .on_requires = PS(VDD_OUT) | PS(PMC_LPD) | PS(PMC_MIO),
        .off_requires = PS(PL) | PS(GTYP),
    },
    [SC_MPPC_PL] = {
        .name = "pl",
        .rails = PS(PL),
        .on_requires = PS(VDD_OUT) | PS(PMC_LPD) | PS(PMC_MIO) | PS(SPD),
        .off_requires = PS(GTYP),
    },
    [SC_MPPC_GTYP] = {
        .name = "gtyp",
        .rails = PS(GTYP),
        .on_requires = PS(VDD_OUT) | PS(PMC_LPD) | PS(PMC_MIO) | PS(SPD) |
                       PS(PL),
    },
};

struct SCMPPCState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    qemu_irq main_reset;
    qemu_irq main_por_b;
    qemu_irq power_enable[SC_MPPC_NUM_DOMAINS];

    /* Hardwired values, configured per board */
    uint32_t ip_version;

    uint32_t intsts;
    uint32_t intenb;
    uint32_t main_reset_ctrl;
    /* Bit N set when domain N is enabled */
    uint32_t enabled;
    /* POWER_STATUS as last evaluated, to detect rails dropping out */
    uint32_t power_status;
    bool power_cycle_req;

    /* Injected supply faults, in POWER_STATUS layout */
    uint32_t power_fault;
};

static uint32_t sc_mppc_rails_enabled(SCMPPCState *s)
{
    /* VDD OUT is on whenever the OBC is powered */
    uint32_t rails = PS(VDD_OUT);
    int i;

    for (i = 0; i < SC_MPPC_NUM_DOMAINS; i++) {
        if (s->enabled & BIT(i)) {
            rails |= sc_mppc_domains[i].rails;
        }
    }

    return rails;
}

static void sc_mppc_update_irq(SCMPPCState *s)
{
    qemu_set_irq(s->irq, !!(s->intsts & s->intenb));
}

static bool sc_mppc_main_reset_asserted(SCMPPCState *s)
{
    return FIELD_EX32(s->main_reset_ctrl, MAIN_RESET, ASSERT);
}

static void sc_mppc_update(SCMPPCState *s)
{
    uint32_t rails = sc_mppc_rails_enabled(s);
    uint32_t status = rails & ~s->power_fault;
    uint32_t dropped = s->power_status & ~status & rails;
    int i;

    /* MIO28 of the main processor is not driven while it is in reset */
    if (sc_mppc_main_reset_asserted(s)) {
        s->power_cycle_req = false;
    }

    if (dropped) {
        trace_sc_mppc_power_error(dropped);
        s->intsts |= dropped;
    }
    if (status != s->power_status) {
        trace_sc_mppc_power_status(status);
        s->power_status = status;
    }

    for (i = 0; i < SC_MPPC_NUM_DOMAINS; i++) {
        qemu_set_irq(s->power_enable[i], !!(s->enabled & BIT(i)));
    }
    qemu_set_irq(s->main_reset, sc_mppc_main_reset_asserted(s));
    qemu_set_irq(s->main_por_b, !sc_mppc_main_reset_asserted(s));
    sc_mppc_update_irq(s);
}

static uint32_t sc_mppc_domain_read(SCMPPCState *s, int dom)
{
    uint32_t rails = sc_mppc_domains[dom].rails;
    uint32_t val = 0;

    val = FIELD_DP32(val, CTRL, ENABLE, !!(s->enabled & BIT(dom)));
    val = FIELD_DP32(val, CTRL, STATUS,
                     (s->power_status & rails) >> ctz32(rails));

    return val;
}

static void sc_mppc_domain_write(SCMPPCState *s, int dom, uint32_t val)
{
    const SCMPPCDomain *d = &sc_mppc_domains[dom];
    bool enable = FIELD_EX32(val, CTRL, ENABLE);

    if (FIELD_EX32(val, CTRL, KEY) != SC_MPPC_WRITE_KEY) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s write without key: 0x%08"
                      PRIx32 "\n", __func__, d->name, val);
        return;
    }
    if (enable == !!(s->enabled & BIT(dom))) {
        return;
    }

    if (enable && (s->power_status & d->on_requires) != d->on_requires) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s enable ignored, required "
                      "rails not good (status 0x%02" PRIx32 ")\n", __func__,
                      d->name, s->power_status);
        return;
    }
    if (!enable && (s->power_status & d->off_requires)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s disable ignored, dependent "
                      "rails still good (status 0x%02" PRIx32 ")\n", __func__,
                      d->name, s->power_status);
        return;
    }

    trace_sc_mppc_power_enable(d->name, enable);
    if (enable) {
        s->enabled |= BIT(dom);
    } else {
        s->enabled &= ~BIT(dom);
    }
}

static uint64_t sc_mppc_read(void *opaque, hwaddr addr, unsigned size)
{
    SCMPPCState *s = opaque;
    uint32_t val;

    switch (addr) {
    case A_IP_VERSION:
        val = s->ip_version;
        break;
    case A_INTSTS:
        val = s->intsts;
        break;
    case A_INTENB:
        val = s->intenb;
        break;
    case A_POWER_STATUS:
        val = s->power_status;
        break;
    case A_MAIN_RESET:
        val = s->main_reset_ctrl;
        break;
    case A_POWER_CYCLE_REQ:
        val = s->power_cycle_req;
        break;
    case A_PMC_LPD_CTRL:
    case A_FPD_CTRL:
    case A_SPD_CTRL:
    case A_PL_CTRL:
    case A_GTYP_CTRL:
        val = sc_mppc_domain_read(s, (addr - A_PMC_LPD_CTRL) / 4);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%" HWADDR_PRIx
                      "\n", __func__, addr);
        val = 0;
        break;
    }

    trace_sc_mppc_read(addr, val);
    return val;
}

static void sc_mppc_write(void *opaque, hwaddr addr, uint64_t val64,
                          unsigned size)
{
    SCMPPCState *s = opaque;
    uint32_t val = val64;

    trace_sc_mppc_write(addr, val);

    switch (addr) {
    case A_INTSTS:
        s->intsts &= ~(val & SC_MPPC_INT_MASK);
        break;
    case A_INTENB:
        s->intenb = val & SC_MPPC_INT_MASK;
        break;
    case A_MAIN_RESET:
        /* Documented as key protected, but the RTL does not check the key */
        s->main_reset_ctrl = val & R_MAIN_RESET_ASSERT_MASK;
        trace_sc_mppc_main_reset(s->main_reset_ctrl);
        break;
    case A_PMC_LPD_CTRL:
    case A_FPD_CTRL:
    case A_SPD_CTRL:
    case A_PL_CTRL:
    case A_GTYP_CTRL:
        sc_mppc_domain_write(s, (addr - A_PMC_LPD_CTRL) / 4, val);
        break;
    case A_IP_VERSION:
    case A_POWER_STATUS:
    case A_POWER_CYCLE_REQ:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write offset 0x%" HWADDR_PRIx
                      "\n", __func__, addr);
        return;
    }

    sc_mppc_update(s);
}

static const MemoryRegionOps sc_mppc_ops = {
    .read = sc_mppc_read,
    .write = sc_mppc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* Power Cycle Request output of the main processor */
static void sc_mppc_power_cycle_req(void *opaque, int n, int level)
{
    SCMPPCState *s = opaque;

    trace_sc_mppc_power_cycle_req(level);
    if (sc_mppc_main_reset_asserted(s)) {
        return;
    }
    if (level && !s->power_cycle_req) {
        s->intsts |= R_INT_POWER_CYCLE_REQ_MASK;
    }
    s->power_cycle_req = level;
    sc_mppc_update_irq(s);
}

static void sc_mppc_get_fault(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    SCMPPCState *s = SC_MPPC(obj);

    visit_type_uint32(v, name, &s->power_fault, errp);
}

static void sc_mppc_set_fault(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    SCMPPCState *s = SC_MPPC(obj);
    uint32_t val;

    if (!visit_type_uint32(v, name, &val, errp)) {
        return;
    }
    s->power_fault = val & SC_MPPC_RAIL_MASK;
    if (DEVICE(obj)->realized) {
        sc_mppc_update(s);
    }
}

static void sc_mppc_reset(DeviceState *dev)
{
    SCMPPCState *s = SC_MPPC(dev);

    s->intsts = 0;
    s->intenb = 0;
    s->main_reset_ctrl = R_MAIN_RESET_ASSERT_MASK;
    s->enabled = 0;
    s->power_status = sc_mppc_rails_enabled(s) & ~s->power_fault;
    sc_mppc_update(s);
}

static void sc_mppc_init(Object *obj)
{
    SCMPPCState *s = SC_MPPC(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &sc_mppc_ops, s, TYPE_SC_MPPC,
                          SC_MPPC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    /* Indexed by domain: PMC/LPD, FPD, SPD, PL, GTYP */
    qdev_init_gpio_out_named(dev, s->power_enable, "power-enable",
                             SC_MPPC_NUM_DOMAINS);
    qdev_init_gpio_out_named(dev, &s->main_reset, "main-reset", 1);
    qdev_init_gpio_out_named(dev, &s->main_por_b, "main-por-b", 1);
    qdev_init_gpio_in_named(dev, sc_mppc_power_cycle_req,
                            "power-cycle-req", 1);

    object_property_add(obj, "power-fault", "uint32", sc_mppc_get_fault,
                        sc_mppc_set_fault, NULL, NULL);
}

static const VMStateDescription vmstate_sc_mppc = {
    .name = TYPE_SC_MPPC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(intsts, SCMPPCState),
        VMSTATE_UINT32(intenb, SCMPPCState),
        VMSTATE_UINT32(main_reset_ctrl, SCMPPCState),
        VMSTATE_UINT32(enabled, SCMPPCState),
        VMSTATE_UINT32(power_status, SCMPPCState),
        VMSTATE_BOOL(power_cycle_req, SCMPPCState),
        VMSTATE_UINT32(power_fault, SCMPPCState),
        VMSTATE_END_OF_LIST()
    }
};

static const FDTGenericGPIOSet sc_mppc_controller_gpios[] = {
    {
        .names = &fdt_generic_gpio_name_set_gpio,
        .gpios = (FDTGenericGPIOConnection[]) {
            { .name = "power-enable", .fdt_index = 0,
              .range = SC_MPPC_NUM_DOMAINS },
            { .name = "main-reset", .fdt_index = SC_MPPC_NUM_DOMAINS },
            { .name = "main-por-b", .fdt_index = SC_MPPC_NUM_DOMAINS + 1 },
            { },
        },
    },
    { },
};

static const FDTGenericGPIOSet sc_mppc_client_gpios[] = {
    {
        .names = &fdt_generic_gpio_name_set_gpio,
        .gpios = (FDTGenericGPIOConnection[]) {
            { .name = "power-cycle-req", .fdt_index = 0 },
            { },
        },
    },
    { },
};

static Property sc_mppc_props[] = {
    DEFINE_PROP_UINT32("ip-version", SCMPPCState, ip_version, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void sc_mppc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    FDTGenericGPIOClass *fggc = FDT_GENERIC_GPIO_CLASS(oc);

    dc->reset = sc_mppc_reset;
    dc->vmsd = &vmstate_sc_mppc;
    device_class_set_props(dc, sc_mppc_props);
    fggc->controller_gpios = sc_mppc_controller_gpios;
    fggc->client_gpios = sc_mppc_client_gpios;
}

static const TypeInfo sc_mppc_info = {
    .name = TYPE_SC_MPPC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SCMPPCState),
    .instance_init = sc_mppc_init,
    .class_init = sc_mppc_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_FDT_GENERIC_GPIO },
        { }
    },
};

static void sc_mppc_register_types(void)
{
    type_register_static(&sc_mppc_info);
}

type_init(sc_mppc_register_types)
