/*
 * Space Cubics SC-OBC Module V1 Boot Memory Select
 *
 * An analog switch between the Versal QSPI controller and the two boot
 * flashes. It connects one of the flashes to the controller and leaves
 * the other one disconnected, with its chip select pulled up. The safety
 * processor drives the select line (Boot Memory Select register).
 *
 * The switch sits on the controller bus like a flash. Bank 0 is attached
 * to its "spi0" bus and chip select 0, bank 1 to "spi1" and chip select 1,
 * i.e. reg = <0 0> and <1 1> in the device tree. GPIO input 0 is the
 * select line.
 *
 * Like the hardware, a change of the select line takes effect at once.
 * The newly selected flash sees the current level of the chip select.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "hw/fdt_generic_util.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TYPE_SC_QSPI_MUX "sc.qspi-mux"
OBJECT_DECLARE_SIMPLE_TYPE(SCQSPIMuxState, SC_QSPI_MUX)

#define SC_QSPI_MUX_NUM_BANKS   2

struct SCQSPIMuxState {
    SSIPeripheral parent_obj;

    SSIBus *bank[SC_QSPI_MUX_NUM_BANKS];
    qemu_irq bank_cs[SC_QSPI_MUX_NUM_BANKS];

    /* Level of the chip select from the controller, active low */
    bool cs_level;
    /* Select line from the safety processor */
    uint8_t sel;
};

static void sc_qspi_mux_update_cs(SCQSPIMuxState *s)
{
    int i;

    /* The disconnected flash is deselected by its pull up */
    for (i = 0; i < SC_QSPI_MUX_NUM_BANKS; i++) {
        qemu_set_irq(s->bank_cs[i], i == s->sel ? s->cs_level : 1);
    }
}

static uint32_t sc_qspi_mux_transfer(SSIPeripheral *dev, uint32_t val)
{
    SCQSPIMuxState *s = SC_QSPI_MUX(dev);

    return ssi_transfer(s->bank[s->sel], val);
}

static int sc_qspi_mux_set_cs(SSIPeripheral *dev, bool level)
{
    SCQSPIMuxState *s = SC_QSPI_MUX(dev);

    s->cs_level = level;
    sc_qspi_mux_update_cs(s);
    return 0;
}

static void sc_qspi_mux_sel(void *opaque, int n, int level)
{
    SCQSPIMuxState *s = opaque;
    uint8_t sel = !!level;

    if (s->sel == sel) {
        return;
    }

    trace_sc_qspi_mux_sel(sel, s->cs_level);
    s->sel = sel;
    sc_qspi_mux_update_cs(s);
}

/*
 * The select line belongs to the safety processor and survives a reset
 * of the main processor, so only the chip select returns to idle.
 */
static void sc_qspi_mux_reset(DeviceState *dev)
{
    SCQSPIMuxState *s = SC_QSPI_MUX(dev);

    s->cs_level = 1;
    sc_qspi_mux_update_cs(s);
}

static void sc_qspi_mux_realize(SSIPeripheral *dev, Error **errp)
{
    SCQSPIMuxState *s = SC_QSPI_MUX(dev);
    char name[8];
    int i;

    for (i = 0; i < SC_QSPI_MUX_NUM_BANKS; i++) {
        snprintf(name, sizeof(name), "spi%d", i);
        s->bank[i] = ssi_create_bus(DEVICE(dev), name);
    }
}

static void sc_qspi_mux_init(Object *obj)
{
    SCQSPIMuxState *s = SC_QSPI_MUX(obj);
    DeviceState *dev = DEVICE(obj);

    /* Unnamed, so flashes connect to them by their chip select index */
    qdev_init_gpio_out(dev, s->bank_cs, SC_QSPI_MUX_NUM_BANKS);
    qdev_init_gpio_in_named(dev, sc_qspi_mux_sel, "sel", 1);
}

static const VMStateDescription vmstate_sc_qspi_mux = {
    .name = TYPE_SC_QSPI_MUX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, SCQSPIMuxState),
        VMSTATE_BOOL(cs_level, SCQSPIMuxState),
        VMSTATE_UINT8(sel, SCQSPIMuxState),
        VMSTATE_END_OF_LIST()
    }
};

static const FDTGenericGPIOSet sc_qspi_mux_client_gpios[] = {
    {
        .names = &fdt_generic_gpio_name_set_gpio,
        .gpios = (FDTGenericGPIOConnection[]) {
            { .name = "sel", .fdt_index = 0 },
            { },
        },
    },
    { },
};

static void sc_qspi_mux_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(oc);
    FDTGenericGPIOClass *fggc = FDT_GENERIC_GPIO_CLASS(oc);

    k->realize = sc_qspi_mux_realize;
    k->transfer = sc_qspi_mux_transfer;
    k->set_cs = sc_qspi_mux_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->reset = sc_qspi_mux_reset;
    dc->vmsd = &vmstate_sc_qspi_mux;
    fggc->client_gpios = sc_qspi_mux_client_gpios;
}

static const TypeInfo sc_qspi_mux_info = {
    .name = TYPE_SC_QSPI_MUX,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SCQSPIMuxState),
    .instance_init = sc_qspi_mux_init,
    .class_init = sc_qspi_mux_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_FDT_GENERIC_GPIO },
        { }
    },
};

static void sc_qspi_mux_register_types(void)
{
    type_register_static(&sc_qspi_mux_info);
}

type_init(sc_qspi_mux_register_types)
