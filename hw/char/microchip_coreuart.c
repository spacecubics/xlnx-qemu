#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/char/microchip_coreuart.h"
#include "hw/qdev-properties-system.h"


static uint64_t microchip_coreuart_read(void *opaque, hwaddr addr,
                                                  unsigned size)
{
    MicrochipCoreUARTState *s = opaque;

    switch (addr) {
    case 0x4:
        s->rx_full = false;
        qemu_chr_fe_accept_input(&s->chr);
        return s->rx;
    case 0x8:
        return s->ctrl1;
    case 0xc:
        return s->ctrl2;
    case 0x10:
        return 0x1 | (s->rx_full ? 0x2 : 0); /* TXRDY | RXFULL */
    default:
        return 0;
    }
}

static void microchip_coreuart_write(void *opaque, hwaddr addr,
                                               uint64_t value, unsigned size)
{
    MicrochipCoreUARTState *s = opaque;
    uint8_t ch = value;

    switch (addr) {
    case 0x0:
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        break;
    case 0x8:
        s->ctrl1 = value;
        break;
    case 0xc:
        s->ctrl2 = value;
        break;
    }
}

static const MemoryRegionOps microchip_coreuart_ops = {
    .read = microchip_coreuart_read,
    .write = microchip_coreuart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static int microchip_coreuart_can_receive(void *opaque)
{
    return !MICROCHIP_COREUART(opaque)->rx_full;
}

static void microchip_coreuart_receive(void *opaque,
                                                 const uint8_t *buf, int size)
{
    MicrochipCoreUARTState *s = opaque;

    s->rx = buf[0];
    s->rx_full = true;
}

static void microchip_coreuart_reset(DeviceState *dev)
{
    MicrochipCoreUARTState *s = MICROCHIP_COREUART(dev);

    s->rx = 0;
    s->ctrl1 = 0;
    s->ctrl2 = 0;
    s->rx_full = false;
}

static void microchip_coreuart_realize(DeviceState *dev,
                                                 Error **errp)
{
    MicrochipCoreUARTState *s = MICROCHIP_COREUART(dev);

    qemu_chr_fe_set_handlers(&s->chr, microchip_coreuart_can_receive,
                             microchip_coreuart_receive, NULL, NULL,
                             s, NULL, true);
}

static void microchip_coreuart_init(Object *obj)
{
    MicrochipCoreUARTState *s = MICROCHIP_COREUART(obj);

    memory_region_init_io(&s->mmio, obj, &microchip_coreuart_ops, s,
                          TYPE_MICROCHIP_COREUART, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static Property microchip_coreuart_props[] = {
    DEFINE_PROP_CHR("chardev", MicrochipCoreUARTState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void microchip_coreuart_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = microchip_coreuart_realize;
    dc->reset = microchip_coreuart_reset;
    device_class_set_props(dc, microchip_coreuart_props);
}

static const TypeInfo microchip_coreuart_type_info = {
    .name = TYPE_MICROCHIP_COREUART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MicrochipCoreUARTState),
    .instance_init = microchip_coreuart_init,
    .class_init = microchip_coreuart_class_init,
};

static void microchip_coreuart_register_types(void)
{
    type_register_static(&microchip_coreuart_type_info);
}

type_init(microchip_coreuart_register_types)
