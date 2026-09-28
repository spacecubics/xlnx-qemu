/*
 * Remote-port SSI peripheral
 *
 * Forwards chip select changes and transfers of an SSI bus to an SPI
 * target on the other side of a remote-port link. Every transfer blocks
 * until the peer returns the bytes shifted in, so the peer can take as
 * long as the transfer takes on the wire.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/remote-port.h"
#include "hw/remote-port-proto.h"
#include "hw/remote-port-device.h"
#include "trace.h"

#define TYPE_REMOTE_PORT_SSI "remote-port-ssi"
OBJECT_DECLARE_SIMPLE_TYPE(RemotePortSSI, REMOTE_PORT_SSI)

/* SSI transfers one word at a time */
#define RP_SSI_MAX_LEN  1

struct RemotePortSSI {
    SSIPeripheral parent_obj;

    struct RemotePort *rp;
    struct rp_peer_state *peer;
    uint32_t rp_dev;

    /* Chip selects of the target driven by this chip select */
    uint32_t cs_mask;
};

static void rp_ssi_xfer(RemotePortSSI *s, uint32_t cs, const uint8_t *tx,
                        uint8_t *rx, uint32_t len)
{
    struct {
        struct rp_pkt_ssi pkt;
        uint8_t data[RP_SSI_MAX_LEN];
    } pay;
    RemotePortRespSlot *rsp_slot;
    struct rp_pkt_ssi *rsp;
    uint32_t id;
    size_t enclen;

    assert(len <= RP_SSI_MAX_LEN);
    if (len) {
        memset(rx, 0, len);
    }

    if (!s->peer->caps.ssi) {
        warn_report_once("%s: remote-port peer does not support SSI",
                         object_get_canonical_path(OBJECT(s)));
        return;
    }

    id = rp_new_id(s->rp);
    enclen = rp_encode_ssi(id, s->rp_dev, &pay.pkt,
                           rp_normalized_vmclk(s->rp), cs, len, 0);
    if (len) {
        memcpy(pay.data, tx, len);
    }

    trace_remote_port_ssi_tx(id, s->rp_dev, cs, len, len ? tx[0] : 0);

    rp_rsp_mutex_lock(s->rp);
    rp_write(s->rp, &pay, enclen + len);

    rsp_slot = rp_dev_wait_resp(s->rp, s->rp_dev, id);
    rsp = &rsp_slot->rsp.pkt->ssi;
    assert(rsp->hdr.id == id);

    if (len) {
        memcpy(rx, rp_ssi_dataptr(rsp), MIN(len, rsp->len));
    }
    trace_remote_port_ssi_rx(id, s->rp_dev, rsp->len, len ? rx[0] : 0);

    rp_resp_slot_done(s->rp, rsp_slot);
    rp_rsp_mutex_unlock(s->rp);
}

static uint32_t rp_ssi_transfer(SSIPeripheral *dev, uint32_t val)
{
    RemotePortSSI *s = REMOTE_PORT_SSI(dev);
    uint8_t tx = val;
    uint8_t rx;

    rp_ssi_xfer(s, s->cs_mask, &tx, &rx, 1);
    return rx;
}

static int rp_ssi_set_cs(SSIPeripheral *dev, bool select)
{
    RemotePortSSI *s = REMOTE_PORT_SSI(dev);

    /* Active low: select is the line level */
    rp_ssi_xfer(s, select ? 0 : s->cs_mask, NULL, NULL, 0);
    return 0;
}

static void rp_ssi_realize(SSIPeripheral *dev, Error **errp)
{
    RemotePortSSI *s = REMOTE_PORT_SSI(dev);

    if (!s->rp) {
        error_setg(errp, "not connected to a remote-port adaptor");
        return;
    }
    s->peer = rp_get_peer(s->rp);
}

static void rp_ssi_init(Object *obj)
{
    RemotePortSSI *s = REMOTE_PORT_SSI(obj);

    object_property_add_link(obj, "rp-adaptor0", "remote-port",
                             (Object **)&s->rp,
                             qdev_prop_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
}

static Property rp_ssi_properties[] = {
    DEFINE_PROP_UINT32("rp-chan0", RemotePortSSI, rp_dev, 0),
    DEFINE_PROP_UINT32("cs-mask", RemotePortSSI, cs_mask, 0x1),
    DEFINE_PROP_END_OF_LIST(),
};

static void rp_ssi_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(oc);

    k->realize = rp_ssi_realize;
    k->transfer = rp_ssi_transfer;
    k->set_cs = rp_ssi_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    device_class_set_props(dc, rp_ssi_properties);
}

static const TypeInfo rp_ssi_info = {
    .name          = TYPE_REMOTE_PORT_SSI,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(RemotePortSSI),
    .instance_init = rp_ssi_init,
    .class_init    = rp_ssi_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_REMOTE_PORT_DEVICE },
        { },
    },
};

static void rp_ssi_register_types(void)
{
    type_register_static(&rp_ssi_info);
}

type_init(rp_ssi_register_types)
