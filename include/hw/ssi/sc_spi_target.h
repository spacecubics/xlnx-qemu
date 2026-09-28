/*
 * Space Cubics Single SPI Target (SC-SPI-TARGET)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_SC_SPI_TARGET_H
#define HW_SSI_SC_SPI_TARGET_H

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_SC_SPI_TARGET "sc.spi-target"
OBJECT_DECLARE_SIMPLE_TYPE(SCSPITargetState, SC_SPI_TARGET)

#define SC_SPI_TARGET_MAX_FRAME_SIZE    32
#define SC_SPI_TARGET_NUM_CS            4
#define SC_SPI_TARGET_NUM_GPO_PORTS     2
#define SC_SPI_TARGET_GPO_WIDTH         4

struct SCSPITargetState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    qemu_irq gpo_out[SC_SPI_TARGET_NUM_GPO_PORTS * SC_SPI_TARGET_GPO_WIDTH];

    /* Synthesis parameters */
    uint32_t ip_version;
    uint32_t max_frame_size;
    uint32_t gpo_init[SC_SPI_TARGET_NUM_GPO_PORTS];

    /* Registers */
    uint32_t rstctrl;
    uint32_t scratch_pad;
    uint32_t intstats;
    uint32_t intenb;
    uint32_t ctrl;
    uint32_t frmthr;
    uint32_t gpo[SC_SPI_TARGET_NUM_GPO_PORTS];

    /* Double buffer, indexed by buffer number */
    uint8_t tx_buf[2][SC_SPI_TARGET_MAX_FRAME_SIZE];
    uint8_t rx_buf[2][SC_SPI_TARGET_MAX_FRAME_SIZE];
    uint8_t frame_size[2];
    uint8_t cs_det[2];
    uint8_t cpu_buf;
    uint8_t spi_buf;

    /* Remote-port link to the SPI controller */
    struct RemotePort *rp;
    uint32_t sck_frequency;
    QEMUTimer *xfer_timer;
    GQueue xfers;

    /* SPI side */
    uint8_t cs;
    bool frame_active;
    uint8_t frame_bytes;
    bool match_thr;
};

/*
 * SPI controller side interface. @cs is the mask of asserted chip
 * selects (active high, bit N for CSn[N]); a frame ends when all chip
 * selects are released. Each transfer shifts one full byte.
 */
void sc_spi_target_set_cs(SCSPITargetState *s, uint8_t cs);
uint8_t sc_spi_target_transfer(SCSPITargetState *s, uint8_t mosi);

#endif /* HW_SSI_SC_SPI_TARGET_H */
