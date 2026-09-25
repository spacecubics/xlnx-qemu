#ifndef HW_MICROCHIP_COREUART_H
#define HW_MICROCHIP_COREUART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"

#define TYPE_MICROCHIP_COREUART "microchip.coreuart"
OBJECT_DECLARE_SIMPLE_TYPE(MicrochipCoreUARTState,
                           MICROCHIP_COREUART)

/* Microchip CoreUARTapb register layout. */
typedef struct MicrochipCoreUARTState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    CharBackend chr;
    uint8_t rx;
    uint8_t ctrl1;
    uint8_t ctrl2;
    bool rx_full;
} MicrochipCoreUARTState;

#endif /* HW_MICROCHIP_COREUART_H */
