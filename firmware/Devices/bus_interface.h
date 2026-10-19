#ifndef BUS_INTERFACE_H
#define BUS_INTERFACE_H

#include <stdint.h>

typedef enum {
    BUS_OK = 0,
    BUS_ERR_NACK = -1,
    BUS_ERR_TIMEOUT = -2,
    BUS_ERR_PARAM = -3
} bus_status_t;

typedef bus_status_t (*bus_read_fn)(void *ctx, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef bus_status_t (*bus_write_fn)(void *ctx, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);

typedef struct {
    void *ctx;
    bus_read_fn read_regs;
    bus_write_fn write_regs;
} reg_bus_t;

#endif