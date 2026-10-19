#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "bus_interface.h"

#define MPU6050_ADDR_DEFAULT    0x68U
#define MPU6050_ADDR_ALT        0x69U

#define MPU6050_REG_SMPLRT_DIV  0x19U
#define MPU6050_REG_CONFIG      0x1AU
#define MPU6050_REG_ACCEL_CFG   0x1CU
#define MPU6050_REG_ACCEL_XOUT  0x3BU
#define MPU6050_REG_PWR_MGMT_1  0x6BU
#define MPU6050_REG_WHO_AM_I    0x75U

#define MPU6050_WHO_AM_I_VAL    0x68U
#define MPU6050_LSB_PER_G_2G    16384

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_BUS = -1,
    MPU6050_ERR_ID = -2,
    MPU6050_ERR_PARAM = -3
} mpu6050_status_t;

typedef struct {
    int16_t x_mg;
    int16_t y_mg;
    int16_t z_mg;
} mpu6050_accel_mg_t;

typedef struct {
    reg_bus_t bus;
    uint8_t dev_addr;
    uint8_t last_who_am_i;
} mpu6050_dev_t;

mpu6050_status_t mpu6050_init(mpu6050_dev_t *dev, const reg_bus_t *bus, uint8_t dev_addr);
mpu6050_status_t mpu6050_read_id(mpu6050_dev_t *dev, uint8_t *who_am_i);
mpu6050_status_t mpu6050_convert_raw_to_mg(const uint8_t raw[6], mpu6050_accel_mg_t *out);
mpu6050_status_t mpu6050_read_accel_mg(mpu6050_dev_t *dev, mpu6050_accel_mg_t *out, uint8_t raw_out[6]);

#endif