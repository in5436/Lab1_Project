#include "mpu6050.h"
#include <stddef.h>

mpu6050_status_t mpu6050_read_id(mpu6050_dev_t *dev, uint8_t *who_am_i)
{
    uint8_t id = 0U;

    if ((dev == NULL) || (who_am_i == NULL) || (dev->bus.read_regs == NULL)) {
        return MPU6050_ERR_PARAM;
    }

    if (dev->bus.read_regs(dev->bus.ctx, dev->dev_addr, MPU6050_REG_WHO_AM_I, &id, 1U) != BUS_OK) {
        return MPU6050_ERR_BUS;
    }

    dev->last_who_am_i = id;
    *who_am_i = id;

    if (id != MPU6050_WHO_AM_I_VAL) {
        return MPU6050_ERR_ID;
    }

    return MPU6050_OK;
}

mpu6050_status_t mpu6050_init(mpu6050_dev_t *dev, const reg_bus_t *bus, uint8_t dev_addr)
{
    uint8_t who_am_i = 0U;
    uint8_t val = 0U;
    mpu6050_status_t st;

    if ((dev == NULL) || (bus == NULL) || (bus->read_regs == NULL) || (bus->write_regs == NULL)) {
        return MPU6050_ERR_PARAM;
    }

    dev->bus = *bus;
    dev->dev_addr = dev_addr;
    dev->last_who_am_i = 0U;

    st = mpu6050_read_id(dev, &who_am_i);
    if (st != MPU6050_OK) {
        return st;
    }

    val = 0x00U;
    if (dev->bus.write_regs(dev->bus.ctx, dev->dev_addr, MPU6050_REG_PWR_MGMT_1, &val, 1U) != BUS_OK) {
        return MPU6050_ERR_BUS;
    }

    val = 0x03U;
    if (dev->bus.write_regs(dev->bus.ctx, dev->dev_addr, MPU6050_REG_CONFIG, &val, 1U) != BUS_OK) {
        return MPU6050_ERR_BUS;
    }

    val = 0x09U;
    if (dev->bus.write_regs(dev->bus.ctx, dev->dev_addr, MPU6050_REG_SMPLRT_DIV, &val, 1U) != BUS_OK) {
        return MPU6050_ERR_BUS;
    }

    val = 0x00U;
    if (dev->bus.write_regs(dev->bus.ctx, dev->dev_addr, MPU6050_REG_ACCEL_CFG, &val, 1U) != BUS_OK) {
        return MPU6050_ERR_BUS;
    }

    return MPU6050_OK;
}

mpu6050_status_t mpu6050_convert_raw_to_mg(const uint8_t raw[6], mpu6050_accel_mg_t *out)
{
    int16_t raw_x, raw_y, raw_z;

    if ((raw == NULL) || (out == NULL)) {
        return MPU6050_ERR_PARAM;
    }

    raw_x = (int16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
    raw_y = (int16_t)(((uint16_t)raw[2] << 8) | (uint16_t)raw[3]);
    raw_z = (int16_t)(((uint16_t)raw[4] << 8) | (uint16_t)raw[5]);

    out->x_mg = (int16_t)(((int32_t)raw_x * 1000) / MPU6050_LSB_PER_G_2G);
    out->y_mg = (int16_t)(((int32_t)raw_y * 1000) / MPU6050_LSB_PER_G_2G);
    out->z_mg = (int16_t)(((int32_t)raw_z * 1000) / MPU6050_LSB_PER_G_2G);

    return MPU6050_OK;
}

mpu6050_status_t mpu6050_read_accel_mg(mpu6050_dev_t *dev, mpu6050_accel_mg_t *out, uint8_t raw_out[6])
{
    uint8_t buf[6];
    uint16_t i;

    if ((dev == NULL) || (out == NULL) || (dev->bus.read_regs == NULL)) {
        return MPU6050_ERR_PARAM;
    }

    if (dev->bus.read_regs(dev->bus.ctx, dev->dev_addr, MPU6050_REG_ACCEL_XOUT, buf, 6U) != BUS_OK) {
        return MPU6050_ERR_BUS;
    }

    if (raw_out != NULL) {
        for (i = 0U; i < 6U; i++) {
            raw_out[i] = buf[i];
        }
    }

    return mpu6050_convert_raw_to_mg(buf, out);
}