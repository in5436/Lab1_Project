#ifndef RTOS_IMU_H
#define RTOS_IMU_H

#include <stdint.h>
#include "mpu6050.h"

typedef enum {
    RTOS_IMU_PROBE = 1,
    RTOS_IMU_SIM_NACK = 2
} rtos_imu_command_t;

typedef struct {
    mpu6050_accel_mg_t accel;
    uint8_t ok;
} rtos_imu_sample_t;

typedef struct {
    rtos_imu_command_t command;
    mpu6050_accel_mg_t accel;
    uint8_t raw[6];
    uint8_t addr;
    uint8_t who;
    uint8_t success;
} rtos_imu_reply_t;

int RTOS_IMU_Init(void);
int RTOS_IMU_Request(rtos_imu_command_t command);

int RTOS_IMU_GetSample(rtos_imu_sample_t *sample);
int RTOS_IMU_GetReply(rtos_imu_reply_t *reply);

void RTOS_IMU_Task(void *argument);

#endif