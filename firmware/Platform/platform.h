#ifndef PLATFORM_H
#define PLATFORM_H

#include "main.h"

#include <stdbool.h>
#include "rtos_imu.h"

void Platform_Init(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc);
void Platform_Step(void);

bool Platform_ImuRead(mpu6050_accel_mg_t *accel);

void Platform_ImuCommand(
    rtos_imu_command_t command,
    rtos_imu_reply_t *reply
);

#endif