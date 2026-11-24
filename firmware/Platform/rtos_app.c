#include "rtos_app.h"
#include "platform.h"
#include "rtos_uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include "rtos_rx.h"
#include "rtos_imu.h"

#include <stdio.h>

static TaskHandle_t s_monitor_task;
static TaskHandle_t s_imu_task;
static TaskHandle_t s_rx_task;
static TaskHandle_t s_tx_task;

static void MonitorTask(void *argument)
{
    TickType_t period = pdMS_TO_TICKS(5U);
    TickType_t scheduled = xTaskGetTickCount();

    (void)argument;

    if (period == 0U) {
        period = 1U;
    }

    for (;;) {
        TickType_t late =
            xTaskGetTickCount() - scheduled;

        scheduled += (late / period) * period;

        Platform_Step();

        vTaskDelayUntil(&scheduled, period);
    }
}

void RTOS_Start(UART_HandleTypeDef *huart)
{
    if (!RTOS_UART_Init(huart)) {
        Error_Handler();
    }

    if (!RTOS_RX_Init(huart)) {
    Error_Handler();
    }

    if (!RTOS_IMU_Init()) {
    Error_Handler();
    }

    if (xTaskCreate(
            RTOS_UART_Task,
            "UartTx",
            192U,
            NULL,
            1U,
            &s_tx_task) != pdPASS) {
        Error_Handler();
    }

    if (xTaskCreate(
        RTOS_RX_Task,
        "UartRx",
        160U,
        NULL,
        2U,
        &s_rx_task) != pdPASS) {
    Error_Handler();
    }

    if (xTaskCreate(
        RTOS_IMU_Task,
        "Imu",
        160U,
        NULL,
        3U,
        &s_imu_task) != pdPASS) {
    Error_Handler();
    }

    if (xTaskCreate(
        MonitorTask,
        "Monitor",
        384U,
        NULL,
        4U,
        &s_monitor_task) != pdPASS) {
    Error_Handler();
    }

    vTaskStartScheduler();

    Error_Handler();
}

static void rtos_print_stack(
    const char *name,
    TaskHandle_t task)
{
    printf("[RTOS_STACK] name=%s free_min_bytes=%lu\r\n",
           name,
           (unsigned long)(
               uxTaskGetStackHighWaterMark(task) *
               sizeof(StackType_t)
           ));
}

void RTOS_PrintMemory(void)
{
    printf("[RTOS_HEAP] total=%lu free=%lu min_free=%lu\r\n",
           (unsigned long)configTOTAL_HEAP_SIZE,
           (unsigned long)xPortGetFreeHeapSize(),
           (unsigned long)xPortGetMinimumEverFreeHeapSize());

    rtos_print_stack("Monitor", s_monitor_task);
    rtos_print_stack("Imu", s_imu_task);
    rtos_print_stack("UartRx", s_rx_task);
    rtos_print_stack("UartTx", s_tx_task);
    rtos_print_stack("Idle", xTaskGetIdleTaskHandle());
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();

    for (;;) {
    }
}

void vApplicationStackOverflowHook(
    TaskHandle_t task,
    char *task_name)
{
    (void)task;
    (void)task_name;

    taskDISABLE_INTERRUPTS();

    for (;;) {
    }
}