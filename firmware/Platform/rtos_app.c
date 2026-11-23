#include "rtos_app.h"
#include "platform.h"

#include "FreeRTOS.h"
#include "task.h"

static void LegacyTask(void *argument)
{
    (void)argument;

    for (;;) {
        Platform_Step();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

void RTOS_Start(void)
{
    BaseType_t result;

    result = xTaskCreate(
        LegacyTask,
        "Legacy",
        384U,
        NULL,
        1U,
        NULL
    );

    if (result != pdPASS) {
        Error_Handler();
    }

    vTaskStartScheduler();

    Error_Handler();
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