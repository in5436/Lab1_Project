#include "rtos_imu.h"
#include "platform.h"
#include "perf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static QueueHandle_t s_samples;
static QueueHandle_t s_requests;
static QueueHandle_t s_replies;

int RTOS_IMU_Init(void)
{
    s_samples = xQueueCreate(1U, sizeof(rtos_imu_sample_t));
    s_requests = xQueueCreate(2U, sizeof(rtos_imu_command_t));
    s_replies = xQueueCreate(2U, sizeof(rtos_imu_reply_t));

    return s_samples != NULL &&
           s_requests != NULL &&
           s_replies != NULL;
}

int RTOS_IMU_Request(rtos_imu_command_t command)
{
    return xQueueSend(s_requests, &command, 0U) == pdPASS;
}

int RTOS_IMU_GetSample(rtos_imu_sample_t *sample)
{
    return xQueueReceive(s_samples, sample, 0U) == pdPASS;
}

int RTOS_IMU_GetReply(rtos_imu_reply_t *reply)
{
    return xQueueReceive(s_replies, reply, 0U) == pdPASS;
}

void RTOS_IMU_Task(void *argument)
{
    const TickType_t period = pdMS_TO_TICKS(20U);
    TickType_t scheduled = xTaskGetTickCount();

    (void)argument;

    for (;;) {
        rtos_imu_command_t command;
        rtos_imu_reply_t reply;
        rtos_imu_sample_t sample = {0};

        TickType_t now;
        TickType_t late;
        TickType_t skipped;

        uint32_t started_us;

        if (xQueueReceive(
                s_requests,
                &command,
                0U) == pdPASS) {

            Platform_ImuCommand(command, &reply);

            (void)xQueueSend(s_replies, &reply, 0U);
        }

        now = xTaskGetTickCount();

        late = now - scheduled;
        skipped = late / period;

        perf_note_schedule(
            PERF_IMU,
            (uint32_t)skipped,
            (uint32_t)(late * portTICK_PERIOD_MS)
        );

        scheduled += skipped * period;

        started_us = perf_now_us();

        sample.ok =
            Platform_ImuRead(&sample.accel) ? 1U : 0U;

        perf_record(PERF_IMU, started_us);

        if ((xTaskGetTickCount() - scheduled) > period) {
            perf_note_deadline_miss(PERF_IMU);
        }

        (void)xQueueOverwrite(s_samples, &sample);

        vTaskDelayUntil(&scheduled, period);
    }
}