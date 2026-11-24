#include "rtos_rx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdbool.h>
#include <string.h>

#define RX_RING_SIZE 128U
#define RX_COMMAND_QUEUE_LENGTH 8U

static UART_HandleTypeDef *s_uart = NULL;
static QueueHandle_t s_commands = NULL;
static TaskHandle_t s_rx_task = NULL;

static volatile uint8_t s_ring[RX_RING_SIZE];
static volatile uint16_t s_head = 0U;
static volatile uint16_t s_tail = 0U;

static volatile uint32_t s_bytes = 0U;
static volatile uint32_t s_overruns = 0U;
static volatile uint32_t s_ring_dropped = 0U;

static uint32_t s_lines = 0U;
static uint32_t s_long_lines = 0U;
static uint32_t s_cmd_dropped = 0U;

int RTOS_RX_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return 0;
    }

    s_uart = huart;
    s_commands = xQueueCreate(
        RX_COMMAND_QUEUE_LENGTH,
        RTOS_RX_CMD_SIZE
    );

    return s_commands != NULL;
}

static int rx_ring_read(uint8_t *value)
{
    uint16_t tail = s_tail;

    if (tail == s_head) {
        return 0;
    }

    *value = s_ring[tail];
    s_tail = (uint16_t)((tail + 1U) % RX_RING_SIZE);

    return 1;
}

void RTOS_RX_IRQHandler(void)
{
    BaseType_t higher_priority_woken = pdFALSE;
    bool received = false;
    USART_TypeDef *uart = s_uart->Instance;

    if ((uart->ISR & USART_ISR_ORE) != 0U) {
        s_overruns++;
        uart->ICR = USART_ICR_ORECF;
    }

    while ((uart->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
        uint8_t value = (uint8_t)(uart->RDR & 0xFFU);
        uint16_t next = (uint16_t)(
            (s_head + 1U) % RX_RING_SIZE
        );

        s_bytes++;

        if (next == s_tail) {
            s_ring_dropped++;
        } else {
            s_ring[s_head] = value;
            s_head = next;
            received = true;
        }
    }

    if (received && s_rx_task != NULL) {
        vTaskNotifyGiveFromISR(
            s_rx_task,
            &higher_priority_woken
        );
    }

    portYIELD_FROM_ISR(higher_priority_woken);
}

void RTOS_RX_Task(void *argument)
{
    char command[RTOS_RX_CMD_SIZE] = {0};
    uint8_t length = 0U;
    uint8_t value;
    bool drop_line = false;

    (void)argument;

    s_rx_task = xTaskGetCurrentTaskHandle();

    NVIC_SetPriority(USART2_IRQn, 2U);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);

    s_uart->Instance->CR1 |= USART_CR1_RXNEIE_RXFNEIE;

    for (;;) {
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        while (rx_ring_read(&value)) {
            if (value == '\r' || value == '\n') {
                if (length != 0U || drop_line) {
                    s_lines++;

                    if (!drop_line) {
                        command[length] = '\0';

                        if (xQueueSend(
                                s_commands,
                                command,
                                0U) != pdPASS) {
                            s_cmd_dropped++;
                        }
                    }

                    length = 0U;
                    drop_line = false;
                }
            } else if (!drop_line) {
                if (length < RTOS_RX_CMD_SIZE - 1U) {
                    command[length++] = (char)value;
                } else {
                    drop_line = true;
                    s_long_lines++;
                }
            }
        }
    }
}

int RTOS_RX_GetCommand(char command[RTOS_RX_CMD_SIZE])
{
    return xQueueReceive(
        s_commands,
        command,
        0U
    ) == pdPASS;
}

void RTOS_RX_GetStats(rtos_rx_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    taskENTER_CRITICAL();

    stats->bytes = s_bytes;
    stats->lines = s_lines;
    stats->overrun_events = s_overruns;
    stats->long_lines = s_long_lines;
    stats->ring_dropped = s_ring_dropped;
    stats->cmd_dropped = s_cmd_dropped;

    taskEXIT_CRITICAL();
}

void RTOS_RX_ResetStats(void)
{
    taskENTER_CRITICAL();

    s_bytes = 0U;
    s_lines = 0U;
    s_overruns = 0U;
    s_long_lines = 0U;
    s_ring_dropped = 0U;
    s_cmd_dropped = 0U;

    taskEXIT_CRITICAL();
}