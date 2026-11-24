#include "rtos_uart.h"
#include "uart_stress.h"
#include "perf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>

#define UART_TX_QUEUE_LENGTH 4U
#define UART_TX_TEXT_SIZE    144U

#define UART_MSG_TEXT 0U
#define UART_MSG_LOAD 1U

typedef struct {
    uint8_t type;
    uint8_t length;
    uint16_t load_bytes;
    char text[UART_TX_TEXT_SIZE];
} uart_tx_message_t;

static UART_HandleTypeDef *s_uart = NULL;
static QueueHandle_t s_tx_queue = NULL;

static char s_pending[UART_TX_TEXT_SIZE];
static uint8_t s_pending_length = 0U;
static uint32_t s_dropped = 0U;

static uint8_t s_diagnostic = 0U;
static uint8_t s_discard_line = 0U;

static void uart_send_text(const char *text)
{
    (void)HAL_UART_Transmit(
        s_uart,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        200U
    );
}

static void uart_send_number(uint32_t value)
{
    char digits[10];
    uint8_t length = 0U;

    do {
        digits[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (length > 0U) {
        length--;
        (void)HAL_UART_Transmit(
            s_uart,
            (uint8_t *)&digits[length],
            1U,
            200U
        );
    }
}

static void uart_send_load(uint16_t requested)
{
    uint8_t chunk[UART_STRESS_CHUNK];
    uint16_t sent = 0U;
    uint32_t started_us;
    uint32_t elapsed_us;
    HAL_StatusTypeDef status = HAL_OK;

    uart_stress_make_chunk(chunk);

    uart_send_text("[BURST] start bytes=");
    uart_send_number(requested);
    uart_send_text("\r\n");

    started_us = perf_now_us();

    while (sent < requested) {
        status = HAL_UART_Transmit(
            s_uart,
            chunk,
            UART_STRESS_CHUNK,
            200U
        );

        if (status != HAL_OK) {
            break;
        }

        sent += UART_STRESS_CHUNK;
    }

    elapsed_us = perf_now_us() - started_us;

    uart_send_text("[BURST] end completed_bytes=");
    uart_send_number(sent);
    uart_send_text(" duration_us=");
    uart_send_number(elapsed_us);
    uart_send_text(" status=");
    uart_send_text(status == HAL_OK ? "OK\r\n" : "ERROR\r\n");
}

static void uart_flush_pending(void)
{
    uart_tx_message_t message = {0};
    TickType_t wait;

    if (s_pending_length == 0U) {
        return;
    }

    message.type = UART_MSG_TEXT;
    message.length = s_pending_length;

    memcpy(message.text, s_pending, s_pending_length);

    s_pending_length = 0U;

    wait = s_diagnostic ? portMAX_DELAY : 0U;

    if (xQueueSend(s_tx_queue, &message, wait) != pdPASS) {
        s_dropped++;
    }
}

int RTOS_UART_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return 0;
    }

    s_uart = huart;

    s_tx_queue = xQueueCreate(
        UART_TX_QUEUE_LENGTH,
        sizeof(uart_tx_message_t)
    );

    return s_tx_queue != NULL;
}

int RTOS_UART_Write(const char *data, int length)
{
    int i;

    if (data == NULL || length < 0 || s_tx_queue == NULL) {
        return -1;
    }

    for (i = 0; i < length; i++) {
        char ch = data[i];

        if (s_discard_line) {
            if (ch == '\n') {
                s_discard_line = 0U;
            }
            continue;
        }

        if (s_pending_length == UART_TX_TEXT_SIZE) {
            if (s_diagnostic) {
                uart_flush_pending();
            } else {
                s_pending_length = 0U;
                s_dropped++;
                s_discard_line = (ch != '\n');
                continue;
            }
        }

        s_pending[s_pending_length++] = ch;

        if (ch == '\n') {
            uart_flush_pending();
        }
    }

    return length;
}

int RTOS_UART_RequestLoad(uint16_t bytes)
{
    uart_tx_message_t message = {0};

    if (s_tx_queue == NULL) {
        return 0;
    }

    message.type = UART_MSG_LOAD;
    message.load_bytes = bytes;

    if (xQueueSend(s_tx_queue, &message, 0U) != pdPASS) {
        s_dropped++;
        return 0;
    }

    return 1;
}

uint32_t RTOS_UART_GetDropped(void)
{
    return s_dropped;
}

void RTOS_UART_Task(void *argument)
{
    uart_tx_message_t message;

    (void)argument;

    for (;;) {
        if (xQueueReceive(
                s_tx_queue,
                &message,
                portMAX_DELAY) != pdPASS) {
            continue;
        }

        if (message.type == UART_MSG_TEXT) {
            (void)HAL_UART_Transmit(
                s_uart,
                (uint8_t *)message.text,
                message.length,
                200U
            );
        } else if (message.type == UART_MSG_LOAD) {
            uart_send_load(message.load_bytes);
        }
    }
}

void RTOS_UART_SetDiagnostic(uint8_t enabled)
{
    s_diagnostic = enabled;
}

void RTOS_UART_ResetDropped(void)
{
    s_dropped = 0U;
}