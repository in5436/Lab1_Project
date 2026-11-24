#include "platform.h"
#include "mpu6050.h"
#include "rack_monitor.h"
#include <stdio.h>
#include <string.h>
#include "perf.h"
#include "uart_stress.h"
#include "memory_probe.h"
#include "rtos_uart.h"
#include "rtos_rx.h"
#include "rtos_app.h"

#ifndef LAB3_BUILD_SHA
#define LAB3_BUILD_SHA "unknown"
#endif

#define IMU_POLL_PERIOD_MS      20U     
#define ADC_POLL_PERIOD_MS      20U     
#define TELEMETRY_PERIOD_MS     1000U   
#define I2C_TIMEOUT_LOOPS       100000UL

static UART_HandleTypeDef *s_huart = NULL;
static I2C_HandleTypeDef  *s_hi2c  = NULL;
static ADC_HandleTypeDef  *s_hadc  = NULL;

static mpu6050_dev_t  s_imu;
static rack_monitor_t s_monitor;

static uint32_t s_last_adc_ms = 0U;
static uint32_t s_last_telemetry_ms = 0U;
static uint16_t s_last_adc_raw = 0U;
static uint32_t s_door_open_seen_us = 0U;
static bool s_door_open_pending = false;

static bool s_first_imu_logged = false;
static bool s_first_adc_logged = false;

static void platform_log_first_sample(const char *sensor, bool *logged)
{
    if (*logged) {
        return;
    }

    uint32_t uptime_ms = HAL_GetTick();
    *logged = true;

    printf("[FIRST_SAMPLE] sensor=%s uptime_ms=%lu\r\n",
           sensor, (unsigned long)uptime_ms);
}

static void platform_delay_us(uint32_t us)
{
    uint32_t reload = SysTick->LOAD + 1U;
    uint32_t total_ticks = us * (SystemCoreClock / 1000000U);
    uint32_t elapsed = 0U;
    uint32_t prev = SysTick->VAL;

    while (elapsed < total_ticks) {
        uint32_t now = SysTick->VAL;
        if (now <= prev) {
            elapsed += (prev - now);
        } else {
            elapsed += (prev + reload - now);
        }
        prev = now;
    }
}

static void i2c_clear_flags(I2C_TypeDef *inst)
{
    inst->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF |
                I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
}

static bus_status_t i2c_raw_write(I2C_TypeDef *inst, uint8_t dev_addr, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint32_t guard;

    i2c_clear_flags(inst);

    inst->CR2 = (((uint32_t)dev_addr & 0x7FU) << 1) |
                (((uint32_t)len << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) |
                I2C_CR2_AUTOEND |
                I2C_CR2_START;

    for (i = 0U; i < len; i++) {
        guard = I2C_TIMEOUT_LOOPS;
        while (((inst->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) == 0U) && (guard > 0U)) {
            guard--;
        }
        if (guard == 0U) {
            i2c_clear_flags(inst);
            return BUS_ERR_TIMEOUT;
        }
        if ((inst->ISR & I2C_ISR_NACKF) != 0U) {
            i2c_clear_flags(inst);
            return BUS_ERR_NACK;
        }

        platform_delay_us(120U);

        if ((inst->ISR & I2C_ISR_NACKF) != 0U) {
            i2c_clear_flags(inst);
            return BUS_ERR_NACK;
        }

        inst->TXDR = buf[i];
    }

    guard = I2C_TIMEOUT_LOOPS;
    while (((inst->ISR & (I2C_ISR_STOPF | I2C_ISR_NACKF)) == 0U) && (guard > 0U)) {
        guard--;
    }
    if ((inst->ISR & I2C_ISR_NACKF) != 0U) {
        i2c_clear_flags(inst);
        return BUS_ERR_NACK;
    }

    if ((inst->ISR & I2C_ISR_STOPF) == 0U) {
        i2c_clear_flags(inst);
        return BUS_ERR_TIMEOUT;
    }

    platform_delay_us(120U);
    i2c_clear_flags(inst);
    return BUS_OK;
}

static bus_status_t platform_i2c_write_regs(void *ctx, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)ctx;
    uint8_t frame[16];
    uint16_t i;

    if ((hi2c == NULL) || (data == NULL) || (len == 0U) || (len > 15U)) {
        return BUS_ERR_PARAM;
    }

    frame[0] = reg_addr;
    for (i = 0U; i < len; i++) {
        frame[i + 1U] = data[i];
    }

    return i2c_raw_write(hi2c->Instance, dev_addr, frame, (uint16_t)(len + 1U));
}

static bus_status_t platform_i2c_read_regs(void *ctx, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)ctx;
    I2C_TypeDef *inst;
    bus_status_t st;
    uint16_t i;
    uint32_t guard;

    if ((hi2c == NULL) || (data == NULL) || (len == 0U)) {
        return BUS_ERR_PARAM;
    }

    inst = hi2c->Instance;
    st = i2c_raw_write(inst, dev_addr, &reg_addr, 1U);
    if (st != BUS_OK) {
        return st;
    }

    i2c_clear_flags(inst);
    inst->CR2 = (((uint32_t)dev_addr & 0x7FU) << 1) |
                I2C_CR2_RD_WRN |
                (((uint32_t)len << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) |
                I2C_CR2_AUTOEND |
                I2C_CR2_START;

    for (i = 0U; i < len; i++) {
        guard = I2C_TIMEOUT_LOOPS;
        while (((inst->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)) == 0U) && (guard > 0U)) {
            guard--;
        }
        if (guard == 0U) {
            i2c_clear_flags(inst);
            return BUS_ERR_TIMEOUT;
        }
        if ((inst->ISR & I2C_ISR_RXNE) != 0U) {
            data[i] = (uint8_t)inst->RXDR;
        } else {
            i2c_clear_flags(inst);
            return BUS_ERR_NACK;
        }
    }

    guard = I2C_TIMEOUT_LOOPS;
    while (((inst->ISR & I2C_ISR_STOPF) == 0U) && (guard > 0U)) {
        guard--;
    }

    if ((inst->ISR & I2C_ISR_NACKF) != 0U) {
        i2c_clear_flags(inst);
        return BUS_ERR_NACK;
    }

    if ((inst->ISR & I2C_ISR_STOPF) == 0U) {
        i2c_clear_flags(inst);
        return BUS_ERR_TIMEOUT;
    }

    i2c_clear_flags(inst);
    return BUS_OK;
}

static bool platform_read_adc_marked(uint16_t *val)
{
    bool ok = false;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    if (HAL_ADC_Start(s_hadc) == HAL_OK) {
        if (HAL_ADC_PollForConversion(s_hadc, 10U) == HAL_OK) {
            *val = (uint16_t)HAL_ADC_GetValue(s_hadc);
            ok = true;
        }
        (void)HAL_ADC_Stop(s_hadc);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    return ok;
}

static uint8_t platform_read_door_pin(void)
{
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) ? 1U : 0U;
}

static void platform_uart_load(uint16_t requested)
{
    if (!RTOS_UART_RequestLoad(requested)) {
        printf("[BURST] error=TX_QUEUE_FULL\r\n");
    }
}

static void platform_handle_command(const char *cmd)
{
    uint32_t now = HAL_GetTick();

    if (strcmp(cmd, "maint") == 0) {
        rack_monitor_set_mode(&s_monitor, RACK_MODE_MAINTENANCE);
        printf("[MODE] ts=%lu ms mode=MAINTENANCE\r\n", (unsigned long)now);
    } else if (strcmp(cmd, "active") == 0) {
        rack_monitor_set_mode(&s_monitor, RACK_MODE_ACTIVE);
        printf("[MODE] ts=%lu ms mode=ACTIVE\r\n", (unsigned long)now);
    } else if (strcmp(cmd, "clear") == 0) {
        rack_monitor_clear_alarms(&s_monitor);
        printf("[INFO] ts=%lu ms alarms_cleared=1\r\n", (unsigned long)now);
    } else if (strcmp(cmd, "probe") == 0) {
        if (!RTOS_IMU_Request(RTOS_IMU_PROBE)) {
            printf("[PROBE] error=QUEUE_FULL\r\n");
        }

    } else if (strcmp(cmd, "sim_nack") == 0) {
        if (!RTOS_IMU_Request(RTOS_IMU_SIM_NACK)) {
            printf("[FAULT] error=QUEUE_FULL\r\n");
        }
        s_imu.dev_addr = MPU6050_ADDR_DEFAULT;
    } else if (strcmp(cmd, "build") == 0) {
        printf("[BUILD] git=%s clock_hz=%lu baud=115200\r\n",
               LAB3_BUILD_SHA,
               (unsigned long)SystemCoreClock);

    } else if (strcmp(cmd, "perf") == 0) {
        uint32_t time_us = perf_now_us();

        printf("[PERF] tick_ms=%lu timer_us=%lu\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)time_us);

    } else if (strcmp(cmd, "stats reset") == 0) {
        perf_stats_reset();
        printf("[STATS] reset=OK\r\n");

    } else if (strcmp(cmd, "stats") == 0) {
        RTOS_UART_SetDiagnostic(1U);
        perf_stats_print();
        RTOS_UART_SetDiagnostic(0U);

    } else if (strcmp(cmd, "uart stats") == 0) {
    rtos_rx_stats_t stats;

    RTOS_RX_GetStats(&stats);

    printf("[UART_RX] bytes=%lu lines=%lu overrun_events=%lu long_lines=%lu ring_dropped=%lu cmd_dropped=%lu\r\n",
           (unsigned long)stats.bytes,
           (unsigned long)stats.lines,
           (unsigned long)stats.overrun_events,
           (unsigned long)stats.long_lines,
           (unsigned long)stats.ring_dropped,
           (unsigned long)stats.cmd_dropped);

    printf("[UART_TX] dropped=%lu\r\n",
       (unsigned long)RTOS_UART_GetDropped());

    } else if (strcmp(cmd, "uart reset") == 0) {
        RTOS_RX_ResetStats();
        RTOS_UART_ResetDropped();

        printf("[UART_RX] reset=OK\r\n");;

    } else if (strncmp(cmd, "load", 4U) == 0 &&
               (cmd[4] == ' ' || cmd[4] == '\0')) {
        uint16_t requested;

        if (uart_stress_parse(cmd, &requested)) {
            platform_uart_load(requested);
        } else {
            printf("[BURST] error=INVALID_SIZE range=256..4096 step=64\r\n");
        }

    } else if (strcmp(cmd, "mem") == 0) {
        RTOS_UART_SetDiagnostic(1U);

        memory_probe_print();
        RTOS_PrintMemory();

        RTOS_UART_SetDiagnostic(0U);

    } else if (strcmp(cmd, "status") == 0) {
        printf("[STATUS] ts=%lu ms mode=%s temp=%d.%dC door=%s ax=%d ay=%d az=%d mg alarms=%lu\r\n",
               (unsigned long)now,
               (s_monitor.mode == RACK_MODE_ACTIVE) ? "ACTIVE" : "MAINT",
               (int)(s_monitor.last_temp_deci_c / 10),
               (int)(s_monitor.last_temp_deci_c % 10),
               (s_monitor.door_stable_state != 0U) ? "OPEN" : "CLOSED",
               (int)s_monitor.last_accel.x_mg,
               (int)s_monitor.last_accel.y_mg,
               (int)s_monitor.last_accel.z_mg,
               (unsigned long)s_monitor.total_alarms_count);
    }
}

static void platform_poll_uart_rx(void)
{
    char command[RTOS_RX_CMD_SIZE];
    uint8_t processed = 0U;

    while (processed < 2U &&
           RTOS_RX_GetCommand(command)) {
        uint32_t started_us;

        if (strcmp(command, "stats") == 0 ||
            strcmp(command, "stats reset") == 0) {
            platform_handle_command(command);
        } else {
            started_us = perf_now_us();
            platform_handle_command(command);
            perf_record(PERF_UART_CMD, started_us);
        }

        processed++;
    }
}

void Platform_Init(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc)
{
    reg_bus_t bus;
    mpu6050_status_t imu_st;
    mpu6050_accel_mg_t init_acc = {0, 0, 0};
    uint8_t raw[6] = {0};
    int16_t temp_dc;
    uint8_t door_state;
    bool adc_ok;
    bool imu_sample_ok;

    s_huart = huart;
    s_hi2c  = hi2c;
    s_hadc  = hadc;

    perf_init();

    rack_monitor_init(&s_monitor);

    bus.ctx = s_hi2c;
    bus.read_regs = platform_i2c_read_regs;
    bus.write_regs = platform_i2c_write_regs;

    printf("\r\n=== Server Rack Health Monitor: Self-Test (Bring-up) ===\r\n");

    printf("[BUILD] git=%s clock_hz=%lu baud=115200\r\n",
       LAB3_BUILD_SHA,
       (unsigned long)SystemCoreClock);

    imu_st = mpu6050_init(&s_imu, &bus, MPU6050_ADDR_DEFAULT);

    imu_sample_ok = (imu_st == MPU6050_OK) &&
        (mpu6050_read_accel_mg(&s_imu, &init_acc, raw) == MPU6050_OK);

    if (imu_sample_ok) {
        s_monitor.last_accel = init_acc;

        platform_log_first_sample("MPU6050", &s_first_imu_logged);
        printf("[SELF-TEST] MPU6050 addr=0x%02X who_am_i=0x%02X ax=%d ay=%d az=%d mg status=OK\r\n",
               (unsigned int)s_imu.dev_addr,
               (unsigned int)s_imu.last_who_am_i,
               (int)init_acc.x_mg, (int)init_acc.y_mg, (int)init_acc.z_mg);
    } else {
        printf("[SELF-TEST] MPU6050 addr=0x%02X who_am_i=0x%02X status=FAIL\r\n",
               (unsigned int)s_imu.dev_addr,
               (unsigned int)s_imu.last_who_am_i);
    }

     adc_ok = platform_read_adc_marked(&s_last_adc_raw);
    if (adc_ok) {
        platform_log_first_sample("LM35_ADC", &s_first_adc_logged);

        temp_dc = rack_monitor_adc_to_deci_c(s_last_adc_raw);
        s_monitor.last_temp_deci_c = temp_dc;
        printf("[SELF-TEST] LM35_ADC ch=IN0 raw=%u temp=%d.%dC status=OK\r\n",
               (unsigned int)s_last_adc_raw, (int)(temp_dc / 10), (int)(temp_dc % 10));
    } else {
        printf("[SELF-TEST] LM35_ADC ch=IN0 status=FAIL\r\n");
    }

    door_state = platform_read_door_pin();
    s_monitor.door_raw_prev = door_state;
    s_monitor.door_stable_state = door_state;
    printf("[SELF-TEST] DOOR_SW pin=PA1 level=%u state=%s status=OK\r\n",
           (unsigned int)door_state, (door_state != 0U) ? "OPEN" : "CLOSED");

    printf("System Init %s | Mode=ACTIVE | Commands: build, status, maint, active, clear, probe, sim_nack, perf, stats, stats reset, load 256, load 1024, load 4096, uart stats, uart reset, mem\r\n",
           ((imu_sample_ok && adc_ok) ? "OK" : "FAIL"));
}

bool Platform_ImuRead(mpu6050_accel_mg_t *accel)
{
    static uint32_t last_reinit_ms = 0U;
    bool ok;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    ok = mpu6050_read_accel_mg(
        &s_imu,
        accel,
        NULL
    ) == MPU6050_OK;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

    if (!ok &&
        (HAL_GetTick() - last_reinit_ms) >= 500U) {

        reg_bus_t bus = s_imu.bus;

        last_reinit_ms = HAL_GetTick();

        (void)mpu6050_init(
            &s_imu,
            &bus,
            MPU6050_ADDR_DEFAULT
        );
    }

    return ok;
}

void Platform_ImuCommand(
    rtos_imu_command_t command,
    rtos_imu_reply_t *reply)
{
    uint8_t who = 0U;

    memset(reply, 0, sizeof(*reply));

    reply->command = command;

    if (command == RTOS_IMU_PROBE) {
        s_imu.dev_addr = MPU6050_ADDR_DEFAULT;
        reply->addr = s_imu.dev_addr;

        if (mpu6050_read_id(&s_imu, &who) == MPU6050_OK &&
            mpu6050_read_accel_mg(
                &s_imu,
                &reply->accel,
                reply->raw) == MPU6050_OK) {

            reply->success = 1U;
            reply->who = who;
        }
    } else if (command == RTOS_IMU_SIM_NACK) {
        s_imu.dev_addr = MPU6050_ADDR_ALT;
        reply->addr = s_imu.dev_addr;

        reply->success =
            mpu6050_read_id(&s_imu, &who) != MPU6050_OK;

        s_imu.dev_addr = MPU6050_ADDR_DEFAULT;
    }
}

void Platform_Step(void)
{
    static uint32_t s_last_sample_ms = 0U;

    uint32_t now = HAL_GetTick();
    uint32_t loop_started_us = perf_now_us();
    uint32_t task_started_us;

    bool sample_loop = (now != s_last_sample_ms);
    bool need_update = true;
    bool adc_sample_ok = false;
    bool imu_ok = s_monitor.imu_online;

    mpu6050_accel_mg_t acc = s_monitor.last_accel;
    uint8_t alarms;

    if (sample_loop) {
        s_last_sample_ms = now;
    }

    platform_poll_uart_rx();


    now = HAL_GetTick();

    if ((now - s_last_adc_ms) >= ADC_POLL_PERIOD_MS) {
        uint32_t elapsed_ms = now - s_last_adc_ms;
        uint32_t due = elapsed_ms / ADC_POLL_PERIOD_MS;
        uint32_t lateness_ms = elapsed_ms - ADC_POLL_PERIOD_MS;

        s_last_adc_ms += due * ADC_POLL_PERIOD_MS;

        perf_note_schedule(
            PERF_ADC,
            due - 1U,
            lateness_ms
        );

        task_started_us = perf_now_us();

        adc_sample_ok = platform_read_adc_marked(&s_last_adc_raw);

        if (adc_sample_ok) {
            need_update = true;
        }

        perf_record(PERF_ADC, task_started_us);

        if ((int32_t)(HAL_GetTick() -
            (s_last_adc_ms + ADC_POLL_PERIOD_MS)) > 0) {
            perf_note_deadline_miss(PERF_ADC);
        }

        if (adc_sample_ok) {
            platform_log_first_sample("LM35_ADC", &s_first_adc_logged);
        }
    }

    {
    rtos_imu_sample_t sample;
    rtos_imu_reply_t reply;

    if (RTOS_IMU_GetSample(&sample)) {
        imu_ok = sample.ok != 0U;

        if (imu_ok) {
            acc = sample.accel;
            platform_log_first_sample(
                "MPU6050",
                &s_first_imu_logged
            );
        }
    }

    while (RTOS_IMU_GetReply(&reply)) {
        if (reply.command == RTOS_IMU_PROBE) {
            if (reply.success) {
                printf("[PROBE] addr=0x%02X who_am_i=0x%02X raw=%02X%02X_%02X%02X_%02X%02X ax=%d ay=%d az=%d mg\r\n",
                       (unsigned int)reply.addr,
                       (unsigned int)reply.who,
                       (unsigned int)reply.raw[0],
                       (unsigned int)reply.raw[1],
                       (unsigned int)reply.raw[2],
                       (unsigned int)reply.raw[3],
                       (unsigned int)reply.raw[4],
                       (unsigned int)reply.raw[5],
                       (int)reply.accel.x_mg,
                       (int)reply.accel.y_mg,
                       (int)reply.accel.z_mg);
            } else {
                printf("[PROBE] addr=0x%02X result=ERROR\r\n",
                       (unsigned int)reply.addr);
            }
        } else if (reply.command == RTOS_IMU_SIM_NACK &&
                   reply.success) {
            printf("[FAULT] ts=%lu ms bus=I2C1 addr=0x%02X error=NACK\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned int)reply.addr);
        }
    }
}

    if (need_update) {
        uint8_t door_pin;

        task_started_us = perf_now_us();

        door_pin = platform_read_door_pin();

        if (door_pin != s_monitor.door_raw_prev) {
            s_door_open_pending = (door_pin != 0U);

            if (s_door_open_pending) {
                s_door_open_seen_us = perf_now_us();
            }
        }

        alarms = rack_monitor_update(
            &s_monitor,
            now,
            &acc,
            imu_ok,
            s_last_adc_raw,
            door_pin
        );

        if ((alarms & RACK_ALARM_SHOCK) != 0U) {
            printf("[ALARM] SHOCK DETECTED | ts=%lu ms ax=%d ay=%d az=%d mg\r\n",
                   (unsigned long)now,
                   (int)acc.x_mg,
                   (int)acc.y_mg,
                   (int)acc.z_mg);
        }

        if ((alarms & RACK_ALARM_HIGH_TEMP) != 0U) {
            printf("[ALARM] HIGH TEMPERATURE | ts=%lu ms temp=%d.%dC\r\n",
                   (unsigned long)now,
                   (int)(s_monitor.last_temp_deci_c / 10),
                   (int)(s_monitor.last_temp_deci_c % 10));
        }

        if ((alarms & RACK_ALARM_DOOR_OPENED) != 0U) {
            uint32_t response_us;

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
            response_us = perf_now_us();

            printf("[ALARM] DOOR OPENED | ts=%lu ms pin=PA1\r\n",
                (unsigned long)now);

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

            if (s_door_open_pending) {
                printf("EVT t_us=%lu name=door_open_observed\r\n",
                    (unsigned long)s_door_open_seen_us);

                printf("EVT t_us=%lu name=door_response\r\n",
                    (unsigned long)response_us);

                s_door_open_pending = false;
            }
        }

        if ((alarms & RACK_ALARM_SENSOR_FAULT) != 0U) {
            printf("[FAULT] MPU6050 I2C ERROR | ts=%lu ms\r\n",
                   (unsigned long)now);
        }

        perf_record(PERF_LOGIC, task_started_us);
    }

    if (sample_loop) {
        task_started_us = perf_now_us();
    }

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_5,
        rack_monitor_get_led_state(&s_monitor, now)
            ? GPIO_PIN_SET
            : GPIO_PIN_RESET
    );

    if (sample_loop) {
        perf_record(PERF_LED, task_started_us);
    }

    now = HAL_GetTick();

    if ((now - s_last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
        uint32_t elapsed_ms = now - s_last_telemetry_ms;
        uint32_t due = elapsed_ms / TELEMETRY_PERIOD_MS;
        uint32_t lateness_ms = elapsed_ms - TELEMETRY_PERIOD_MS;

        s_last_telemetry_ms += due * TELEMETRY_PERIOD_MS;

        perf_note_schedule(
            PERF_TELEMETRY,
            due - 1U,
            lateness_ms
        );

        task_started_us = perf_now_us();

        printf("[TELEMETRY] ts=%lu ms mode=%s temp=%d.%dC door=%s ax=%d ay=%d az=%d mg\r\n",
                (unsigned long)now,
                (s_monitor.mode == RACK_MODE_ACTIVE) ? "ACTIVE" : "MAINT",
                (int)(s_monitor.last_temp_deci_c / 10),
                (int)(s_monitor.last_temp_deci_c % 10),
                (s_monitor.door_stable_state != 0U) ? "OPEN" : "CLOSED",
                (int)s_monitor.last_accel.x_mg,
                (int)s_monitor.last_accel.y_mg,
                (int)s_monitor.last_accel.z_mg);

        perf_record(PERF_TELEMETRY, task_started_us);

        if ((int32_t)(HAL_GetTick() -
            (s_last_telemetry_ms + TELEMETRY_PERIOD_MS)) > 0) {
            perf_note_deadline_miss(PERF_TELEMETRY);
        }
    }

    perf_record(PERF_LOOP, loop_started_us);
}