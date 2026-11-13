#include "platform.h"
#include "mpu6050.h"
#include "rack_monitor.h"
#include <stdio.h>
#include <string.h>
#include "perf.h"

#define IMU_POLL_PERIOD_MS      20U     
#define ADC_POLL_PERIOD_MS      20U     
#define TELEMETRY_PERIOD_MS     1000U   
#define I2C_TIMEOUT_LOOPS       100000UL

static UART_HandleTypeDef *s_huart = NULL;
static I2C_HandleTypeDef  *s_hi2c  = NULL;
static ADC_HandleTypeDef  *s_hadc  = NULL;

static mpu6050_dev_t  s_imu;
static rack_monitor_t s_monitor;

static uint32_t s_last_imu_ms = 0U;
static uint32_t s_last_adc_ms = 0U;
static uint32_t s_last_telemetry_ms = 0U;
static uint16_t s_last_adc_raw = 0U;

static char s_cmd_buf[32];
static uint8_t s_cmd_len = 0U;

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

static void platform_handle_command(const char *cmd)
{
    uint32_t now = HAL_GetTick();
    uint8_t who = 0U;
    uint8_t raw[6] = {0};
    mpu6050_accel_mg_t acc = {0, 0, 0};

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
        s_imu.dev_addr = MPU6050_ADDR_DEFAULT;
        if ((mpu6050_read_id(&s_imu, &who) == MPU6050_OK) &&
            (mpu6050_read_accel_mg(&s_imu, &acc, raw) == MPU6050_OK)) {
            printf("[PROBE] addr=0x%02X who_am_i=0x%02X raw=%02X%02X_%02X%02X_%02X%02X ax=%d ay=%d az=%d mg\r\n",
                   (unsigned int)s_imu.dev_addr, (unsigned int)who,
                   (unsigned int)raw[0], (unsigned int)raw[1],
                   (unsigned int)raw[2], (unsigned int)raw[3],
                   (unsigned int)raw[4], (unsigned int)raw[5],
                   (int)acc.x_mg, (int)acc.y_mg, (int)acc.z_mg);
        } else {
            printf("[PROBE] addr=0x%02X result=ERROR\r\n", (unsigned int)s_imu.dev_addr);
        }
    } else if (strcmp(cmd, "sim_nack") == 0) {
        s_imu.dev_addr = MPU6050_ADDR_ALT;
        if (mpu6050_read_id(&s_imu, &who) != MPU6050_OK) {
            printf("[FAULT] ts=%lu ms bus=I2C1 addr=0x%02X error=NACK\r\n",
                   (unsigned long)now, (unsigned int)s_imu.dev_addr);
        }
        s_imu.dev_addr = MPU6050_ADDR_DEFAULT;
    } else if (strcmp(cmd, "perf") == 0) {
        uint32_t time_us = perf_now_us();

        printf("[PERF] tick_ms=%lu timer_us=%lu\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)time_us);

    } else if (strcmp(cmd, "stats reset") == 0) {
        perf_stats_reset();
        printf("[STATS] reset=OK\r\n");

    } else if (strcmp(cmd, "stats") == 0) {
        perf_stats_print();

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
    uint8_t ch;

    while ((s_huart != NULL) &&
           ((s_huart->Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U)) {

        ch = (uint8_t)(s_huart->Instance->RDR & 0xFFU);

        if ((ch == '\r') || (ch == '\n')) {
            if (s_cmd_len > 0U) {
                uint32_t start_us;

                s_cmd_buf[s_cmd_len] = '\0';

                if ((strcmp(s_cmd_buf, "stats") == 0) ||
                    (strcmp(s_cmd_buf, "stats reset") == 0)) {
                    platform_handle_command(s_cmd_buf);
                } else {
                    start_us = perf_now_us();
                    platform_handle_command(s_cmd_buf);
                    perf_record(PERF_UART_CMD, start_us);
                }

                s_cmd_len = 0U;
            }
        } else if (s_cmd_len < (sizeof(s_cmd_buf) - 1U)) {
            s_cmd_buf[s_cmd_len++] = (char)ch;
        }
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

    s_huart = huart;
    s_hi2c  = hi2c;
    s_hadc  = hadc;

    perf_init();

    rack_monitor_init(&s_monitor);

    bus.ctx = s_hi2c;
    bus.read_regs = platform_i2c_read_regs;
    bus.write_regs = platform_i2c_write_regs;

    printf("\r\n=== Server Rack Health Monitor: Self-Test (Bring-up) ===\r\n");

    imu_st = mpu6050_init(&s_imu, &bus, MPU6050_ADDR_DEFAULT);
    if (imu_st == MPU6050_OK) {
        (void)mpu6050_read_accel_mg(&s_imu, &init_acc, raw);
        s_monitor.last_accel = init_acc;
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

    printf("System Init %s | Mode=ACTIVE | Commands: status, maint, active, clear, probe, sim_nack, perf, stats, stats reset\r\n",
           ((imu_st == MPU6050_OK) && adc_ok) ? "OK" : "FAIL");
}

void Platform_Step(void)
{
    static uint32_t s_last_sample_ms = 0U;

    uint32_t now = HAL_GetTick();
    uint32_t loop_started_us = 0U;
    uint32_t task_started_us;

    bool sample_loop = (now != s_last_sample_ms);
    bool need_update = false;
    bool imu_ok = s_monitor.imu_online;

    mpu6050_accel_mg_t acc = s_monitor.last_accel;
    uint8_t alarms;

    if (sample_loop) {
        s_last_sample_ms = now;
        loop_started_us = perf_now_us();
    }

    platform_poll_uart_rx();

    if ((now - s_last_adc_ms) >= ADC_POLL_PERIOD_MS) {
        s_last_adc_ms = now;

        task_started_us = perf_now_us();

        if (platform_read_adc_marked(&s_last_adc_raw)) {
            need_update = true;
        }

        perf_record(PERF_ADC, task_started_us);
    }

    if ((now - s_last_imu_ms) >= IMU_POLL_PERIOD_MS) {
        s_last_imu_ms = now;

        task_started_us = perf_now_us();

        if (mpu6050_read_accel_mg(&s_imu, &acc, NULL) == MPU6050_OK) {
            imu_ok = true;
        } else {
            imu_ok = false;
        }

        perf_record(PERF_IMU, task_started_us);
        need_update = true;
    }

    if (need_update) {
        task_started_us = perf_now_us();

        alarms = rack_monitor_update(
            &s_monitor,
            now,
            &acc,
            imu_ok,
            s_last_adc_raw,
            platform_read_door_pin()
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
            printf("[ALARM] DOOR OPENED | ts=%lu ms pin=PA1\r\n",
                   (unsigned long)now);
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

    if ((now - s_last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
        s_last_telemetry_ms = now;

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
    }

    if (sample_loop) {
        perf_record(PERF_LOOP, loop_started_us);
    }
}