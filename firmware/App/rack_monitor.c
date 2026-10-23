#include "rack_monitor.h"
#include <stddef.h>

void rack_monitor_init(rack_monitor_t *mon)
{
    if (mon == NULL) {
        return;
    }

    mon->mode = RACK_MODE_ACTIVE;
    mon->temp_limit_deci_c = RACK_DEFAULT_TEMP_LIMIT_DC;
    mon->shock_threshold_mg = RACK_DEFAULT_SHOCK_DELTA_MG;

    mon->door_raw_prev = 0U;
    mon->door_stable_state = 0U;
    mon->door_change_ms = 0U;

    mon->last_shock_alarm_ms = 0U;
    mon->last_temp_alarm_ms = 0U;
    mon->last_door_alarm_ms = 0U;
    mon->shock_cooldown_active = false;
    mon->temp_cooldown_active = false;
    mon->door_cooldown_active = false;

    mon->last_accel.x_mg = 0;
    mon->last_accel.y_mg = 0;
    mon->last_accel.z_mg = 1000;
    mon->last_accel_mag_sq_mg2 = 1000000UL;
    mon->last_temp_deci_c = 250;
    mon->imu_online = true;
    mon->alarm_latched = false;
    mon->total_alarms_count = 0U;
}

void rack_monitor_set_mode(rack_monitor_t *mon, rack_mode_t mode)
{
    if (mon == NULL) {
        return;
    }
    mon->mode = mode;
    if (mode == RACK_MODE_MAINTENANCE) {
        mon->alarm_latched = false;
    }
}

void rack_monitor_clear_alarms(rack_monitor_t *mon)
{
    if (mon == NULL) {
        return;
    }
    mon->alarm_latched = false;
}

int16_t rack_monitor_adc_to_deci_c(uint16_t adc_raw)
{
    uint32_t clamped = (adc_raw > 4095U) ? 4095U : (uint32_t)adc_raw;
    return (int16_t)((clamped * 3300UL) / 4095UL);
}

bool rack_monitor_check_shock(const mpu6050_accel_mg_t *accel, uint16_t threshold_mg, uint32_t *mag_sq_out)
{
    int32_t x, y, z;
    uint32_t mag_sq;
    uint32_t upper_limit, lower_limit;
    uint32_t upper_sq, lower_sq;

    if (accel == NULL) {
        return false;
    }

    x = (int32_t)accel->x_mg;
    y = (int32_t)accel->y_mg;
    z = (int32_t)accel->z_mg;
    mag_sq = (uint32_t)(x * x + y * y + z * z);

    if (mag_sq_out != NULL) {
        *mag_sq_out = mag_sq;
    }

    upper_limit = 1000UL + (uint32_t)threshold_mg;
    upper_sq = upper_limit * upper_limit;

    lower_limit = (threshold_mg < 1000U) ? (1000UL - (uint32_t)threshold_mg) : 0UL;
    lower_sq = lower_limit * lower_limit;

    if ((mag_sq >= upper_sq) || (mag_sq <= lower_sq)) {
        return true;
    }

    return false;
}

static bool can_trigger_alarm(bool *cooldown_active, uint32_t *last_ms, uint32_t now_ms)
{
    if (!(*cooldown_active) || ((now_ms - *last_ms) >= RACK_ALARM_COOLDOWN_MS)) {
        *cooldown_active = true;
        *last_ms = now_ms;
        return true;
    }
    return false;
}

uint8_t rack_monitor_update(rack_monitor_t *mon,
                            uint32_t now_ms,
                            const mpu6050_accel_mg_t *accel,
                            bool imu_read_ok,
                            uint16_t adc_raw,
                            uint8_t door_pin_state)
{
    uint8_t triggered = RACK_ALARM_NONE;
    uint8_t raw_door = (door_pin_state != 0U) ? 1U : 0U;
    bool shock_detected = false;

    if (mon == NULL) {
        return RACK_ALARM_NONE;
    }

    if (!imu_read_ok || (accel == NULL)) {
        if (mon->imu_online) {
            mon->imu_online = false;
            triggered |= RACK_ALARM_SENSOR_FAULT;
        }
    } else {
        mon->imu_online = true;
        mon->last_accel = *accel;
        shock_detected = rack_monitor_check_shock(accel, mon->shock_threshold_mg, &mon->last_accel_mag_sq_mg2);
    }

    mon->last_temp_deci_c = rack_monitor_adc_to_deci_c(adc_raw);

    if (raw_door != mon->door_raw_prev) {
        mon->door_raw_prev = raw_door;
        mon->door_change_ms = now_ms;
    } else if (mon->door_stable_state != raw_door) {
        if ((now_ms - mon->door_change_ms) >= RACK_DEBOUNCE_MS) {
            mon->door_stable_state = raw_door;
        }
    }

    if (mon->mode == RACK_MODE_MAINTENANCE) {
        return triggered;
    }

    if (shock_detected) {
        if (can_trigger_alarm(&mon->shock_cooldown_active, &mon->last_shock_alarm_ms, now_ms)) {
            triggered |= RACK_ALARM_SHOCK;
            mon->alarm_latched = true;
            mon->total_alarms_count++;
        }
    }

    if (mon->last_temp_deci_c >= mon->temp_limit_deci_c) {
        if (can_trigger_alarm(&mon->temp_cooldown_active, &mon->last_temp_alarm_ms, now_ms)) {
            triggered |= RACK_ALARM_HIGH_TEMP;
            mon->alarm_latched = true;
            mon->total_alarms_count++;
        }
    }

    if (mon->door_stable_state == 1U) {
        if (can_trigger_alarm(&mon->door_cooldown_active, &mon->last_door_alarm_ms, now_ms)) {
            triggered |= RACK_ALARM_DOOR_OPENED;
            mon->alarm_latched = true;
            mon->total_alarms_count++;
        }
    }

    return triggered;
}

bool rack_monitor_get_led_state(const rack_monitor_t *mon, uint32_t now_ms)
{
    if (mon == NULL) {
        return false;
    }

    if ((mon->mode == RACK_MODE_ACTIVE) && mon->alarm_latched) {
        return true;
    }

    if (mon->mode == RACK_MODE_MAINTENANCE) {
        return ((now_ms / 500U) % 2U) == 0U;
    }

    return ((now_ms / 100U) % 2U) == 0U;
}