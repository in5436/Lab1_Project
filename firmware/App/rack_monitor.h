#ifndef RACK_MONITOR_H
#define RACK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "mpu6050.h"

#define RACK_DEBOUNCE_MS            50U     
#define RACK_ALARM_COOLDOWN_MS      5000U   
#define RACK_DEFAULT_TEMP_LIMIT_DC  450     
#define RACK_DEFAULT_SHOCK_DELTA_MG 500U    

typedef enum {
    RACK_MODE_ACTIVE = 0,       
    RACK_MODE_MAINTENANCE = 1  
} rack_mode_t;

typedef enum {
    RACK_ALARM_NONE          = 0x00U,
    RACK_ALARM_SHOCK         = 0x01U,  
    RACK_ALARM_HIGH_TEMP     = 0x02U,   
    RACK_ALARM_DOOR_OPENED   = 0x04U,   
    RACK_ALARM_SENSOR_FAULT  = 0x08U    
} rack_alarm_flags_t;

typedef struct {
    rack_mode_t mode;
    int16_t temp_limit_deci_c;
    uint16_t shock_threshold_mg;

    uint8_t door_raw_prev;
    uint8_t door_stable_state;
    uint32_t door_change_ms;

    uint32_t last_shock_alarm_ms;
    uint32_t last_temp_alarm_ms;
    uint32_t last_door_alarm_ms;
    bool shock_cooldown_active;
    bool temp_cooldown_active;
    bool door_cooldown_active;

    mpu6050_accel_mg_t last_accel;
    uint32_t last_accel_mag_sq_mg2;
    int16_t last_temp_deci_c;
    bool imu_online;
    bool alarm_latched;
    uint32_t total_alarms_count;
} rack_monitor_t;

void rack_monitor_init(rack_monitor_t *mon);
void rack_monitor_set_mode(rack_monitor_t *mon, rack_mode_t mode);
void rack_monitor_clear_alarms(rack_monitor_t *mon);

int16_t rack_monitor_adc_to_deci_c(uint16_t adc_raw);

bool rack_monitor_check_shock(const mpu6050_accel_mg_t *accel, uint16_t threshold_mg, uint32_t *mag_sq_out);

uint8_t rack_monitor_update(rack_monitor_t *mon,
                            uint32_t now_ms,
                            const mpu6050_accel_mg_t *accel,
                            bool imu_read_ok,
                            uint16_t adc_raw,
                            uint8_t door_pin_state);

bool rack_monitor_get_led_state(const rack_monitor_t *mon, uint32_t now_ms);

#endif