#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "rack_monitor.h"
#include "mpu6050.h"

static uint8_t fake_who_am_i_response = 0x68;
static uint8_t fake_accel_regs[6] = {0};
static bus_status_t fake_bus_status = BUS_OK;

bus_status_t fake_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    (void)ctx; (void)dev_addr;
    if (fake_bus_status != BUS_OK) return fake_bus_status;

    if (reg_addr == MPU6050_REG_WHO_AM_I && len == 1) {
        data[0] = fake_who_am_i_response;
        return BUS_OK;
    }
    if (reg_addr == MPU6050_REG_ACCEL_XOUT && len == 6) {
        memcpy(data, fake_accel_regs, 6);
        return BUS_OK;
    }
    return BUS_ERR_PARAM;
}

bus_status_t fake_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len) {
    (void)ctx; (void)dev_addr; (void)reg_addr; (void)data; (void)len;
    return fake_bus_status; 
}

void test_mpu6050_driver() {
    mpu6050_dev_t dev;
    reg_bus_t bus = { .ctx = NULL, .read_regs = fake_read, .write_regs = fake_write };
    uint8_t who_am_i;

    printf("Running test_mpu6050_driver...\n");

    assert(mpu6050_read_id(NULL, &who_am_i) == MPU6050_ERR_PARAM);
    assert(mpu6050_init(NULL, &bus, 0x68) == MPU6050_ERR_PARAM);
    assert(mpu6050_read_accel_mg(NULL, NULL, NULL) == MPU6050_ERR_PARAM);
    assert(mpu6050_convert_raw_to_mg(NULL, NULL) == MPU6050_ERR_PARAM);

    fake_who_am_i_response = 0x68;
    assert(mpu6050_init(&dev, &bus, 0x68) == MPU6050_OK);
    assert(mpu6050_read_id(&dev, &who_am_i) == MPU6050_OK);
    assert(who_am_i == 0x68);

    fake_who_am_i_response = 0xFF;
    assert(mpu6050_read_id(&dev, &who_am_i) == MPU6050_ERR_ID);

    uint8_t logic_analyzer_vector[6] = {0x00, 0x00, 0x00, 0x00, 0x40, 0x00};
    mpu6050_accel_mg_t acc;
    assert(mpu6050_convert_raw_to_mg(logic_analyzer_vector, &acc) == MPU6050_OK);
    assert(acc.x_mg == 0);
    assert(acc.y_mg == 0);
    assert(acc.z_mg >= 999 && acc.z_mg <= 1001);

    fake_bus_status = BUS_ERR_NACK;
    assert(mpu6050_read_accel_mg(&dev, &acc, NULL) == MPU6050_ERR_BUS);
    fake_bus_status = BUS_OK; 
}

void test_rack_monitor_logic() {
    rack_monitor_t mon;
    mpu6050_accel_mg_t acc = {0, 0, 1000}; 
    uint8_t alarm;

    printf("Running test_rack_monitor_logic...\n");

    rack_monitor_init(NULL); 
    assert(rack_monitor_update(NULL, 0, &acc, true, 0, 0) == RACK_ALARM_NONE);

    rack_monitor_init(&mon);
    assert(rack_monitor_adc_to_deci_c(500) == 402); 

    alarm = rack_monitor_update(&mon, 100, &acc, true, 500, 0);
    assert(alarm == RACK_ALARM_NONE);
    assert(mon.door_stable_state == 0);
    assert(rack_monitor_get_led_state(&mon, 100) == false); 

    alarm = rack_monitor_update(&mon, 200, &acc, true, 4095, 0);
    assert((alarm & RACK_ALARM_HIGH_TEMP) != 0);
    assert(mon.alarm_latched == true);
    assert(rack_monitor_get_led_state(&mon, 200) == true); 

    alarm = rack_monitor_update(&mon, 210, &acc, true, 4095, 0);
    assert((alarm & RACK_ALARM_HIGH_TEMP) == 0); 

    alarm = rack_monitor_update(&mon, 300, &acc, true, 500, 1);
    assert((alarm & RACK_ALARM_DOOR_OPENED) == 0); 
    
    alarm = rack_monitor_update(&mon, 360, &acc, true, 500, 1);
    assert((alarm & RACK_ALARM_DOOR_OPENED) != 0);

    rack_monitor_clear_alarms(&mon);
    assert(mon.alarm_latched == false);

    rack_monitor_set_mode(&mon, RACK_MODE_MAINTENANCE);
    assert(mon.mode == RACK_MODE_MAINTENANCE);
    
    alarm = rack_monitor_update(&mon, 10000, &acc, true, 4095, 1);
    assert(alarm == RACK_ALARM_NONE);
    
    assert(rack_monitor_get_led_state(&mon, 1000) == true);
    assert(rack_monitor_get_led_state(&mon, 1500) == false);
}

int main(void) {
    printf("=== Starting Tests ===\n");
    test_mpu6050_driver();
    test_rack_monitor_logic();
    printf("=== All Assertions Passed! ===\n");
    return 0;
}