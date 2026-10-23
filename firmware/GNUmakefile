include Makefile

ifeq ($(filter Platform/platform.c,$(C_SOURCES)),)
C_SOURCES += Platform/platform.c
endif

ifeq ($(filter Devices/mpu6050.c,$(C_SOURCES)),)
C_SOURCES += Devices/mpu6050.c
endif

ifeq ($(filter App/rack_monitor.c,$(C_SOURCES)),)
C_SOURCES += App/rack_monitor.c
endif

C_INCLUDES += -IPlatform -IDevices -IApp

vpath %.c Platform Devices App

$(BUILD_DIR)/$(TARGET).elf: \
    $(BUILD_DIR)/platform.o \
    $(BUILD_DIR)/mpu6050.o \
    $(BUILD_DIR)/rack_monitor.o

$(BUILD_DIR)/main.o $(BUILD_DIR)/platform.o \
$(BUILD_DIR)/mpu6050.o $(BUILD_DIR)/rack_monitor.o: CFLAGS += -Wextra