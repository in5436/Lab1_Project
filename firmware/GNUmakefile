include Makefile

CFLAGS += -fstack-usage -Wstack-usage=512
LDFLAGS += -Wl,--print-memory-usage

ifeq ($(filter Platform/platform.c,$(C_SOURCES)),)
C_SOURCES += Platform/platform.c
endif

ifeq ($(filter Platform/perf.c,$(C_SOURCES)),)
C_SOURCES += Platform/perf.c
endif

ifeq ($(filter Platform/uart_stress.c,$(C_SOURCES)),)
C_SOURCES += Platform/uart_stress.c
endif

ifeq ($(filter Platform/memory_probe.c,$(C_SOURCES)),)
C_SOURCES += Platform/memory_probe.c
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
    $(BUILD_DIR)/perf.o \
    $(BUILD_DIR)/uart_stress.o \
    $(BUILD_DIR)/memory_probe.o \
    $(BUILD_DIR)/mpu6050.o \
    $(BUILD_DIR)/rack_monitor.o

$(BUILD_DIR)/main.o $(BUILD_DIR)/platform.o \
$(BUILD_DIR)/perf.o $(BUILD_DIR)/uart_stress.o \
$(BUILD_DIR)/memory_probe.o $(BUILD_DIR)/mpu6050.o \
$(BUILD_DIR)/rack_monitor.o: CFLAGS += -Wextra