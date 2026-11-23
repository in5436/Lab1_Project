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

LAB3_GIT_SHA := $(shell git rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
CFLAGS += -DLAB3_BUILD_SHA=\"$(LAB3_GIT_SHA)\"

FREERTOS_DIR := Middlewares/FreeRTOS/FreeRTOS-Kernel

C_SOURCES += \
$(FREERTOS_DIR)/tasks.c \
$(FREERTOS_DIR)/queue.c \
$(FREERTOS_DIR)/list.c \
$(FREERTOS_DIR)/portable/GCC/ARM_CM0/port.c \
$(FREERTOS_DIR)/portable/GCC/ARM_CM0/portasm.c \
$(FREERTOS_DIR)/portable/MemMang/heap_4.c \
Platform/rtos_app.c \
Platform/hal_timebase_tim14.c

C_INCLUDES += \
-I$(FREERTOS_DIR)/include \
-I$(FREERTOS_DIR)/portable/GCC/ARM_CM0

vpath %.c \
$(FREERTOS_DIR) \
$(FREERTOS_DIR)/portable/GCC/ARM_CM0 \
$(FREERTOS_DIR)/portable/MemMang

OPT = -Os

$(BUILD_DIR)/$(TARGET).elf: \
    $(BUILD_DIR)/tasks.o \
    $(BUILD_DIR)/queue.o \
    $(BUILD_DIR)/list.o \
    $(BUILD_DIR)/port.o \
    $(BUILD_DIR)/portasm.o \
    $(BUILD_DIR)/heap_4.o \
    $(BUILD_DIR)/rtos_app.o \
    $(BUILD_DIR)/hal_timebase_tim14.o