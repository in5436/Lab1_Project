CC ?= gcc
GCOV ?= gcov
CFLAGS = -Wall -Wextra -g -I../firmware/App -I../firmware/Devices -I. -fprofile-arcs -ftest-coverage
LDFLAGS = -fprofile-arcs -ftest-coverage

SRC = test_main.c ../firmware/App/rack_monitor.c ../firmware/Devices/mpu6050.c
OBJ = $(notdir $(SRC:.c=.o))
EXEC = test_runner

vpath %.c ../firmware/App ../firmware/Devices .

all: check-deps $(EXEC) run coverage uart-stress log-analysis

$(EXEC): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(EXEC)
	./$(EXEC)

coverage: run
	@echo "\n=== Code Coverage Report ==="
	$(GCOV) -b rack_monitor.c mpu6050.c

check-deps:
	@echo "=== Checking Architecture Dependencies ==="
	@if grep -rE '#include.*(stm32|hal|platform\.h|main\.h)' ../firmware/App ../firmware/Devices; then \
		echo "ERROR: Architecture violation! App/Devices layer must not include HAL/Platform headers."; \
		exit 1; \
	else \
		echo "OK: Dependency rules passed."; \
	fi

UART_STRESS_TEST = uart_stress_runner

$(UART_STRESS_TEST): test_uart_stress.c ../firmware/Platform/uart_stress.c
	$(CC) -std=c11 -Wall -Wextra -Werror \
	    -I../firmware/Platform $^ -o $@

uart-stress: $(UART_STRESS_TEST)
	./$(UART_STRESS_TEST)

log-analysis:
	python3 -m unittest -v test_analyze_lab3

clean:
	rm -f $(OBJ) $(EXEC) $(UART_STRESS_TEST) *.gcno *.gcda *.gcov