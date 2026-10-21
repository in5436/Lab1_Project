CC = gcc
CFLAGS = -Wall -Wextra -g -I../firmware/App -I../firmware/Devices -I. -fprofile-arcs -ftest-coverage
LDFLAGS = -fprofile-arcs -ftest-coverage

SRC = test_main.c ../firmware/App/rack_monitor.c ../firmware/Devices/mpu6050.c
OBJ = $(notdir $(SRC:.c=.o))
EXEC = test_runner

vpath %.c ../firmware/App ../firmware/Devices .

all: check-deps $(EXEC) run coverage

$(EXEC): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(EXEC)
	./$(EXEC)

coverage: run
	@echo "\n=== Code Coverage Report ==="
	gcov-9 -b rack_monitor.c mpu6050.c

check-deps:
	@echo "=== Checking Architecture Dependencies ==="
	@if grep -rE '#include.*(stm32|hal|platform\.h|main\.h)' ../firmware/App ../firmware/Devices; then \
		echo "ERROR: Architecture violation! App/Devices layer must not include HAL/Platform headers."; \
		exit 1; \
	else \
		echo "OK: Dependency rules passed."; \
	fi

clean:
	rm -f $(OBJ) $(EXEC) *.gcno *.gcda *.gcov