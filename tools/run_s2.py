
import argparse
import csv
import re
import time
from pathlib import Path

import serial

parser = argparse.ArgumentParser()
parser.add_argument("--minutes", type=int, default=60)
parser.add_argument("--out", default="docs/metrics/lab3/S2.log")
args = parser.parse_args()

if args.minutes < 1:
    parser.error("--minutes must be at least 1")

duration_ms = args.minutes * 60_000
output = Path(args.out)
output.parent.mkdir(parents=True, exist_ok=True)

actions_path = output.with_name(output.stem + "-actions.csv")

schedule = [
    (0, "build"),
    (1000, "stats reset"),
    (3000, "mem"),
    (300000, "status"),
    (600000, "probe"),
    (900000, "maint"),
    (960000, "status"),
    (1020000, "active"),
    (1200000, "mem"),
    (1500000, "status"),
    (1800000, "probe"),
    (2400000, "mem"),
    (2700000, "status"),
    (3000000, "status"),
]

schedule = [(t, cmd) for t, cmd in schedule if t < duration_ms]

schedule += [
    (duration_ms, "stats"),
    (duration_ms + 5000, "mem"),
    (duration_ms + 10000, "uart stats"),
]

timestamp_re = re.compile(r"\[TELEMETRY\] ts=(\d+) ms")

print("Connecting to Wokwi...")
print("Keep the Wokwi simulator running and visible.")

with serial.serial_for_url(
    "rfc2217://localhost:4000",
    baudrate=115200,
    timeout=1
) as port, output.open("w", encoding="utf-8") as log, \
        actions_path.open("w", encoding="utf-8", newline="") as action_file:

    actions = csv.writer(action_file)
    actions.writerow(["elapsed_ms", "mcu_ts_ms", "command"])

    start_ms = None
    last_activity = time.monotonic()
    next_action = 0
    next_progress = 0
    final_uart_received = False

    while True:
        raw = port.readline()

        if raw:
            last_activity = time.monotonic()
            line = raw.decode("utf-8", errors="replace")
            log.write(line)
            log.flush()

            if "[UART_RX] bytes=" in line and next_action == len(schedule):
                final_uart_received = True

            match = timestamp_re.search(line)

            if not match:
                continue

            now_ms = int(match.group(1))

            if start_ms is None:
                start_ms = now_ms
                print(f"S2 started at MCU time {start_ms} ms")

            elapsed_ms = now_ms - start_ms

            while (next_action < len(schedule) and
                   elapsed_ms >= schedule[next_action][0]):

                command = schedule[next_action][1]

                port.write((command + "\n").encode("ascii"))
                actions.writerow([elapsed_ms, now_ms, command])
                action_file.flush()

                print(f"[{elapsed_ms // 1000}s] -> {command}")
                next_action += 1

            if elapsed_ms >= next_progress:
                print(f"Progress: {elapsed_ms / 60000:.1f} / "
                      f"{args.minutes} simulated minutes")
                next_progress += 60_000

            if (elapsed_ms >= duration_ms + 15000 and
                    final_uart_received):
                print("S2 finished successfully.")
                break

        elif time.monotonic() - last_activity > 90:
            raise TimeoutError(
                "No UART output for 90 seconds. "
                "Check whether Wokwi is running."
            )

print(f"UART log: {output}")
print(f"Commands: {actions_path}")
