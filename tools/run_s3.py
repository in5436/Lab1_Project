
import argparse
import csv
import re
import time
from pathlib import Path

import serial

parser = argparse.ArgumentParser()
parser.add_argument("--minutes", type=int, default=3)
parser.add_argument("--out", default="docs/metrics/lab3/S3.log")
args = parser.parse_args()

if args.minutes < 1:
    parser.error("--minutes must be at least 1")

duration_ms = args.minutes * 60000
output = Path(args.out)
output.parent.mkdir(parents=True, exist_ok=True)

actions_path = output.with_name(output.stem + "-actions.csv")
timestamp_re = re.compile(r"\[TELEMETRY\] ts=(\d+) ms")

schedule = [
    (0, "build"),
    (1000, "stats reset"),
    (2000, "uart reset"),
]

for t in range(5000, duration_ms - 12000, 5000):
    schedule.append((t, "load 4096"))

for t in (25000, 55000, 85000, 115000, 145000):
    if t < duration_ms - 12000:
        schedule.append((t, "status"))

for t in (45000, 105000):
    if t < duration_ms - 12000:
        schedule.append((t, "probe"))

for t in (70000, 130000):
    if t < duration_ms - 12000:
        for _ in range(12):
            schedule.append((t, "status"))

schedule.extend([
    (duration_ms, "stats"),
    (duration_ms + 4000, "mem"),
    (duration_ms + 8000, "uart stats"),
])
schedule.sort(key=lambda item: item[0])

print("Connecting to Wokwi...")
print("Keep Wokwi running. Duration:", args.minutes, "minutes")

with serial.serial_for_url(
    "rfc2217://localhost:4000",
    baudrate=115200,
    timeout=1
) as port, output.open("w", encoding="utf-8") as log, \
        actions_path.open("w", encoding="utf-8", newline="") as action_file:

    actions = csv.writer(action_file)
    actions.writerow(["elapsed_ms", "mcu_ts_ms", "command"])

    start_ms = None
    next_action = 0
    next_progress = 0
    last_activity = time.monotonic()

    final_sent = {"stats": False, "mem": False, "uart stats": False}
    final_received = {"stats": False, "mem": False, "uart stats": False}

    while True:
        raw = port.readline()

        if not raw:
            if time.monotonic() - last_activity > 90:
                raise TimeoutError("No UART output for 90 seconds")
            continue

        last_activity = time.monotonic()
        line = raw.decode("utf-8", errors="replace")
        log.write(line)
        log.flush()

        if final_sent["stats"] and "[STATS] window_ms=" in line:
            final_received["stats"] = True
        if final_sent["mem"] and "[MEM] heap_brk_used=" in line:
            final_received["mem"] = True
        if final_sent["uart stats"] and "[UART_RX] bytes=" in line:
            final_received["uart stats"] = True

        match = timestamp_re.search(line)
        if not match:
            continue

        now_ms = int(match.group(1))

        if start_ms is None:
            start_ms = now_ms
            print("S3 started at MCU time", start_ms, "ms")

        elapsed_ms = now_ms - start_ms

        while (next_action < len(schedule) and
               elapsed_ms >= schedule[next_action][0]):

            command = schedule[next_action][1]
            port.write((command + "\n").encode("ascii"))

            actions.writerow([elapsed_ms, now_ms, command])
            action_file.flush()

            if schedule[next_action][0] >= duration_ms:
                final_sent[command] = True

            if command != "load 4096":
                print(f"[{elapsed_ms / 1000:.0f}s] -> {command}")

            next_action += 1

        if elapsed_ms >= next_progress:
            print(f"Progress: {elapsed_ms / 1000:.0f} / "
                  f"{duration_ms / 1000:.0f} seconds")
            next_progress += 30000

        if elapsed_ms >= duration_ms + 12000:
            if all(final_received.values()):
                print("S3 finished successfully.")
                break

            if elapsed_ms >= duration_ms + 30000:
                print("S3 finished with missing responses:")
                for name, received in final_received.items():
                    if not received:
                        print(" -", name)
                break

print("UART log:", output)
print("Actions:", actions_path)
