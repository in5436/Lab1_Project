import csv
import re
import time
from pathlib import Path

import serial

OUT = Path("docs/metrics/lab3/breakpoint.log")
ACTIONS = Path("docs/metrics/lab3/breakpoint-actions.csv")
STAGES = [("baseline", 0), ("burst_256", 256), ("burst_512", 512),
          ("burst_1024", 1024), ("burst_2048", 2048), ("burst_4096", 4096)]
STAGE_SPACING_MS = 25000
TELEMETRY_RE = re.compile(r"\[TELEMETRY\] ts=(\d+) ms")

schedule = [(0, "build", "initial")]
for idx, (name, burst) in enumerate(STAGES):
    start = idx * STAGE_SPACING_MS
    schedule.append((start + 1000, "stats reset", name))
    if burst:
        for offset in range(3000, 20000, 2000):
            schedule.append((start + offset, f"load {burst}", name))
    schedule.append((start + 21000, "stats", name))
schedule.sort(key=lambda x: x[0])
stop_after = len(STAGES) * STAGE_SPACING_MS

OUT.parent.mkdir(parents=True, exist_ok=True)

with serial.serial_for_url("rfc2217://localhost:4000", baudrate=115200,
                           timeout=1) as port, \
     OUT.open("w", encoding="utf-8") as log, \
     ACTIONS.open("w", newline="", encoding="utf-8") as action_file:
    writer = csv.writer(action_file)
    writer.writerow(["elapsed_ms", "mcu_ts_ms", "stage", "command"])
    t0 = None
    next_action = 0
    last_rx = time.monotonic()
    while True:
        raw = port.readline()
        if not raw:
            if time.monotonic() - last_rx > 90:
                raise TimeoutError("No Wokwi UART output for 90 s")
            continue
        last_rx = time.monotonic()
        line = raw.decode("utf-8", errors="replace")
        log.write(line)
        log.flush()
        match = TELEMETRY_RE.search(line)
        if not match:
            continue
        now = int(match.group(1))
        if t0 is None:
            t0 = now
            print(f"MCU timestamp at start: {t0} ms")
        elapsed = now - t0
        while next_action < len(schedule) and elapsed >= schedule[next_action][0]:
            _, cmd, stage = schedule[next_action]
            port.write((cmd + "\n").encode("ascii"))
            writer.writerow([elapsed, now, stage, cmd])
            action_file.flush()
            if cmd in ("build", "stats", "stats reset"):
                print(f"[{elapsed / 1000:.0f}s] {stage}: {cmd}")
            next_action += 1
        if elapsed >= stop_after and next_action == len(schedule):
            print("Breakpoint ramp finished.")
            break

print("UART log:", OUT)
print("Action timeline:", ACTIONS)
