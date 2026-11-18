import argparse
import csv
import re
import statistics
import threading
import time
from pathlib import Path

import serial

EXPECTED_SHA = "c1d1d3a82c1e"
UART_RE = re.compile(
    r"\[UART_RX\] bytes=(\d+) lines=(\d+) "
    r"overrun_events=(\d+) long_lines=(\d+)"
)

parser = argparse.ArgumentParser(description="UART flood test (Lab 3, 6.2.3)")
parser.add_argument("--out", default="docs/metrics/lab3/uart_flood.log")
parser.add_argument("--port", default="rfc2217://localhost:4000")
args = parser.parse_args()

out = Path(args.out)
out.parent.mkdir(parents=True, exist_ok=True)
actions_path = out.with_name(out.stem + "-actions.csv")
summary_path = out.with_name(out.stem + "-summary.csv")

start = time.monotonic()
events = []
lock = threading.Lock()
stop = threading.Event()
reader_error = []

def host_ms():
    return round((time.monotonic() - start) * 1000, 3)


def event_index():
    with lock:
        return len(events)


def get_events(index=0):
    with lock:
        return list(events[index:])


def await_line(index, predicate, timeout=6.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if reader_error:
            raise RuntimeError(f"UART reader failed: {reader_error[0]}")
        for ts, line in get_events(index):
            if predicate(line):
                return ts, line
        time.sleep(0.015)
    return None


def reader(port, log):
    try:
        while not stop.is_set():
            raw = port.readline()
            if not raw:
                continue
            s = raw.decode("utf-8", errors="replace")
            t = host_ms()
            with lock:
                events.append((t, s))
            log.write(s)
            log.flush()
    except Exception as exc:
        reader_error.append(str(exc))


def send(port, writer, phase, command):
    t = host_ms()
    port.write((command + "\n").encode("ascii"))
    writer.writerow([phase, t, command])
    return t


def reset_uart(port, writer, phase):
    for attempt in range(3):
        at = event_index()
        send(port, writer, phase, "uart reset")
        got = await_line(at, lambda s: "[UART_RX] reset=OK" in s, 3.0)
        if got:
            return True
    return False


def get_uart_stats(port, writer, phase):
    for attempt in range(3):
        at = event_index()
        send(port, writer, phase, "uart stats")
        got = await_line(at, lambda s: UART_RE.search(s) is not None, 4.0)
        if got:
            m = UART_RE.search(got[1])
            return tuple(int(x) for x in m.groups())
    return None


def collect_statuses(index):
    return [t for t, line in get_events(index) if line.startswith("[STATUS]")]


def settle(wait=7.0):
    time.sleep(wait)


def summary_row(phase, sent, received, stats, latencies):
    arr = latencies or []
    return [phase, sent, received, max(0, sent - received),
            stats[0] if stats else "", stats[1] if stats else "",
            stats[2] if stats else "", stats[3] if stats else "",
            round(min(arr), 3) if arr else "",
            round(statistics.mean(arr), 3) if arr else "",
            round(max(arr), 3) if arr else ""]

print("Saving:", out)
print("         ", actions_path)
print("         ", summary_path)

with serial.serial_for_url(args.port, baudrate=115200, timeout=0.2) as port, \
     out.open("w", encoding="utf-8") as log, \
     actions_path.open("w", encoding="utf-8", newline="") as action_file, \
     summary_path.open("w", encoding="utf-8", newline="") as summary_file:
    actions = csv.writer(action_file)
    actions.writerow(["phase", "host_elapsed_ms", "command"])
    summaries = csv.writer(summary_file)
    summaries.writerow(["phase", "sent_status", "received_status", "missing_response",
                        "uart_rx_bytes", "uart_rx_lines", "uart_overrun_events",
                        "uart_long_lines", "rtt_min_host_ms", "rtt_avg_host_ms", "rtt_max_host_ms"])

    thread = threading.Thread(target=reader, args=(port, log), daemon=True)
    thread.start()
    try:
        at = event_index()
        send(port, actions, "setup", "build")
        check = await_line(at, lambda s: s.startswith("[BUILD]"), timeout=7)
        if not check or f"git={EXPECTED_SHA}" not in check[1]:
            raise RuntimeError("Wrong/missing [BUILD]. Expected " + EXPECTED_SHA)
        print("Verified:", check[1].strip())
        send(port, actions, "setup", "stats reset")
        settle(1.0)

        phase = "control"
        if not reset_uart(port, actions, phase):
            print("WARNING: no UART reset ACK for control phase")
        successes = 0
        rtts = []
        for i in range(20):
            at = event_index()
            tx = send(port, actions, phase, "status")
            ack = await_line(at, lambda s: s.startswith("[STATUS]"), 3.0)
            if ack:
                successes += 1
                rtts.append(ack[0] - tx)
        settle(1.0)
        rx = get_uart_stats(port, actions, phase)
        summaries.writerow(summary_row(phase, 20, successes, rx, rtts))
        summary_file.flush()
        print(f"CONTROL: status {successes}/20, host RTT avg={statistics.mean(rtts):.2f} ms" if rtts else "CONTROL: no responses")

        phase = "flood_only"
        if not reset_uart(port, actions, phase):
            print("WARNING: no UART reset ACK for flood-only phase")
        settle(0.5)
        begin = event_index()
        for i in range(100):
            send(port, actions, phase, "status")
            time.sleep(0.002)
        settle(8)
        got = len(collect_statuses(begin))
        rx = get_uart_stats(port, actions, phase)
        summaries.writerow(summary_row(phase, 100, got, rx, []))
        summary_file.flush()
        print(f"FLOOD ONLY: status {got}/100, missing={max(0, 100-got)}, uart={rx}")

        phase = "flood_during_load"
        if not reset_uart(port, actions, phase):
            print("WARNING: no UART reset ACK for loaded phase")
        settle(0.5)
        begin = event_index()
        send(port, actions, phase, "load 4096")
        time.sleep(0.008)
        for i in range(100):
            send(port, actions, phase, "status")
            time.sleep(0.002)
        settle(10)
        got = len(collect_statuses(begin))
        rx = get_uart_stats(port, actions, phase)
        summaries.writerow(summary_row(phase, 100, got, rx, []))
        summary_file.flush()
        print(f"LOADED FLOOD: status {got}/100, missing={max(0, 100-got)}, uart={rx}")

        at = event_index()
        send(port, actions, "final", "stats")
        if not await_line(at, lambda s: s.startswith("[STATS] window_ms="), 7):
            print("WARNING: [STATS] final block not received")
        settle(2)
    finally:
        stop.set()
        thread.join(timeout=2)

print("Done. Send these three files for review:")
print(out)
print(actions_path)
print(summary_path)
