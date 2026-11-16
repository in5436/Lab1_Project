import argparse
import csv
import math
import re
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from statistics import mean
import subprocess

FIELD_RE = re.compile(r"\b([a-zA-Z][a-zA-Z0-9_]*)=([^\s]+)")
TAG_RE = re.compile(r"^\[([A-Z_]+)\]\s*(.*)$")
TASK_REQUIRED = {"ADC", "IMU", "TELEMETRY", "LOOP_SAMPLE"}
PERIOD_REQUIRED = {"ADC", "IMU", "TELEMETRY"}
U32 = 1 << 32


@dataclass
class Snapshot:
    window_ms: int
    tasks: dict = field(default_factory=dict)
    periods: dict = field(default_factory=dict)
    work_min_pct: str = ""

    def complete(self):
        return TASK_REQUIRED.issubset(self.tasks) and PERIOD_REQUIRED.issubset(self.periods)


@dataclass
class Run:
    scenario: str
    source: str
    snapshots: list = field(default_factory=list)
    stack: dict = field(default_factory=dict)
    heap: dict = field(default_factory=dict)
    uart: dict = field(default_factory=dict)
    build: dict = field(default_factory=dict)
    bursts: list = field(default_factory=list)
    events: list = field(default_factory=list)
    alarm_count: int = 0
    warnings: list = field(default_factory=list)

    def last_snapshot(self):
        return next((s for s in reversed(self.snapshots) if s.complete()), None)


def fields(text):
    return dict(FIELD_RE.findall(text))


def integer(value, default=""):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def parse_log(scenario, path):
    run = Run(scenario=scenario, source=str(path))
    pending_bytes = None
    with Path(path).open("r", encoding="utf-8", errors="replace") as stream:
        for line_no, original in enumerate(stream, 1):
            line = original.strip()
            if line.startswith("EVT ") or line.startswith("[EVT] "):
                body = line[4:] if line.startswith("EVT ") else line[6:]
                data = fields(body)
                t = integer(data.get("t_us"))
                if t != "" and t >= 0 and data.get("name"):
                    run.events.append((t, data["name"]))
                else:
                    run.warnings.append(f"line {line_no}: invalid EVT")
                continue

            match = TAG_RE.match(line)
            if not match:
                continue
            tag, body = match.groups()
            data = fields(body)

            if tag == "STATS" and "window_ms" in data:
                window = integer(data["window_ms"])
                if window != "" and window > 0:
                    run.snapshots.append(Snapshot(window_ms=window))
            elif tag == "LOAD" and run.snapshots and "work_min_pct" in data:
                run.snapshots[-1].work_min_pct = data["work_min_pct"]
            elif tag in {"TASK", "PERIOD"} and run.snapshots and "name" in data:
                target = run.snapshots[-1].tasks if tag == "TASK" else run.snapshots[-1].periods
                target[data["name"]] = data
            elif tag == "MEM":
                if "stack_reserved" in data:
                    run.stack = data
                if "heap_brk_used" in data:
                    run.heap = data
            elif tag == "UART_RX" and "bytes" in data:
                run.uart = data
            elif tag == "BUILD":
                run.build = data
                run.build["origin"] = "UART"
            elif tag == "ALARM":
                run.alarm_count += 1
            elif tag == "BURST":
                if body.startswith("start "):
                    pending_bytes = integer(data.get("bytes"))
                elif body.startswith("end "):
                    run.bursts.append({
                        "requested_bytes": pending_bytes if pending_bytes is not None else "",
                        "completed_bytes": integer(data.get("completed_bytes")),
                        "duration_us": integer(data.get("duration_us")),
                        "status": data.get("status", ""),
                    })
                    pending_bytes = None

    if pending_bytes is not None:
        run.warnings.append("unfinished BURST: no end marker")
    if run.snapshots and not run.snapshots[-1].complete():
        run.warnings.append("last STATS block incomplete; using last complete block")
    if not run.last_snapshot():
        run.warnings.append("no complete STATS block (ADC, IMU, TELEMETRY, LOOP_SAMPLE)")
    snap = run.last_snapshot()
    if snap:
        for name in ("ADC", "IMU"):
            n = integer(snap.tasks[name].get("n"))
            if n != "" and n < 1000:
                run.warnings.append(f"{name}: only {n} samples; periodic tasks need thousands")
    if not run.build.get("git"):
        run.warnings.append("no [BUILD] git=... in UART log; firmware revision unverified")
    return run


def event_pairs(events, start, end):
    """FIFO matching, never use arrival time from the host."""
    starts = deque()
    samples = []
    unmatched_ends = 0
    invalid = 0
    for t, name in events:
        if name == start:
            starts.append(t)
        elif name == end:
            if not starts:
                unmatched_ends += 1
                continue
            t0 = starts.popleft()
            diff = (t - t0) if t >= t0 else (t + U32 - t0 if t < U32 and t0 < U32 else -1)
            if diff < 0 or diff >= U32 // 2:
                invalid += 1
            else:
                samples.append(diff)
    return samples, len(starts), unmatched_ends, invalid


def percentile_nearest_rank(values, percent):
    if not values:
        return ""
    ordered = sorted(values)
    return ordered[math.ceil(len(ordered) * percent / 100) - 1]


def write_csv(path, columns, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def make_outputs(runs, out_dir, baud, pairs):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    tasks, periods, systems, bursts, latencies, samples = [], [], [], [], [], []
    report = ["Server Rack Health Monitor | analysis of MCU UART logs", ""]

    for run in runs:
        snap = run.last_snapshot()
        common = {"scenario": run.scenario, "source": run.source}
        system = dict(common)
        system.update({
            "git": run.build.get("git", ""),
            "git_origin": run.build.get("origin", "UNKNOWN"),
            "window_ms": snap.window_ms if snap else "",
            "work_min_pct": snap.work_min_pct if snap else "",
            "stack_reserved": run.stack.get("stack_reserved", ""),
            "stack_depth_est": run.stack.get("depth_est", ""),
            "stack_spare": run.stack.get("spare", ""),
            "stack_state": run.stack.get("state", ""),
            "heap_brk_used": run.heap.get("heap_brk_used", ""),
            "heap_room": run.heap.get("heap_room", ""),
            "uart_rx_bytes": run.uart.get("bytes", ""),
            "uart_rx_lines": run.uart.get("lines", ""),
            "uart_rx_overrun_events": run.uart.get("overrun_events", ""),
            "uart_rx_long_lines": run.uart.get("long_lines", ""),
            "alarm_messages": run.alarm_count,
        })
        systems.append(system)
        report.append(f"[{run.scenario}] source={run.source}")
        report.append(
            f"  git={system['git'] or 'UNKNOWN'} "
            f"origin={system['git_origin']} "
            f"window_ms={system['window_ms'] or 'N/A'}"
        )
        if snap:
            for name, data in sorted(snap.tasks.items()):
                row = dict(common, window_ms=snap.window_ms, **data)
                tasks.append(row)
            for name, data in sorted(snap.periods.items()):
                row = dict(common, window_ms=snap.window_ms, **data)
                periods.append(row)
            for name in ("ADC", "IMU"):
                p = snap.periods.get(name, {})
                report.append(f"  {name}: avg_period_us={p.get('avg_us', 'N/A')} "
                              f"jitter_max_us={p.get('jitter_max_us', 'N/A')} "
                              f"missed={p.get('missed', 'N/A')}")
            report.append(f"  work_min_pct={snap.work_min_pct or 'N/A'} "
                          "(lower bound, not total CPU load)")
        report.append(f"  stack_depth_est={system['stack_depth_est'] or 'N/A'} "
                      f"heap_brk_used={system['heap_brk_used'] or 'N/A'}")

        for burst in run.bursts:
            completed, elapsed = burst["completed_bytes"], burst["duration_us"]
            theory = ""
            overhead = ""
            if (isinstance(completed, int) and isinstance(elapsed, int) and completed > 0):
                theory = round(completed * 10_000_000 / baud)  # UART 8N1, 10 bits/byte
                if burst["status"] == "OK" and theory > 0:
                    overhead = f"{(elapsed - theory) * 100 / theory:.2f}"
            bursts.append(dict(common, **burst, theory_us=theory, overhead_pct=overhead))

        for start, end in pairs:
            delays, missing_starts, missing_ends, invalid = event_pairs(run.events, start, end)
            latency_row = dict(common, start_event=start, end_event=end, n=len(delays),
                               min_us=min(delays) if delays else "",
                               avg_us=f"{mean(delays):.2f}" if delays else "",
                               max_us=max(delays) if delays else "",
                               p99_us=percentile_nearest_rank(delays, 99),
                               unmatched_starts=missing_starts, unmatched_ends=missing_ends,
                               invalid_pairs=invalid)
            latencies.append(latency_row)
            if 0 < len(delays) < 30:
                run.warnings.append(f"EVT {start}->{end}: only {len(delays)} samples; need >=30")
            for idx, delta in enumerate(delays, 1):
                samples.append(dict(common, start_event=start, end_event=end,
                                    sample=idx, latency_us=delta))
            report.append(f"  EVT {start}->{end}: n={len(delays)} "
                          f"max_us={latency_row['max_us'] or 'N/A'}")
        if not pairs:
            report.append("  EVT latency: not requested (use --pair START:END)")
        for warning in run.warnings:
            report.append(f"  WARNING: {warning}")
        report.append("")

    write_csv(out / "tasks.csv", ["scenario", "source", "window_ms", "name", "n", "min_us", "avg_us", "max_us"], tasks)
    write_csv(out / "periods.csv", ["scenario", "source", "window_ms", "name", "n", "min_us", "avg_us", "max_us", "jitter_max_us", "missed", "late_max_ms", "deadline_miss", "overruns"], periods)
    write_csv(out / "system.csv", ["scenario", "source", "git", "git_origin", "window_ms", "work_min_pct", "stack_reserved", "stack_depth_est", "stack_spare", "stack_state", "heap_brk_used", "heap_room", "uart_rx_bytes", "uart_rx_lines", "uart_rx_overrun_events", "uart_rx_long_lines", "alarm_messages"], systems)
    write_csv(out / "bursts.csv", ["scenario", "source", "requested_bytes", "completed_bytes", "duration_us", "status", "theory_us", "overhead_pct"], bursts)
    write_csv(out / "latencies.csv", ["scenario", "source", "start_event", "end_event", "n", "min_us", "avg_us", "max_us", "p99_us", "unmatched_starts", "unmatched_ends", "invalid_pairs"], latencies)
    write_csv(out / "latency_samples.csv", ["scenario", "source", "start_event", "end_event", "sample", "latency_us"], samples)

    # Only compare scenarios with compatible, explicitly identified firmware revisions.
    named = {run.scenario: run for run in runs}
    comp = []
    if "S1" in named and "S3" in named:
        a, b = named["S1"], named["S3"]
        if a.build.get("git") and a.build.get("git") == b.build.get("git") and a.last_snapshot() and b.last_snapshot():
            for name in ("ADC", "IMU", "TELEMETRY"):
                pa = a.last_snapshot().periods.get(name, {})
                pb = b.last_snapshot().periods.get(name, {})
                for metric in ("jitter_max_us", "max_us"):
                    va, vb = integer(pa.get(metric)), integer(pb.get(metric))
                    if va != "" and vb != "":
                        comp.append(dict(task=name, metric=metric, s1=va, s3=vb, delta_s3_minus_s1=vb - va))
                # Rate normalized to the size of each observation window.
                va, vb = integer(pa.get("missed")), integer(pb.get("missed"))
                if va != "" and vb != "":
                    ra = va * 60000 / a.last_snapshot().window_ms
                    rb = vb * 60000 / b.last_snapshot().window_ms
                    comp.append(dict(task=name, metric="missed_per_min", s1=f"{ra:.3f}",
                                     s3=f"{rb:.3f}", delta_s3_minus_s1=f"{rb - ra:.3f}"))
        else:
            report.append("WARNING: S1/S3 comparison skipped: firmware git hash missing/different or incomplete STATS")
    write_csv(out / "comparison.csv", ["task", "metric", "s1", "s3", "delta_s3_minus_s1"], comp)
    (out / "summary.txt").write_text("\n".join(report) + "\n", encoding="utf-8")
    return out


def parse_run(arg):
    if "=" not in arg:
        raise argparse.ArgumentTypeError("--run must be SCENARIO=path/to/log.txt")
    scenario, path = arg.split("=", 1)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", scenario) or not path:
        raise argparse.ArgumentTypeError("invalid scenario/path in --run")
    return scenario, path


def parse_pair(arg):
    if ":" not in arg:
        raise argparse.ArgumentTypeError("--pair must be START:END")
    a, b = arg.split(":", 1)
    if not a or not b or a == b:
        raise argparse.ArgumentTypeError("--pair needs two different event names")
    return a, b

def elf_git_sha(elf_path):
    if not elf_path.is_file():
        raise ValueError(f"ELF file not found: {elf_path}")

    result = subprocess.run(
        ["strings", str(elf_path)],
        capture_output=True,
        text=True,
        check=True
    )

    candidates = set(
        re.findall(r"(?m)^([0-9a-f]{12})$", result.stdout)
    )

    if len(candidates) != 1:
        raise ValueError(
            f"Cannot identify one unique build SHA in {elf_path}: "
            f"{sorted(candidates)}"
        )

    return candidates.pop()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        "--run",
        type=parse_run,
        action="append",
        required=True,
        metavar="S1=PATH"
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("docs/metrics/lab3/analysis")
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--pair",
        type=parse_pair,
        action="append",
        default=[],
        metavar="START:END"
    )

    parser.add_argument(
        "--elf",
        type=Path,
        help="Check firmware revision against the compiled ELF"
    )
    parser.add_argument(
        "--acknowledge-legacy-build",
        action="store_true",
        help=(
            "Operator attests that logs without a BUILD banner "
            "were recorded using the supplied ELF"
        )
    )

    args = parser.parse_args(argv)

    if args.baud <= 0:
        parser.error("--baud must be positive")

    if args.acknowledge_legacy_build and args.elf is None:
        parser.error(
            "--acknowledge-legacy-build requires --elf"
        )

    names = [name for name, path in args.run]

    if len(set(names)) != len(names):
        parser.error("scenario names must be unique")

    runs = []

    for name, path in args.run:
        try:
            runs.append(parse_log(name, path))
        except OSError as exc:
            parser.error(str(exc))

    if args.elf is not None:
        try:
            expected_sha = elf_git_sha(args.elf)
        except (OSError, ValueError, subprocess.CalledProcessError) as exc:
            parser.error(str(exc))

        print(f"ELF build: {expected_sha}")

        for current in runs:
            logged_sha = current.build.get("git")

            # Never override an actual, different firmware revision.
            if logged_sha and logged_sha != expected_sha:
                parser.error(
                    f"{current.scenario}: firmware mismatch! "
                    f"Log={logged_sha}, ELF={expected_sha}"
                )

            # Old logs have no captured startup banner.
            # Attribution requires explicit operator acknowledgement.
            if not logged_sha and args.acknowledge_legacy_build:
                current.build["git"] = expected_sha
                current.build["origin"] = "ELF_OPERATOR_ATTESTED"

                current.warnings = [
                    warning for warning in current.warnings
                    if not warning.startswith("no [BUILD]")
                ]

                current.warnings.append(
                    "Firmware revision attributed by operator "
                    "to the supplied ELF; not verified from MCU UART"
                )

    out = make_outputs(runs, args.out, args.baud, args.pair)

    print(f"Saved analysis to: {out}")

    for current in runs:
        print(
            f"  {current.scenario}: "
            f"git={current.build.get('git', 'UNKNOWN')} "
            f"origin={current.build.get('origin', 'UNKNOWN')} "
            f"stats={len(current.snapshots)} "
            f"bursts={len(current.bursts)} "
            f"events={len(current.events)}"
        )

        for warning in current.warnings:
            print(
                f"  WARNING {current.scenario}: {warning}",
                file=sys.stderr
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
