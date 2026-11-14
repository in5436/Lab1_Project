import csv
import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import analyze_lab3 as analyzer  # noqa: E402


def sample_log(window_ms=20000, git="abcdef123456", jitter=110, missed=0):
    return f"""[BUILD] git={git} clock_hz=48000000 baud=115200
[STATS] window_ms={window_ms}
[LOAD] mode=BUSY_LOOP work_min_pct=7.2
[TASK] name=UART_CMD n=2 min_us=10 avg_us=100 max_us=190
[TASK] name=ADC n=1000 min_us=69 avg_us=71 max_us=77
[PERIOD] name=ADC n=999 min_us=19900 avg_us=20000 max_us=20100 jitter_max_us={jitter} missed={missed} late_max_ms=0 deadline_miss=0 overruns=0
[TASK] name=IMU n=1000 min_us=1060 avg_us=1065 max_us=1070
[PERIOD] name=IMU n=999 min_us=19900 avg_us=20000 max_us=20100 jitter_max_us=120 missed={missed} late_max_ms=0 deadline_miss=0 overruns=0
[TASK] name=LOGIC n=1000 min_us=60 avg_us=65 max_us=70
[TASK] name=LED_SAMPLE n=19800 min_us=27 avg_us=29 max_us=35
[TASK] name=TELEMETRY n=19 min_us=9010 avg_us=9100 max_us=9200
[PERIOD] name=TELEMETRY n=18 min_us=999900 avg_us=1000000 max_us=1000100 jitter_max_us=100 missed=0 late_max_ms=0 deadline_miss=0 overruns=0
[TASK] name=LOOP_SAMPLE n=19800 min_us=59 avg_us=121 max_us=9500
[MEM] stack_reserved=1024 depth_est=696 spare=328 state=MEASURED
[MEM] heap_brk_used=1468 heap_room=8628 heap_min_reserve=512
[UART_RX] bytes=33 lines=3 overrun_events=0 long_lines=0
[BURST] start bytes=256
ABCDEFGHIJKLMNOPQRSTUVWXYZ
[BURST] end completed_bytes=256 duration_us=24144 status=OK
EVT t_us=100 name=door_open
EVT t_us=190 name=door_alarm
EVT t_us=300 name=door_open
EVT t_us=410 name=door_alarm
"""


class ParserTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def save(self, name, text):
        p = self.root / name
        p.write_text(text, encoding="utf-8")
        return p

    def read_csv(self, path):
        with path.open(encoding="utf-8", newline="") as f:
            return list(csv.DictReader(f))

    def test_native_logs_and_all_outputs(self):
        p = self.save("s1.txt", sample_log())
        run = analyzer.parse_log("S1", p)
        self.assertTrue(run.last_snapshot().complete())
        self.assertEqual(run.last_snapshot().periods["ADC"]["avg_us"], "20000")
        out = analyzer.make_outputs([run], self.root / "out", 115200, [("door_open", "door_alarm")])
        system = self.read_csv(out / "system.csv")
        self.assertEqual(system[0]["stack_depth_est"], "696")
        self.assertEqual(system[0]["git"], "abcdef123456")
        self.assertEqual(len(self.read_csv(out / "tasks.csv")), 7)
        self.assertEqual(len(self.read_csv(out / "periods.csv")), 3)
        burst = self.read_csv(out / "bursts.csv")[0]
        self.assertEqual(burst["completed_bytes"], "256")
        self.assertEqual(burst["theory_us"], "22222")
        latency = self.read_csv(out / "latencies.csv")[0]
        self.assertEqual(latency["n"], "2")
        self.assertEqual(latency["min_us"], "90")
        self.assertEqual(latency["max_us"], "110")
        self.assertEqual(latency["p99_us"], "110")

    def test_latest_complete_snapshot_not_truncated_tail(self):
        p = self.save("truncated.txt", sample_log() + "\n[STATS] window_ms=40000\n[TASK] name=ADC n=2000\n")
        run = analyzer.parse_log("S2", p)
        self.assertEqual(run.last_snapshot().window_ms, 20000)
        self.assertTrue(any("incomplete" in w for w in run.warnings))

    def test_missing_metrics_are_not_invented(self):
        p = self.save("empty.txt", "[TELEMETRY] ts=1000 ms mode=ACTIVE\n")
        run = analyzer.parse_log("S1", p)
        out = analyzer.make_outputs([run], self.root / "out", 115200, [])
        self.assertEqual(self.read_csv(out / "tasks.csv"), [])
        self.assertEqual(self.read_csv(out / "periods.csv"), [])
        self.assertEqual(self.read_csv(out / "system.csv")[0]["window_ms"], "")
        self.assertTrue(any("no complete" in w for w in run.warnings))

    def test_fifo_unmatched_and_u32_wrap(self):
        events = [(1, "end"), (4294967290, "start"), (5, "end"),
                  (20, "start"), (23, "start"), (27, "end")]
        values, open_starts, stray_ends, invalid = analyzer.event_pairs(events, "start", "end")
        self.assertEqual(values, [11, 7])
        self.assertEqual((open_starts, stray_ends, invalid), (1, 1, 0))

    def test_comparison_requires_same_identified_build(self):
        a = analyzer.parse_log("S1", self.save("s1.txt", sample_log(jitter=50, missed=0)))
        b = analyzer.parse_log("S3", self.save("s3.txt", sample_log(jitter=500, missed=4)))
        out = analyzer.make_outputs([a, b], self.root / "out", 115200, [])
        compare = self.read_csv(out / "comparison.csv")
        self.assertTrue(any(r["task"] == "ADC" and r["metric"] == "jitter_max_us"
                            and r["delta_s3_minus_s1"] == "450" for r in compare))
        c = analyzer.parse_log("S3", self.save("other.txt", sample_log(git="different")))
        analyzer.make_outputs([a, c], self.root / "out2", 115200, [])
        self.assertEqual(self.read_csv(self.root / "out2" / "comparison.csv"), [])

    def test_main_rejects_invalid_arguments_and_handles_unicode(self):
        text = "Привіт\n" + sample_log()
        p = self.save("unicode.txt", text)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(analyzer.main(["--run", f"S1={p}", "--out", str(self.root / "cli")]), 0)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            analyzer.main(["--run", f"S1={p}", "--run", f"S1={p}"])
        self.assertEqual(analyzer.percentile_nearest_rank([1, 2, 3, 100], 99), 100)


if __name__ == "__main__":
    unittest.main()
