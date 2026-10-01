import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "benchmarks"))
from report_motion import trace_error


def test_lateral_error_excludes_startup_and_finish_delay():
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "trace.json"
        path.write_text(json.dumps({"time_series": [
            {"sim_t_s": 9.9, "lateral_global_m": 10},
            {"sim_t_s": 10, "lateral_global_m": -0.03},
            {"sim_t_s": 11, "lateral_global_m": 0.03},
            {"sim_t_s": 12.1, "lateral_global_m": 10}]}))
        rms, p95, count = trace_error(path, "lateral_global_m", (10, 12))
        assert abs(rms - 0.03) < 1e-10
        assert p95 == 0.03 and count == 2


def test_relative_trace_is_not_claimed_as_referee_aligned():
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "trace.json"
        path.write_text(json.dumps({"time_series": [
            {"t_s": 1, "lateral_global_m": 0.03}]}))
        assert trace_error(path, "lateral_global_m", (10, 12)) == (None, None, 0)
