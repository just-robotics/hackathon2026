import importlib.util
import json
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "capture_report", Path(__file__).parents[1] / "benchmarks/report_capture_geometry.py")
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


def test_interpolation_does_not_bridge_missing_data_or_extrapolate():
    points = [(10, {"own_x_m": 0, "own_y_m": 0}),
              (10.2, {"own_x_m": 2, "own_y_m": 1}),
              (12, {"own_x_m": 4, "own_y_m": 2})]
    times = [t for t, _ in points]
    assert report.interpolate(points, times, 10.1, 0.3) == pytest.approx((1, 0.5))
    for stamp in (9, 11, 13):
        assert report.interpolate(points, times, stamp, 0.3) is None


@pytest.mark.parametrize("roles", [("guardian", "explorer"), ("explorer", "guardian")])
def test_role_assignment_and_referee_window_exclude_post_finish_capture(tmp_path, roles):
    record = {"seed": 4, "outcome": {"event": "explorer_goal"},
              "robots": [{"role": role, "window_start_sim_s": 10,
                          "window_end_sim_s": 10.2} for role in roles]}
    (tmp_path / "index.json").write_text(json.dumps([record]))
    for side, role in zip(("first", "second"), roles):
        samples = [{"sim_t_s": t, "own_x_m": (distance if role == "explorer" else 0),
                    "own_y_m": 0, "own_yaw_rad": 0}
                   for t, distance in [(9.9, 0.1), (10, 1), (10.2, 0.8), (10.3, 0.1)]]
        (tmp_path / f"00-trace-{side}.json").write_text(json.dumps({"time_series": samples}))
    result = report.analyze(tmp_path)["runs"][0]
    assert result["event"] == "explorer_goal"
    assert result["paired_samples"] == 2
    assert result["closest"]["distance_m"] == pytest.approx(0.8)
    assert result["samples_with_distance_and_heading"] == 0
