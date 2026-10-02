import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "benchmarks"))
import json
import pytest
from run_duel_series import classify_runtime, validate_runtime_metadata, roles_for_run, container_name
from match_config import load_config, configuration_environment


def test_stopped_evaluation_is_terminal():
    expected = {"gazebo": {"container_id": "a"}, "referee": {"container_id": "b"}}
    observed = {"gazebo": {"id": "a", "running": False},
                "referee": {"id": "b", "running": True}}
    assert classify_runtime(expected, observed) == "stopped"


def test_manual_replacement_is_not_owned_by_evaluation():
    expected = {"gazebo": {"container_id": "a"}}
    assert classify_runtime(expected, {"gazebo": {"id": "new", "running": True}}) == "replaced"
    assert classify_runtime(expected, {}) == "replaced"


def test_observation_timeout_does_not_claim_terminal_runtime():
    assert classify_runtime({"gazebo": {"container_id": "a"}}, None) == "unknown"
    assert classify_runtime({"gazebo": {"container_id": "a"}},
                            {"gazebo": {"id": "a", "running": True}}) == "running"


def test_manual_referee_is_rejected_before_motion_can_be_enabled():
    env = {"DUEL_RUN_ID": "evaluation-20", "DUEL_SEED": "20", "DUEL_REVISION": "abc",
           "DUEL_MAX_ACTIVE_S": "90", "HSL_ROLE": "explorer", "HSL_OPPONENT_ROLE": "guardian",
           "DUEL_OPPONENT_SEED": "1000023"}
    settings = configuration_environment(load_config())
    settings.update(env)
    env = settings
    referee = {"run_id": "evaluation-20", "seed": 20, "code_revision": "abc",
               "max_active_s": 90.0, "first_role": "explorer", "second_role": "guardian",
               "spawn_x": float(env["MAP_ORIGIN_X"]), "spawn_y": float(env["MAP_ORIGIN_Y"]),
               "second_start": json.loads(env["DUEL_SECOND_START"]),
               "first_start": json.loads(env["DUEL_FIRST_START"])}
    params = {"/duel_referee": referee}
    for prefix, role, seed in (("/", "explorer", 20), ("/opponent/", "guardian", 1000023)):
        params[prefix + "trajectory_planner"] = {
            "role": role, "random_seed": seed, "arena_bounds": json.loads(env["DUEL_ARENA_BOUNDS"]), "require_match_active": True}
        params[prefix + "decision_manager"] = {"role": role, "own_max_speed": 0.5,
            "own_start": json.loads(env["DUEL_FIRST_START"] if prefix == "/" else env["DUEL_SECOND_START"]),
            "opponent_start": json.loads(env["DUEL_SECOND_START"] if prefix == "/" else env["DUEL_FIRST_START"])}
        params[prefix + "opponent_detector"] = {"use_sim_time": True, "opponent_max_height": 0.46, "robot.max_gap_share": 0.12, "robot.line_ratio": 0.35, "strong_arc_min_span_deg": 90.0, "allow_merged_strong": False, "strong_min_inlier_fraction": 0.95, "strong_rectangle_ratio": 0.70}
        params[prefix + "hsl_motion_gate"] = {"require_match_active": True}
        params[prefix + "native_mppi"] = {
            "role": role, "random_seed": seed,
            "MPPI": {"PathAngleCritic": {"forward_preference": role == "guardian"},
                     "PreferForwardCritic": {"enabled": False},
                     "GoalAngleCritic": {"enabled": False},
                     "wz_max": 1.5, "vx_max": 0.5, "vx_min": -0.5 if role == "explorer" else 0.0}}
    runtime = {"effective_parameters": params}
    validate_runtime_metadata(runtime, env)
    params["/opponent/decision_manager"]["own_max_speed"] = 0.3
    with pytest.raises(RuntimeError, match="own_max_speed"):
        validate_runtime_metadata(runtime, env)
    params["/opponent/decision_manager"]["own_max_speed"] = 0.5
    params["/native_mppi"]["MPPI"]["PreferForwardCritic"]["enabled"] = True
    with pytest.raises(RuntimeError, match="PreferForwardCritic.enabled"):
        validate_runtime_metadata(runtime, env)
    params["/native_mppi"]["MPPI"]["PreferForwardCritic"]["enabled"] = False
    params["/native_mppi"]["MPPI"]["vx_min"] = -0.35
    with pytest.raises(RuntimeError, match="vx_min"):
        validate_runtime_metadata(runtime, env)
    params["/native_mppi"]["MPPI"]["vx_min"] = -0.5
    params["/opponent/native_mppi"]["MPPI"]["vx_min"] = -0.5
    with pytest.raises(RuntimeError, match="vx_min"):
        validate_runtime_metadata(runtime, env)
    params["/opponent/native_mppi"]["MPPI"]["vx_min"] = 0.0
    referee["run_id"] = "manual"
    with pytest.raises(RuntimeError, match="referee run_id"):
        validate_runtime_metadata(runtime, env)
    referee["run_id"] = "evaluation-20"
    params["/opponent/native_mppi"]["random_seed"] = 1
    with pytest.raises(RuntimeError, match="random_seed"):
        validate_runtime_metadata(runtime, env)


def test_compose_preserves_scientific_looking_revision_as_ros_string():
    import shlex
    import yaml
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             "docker/docker-compose.yaml").read_text())
    for revision in ("5787e30", "0123456", "abc1234"):
        for service in ("hsl-referee", "hsl-metrics", "hsl-opponent-metrics"):
            command = config["services"][service]["command"]
            command = command.replace("$CMD", "/bin/bash -ic").replace(
                "${DUEL_REVISION:-unknown}", revision)
            shell_command = shlex.split(command)[-1]
            ros_arg = next(arg for arg in shlex.split(shell_command)
                           if arg.startswith("code_revision:="))
            assert yaml.safe_load(ros_arg.split(":=", 1)[1]) == revision


def test_fixed_assignment_reproduces_explorer_on_the_second_start():
    assert roles_for_run(0, "guardian") == roles_for_run(1) == ("guardian", "explorer")
    assert roles_for_run(19, "guardian") == ("guardian", "explorer")
    assert roles_for_run(0) == ("explorer", "guardian")


def test_isolation_changes_only_evaluation_container_names():
    env = {"COMPOSE_PROJECT_NAME": "hsl-eval"}
    assert container_name("docker-hsl-control-1", env) == "hsl-eval-hsl-control-1"
    assert container_name("docker-hsl-adapter-1:/tmp/trace.py", env) == "hsl-eval-hsl-adapter-1:/tmp/trace.py"
    assert container_name("docker-gazebo-duel-1", env) == "hsl-eval-gazebo-duel-1"
    assert container_name("docker", env) == "docker"
    assert container_name("/autoware/src/file.py", env) == "/autoware/src/file.py"
    assert container_name("docker-hsl-control-1", {}) == "docker-hsl-control-1"


def test_live_launch_can_wait_for_gazebo_to_appear(monkeypatch):
    import types
    import run_duel_series as runner
    ticks = iter([0, 20, 20, 25, 25, 30, 30])
    monkeypatch.setattr(runner.time, 'monotonic', lambda: next(ticks))
    monkeypatch.setattr(runner.time, 'sleep', lambda _: None)
    # The launcher is alive but gzserver has not appeared yet.
    monkeypatch.setattr(runner.subprocess, 'run', lambda *a, **k: types.SimpleNamespace(returncode=1))
    monkeypatch.setattr(runner, 'command', lambda *a, **k: 'true\n')
    readiness = iter([False, True, True, True, True, True])
    monkeypatch.setattr(runner, 'ready_topic', lambda *a, **k: next(readiness))
    runner.wait_ready({}, 100)


@pytest.mark.parametrize('value,extra,message', [
    ('0.14', ['--unknown-obstacle'], 'at least 0.15'),
    ('nan', ['--unknown-obstacle'], 'finite'),
    ('0.15', [], 'requires unknown-obstacle'),
])
def test_invalid_fixture_height_is_rejected_before_launch(value, extra, message):
    import subprocess
    result = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] /
                            'benchmarks/run_duel_series.py'), '--obstacle-height', value, *extra],
                            text=True, capture_output=True, timeout=10)
    assert result.returncode == 2
    assert message in result.stderr
