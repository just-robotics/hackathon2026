import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "benchmarks"))
import json
import pytest
from run_duel_series import classify_runtime, validate_runtime_metadata, roles_for_run
from scenarios import scenario_environment


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
           "DUEL_OPPONENT_SEED": "1000023", "HSL_LOCAL_BACKEND": "nav2_cpp"}
    env.update(scenario_environment(3))
    referee = {"run_id": "evaluation-20", "seed": 20, "code_revision": "abc",
               "max_active_s": 90.0, "first_role": "explorer", "second_role": "guardian",
               "spawn_x": float(env["SPAWN_X"]), "spawn_y": float(env["DUEL_SPAWN_Y"]),
               "second_start": json.loads(env["DUEL_SECOND_START"])}
    params = {"/duel_referee": referee}
    for prefix, role, seed in (("/", "explorer", 20), ("/opponent/", "guardian", 1000023)):
        params[prefix + "trajectory_planner"] = {
            "role": role, "random_seed": seed, "local_backend": "nav2_cpp",
            "arena_bounds": json.loads(env["DUEL_ARENA_BOUNDS"]), "require_match_active": True}
        params[prefix + "decision_manager"] = {"role": role}
        params[prefix + "hsl_mpc_gate"] = {"require_match_active": True}
        params[prefix + "native_mppi"] = {
            "role": role, "random_seed": seed,
            "MPPI": {"PathAngleCritic": {"forward_preference": role == "guardian"},
                     "PreferForwardCritic": {"enabled": True},
                     "GoalAngleCritic": {"enabled": role == "guardian"}}}
    runtime = {"effective_parameters": params}
    validate_runtime_metadata(runtime, env)
    params["/native_mppi"]["MPPI"]["PreferForwardCritic"]["enabled"] = False
    with pytest.raises(RuntimeError, match="PreferForwardCritic.enabled"):
        validate_runtime_metadata(runtime, env)
    params["/native_mppi"]["MPPI"]["PreferForwardCritic"]["enabled"] = True
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
