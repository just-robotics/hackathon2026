import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'straight_report', Path(__file__).parents[1]/'benchmarks/report_straight_motion.py')
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


def sample(t, omega, **kwargs):
    return dict(sim_t_s=t, speed_mps=0.3, measured_omega_radps=omega,
                own_x_m=0., own_y_m=0., lateral_global_m=0.1,
                global_head_xy_m=[[0., 0.], [0.15, 0.], [0.3, 0.], [0.6, 0.]], **kwargs)


def test_classification_uses_remaining_reference_and_rejects_turn_and_missing_geometry():
    s = sample(0, 0.1)
    assert report.straight_reference(s)
    assert report.straight_reference(dict(s, global_head_xy_m=[[0,0],[0,0],[0.6,0]]))
    assert not report.straight_reference(dict(s, own_x_m=0.4))
    assert not report.straight_reference(dict(s, global_head_xy_m=[[0,0],[0.3,0],[0.3,0.6]]))
    assert not report.straight_reference(dict(s, global_head_xy_m=[]))


def test_deadband_does_not_count_noise_and_time_weights_are_irregular():
    samples = [sample(0, 0.1), sample(0.1, 0.02), sample(0.2, -0.01), sample(0.4, -0.1)]
    r = report.analyze_samples(samples, 0, 0.4)
    assert r['straight_omega_sign_changes'] == 1
    assert r['straight_moving_s'] == pytest.approx(0.4)
    assert r['straight_sign_changes_per_minute'] == pytest.approx(150.)
    assert r['straight_lateral_rms_m'] == pytest.approx(0.1)
    assert r['straight_measured_omega_rms_radps'] == pytest.approx((((.01+.0004)*.1/2+(.0004+.0001)*.1/2+(.0001+.01)*.2/2)/.4)**0.5)


def test_gaps_turns_stops_and_referee_window_reset_sign():
    samples = [sample(0, 0.2), sample(0.1, -0.2), sample(1, 0.2), sample(1.1, -0.2)]
    r = report.analyze_samples(samples, 0.1, 1)
    assert r['straight_omega_sign_changes'] == 0
    assert r['straight_moving_s'] == 0
    for middle in (dict(sample(.1, 0), speed_mps=0),
                   dict(sample(.1, 0), global_head_xy_m=[[0,0],[.2,0],[.2,.6]])):
        r = report.analyze_samples([sample(0,.2), middle, sample(.2,-.2)], 0,.2)
        assert r['straight_omega_sign_changes'] == 0
        assert r['straight_moving_s'] == 0


def test_arc_fraction_is_time_not_sample_count():
    samples = [sample(0, .2), sample(.1, .3), sample(.3, .4), sample(.6, 0)]
    r = report.analyze_samples(samples, 0,.6)
    assert r['observed_interval_s'] == pytest.approx(.6)
    assert r['arc_motion_s'] == pytest.approx(.3)
    assert r['arc_fraction_of_observed_time'] == pytest.approx(.5)


def test_two_straight_paths_with_changed_direction_do_not_count_as_zigzag():
    first = sample(0, .2)
    second = dict(sample(.1, -.2), global_head_xy_m=[[0,0],[0,.6]])
    r = report.analyze_samples([first, second], 0,.1)
    assert r['straight_omega_sign_changes'] == 0
    assert r['straight_moving_s'] == 0
    r = report.analyze_samples([dict(first, behavior=6),
                               dict(sample(.1,-.2), behavior=7)], 0,.1)
    assert r['straight_omega_sign_changes'] == 0


def test_series_uses_each_role_referee_window_and_skips_incomplete_records(tmp_path):
    import json
    records = [{'error': 'startup'},
               {'seed': 3, 'outcome': {'event': 'guardian_capture'},
                'robots': [{'role': 'guardian', 'window_start_sim_s': 10,
                            'window_end_sim_s': 10.1},
                           {'role': 'explorer', 'window_start_sim_s': 10,
                            'window_end_sim_s': 10.1}]}]
    (tmp_path/'index.json').write_text(json.dumps(records))
    for side in ('first', 'second'):
        samples = [sample(9.9, -.2), sample(10, .2), sample(10.1, .2), sample(10.2, -.2)]
        (tmp_path/f'01-trace-{side}.json').write_text(json.dumps({'time_series': samples}))
    rows = report.analyze(tmp_path)['runs']
    assert [r['role'] for r in rows] == ['guardian', 'explorer']
    assert all(r['straight_omega_sign_changes'] == 0 for r in rows)
    assert all(r['straight_moving_s'] == pytest.approx(.1) for r in rows)
