"""Real-bag launcher checks without Docker, ROS nodes, or a graphical session."""

import hashlib
import importlib.util
import json
from pathlib import Path
import shlex
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('replay_real_detector', ROOT / 'tools/replay_real_detector.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def make_bag(root, name='20261003T095724.528830Z-bag'):
    bag = root / name / 'bag'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    (bag / 'scan.mcap').write_bytes(b'real bag fixture placeholder')
    return bag


def test_discovery_console_choice_and_explicit_selection(tmp_path):
    a = make_bag(tmp_path, 'a')
    b = make_bag(tmp_path, 'b with spaces')
    bags = replay.discover_bags(tmp_path)
    assert bags == [a, b]
    answers = iter(['bad', '3', '2'])
    assert replay.select_bag(None, bags, lambda _: next(answers)) == b
    assert replay.select_bag(str(b), bags, lambda _: pytest.fail('Unexpected prompt')) == b
    assert replay.select_bag('a', bags) == a
    assert replay.select_bag(str(a.parent), bags) == a


def test_headless_command_is_scoped_to_current_source_and_own_domain(tmp_path):
    bag = make_bag(tmp_path, 'robot with spaces')
    args = replay.parser().parse_args(['--headless', '--rate', '.5', '--from-seconds', '2',
                                      '--ros-domain-id', '185'])
    output = replay.REPOSITORY / 'results/real-detector-replay/test-run'
    command = replay.container_command(args, bag, 'hsl-detector-replay-own', output)
    assert command[:5] == ['docker', 'run', '--rm', '--init', '--name']
    assert 'ROS_DOMAIN_ID=185' in command and 'ROS_LOCALHOST_ONLY=1' in command
    assert '--privileged' not in command and '--device' not in command
    mounts = [command[i + 1] for i, arg in enumerate(command) if arg == '--mount']
    assert any(m.endswith('dst=/replay,readonly') for m in mounts)
    assert any(f'src={tmp_path}' in m and m.endswith(',readonly') for m in mounts)
    script = command[-1]
    assert '--packages-select jr_perception --parallel-workers 1' in script
    assert '--count 0 --expect any --rate 0.5 --from-seconds 2.0' in script
    assert 'rviz-config' not in script and 'ros2 launch' not in script
    assert shlex.quote(str(bag)) in script
    assert '--source-bag ' + shlex.quote(str(bag)) in script


def test_list_and_dry_run_never_start_docker(tmp_path, monkeypatch, capsys):
    bag = make_bag(tmp_path)
    monkeypatch.setattr(replay.subprocess, 'Popen', lambda *_a, **_k: pytest.fail('Unexpected Docker'))
    assert replay.main(['--bag-root', str(tmp_path), '--list']) == 0
    assert bag.parent.name in capsys.readouterr().out
    assert replay.main(['--bag', str(bag), '--headless', '--dry-run']) == 0
    command = shlex.split(capsys.readouterr().out)
    assert command[:2] == ['docker', 'run']


def test_xauthority_is_mounted_without_xhost(tmp_path, monkeypatch):
    bag = make_bag(tmp_path)
    authority = tmp_path / 'xauthority'
    authority.write_bytes(b'cookie')
    monkeypatch.setenv('DISPLAY', ':0')
    monkeypatch.setenv('XAUTHORITY', str(authority))
    args = replay.parser().parse_args([])
    output = replay.REPOSITORY / 'results/real-detector-replay/test-run'
    command = replay.container_command(args, bag, 'hsl-detector-replay-own', output)
    assert 'XAUTHORITY=/tmp/replay.xauthority' in command
    assert any(f'src={authority}' in value and value.endswith(',readonly') for value in command)
    assert '--hold-seconds -1' in command[-1] and '--rviz-config' in command[-1]
    assert 'xhost' not in ' '.join(command)


def test_cache_rejects_changed_filter_and_corrupt_payload(tmp_path, monkeypatch):
    monkeypatch.setattr(replay, 'REPOSITORY', tmp_path)
    filter_source = tmp_path / 'filter.py'
    filter_source.write_bytes(b'filter v1')
    cache = tmp_path / 'cache'
    cache.mkdir()
    payload = cache / 'clouds.zstbin'
    payload.write_bytes(b'payload')
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = dict(cloud_mode='both', truncated=False, counts={'clouds': {'filtered': 1}},
                    processing_sources=[{'path': 'filter.py', 'sha256': digest(filter_source)}],
                    artifacts=[{'name': payload.name, 'bytes': 7, 'sha256': digest(payload)}])
    replay.verify_cache(cache, manifest, digest)
    filter_source.write_bytes(b'filter v2')
    with pytest.raises(ValueError, match='другим препроцессором'):
        replay.verify_cache(cache, manifest, digest)
    filter_source.write_bytes(b'filter v1')
    payload.write_bytes(b'changed')
    with pytest.raises(ValueError, match='Повреждён'):
        replay.verify_cache(cache, manifest, digest)


def test_interrupt_stops_only_this_replay_container(tmp_path, monkeypatch):
    bag = make_bag(tmp_path)
    fake_repository = tmp_path / 'repo'
    fake_repository.mkdir()
    monkeypatch.setattr(replay, 'REPOSITORY', fake_repository)
    calls = []

    class Process:
        def __init__(self):
            self.count = 0

        def wait(self):
            self.count += 1
            if self.count == 1:
                raise KeyboardInterrupt
            return 0

    def start(command):
        calls.append(command)
        return Process()

    monkeypatch.setattr(replay.subprocess, 'Popen', start)
    monkeypatch.setattr(replay.subprocess, 'run', lambda command, **_kw: calls.append(command))
    assert replay.main(['--bag', str(bag), '--headless']) == 130
    own_name = calls[0][calls[0].index('--name') + 1]
    assert calls[1] == ['docker', 'stop', '--time', '10', own_name]


@pytest.mark.parametrize('option,value', [('--rate', 'nan'), ('--rate', '0'),
                                         ('--from-seconds', '-1'), ('--ros-domain-id', '26')])
def test_bad_replay_parameters_stop_before_docker(option, value):
    with pytest.raises(SystemExit) as error:
        replay.main([option, value])
    assert error.value.code == 2
