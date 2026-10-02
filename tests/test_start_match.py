"""Manual launch must verify the runtime before granting either permission."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'benchmarks'))
import start_match


@pytest.mark.parametrize('prepare_only', [False, True])
def test_manual_start_verifies_before_permissions(monkeypatch, tmp_path, prepare_only):
    events = []
    monkeypatch.setattr(sys, 'argv', ['start_match'] + (['--prepare-only'] if prepare_only else []))
    monkeypatch.setattr(start_match, 'RESULTS', tmp_path)
    monkeypatch.setattr(start_match, 'revision', lambda: 'test-revision')
    monkeypatch.setattr(start_match.subprocess, 'run', lambda args, **kw: events.append(args[1]))
    monkeypatch.setattr(start_match, 'wait_ready', lambda *args: events.append('ready'))
    monkeypatch.setattr(start_match, 'runtime_snapshot', lambda *args: events.append('verified') or {})
    monkeypatch.setattr(start_match, 'allow_motion', lambda *args: events.append('allowed'))
    start_match.main()
    assert events == ['clean', 'up', 'ready', 'verified'] + ([] if prepare_only else ['allowed'])
    timing = json.loads(next(tmp_path.glob('*/startup-timing.json')).read_text())
    assert ('prepared_s' if prepare_only else 'permissions_granted_s') in timing


def test_invalid_runtime_never_grants_motion(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, 'argv', ['start_match'])
    monkeypatch.setattr(start_match, 'RESULTS', tmp_path)
    monkeypatch.setattr(start_match, 'revision', lambda: 'test-revision')
    monkeypatch.setattr(start_match.subprocess, 'run', lambda *args, **kw: None)
    monkeypatch.setattr(start_match, 'wait_ready', lambda *args: None)
    def rejected(*args):
        raise RuntimeError('image does not match source')
    monkeypatch.setattr(start_match, 'runtime_snapshot', rejected)
    monkeypatch.setattr(start_match, 'allow_motion', lambda *args: pytest.fail('grant on invalid image'))
    with pytest.raises(RuntimeError, match='image does not match'):
        start_match.main()
