import copy
import json
import sys
from pathlib import Path
import pytest
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'benchmarks'))
from match_config import load_config, configuration_environment


def test_map_origin_is_independent_of_robot_start(tmp_path):
    cfg = load_config()
    cfg['robot']['start'] = [1.0, 2.0, 0.7]
    cfg['motion']['allow_reverse'] = False
    path = tmp_path/'match.yaml'
    path.write_text(yaml.safe_dump(cfg))
    env = configuration_environment(load_config(path))
    assert float(env['SPAWN_X']) == pytest.approx(0.66)
    assert float(env['DUEL_SPAWN_Y']) == pytest.approx(2.4)
    assert float(env['MAP_ORIGIN_X']) == -0.34
    assert env['DUEL_SPAWN_YAW'] == '0.7'
    assert json.loads(env['DUEL_FIRST_START']) == [.5,1.5,1.5,1.5,1.5,2.5,.5,2.5]
    assert env['HSL_ALLOW_REVERSE'] == 'false'
    cfg['robot']['role'] = 'guardian'
    path.write_text(yaml.safe_dump(cfg))
    assert configuration_environment(load_config(path))['HSL_OPPONENT_ROLE'] == 'explorer'


@pytest.mark.parametrize('section,key,value', [
    ('robot','start',[float('nan'),0,0]), ('robot','role','invalid'),
    ('motion','allow_reverse','false'), ('motion','max_speed',0),
    ('match','seed',True), ('simulation','world','maze;echo bad'),
    ('simulation','arena_bounds',[3,0,1,2]), ('robot','typo',1)])
def test_invalid_configuration_is_rejected_before_start(tmp_path,section,key,value):
    cfg = copy.deepcopy(load_config())
    cfg[section][key] = value
    path = tmp_path/'match.yaml'
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):
        load_config(path)
