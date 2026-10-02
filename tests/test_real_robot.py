import copy
import sys
from pathlib import Path
import yaml
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hsl_real'))
from hsl_real.config import load_config,start_polygon


def fixture(tmp_path):
    cfg=yaml.safe_load((ROOT/'config/real.yaml').read_text())
    (tmp_path/'match.yaml').write_text((ROOT/'config/match.yaml').read_text())
    (tmp_path/'livox_mid360.json').write_text((ROOT/'config/livox_mid360.json').read_text())
    path=tmp_path/'real.yaml'
    path.write_text(yaml.safe_dump(cfg))
    return cfg,path


def test_real_config_uses_no_simulation_map_or_opponent_truth(tmp_path):
    _,path=fixture(tmp_path)
    cfg,mission=load_config(path)
    assert cfg['map_file']=='' and cfg['localization']=='odometry'
    assert cfg['ros_domain_id']==26
    assert mission['motion']['max_speed']==.5
    assert start_polygon(mission['opponent'])==pytest.approx([2.59,1.85,3.09,1.85,3.09,2.35,2.59,2.35])


@pytest.mark.parametrize('key,value', [('localization','gazebo'),('ros_domain_id',True),
    ('arena_bounds',[0,1,0,3]),('lidar_mount',[0,0,float('nan'),0,0,0]),
    ('mission_file','../outside.yaml'),('rviz','false')])
def test_bad_real_config_is_rejected(tmp_path,key,value):
    cfg,path=fixture(tmp_path);cfg[key]=value;path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):load_config(path)


def test_static_map_can_be_added_without_changing_code(tmp_path):
    cfg,path=fixture(tmp_path)
    (tmp_path/'arena.yaml').write_text('image: arena.pgm\n')
    cfg['map_file']='arena.yaml';cfg['localization']='external_tf'
    path.write_text(yaml.safe_dump(cfg))
    loaded,_=load_config(path)
    assert loaded['map_file']==str(tmp_path/'arena.yaml')


def test_stop_does_not_require_valid_config(monkeypatch,tmp_path):
    import importlib.util
    spec=importlib.util.spec_from_file_location('real_robot',ROOT/'tools/real_robot.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(sys,'argv',['real_robot','stop','--config',str(tmp_path/'missing.yaml')])
    commands=[]
    monkeypatch.setattr(module.subprocess,'run',lambda args,**kw:commands.append(args))
    module.main()
    assert any('stop' in command for command in commands)


def test_selected_driver_workspace_has_unique_ros_packages():
    import xml.etree.ElementTree as ET
    packages=[ET.parse(p).getroot().findtext('name') for p in (ROOT/'drivers/src').rglob('package.xml')]
    assert len(packages)==len(set(packages))==5
    assert 'kobuki_node' in packages and 'livox_ros_driver2' in packages
