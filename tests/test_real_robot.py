import copy
import shutil
import sys
from pathlib import Path
import yaml
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hsl_real'))
from hsl_real.config import load_config,start_polygon


def fixture(tmp_path):
    cfg=yaml.safe_load((ROOT/'config/real.yaml').read_text())
    for name in ('match.yaml','real_match.yaml','localization.yaml'):
        shutil.copy2(ROOT/'config'/name,tmp_path/name)
    shutil.copytree(ROOT/'config/maps',tmp_path/'maps')
    (tmp_path/'livox_mid360.json').write_text((ROOT/'config/livox_mid360.json').read_text())
    path=tmp_path/'real.yaml'
    path.write_text(yaml.safe_dump(cfg))
    return cfg,path


def test_real_config_uses_no_simulation_map_or_opponent_truth(tmp_path):
    _,path=fixture(tmp_path)
    cfg,mission=load_config(path)
    assert cfg['map_file']==str(tmp_path/'maps/maze_bag_v1.yaml') and cfg['localization']=='amcl'
    assert 'simulation' not in mission
    assert cfg['ros_domain_id']==26
    assert mission['motion']['max_speed']==.5
    assert start_polygon(mission['opponent'])==pytest.approx([.25,3.25,.75,3.25,.75,3.75,.25,3.75])


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


def test_record_config_keeps_raw_sensor_topics(tmp_path):
    from hsl_real.config import load_recording
    path=tmp_path/'recording.yaml'
    path.write_text((ROOT/'config/recording.yaml').read_text())
    cfg=load_recording(path)
    assert {'/livox/lidar','/livox/imu','/odom','/tf_static'}<=set(cfg['bag_topics'])


@pytest.mark.parametrize('key,value',[('keyboard_timeout_s',0),('keyboard_timeout_s',float('nan')),
    ('bag_topics',['/odom','/odom']),('bag_topics',['relative'])])
def test_record_config_rejects_bad_contract(tmp_path,key,value):
    from hsl_real.config import load_recording
    cfg=yaml.safe_load((ROOT/'config/recording.yaml').read_text())
    cfg[key]=value
    path=tmp_path/'recording.yaml';path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):load_recording(path)


def test_stop_bag_uses_running_session_without_reading_config(monkeypatch,tmp_path):
    import importlib.util
    from types import SimpleNamespace
    spec=importlib.util.spec_from_file_location('real_robot_record',ROOT/'tools/real_robot.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(sys,'argv',['real_robot','stop','--config',str(tmp_path/'missing.yaml')])
    calls=[]
    def run(args,**kw):
        calls.append((args,kw))
        if 'ps' in args:return SimpleNamespace(stdout='example-container\n')
        if 'inspect' in args:return SimpleNamespace(stdout='[{"Config":{"Labels":{"org.hsl.real.mode":"bag","org.hsl.real.session":"test"}}}]')
        return SimpleNamespace(stdout='')
    monkeypatch.setattr(module.subprocess,'run',run)
    with pytest.raises(SystemExit):module.main()  # mock service success cannot replace an actual output file
    assert any('python3 - pause' in ' '.join(command) for command,_ in calls)
    assert any('stop' in command and '90' in command for command,_ in calls)


def test_amcl_requires_map_and_parameters(tmp_path):
    cfg,path=fixture(tmp_path)
    cfg['map_file']=''
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='amcl requires'):load_config(path)


def test_localization_rejects_rotated_large_uncertainty():
    # Load the ROS-independent predicate without requiring rclpy on the host.
    import ast
    from math import sqrt,isfinite
    source=ast.parse((ROOT/'src/hsl_real/hsl_real/localization.py').read_text())
    function=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='uncertainty_ok')
    namespace={'sqrt':sqrt,'isfinite':isfinite}
    exec(compile(ast.Module(body=[function],type_ignores=[]),'localization','exec'),namespace)
    cov=[0.]*36;cov[0]=cov[7]=.03;cov[1]=.025;cov[35]=.01
    assert not namespace['uncertainty_ok'](cov,.2,.35)
    cov[1]=0.
    assert namespace['uncertainty_ok'](cov,.2,.35)
    cov[0]=float('nan')
    assert not namespace['uncertainty_ok'](cov,.2,.35)
