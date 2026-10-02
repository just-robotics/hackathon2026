import sys
from pathlib import Path
import copy
import pytest
import yaml
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hsl_sim_adapter'))
from hsl_sim_adapter.scenario_obstacles import load_obstacles,box_sdf


def test_boxes_are_separate_from_static_world():
    cfg=load_obstacles(ROOT/'config/simulation_obstacles.yaml')
    assert cfg['enabled']
    assert [b['size'] for b in cfg['boxes']].count([.15,.15,.4])==3
    assert [b['size'] for b in cfg['boxes']].count([.4,.6,.2])==1
    static=(ROOT/'src/sim_kobuki/worlds/polygon_rosbag.world').read_text()
    for box in cfg['boxes']:
        assert box['name'] not in static
        assert list(map(float,ET.fromstring(box_sdf(box)).findtext('model/link/collision/geometry/box/size').split()))==box['size']


@pytest.mark.parametrize('key,value',[('size',[0,.15,.4]),('pose',[0,0,float('nan')]),('name','kobuki')])
def test_bad_box_configuration_rejected(tmp_path,key,value):
    cfg=copy.deepcopy(load_obstacles(ROOT/'config/simulation_obstacles.yaml'))
    cfg['boxes'][0][key]=value
    path=tmp_path/'boxes.yaml';path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):load_obstacles(path)


def test_small_boxes_have_physical_collision_and_can_be_pushed():
    cfg=load_obstacles(ROOT/'config/simulation_obstacles.yaml')
    for box in cfg['boxes']:
        model=ET.fromstring(box_sdf(box)).find('model')
        assert model.find('link/collision') is not None
        if box['size']==[.15,.15,.4]:
            assert model.findtext('static')=='false'
            assert float(model.findtext('link/inertial/mass'))>0
        else:
            assert model.findtext('static')=='true'
