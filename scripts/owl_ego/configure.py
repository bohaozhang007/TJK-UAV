#!/usr/bin/env python3
"""Build planner parameters and refresh repository artifact paths, no ROS writes."""
import argparse
import hashlib
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
p=argparse.ArgumentParser()
p.add_argument('--workspace',required=True)
p.add_argument('--upstream',required=True)
a=p.parse_args()
root=Path(__file__).resolve().parents[2]
ws=Path(a.workspace).resolve()
source=Path(a.upstream)/'swarm-playground/main_ws/src/planner/plan_manage/launch/include/advanced_param.xml'
args=dict(map_size_x_=40,map_size_y_=40,map_size_z_=6,cx=0,cy=0,fx=1,fy=1,max_vel=.5,max_acc=.5,max_jer=2,
          planning_horizon=5,point_num=0,flight_type=1,use_multitopology_trajs=False,drone_id=0)
for i in range(5):
    for axis in 'xyz': args[f'point{i}_{axis}']=0
params={}
for element in ET.parse(source).findall('.//param'):
    value=element.attrib['value']
    params[element.attrib['name']]=args[value[6:-1]] if value.startswith('$(arg ') else yaml.safe_load(value)
params.update({'fsm/realworld_experiment':True,'fsm/fail_safe':False,
               'grid_map/obstacles_inflation':.3,'optimization/obstacle_clearance':.3,
               'grid_map/virtual_ceil':3.,'grid_map/virtual_ground':-.2,
               'optimization/record_opt':False})
# Depth topics are private and unused; fx/fy above are never observation calibration.
(ws/'planner.yaml').write_text(yaml.safe_dump(params,sort_keys=True))
config_path=root/'src/robot/config/owl_ego.yaml'
config_text=config_path.read_text()
c=yaml.safe_load(config_text)
binary=ws/'devel/lib/ego_planner/ego_planner_node'
c['planner'].update(executable=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                    parameters=str(ws/'planner.yaml'))
# Preserve user control settings and comments; never generate a second Robot config.
planner_block=yaml.safe_dump({'planner':c['planner']},sort_keys=False)
updated,count=re.subn(r'^planner:\n.*?(?=^[^\s#]|\Z)',lambda _:planner_block,
                      config_text,flags=re.MULTILINE|re.DOTALL)
if count != 1:
    raise RuntimeError('Expected one top-level planner section in Robot config')
config_path.write_text(updated)
print('Robot configuration (planner artifacts refreshed):',config_path)
