#!/usr/bin/env python3
"""Generate reviewable passive config from pinned upstream parameters, no ROS writes."""
import argparse
import hashlib
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
c=yaml.safe_load((root/'src/robot/config/owl_ego.yaml').read_text())
binary=ws/'devel/lib/ego_planner/ego_planner_node'
c['planner'].update(executable=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                    parameters=str(ws/'planner.yaml'))
(ws/'owl_ego.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
print('Passive configuration:',ws/'owl_ego.yaml')
