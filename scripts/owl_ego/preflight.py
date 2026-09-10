#!/usr/bin/env python3
"""Read-only deployment report; does not arm, set modes, or stop any service."""
import argparse
import json
import os
import time
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"src"))
from robot.controllers.owl_ego_observation import camera_intrinsics,fixed_optical_rotation
from types import SimpleNamespace
import rospy
import rosgraph
from sensor_msgs.msg import CameraInfo,Image,PointCloud2
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,ExtendedState
import yaml
p=argparse.ArgumentParser()
p.add_argument('--config',required=True)
p.add_argument('--require-flight-ready',action='store_true')
a=p.parse_args()
with open(a.config) as f:c=yaml.safe_load(f)
rospy.init_node('owl_ego_readonly_preflight',anonymous=True,disable_signals=True)
report={'ros_master':os.environ.get('ROS_MASTER_URI'),'errors':[],'warnings':[],'sensors':{}}
report['mavros_frame_profile']=c['control'].get('mavros_frame_profile','standard_enu')
if report['mavros_frame_profile'] not in ('standard_enu','owl_vendor_world'):
 report['errors'].append('invalid mavros_frame_profile')
pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
report['motion_publishers']={t:n for t,n in pubs if t.startswith('/mavros/setpoint_') and
 t not in ('/mavros/setpoint_raw/target_local','/mavros/setpoint_raw/target_global',
           '/mavros/setpoint_raw/target_attitude','/mavros/setpoint_trajectory/desired')}
if any(report['motion_publishers'].values()):report['errors'].append('existing MAVROS motion publishers; resolve controller ownership')
for key,cls in [('odom',Odometry),('rgb',Image),('camera_info',CameraInfo),('cloud',PointCloud2),('state',State),('extended_state',ExtendedState)]:
 try:
  m=rospy.wait_for_message(c['topics'][key],cls,timeout=2)
  value={'frame':m.header.frame_id,'age_s':rospy.Time.now().to_sec()-m.header.stamp.to_sec()}
  if cls in (Image,CameraInfo):value.update(width=m.width,height=m.height)
  if cls==Image:value['encoding']=m.encoding
  if cls==CameraInfo:
   value.update(K=list(m.K),D=list(m.D),distortion_model=m.distortion_model)
   if m.K[0]<=0 or m.K[4]<=0:
    report['warnings' if c['hardware'].get('intrinsics_mode')=='approximate_fov' else 'errors'].append('RGB CameraInfo is uncalibrated')
  if cls==State:value.update(armed=m.armed,mode=m.mode,connected=m.connected)
  if cls==ExtendedState:value['landed_state']=m.landed_state
  report['sensors'][key]=value
 except Exception as e:
  report['warnings' if key=='camera_info' and c['hardware'].get('intrinsics_mode')=='approximate_fov' else 'errors'].append(key+': '+str(e))
mode=c['hardware'].get('extrinsics_mode','tf')
if mode=='tf' and not c['hardware']['camera_optical_frame']:
 report['errors'].append('TF mode requires calibrated optical frame and stamped body->camera TF')
elif mode=='body_coincident_fixed':
 try:fixed_optical_rotation(c['hardware'])
 except ValueError as e:report['errors'].append(str(e))
elif mode!='tf':
 report['errors'].append('unknown extrinsics mode')
if c['hardware'].get('intrinsics_mode')=='approximate_fov' and 'rgb' in report['sensors']:
 try:
  rgb=report['sensors']['rgb']
  k,_,_,assumptions=camera_intrinsics(c['hardware'],SimpleNamespace(width=rgb['width'],height=rgb['height']),None)
  report['approximate_intrinsics_at_source_resolution']=k.tolist()
  report['geometry_assumptions']=assumptions
  report['warnings'].append('Approximate geometry: current strict Agent must explicitly support rectified:false before integration')
 except (ValueError,KeyError) as e:report['errors'].append(str(e))
for key in ('flight_enabled','failsafe_validated','sensors_validated'):
 if not c['control'][key]:report['errors'].append('control/'+key+' is false')
if not c['planner']['executable'] or not c['planner']['sha256']:report['errors'].append('build pinned upstream and generate planner config')
print(json.dumps(report,indent=2,ensure_ascii=False))
if a.require_flight_ready and report['errors']:raise SystemExit(1)
