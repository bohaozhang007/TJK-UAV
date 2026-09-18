#!/usr/bin/env python3
"""Read-only sampled OWL world/map/LIO alignment report; never commands flight."""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True)
    p.add_argument('--duration',type=float,default=8.)
    p.add_argument('--output')
    a=p.parse_args()
    if not 2<=a.duration<=30:p.error('duration must be 2..30 seconds')
    cfg=yaml.safe_load(Path(a.config).read_text())
    if cfg['control'].get('mavros_frame_profile')!='owl_vendor_world':
        p.error('this report checks the owl_vendor_world profile')
    rospy.init_node('owl_frame_check',anonymous=True,disable_signals=True)
    data={k:[] for k in ('odom','map','lio','cloud')};lock=threading.Lock()
    def callback(kind,m):
        item=dict(stamp=m.header.stamp.to_sec(),frame=m.header.frame_id,age_s=rospy.Time.now().to_sec()-m.header.stamp.to_sec())
        if kind=='cloud':item['points']=m.width*m.height
        else:
            pose=m.pose.pose if kind=='odom' else m.pose
            q=pose.orientation
            item.update(xyz=[pose.position.x,pose.position.y,pose.position.z],
                        yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2])
        with lock:data[kind].append(item)
    topics=[('odom',cfg['topics']['odom'],Odometry),('map','/mavros/local_position/pose',PoseStamped),
            ('lio','/mavros/vision_pose/pose',PoseStamped),('cloud',cfg['topics']['cloud'],PointCloud2)]
    subs=[rospy.Subscriber(topic,cls,lambda m,k=k:callback(k,m),queue_size=20) for k,topic,cls in topics]
    time.sleep(a.duration)
    for sub in subs:sub.unregister()
    report=dict(profile='owl_vendor_world',duration_s=a.duration,errors=[],counts={k:len(v) for k,v in data.items()},comparisons={})
    report['mavros_initial_offsets']={k:rospy.get_param('/mavros/'+k,0.) for k in ('x','y','z','R','P','Y')}
    if any(not np.isfinite(float(v)) or abs(float(v))>1e-9 for v in report['mavros_initial_offsets'].values()):
        report['errors'].append('nonzero or invalid vendor initial offsets')
    for k,values in data.items():
        expected='map' if k=='map' else cfg['control']['world_frame']
        if len(values)<10 or any(v['frame']!=expected or not 0<=v['age_s']<=.5 for v in values):
            report['errors'].append(k+': insufficient, stale or wrong-frame data')
    for kind in ('map','lio'):
        pairs=[]
        for o in data['odom']:
            if not data[kind]:break
            r=min(data[kind],key=lambda r:abs(r['stamp']-o['stamp']))
            if abs(r['stamp']-o['stamp'])>(1e-6 if kind=='map' else .05):continue
            xyz,yaw=r['xyz'],r['yaw']
            if kind=='map':xyz=[xyz[1],-xyz[0],xyz[2]];yaw-=math.pi/2
            pairs.append([float(np.linalg.norm(np.array(o['xyz'])-xyz)),math.degrees(abs((o['yaw']-yaw+math.pi)%(2*math.pi)-math.pi))])
        if len(pairs)<10:report['errors'].append(kind+': insufficient synchronized pairs');continue
        v=np.array(pairs);maximum=v.max(axis=0)
        report['comparisons'][kind]=dict(samples=len(pairs),max_position_error_m=float(maximum[0]),max_yaw_error_deg=float(maximum[1]))
        if not np.isfinite(v).all() or maximum[0]>(.03 if kind=='map' else .25) or maximum[1]>(2 if kind=='map' else 10):
            report['errors'].append(kind+': alignment mismatch')
    report['ok']=not report['errors']
    report['scope']='sampled coordinate consistency only; not flight, obstacle coverage or failsafe certification'
    text=json.dumps(report,indent=2,allow_nan=False)
    if a.output:Path(a.output).write_text(text+'\n')
    print(text)
    return 0 if report['ok'] else 1

if __name__=='__main__':raise SystemExit(main())
