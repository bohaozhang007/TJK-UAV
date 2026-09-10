#!/usr/bin/env python3
"""Offline bag export; compares odometry twist with windowed position differences.

These estimates are diagnostics, never an alternative flight stopping gate.
"""
import argparse
from collections import Counter, deque, defaultdict
import csv
import json
import math
from pathlib import Path
import statistics
import sys
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros/owl_nav/src'))
from owl_nav.frames import Frames


def window_velocity(history, seconds):
    if len(history)<2:return None
    end,position=history[-1];start=end-seconds
    for i,((a,p),(b,q)) in enumerate(zip(history, list(history)[1:])):
        if a<=start<=b:
            segment=list(history)[i:]
            if any(not 0<y[0]-x[0]<=.2 for x,y in zip(segment,segment[1:])):return None
            initial=np.asarray(p)+(np.asarray(q)-p)*(start-a)/(b-a)
            return (np.asarray(position)-initial)/seconds
    return None


def export(folder):
    import rosbag
    from tf.transformations import quaternion_matrix,euler_from_quaternion
    folder=Path(folder)
    cfg=yaml.safe_load((folder/'config.yaml').read_text());topics=cfg['topics']
    frames=Frames(cfg['control'].get('mavros_frame_profile','standard_enu'))
    counts=Counter();rows=[];references=[];velocities=[];history=deque();snapshot={};snapshot_at=None
    setpoint=None;setpoint_at=None;state=None;epoch=None
    with rosbag.Bag(str(folder/'flight.bag')) as bag, (folder/'bridge_status.jsonl').open('w') as status_log:
        for topic,m,receipt in bag.read_messages():
            counts[topic]+=1;t=receipt.to_sec()
            if topic==topics['bridge_status']:
                snapshot=json.loads(m.data);snapshot_at=t
                new_epoch=snapshot.get('health',{}).get('localization_epoch')
                if new_epoch!=epoch:history.clear();epoch=new_epoch
                status_log.write(json.dumps(dict(receipt_ros_s=t,snapshot=snapshot))+'\n')
            elif topic==topics['setpoint']:setpoint=m;setpoint_at=t
            elif topic==topics['state']:state=m
            elif topic in (topics['localization_reset'],topics['vision_pose_reset']):history.clear()
            elif topic in ('/mavros/local_position/velocity_local','/mavros/local_position/velocity_body'):
                v=m.twist.linear;w=m.twist.angular
                velocities.append(dict(topic=topic,receipt_ros_s=t,stamp_ros_s=m.header.stamp.to_sec(),
                    frame=m.header.frame_id,vx_m_s=v.x,vy_m_s=v.y,vz_m_s=v.z,wx_rad_s=w.x,wy_rad_s=w.y,wz_rad_s=w.z))
            elif topic in ('/mavros/local_position/pose','/mavros/vision_pose/pose'):
                p=m.pose.position;q=m.pose.orientation
                references.append(dict(topic=topic,receipt_ros_s=t,stamp_ros_s=m.header.stamp.to_sec(),
                    frame=m.header.frame_id,x_m=p.x,y_m=p.y,z_m=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w))
            elif topic==topics['odom']:
                p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular
                stamp=m.header.stamp.to_sec();xyz=np.array([p.x,p.y,p.z]);quat=[q.x,q.y,q.z,q.w]
                row=dict(receipt_ros_s=t,stamp_ros_s=stamp,frame=m.header.frame_id,child_frame=m.child_frame_id,
                    x_m=p.x,y_m=p.y,z_m=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w,
                    raw_vx=v.x,raw_vy=v.y,raw_vz=v.z,raw_wx=w.x,raw_wy=w.y,raw_wz=w.z,
                    epoch=epoch,mode=state.mode if state else '',armed=state.armed if state else None)
                valid=np.isfinite([stamp,*xyz,*quat,v.x,v.y,v.z,w.x,w.y,w.z]).all() and abs(np.linalg.norm(quat)-1)<.02
                row['valid_geometry']=bool(valid)
                if valid:
                    rot=quaternion_matrix(quat)[:3,:3];vel=frames.world_velocity(rot,[v.x,v.y,v.z])
                    row.update(vx_world_m_s=vel[0],vy_world_m_s=vel[1],vz_world_m_s=vel[2],
                        twist_speed_m_s=float(np.linalg.norm(vel)),yaw_rad=euler_from_quaternion(quat)[2])
                    try:row['yaw_rate_deg_s']=math.degrees(frames.yaw_rate(rot,[w.x,w.y,w.z]))
                    except ValueError:pass
                    if history and (stamp<=history[-1][0] or stamp-history[-1][0]>.2):history.clear()
                    history.append((stamp,xyz))
                    while len(history)>2 and history[1][0]<stamp-1.2:history.popleft()
                    for duration,label in ((.2,'200ms'),(.5,'500ms'),(1.,'1000ms')):
                        speed=window_velocity(history,duration)
                        if speed is not None:
                            row.update({f'fd_{label}_{axis}_m_s':float(value) for axis,value in zip(('vx','vy','vz'),speed)})
                            row[f'fd_{label}_speed_m_s']=float(np.linalg.norm(speed))
                else:history.clear()
                h=snapshot.get('health',{});tid=h.get('active_task_id');task=snapshot.get('tasks',{}).get(tid,{})
                row.update(status_age_s=t-snapshot_at if snapshot_at is not None else None,
                    task_id=tid,task_status=task.get('status'),stopped=h.get('stopped'),hold_ready=h.get('hold_ready'),
                    **{'stop_'+k:value for k,value in h.get('stop_diagnostics',{}).items()})
                if setpoint is not None:
                    sp=setpoint.position;sv=setpoint.velocity;sa=setpoint.acceleration_or_force
                    position=np.array([sp.x,sp.y,sp.z]);heading=setpoint.yaw
                    if frames.vendor:position=np.array([sp.y,-sp.x,sp.z]);heading=(heading-math.pi/2+math.pi)%(2*math.pi)-math.pi
                    row.update(setpoint_age_s=t-setpoint_at,setpoint_stamp_ros_s=setpoint.header.stamp.to_sec(),
                        setpoint_coordinate_frame=setpoint.coordinate_frame,setpoint_type_mask=setpoint.type_mask,
                        setpoint_x_raw_m=sp.x,setpoint_y_raw_m=sp.y,setpoint_z_raw_m=sp.z,
                        setpoint_vx_raw_m_s=sv.x,setpoint_vy_raw_m_s=sv.y,setpoint_vz_raw_m_s=sv.z,
                        setpoint_ax_raw_m_s2=sa.x,setpoint_ay_raw_m_s2=sa.y,setpoint_az_raw_m_s2=sa.z,
                        setpoint_yaw_raw_rad=setpoint.yaw,setpoint_yaw_rate_raw_rad_s=setpoint.yaw_rate,
                        setpoint_x_world_m=position[0],setpoint_y_world_m=position[1],setpoint_z_world_m=position[2],
                        setpoint_yaw_world_rad=heading)
                    # Conversion applies only to this bridge's LOCAL_NED input
                    # profile. Ignore position-masked/stale or differently framed targets.
                    if setpoint.coordinate_frame==1 and not setpoint.type_mask&7 and t-setpoint_at<.2:
                        row['reference_error_m']=float(np.linalg.norm(position-xyz))
                rows.append(row)
    for filename,data in [('odometry.csv',rows),('references.csv',references),('velocity_sources.csv',velocities)]:
        fields=list(dict.fromkeys(k for row in data for k in row))
        with (folder/filename).open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(data)
    groups=defaultdict(list)
    for row in rows:
        if row.get('task_status')=='stopping' and row.get('status_age_s',1)<.3:
            groups[(row['epoch'],row['task_id'])].append(row)
    stop_summary=[]
    for (epoch,tid),samples in groups.items():
        stats={}
        for key in ('twist_speed_m_s','vx_world_m_s','vy_world_m_s','vz_world_m_s',
                    'fd_200ms_speed_m_s','fd_500ms_speed_m_s','fd_1000ms_speed_m_s','reference_error_m'):
            values=[r[key] for r in samples if key in r and math.isfinite(r[key])]
            if values:stats[key]=dict(samples=len(values),min=min(values),median=statistics.median(values),max=max(values))
        stop_summary.append(dict(epoch=epoch,task_id=tid,samples=len(samples),statistics=stats))
    missing=[topics[k] for k in ('odom','bridge_status','state','setpoint') if not counts[topics[k]]]
    summary=dict(counts=dict(counts),missing_topics=missing,stopping=stop_summary,
        scope='Offline evidence only. FD is window-averaged displacement, not proof of stop; oscillation may average out. Cached state/setpoint ages are explicit. Receipt order alignment is approximate. No replacement of Robot stopping thresholds.')
    (folder/'analysis_summary.json').write_text(json.dumps(summary,indent=2))
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',help='Directory created by run_owl_ego.sh record')
    args=parser.parse_args();summary=export(args.folder)
    print(json.dumps(summary,indent=2))
    return 1 if summary['missing_topics'] else 0


if __name__=='__main__':raise SystemExit(main())
