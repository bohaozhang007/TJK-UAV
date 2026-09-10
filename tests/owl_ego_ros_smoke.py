#!/usr/bin/env python3
"""Real EGO + ROS/HTTP integration against a mock FCU on an isolated ROS master.

NOT PX4 SITL and NOT a flight dynamics validation. Refuses port 11311 and requires
an explicit isolated workspace config. No process uses the live robot master.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import hashlib
import urllib.error
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
import uuid
import yaml
import numpy as np
p=argparse.ArgumentParser()
p.add_argument('--config',required=True)
p.add_argument('--port',type=int,default=11421)
p.add_argument('--rounds',type=int,default=3)
p.add_argument('--output',required=True)
p.add_argument('--obstacle-route',action='store_true')
p.add_argument('--vendor-frames',action='store_true')
p.add_argument('--takeoff-retry',action='store_true',help='Console repeats takeoff after landing with FCU spool delay and delayed map delivery')
p.add_argument('--settle-after-takeoff',action='store_true',help='Inject two seconds of drift after takeoff arrival to verify current-stop gating')
p.add_argument('--record-diagnostics',action='store_true',help='Exercise the read-only recorder and offline bag analysis alongside the mock flight')
p.add_argument('--biased-vz',action='store_true',help='Add the observed -0.124 m/s vertical twist bias, without changing simulated position')
p.add_argument('--altitude-offset',action='store_true',help='Mock actual OFFBOARD altitude settles 10 cm above its position setpoint')
p.add_argument('--yaw-lag',action='store_true',help='Mock yaw response limited to 24 deg/s with a proportional settling tail')
p.add_argument('--console-client',action='store_true')
p.add_argument('--console-relative',action='store_true',help='Exercise manual relative console commands instead of test route')
p.add_argument('--landing-frame-mismatch',action='store_true',help='Inject a persistent LIO mismatch during the pending AUTO.LAND service')
p.add_argument('--live-client',action='store_true',help='Run the actual HTTP-only live_sequence client against this mock fixture')
p.add_argument('--faults-only',action='store_true',help='Run fault injection scenarios instead of the three route rounds')
a=p.parse_args()
if a.landing_frame_mismatch and (not a.console_client or a.takeoff_retry):
    p.error('--landing-frame-mismatch requires --console-client without takeoff-retry')
if a.console_relative and (not a.console_client or a.takeoff_retry or a.settle_after_takeoff):
    p.error('--console-relative requires --console-client without takeoff-retry/settle-after-takeoff')
if a.rounds < 3:raise SystemExit('At least three interruption rounds are required')
if not 1024 <= a.port <= 65535 or a.port==11311:raise SystemExit('Refusing live/invalid ROS port')
with socket.socket() as probe:
    probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    try:probe.bind(('127.0.0.1',a.port))
    except OSError:raise SystemExit('ROS port already occupied; refusing to attach to an existing master')
os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(a.port)
os.environ['ROS_IP']='127.0.0.1'
os.environ.pop('ROS_HOSTNAME',None)
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
processes=[]
stop=threading.Event()
temp=Path(a.output).resolve();temp.mkdir(parents=True,exist_ok=True)
events=(temp/'events.jsonl').open('w',buffering=1)
event_lock=threading.Lock()
started=time.monotonic()
phase='startup'
latencies={}
monitor_errors=[]
def record(kind,**values):
    with event_lock:events.write(json.dumps(dict(t_s=time.monotonic()-started,phase=phase,event=kind,**values),allow_nan=False)+'\n')
record('source',git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
       options=vars(a),
       files={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for folder in ['ros/owl_nav','src/robot','tests'] for f in (root/folder).rglob('*.py')})
try:
    master=subprocess.Popen(['roscore','-p',str(a.port)],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(master)
    import rosgraph
    for _ in range(100):
        try:
            rosgraph.Master('/smoke').getPid();break
        except Exception:time.sleep(.1)
    else:raise RuntimeError('isolated master did not start')
    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped,PoseStamped
    from mavros_msgs.msg import State,ExtendedState,PositionTarget
    from mavros_msgs.srv import SetMode,SetModeResponse,CommandBool,CommandBoolResponse
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header, String, Empty
    import tf2_ros
    rospy.init_node('owl_mock_fcu',disable_signals=True)
    cfg=yaml.safe_load(Path(a.config).read_text())
    cfg['control'].update(mavros_frame_profile='owl_vendor_world' if (a.console_client or a.vendor_frames) else 'standard_enu',flight_enabled=True,failsafe_validated=True,sensors_validated=True)
    assert cfg['control']['planning_timeout_s']==10.
    assert cfg['controller']['motion_timeout_s']==15.
    cfg['topics'].update(cloud='/mock/cloud')
    cfg['planner']['log_dir']=str(temp/'planner')
    cfg['hardware'].update(intrinsics_mode='approximate_fov',extrinsics_mode='body_coincident_fixed',
                           fixed_camera_pitch_deg=0.,assumed_horizontal_fov_deg=90.,body_from_camera_optical_rotation=None)
    cfg['hardware']['camera_optical_frame']='mock_camera_optical'
    config=temp/'config.yaml';config.write_text(yaml.safe_dump(cfg))
    diagnostics_process=None
    if a.record_diagnostics:
        diagnostics_log=(temp/'diagnostic_recorder.log').open('w')
        diagnostics_process=subprocess.Popen([sys.executable,str(root/'scripts/owl_ego/record.py'),
            '--config',str(config),'--output',str(temp/'diagnostics')],stdout=diagnostics_log,
            stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(diagnostics_process)
    output=(temp/'bridge.log').open('w')
    bridge=subprocess.Popen(['rosrun','owl_nav','owl_nav_node.py','_config:='+str(config)],stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(bridge)
    topics=cfg['topics']
    odom_pub=rospy.Publisher(topics['odom'],Odometry,queue_size=5)
    map_pub=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=5)
    lio_pub=rospy.Publisher('/mavros/vision_pose/pose',PoseStamped,queue_size=5)
    state_pub=rospy.Publisher(topics['state'],State,queue_size=5)
    ext_pub=rospy.Publisher(topics['extended_state'],ExtendedState,queue_size=5)
    cloud_pub=rospy.Publisher(topics['cloud'],PointCloud2,queue_size=1)
    image_pub=rospy.Publisher(topics['rgb'],Image,queue_size=1)
    info_pub=rospy.Publisher(topics['camera_info'],CameraInfo,queue_size=1)
    mock_lock=threading.Lock()
    xyz=np.zeros(3);yaw=0.;target=np.zeros(3);target_yaw=0.;mode='OFFBOARD';armed=True
    if a.console_client:mode='POSCTL';armed=False;yaw=.7;target_yaw=.7
    fcu_calls=[]
    arm_at=-float('inf')
    landed_override=None;ext_enabled=True
    cloud_enabled=True;lio_offset=0.
    obstacle_points=[(4.5,4.5,1.),(-4.5,-4.5,1.),(4.5,-4.5,1.)]
    if a.obstacle_route:obstacle_points=[(.9,float(y),float(z)) for y in np.arange(-.35,.36,.1) for z in np.arange(.3,1.81,.1)]
    measured_positions=[]
    setpoint_times=[]
    bridge_snap={}
    def bridge_status(m):
        global bridge_snap
        bridge_snap=json.loads(m.data)
        record('bridge',snapshot=bridge_snap)
    rospy.Subscriber(topics['bridge_status'],String,bridge_status,queue_size=50)
    def setpoint(m):
        global target,target_yaw
        with mock_lock:
            target=np.array([m.position.x,m.position.y,m.position.z]);target_yaw=m.yaw
            if cfg['control']['mavros_frame_profile']=='owl_vendor_world':
                # Inverse of vendor transport, independently expressed as map->world.
                target=np.array([m.position.y,-m.position.x,m.position.z])
                target_yaw=(m.yaw-math.pi/2+math.pi)%(2*math.pi)-math.pi
            setpoint_times.append(time.monotonic())
            record('setpoint',position=target.tolist(),yaw=target_yaw)
    rospy.Subscriber(topics['setpoint'],PositionTarget,setpoint,queue_size=1)
    def setmode(req):
        global mode,lio_offset
        if a.console_client and req.custom_mode=='AUTO.LAND':
            begin=time.monotonic()
            if a.landing_frame_mismatch:
                lio_offset=1.
                record('landing_frame_mismatch_injected')
            time.sleep(.7)
            recent=[t for t in setpoint_times if t>=begin]
            if a.landing_frame_mismatch:
                active=bridge_snap.get('health',{}).get('active_task_id')
                task=bridge_snap.get('tasks',{}).get(active,{})
                assert task.get('kind')=='land' and task.get('localization_error'),task
                assert not bridge_snap['health']['control_ready']
                assert not any(t>begin+.3 for t in recent),recent
                record('landing_survives_reset_with_world_output_revoked',task_id=active)
            else:
                assert len(recent)>10 and max(np.diff(recent))<.15, recent
                record('landing_pending_hold_verified',samples=len(recent),max_gap_s=max(np.diff(recent)))
        with mock_lock:
            if a.console_client and req.custom_mode=='OFFBOARD':
                assert len(setpoint_times)>20 and setpoint_times[-1]-setpoint_times[0]>=1.0
                assert abs(xyz[2])<.01 and not armed
                assert abs((target_yaw-yaw+math.pi)%(2*math.pi)-math.pi)<.01
            mode=req.custom_mode
            fcu_calls.append(mode)
            record('mock_mode',mode=mode)
        return SetModeResponse(mode_sent=True)
    rospy.Service('/mavros/set_mode',SetMode,setmode)
    def arm(req):
        global armed,arm_at
        with mock_lock:
            assert req.value and mode=='OFFBOARD' and not armed and abs(xyz[2])<.01
            armed=True;arm_at=time.monotonic()
            fcu_calls.append('ARM')
            record('mock_arm',armed=True)
        return CommandBoolResponse(success=True,result=0)
    rospy.Service('/mavros/cmd/arming',CommandBool,arm)
    static=tf2_ros.StaticTransformBroadcaster()
    tf=TransformStamped();tf.header.stamp=rospy.Time.now();tf.header.frame_id='base_link'
    tf.child_frame_id='mock_camera_optical';tf.transform.translation.x=.10
    # ROS body FLU <- camera optical RDF.
    tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5
    tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5
    static.sendTransform(tf)
    def sensors():
        global xyz,yaw,armed
        seq=0;pending_maps=[];drift_at=None
        while not stop.wait(.02):
            with mock_lock:
                vel=np.clip((target-xyz)*4,-.6,.6)
                if a.altitude_offset and mode=='OFFBOARD' and armed and target[2]>.05:
                    vel[2]=np.clip((target[2]+.10-xyz[2])*4,-.6,.6)
                if a.console_client and (not armed or mode!='OFFBOARD'):vel=np.zeros(3)
                if a.takeoff_retry and mode=='OFFBOARD' and time.monotonic()-arm_at<2.5:vel=np.zeros(3)
                if a.settle_after_takeoff and mode=='OFFBOARD' and armed:
                    if drift_at is None and any(t.get('kind')=='takeoff' and t.get('status')=='arrived' for t in bridge_snap.get('tasks',{}).values()):
                        drift_at=time.monotonic()
                        record('post_takeoff_drift',speed_m_s=.12,duration_s=2.)
                    if drift_at is not None and time.monotonic()-drift_at<2.:
                        vel[0]=.12
                if mode=='AUTO.LAND':vel=np.array([0,0,-.4 if xyz[2]>.01 else 0])
                xyz+=vel*.02
                xyz[2]=max(0.,xyz[2])
                if mode=='AUTO.LAND' and xyz[2]<.02:armed=False
                dy=(target_yaw-yaw+math.pi)%(2*math.pi)-math.pi
                yaw_step=float(np.clip(dy,-.6*.02,.6*.02))
                if a.yaw_lag:
                    yaw_step=float(np.clip(dy*3,-np.deg2rad(24),np.deg2rad(24))*.02)
                if a.takeoff_retry and mode=='AUTO.LAND' and armed:yaw_step=1.5*.02
                if a.console_client and not armed:yaw_step=0.
                yaw+=yaw_step
                pos=xyz.copy();angle=yaw;current_mode=mode;is_armed=armed
                measured_positions.append(pos.tolist())
            stamp=rospy.Time.now();header=Header(seq=seq,stamp=stamp,frame_id='world')
            odom=Odometry(header=header,child_frame_id='base_link')
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=pos
            odom.pose.pose.orientation.z=math.sin(angle/2);odom.pose.pose.orientation.w=math.cos(angle/2)
            # Convert synthetic ENU velocity to body FLU, matching nav_msgs contract.
            odom.twist.twist.linear.x=math.cos(angle)*vel[0]+math.sin(angle)*vel[1]
            odom.twist.twist.linear.y=-math.sin(angle)*vel[0]+math.cos(angle)*vel[1]
            odom.twist.twist.linear.z=vel[2]
            if a.biased_vz:odom.twist.twist.linear.z-=.124
            if cfg['control']['mavros_frame_profile']=='owl_vendor_world':
                odom.twist.twist.linear.x,odom.twist.twist.linear.y=vel[:2]
            odom.twist.twist.angular.z=yaw_step/.02
            if cfg['control']['mavros_frame_profile']=='owl_vendor_world':
                lio=PoseStamped(header=header)
                import copy
                lio.pose=copy.deepcopy(odom.pose.pose);lio.pose.position.x+=lio_offset
                lio_pub.publish(lio)
                map_pose=PoseStamped(header=Header(stamp=stamp,frame_id='map'))
                map_pose.pose.position.x,map_pose.pose.position.y,map_pose.pose.position.z=-pos[1],pos[0],pos[2]
                map_pose.pose.orientation.z=math.sin((angle+math.pi/2)/2)
                map_pose.pose.orientation.w=math.cos((angle+math.pi/2)/2)
                if a.takeoff_retry:pending_maps.append((time.monotonic()+.06,map_pose))
                else:map_pub.publish(map_pose)
            odom_pub.publish(odom)
            while pending_maps and pending_maps[0][0]<=time.monotonic():map_pub.publish(pending_maps.pop(0)[1])
            state_pub.publish(State(header=header,connected=True,armed=is_armed,mode=current_mode))
            if ext_enabled:
                ext_pub.publish(ExtendedState(header=header,landed_state=landed_override if landed_override is not None else (2 if pos[2]>.05 else 1)))
            if seq%5==0:
                if cloud_enabled:cloud_pub.publish(point_cloud2.create_cloud_xyz32(header,obstacle_points))
                # RGB deliberately leads the latest odom by 7 ms.
                h=Header(seq=seq,stamp=rospy.Time.from_sec(stamp.to_sec()+.007),frame_id='mock_camera_optical')
                info_pub.publish(CameraInfo(header=h,width=320,height=240,distortion_model='plumb_bob',D=[0.]*5,
                                  K=[300.,0.,160.,0.,300.,120.,0.,0.,1.],R=[1.,0.,0.,0.,1.,0.,0.,0.,1.]))
                image_pub.publish(Image(header=h,width=320,height=240,encoding='bgr8',step=960,data=bytes(320*240*3)))
            seq+=1
    threading.Thread(target=sensors,daemon=True).start()
    rospy.wait_for_service(topics['command'],timeout=10)
    from robot.controllers.owl_ego import OwlEgoController
    from robot.server import run_http_server,NullKeepalive
    controller=OwlEgoController(config=cfg)
    server=run_http_server(controller,NullKeepalive(),'127.0.0.1',0)
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    url='http://127.0.0.1:'+str(server.server_port)
    def rpc(method,path,data=None,timeout=100):
        request=urllib.request.Request(url+path,data=json.dumps(data).encode() if data is not None else None,method=method)
        begin=time.monotonic()
        try:
            with opener.open(request,timeout=timeout) as r:
                result=json.load(r);code=r.status
        except urllib.error.HTTPError as e:
            result=json.load(e);code=e.code
        elapsed=time.monotonic()-begin
        latencies.setdefault(path.split('?')[0],[]).append(elapsed)
        logged={k:v for k,v in result.items() if k!='rgb_jpeg_base64'}
        record('http',method=method,path=path,latency_s=elapsed,http_status=code,result=logged)
        if code!=200:raise RuntimeError(str(code)+': '+json.dumps(result))
        return result
    def wait(predicate,timeout=25):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            try:
                result=predicate()
                if result:return result
            except Exception:pass
            time.sleep(.1)
        raise RuntimeError('condition timed out; logs: '+str(temp))
    obs=wait(lambda:rpc('GET','/v21/observation'))
    assert not obs['rectified'] and obs['calibration_quality']=='approximate'
    assert obs['geometry_assumptions']['intrinsics']=='approximate_fov'
    assert obs['sync_error_s']<=.05 and obs['image_size']==[320,240]
    if a.console_client:
        wait(lambda:rpc('GET','/health')['health'].get('stopped'))
        commands=temp/'commands.txt'
        if a.console_relative:
            commands.write_text('init\ntakeoff\nwait\nmove_rel_xyz_yaw 50 0 0 0\nwait\n'
                'move_rel_xyz 0 50 0\nwait\nmove_relative_xyz_yaw 0 -50 0 0\nwait\n'
                'move_rel_xyz_yaw 0 0 0 30\nwait\nmove_rel_xyz_yaw 0 0 0 -30\nwait\nland\nwait\nquit\n')
        else:
            commands.write_text('init\ntakeoff\nwait\ntest\nwait\nland\nwait\n'+('init\ntakeoff\nwait\nland\nwait\n' if a.takeoff_retry else '')+'quit\n')
        console_args=[sys.executable,str(root/'scripts/owl_ego/console.py'),'--url',url,'--output',str(temp/'console')]
        if a.settle_after_takeoff:
            # Submit test only after fresh telemetry has observed the injected
            # post-arrival drift, reproducing an operator's delayed command.
            client=subprocess.Popen(console_args,stdin=subprocess.PIPE,text=True,start_new_session=True)
            processes.append(client)
            client.stdin.write('init\ntakeoff\nwait\n');client.stdin.flush()
            wait(lambda:any(t.get('kind')=='takeoff' and t.get('status')=='arrived' for t in bridge_snap.get('tasks',{}).values())
                 and bridge_snap.get('health',{}).get('stopped') is False)
            client.communicate(commands.read_text().split('wait\n',1)[1],timeout=180)
            assert client.returncode==0,client.returncode
        else:
            subprocess.run(console_args+['--commands',str(commands)],check=True,timeout=180)
        assert fcu_calls==['OFFBOARD','ARM','AUTO.LAND']*(2 if a.takeoff_retry else 1),fcu_calls
        assert not armed and xyz[2]<.02
        if a.landing_frame_mismatch:
            assert any(t.get('kind')=='land' and t.get('status')=='arrived' and t.get('localization_error')
                       for t in bridge_snap.get('tasks',{}).values()),bridge_snap
            record('landing_frame_mismatch_verified')
        if not a.takeoff_retry:assert abs((yaw-.7+math.pi)%(2*math.pi)-math.pi)<math.radians(5)
        if a.settle_after_takeoff:
            client_events=[json.loads(line) for line in (temp/'console/events.jsonl').open()]
            assert any(r['event']=='waiting_for_stop' for r in client_events)
            assert any(r['event']=='motion_ready' for r in client_events)
        if diagnostics_process is not None:
            os.killpg(diagnostics_process.pid,signal.SIGINT)
            assert diagnostics_process.wait(timeout=20)==0
            diagnostics_log.close()
            subprocess.run([sys.executable,str(root/'scripts/owl_ego/analyze_recording.py'),str(temp/'diagnostics')],check=True,timeout=30)
            client_events=[json.loads(line) for line in (temp/'console/events.jsonl').open()]
            expected_path='/move_relative_xyz_yaw' if a.console_relative else '/v21/navigation'
            assert any(r['event']=='http_request' and r.get('path')==expected_path for r in client_events)
            if not a.console_relative:assert any(r['event']=='stop_wait_progress' for r in client_events)
            record('diagnostic_recording_verified',folder=str(temp/'diagnostics'))
        if a.console_relative:
            client_events=[json.loads(line) for line in (temp/'console/events.jsonl').open()]
            summaries=[e for e in client_events if e['event']=='relative_motion_summary']
            assert len(summaries)==5 and all(e['completed'] and e['sample_count']>2 for e in summaries)
            requests=[e['request'] for e in client_events if e['event']=='http_request' and e['path']=='/move_relative_xyz_yaw']
            assert [[e[k] for k in ('x','y','z','yaw')] for e in requests]==[[50,0,0,0],[0,50,0,0],[0,-50,0,0],[0,0,0,30],[0,0,0,-30]]
            assert all('on_poll' not in e for e in requests)
            record('console_relative_verified',summaries=summaries)
        record('console_verified',fcu_calls=fcu_calls,landed=True)
        print('PASS: actual console software takeoff + complete EGO sequence + landing, mock FCU',flush=True)
        raise SystemExit(0)
    if a.live_client:
        wait(lambda:rpc('GET','/health')['health'].get('stopped'))
        client=root/'scripts/owl_ego/live_sequence.py'
        subprocess.run([sys.executable,str(client),'--url',url,'--output',str(temp/'client_preview')],check=True)
        subprocess.run([sys.executable,str(client),'--url',url,'--execute','--takeoff','--yes',
                        '--finish','land','--output',str(temp/'client_execute')],check=True)
        print('PASS: actual HTTP-only live client preview + complete sequence against real EGO/mock FCU',flush=True)
        raise SystemExit(0)
    sid=rpc('POST','/v21/session',{'request_id':str(uuid.uuid4())})['session_id']
    lease_stop=threading.Event()
    monitor_stop=threading.Event()
    def heartbeats():
        while not lease_stop.wait(.5):
            try:rpc('POST','/v21/heartbeat',{'session_id':sid},timeout=2)
            except Exception as e:monitor_errors.append('heartbeat: '+str(e))
    def monitor(path):
        while not monitor_stop.wait(.10):
            try:
                value=rpc('GET',path,timeout=2)
                if path=='/health' and phase not in ('startup','takeoff','landing','released','fault'):
                    h=value['health']
                    assert h['control_ready'] and h['hold_ready'] and h['odom_ok'],h
                    assert h['planner_state'] in ('not_required','starting','ready'),h
                    assert not h['planner_ok'] or h['planner_state']=='ready'
                if path=='/v21/observation':
                    assert value['localization_epoch']==obs['localization_epoch']
                    assert value['sync_error_s']<=.05 and value['age_s']<=.4
            except Exception as e:monitor_errors.append(path+': '+str(e))
    workers=[threading.Thread(target=heartbeats,daemon=True),
             threading.Thread(target=monitor,args=('/health',),daemon=True),
             threading.Thread(target=monitor,args=('/v21/observation',),daemon=True)]
    for worker in workers:worker.start()
    def body(**kw):return dict(session_id=sid,request_id=str(uuid.uuid4()),**kw)
    rpc('POST','/init',body())
    phase='takeoff'
    rpc('POST','/takeoff',body())
    home=rpc('GET','/get_pose')['pose']
    print('PASS: approximate observation with RGB ahead of odometry, takeoff',flush=True)
    def navigate(pose):
        data=body(localization_epoch=obs['localization_epoch'],pose=pose)
        value=rpc('POST','/v21/navigation',data)
        assert rpc('POST','/v21/navigation',data)['task_id']==value['task_id']
        return value['task_id']
    def status(tid):return rpc('GET','/v21/navigation/status?task_id='+tid)
    def terminal(tid,expected='arrived',timeout=45):
        def poll():
            value=status(tid)
            if value['status']=='failed':raise AssertionError(value)
            return value if value['status'] in ('arrived','cancelled','failed') else None
        # Unlike startup probing, terminal flight failures must not be swallowed.
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            value=poll()
            if value:
                assert value['status']==expected and value['stopped'],value
                return value
            time.sleep(.08)
        raise AssertionError('task timeout '+tid)
    def current():return rpc('GET','/get_pose')['pose']
    def distance(a,b):return float(np.linalg.norm([a[k]-b[k] for k in ('x','y','z')]))
    def reached(pose,tid):
        value=terminal(tid)
        actual=current()
        error=distance(actual,pose)
        yaw_error=abs((actual['yaw']-pose['yaw']+180)%360-180)
        assert error<=15 and yaw_error<=5,(error,yaw_error)
        record('arrival',task_id=tid,goal=pose,actual=actual,position_error_cm=error,yaw_error_deg=yaw_error,task=value)
        return value
    if a.obstacle_route:
        begin=len(measured_positions)
        goal=dict(x=200,y=0,z=100,yaw=0)
        tid=navigate(goal)
        reached(goal,tid)
        positions=np.asarray(measured_positions[begin:]);obstacles=np.asarray(obstacle_points)
        distances=np.linalg.norm(positions[:,None,:]-obstacles[None,:,:],axis=2).min(axis=1)
        assert distances.min()>.25,float(distances.min())
        record('obstacle_route_verified',min_clearance_m=float(distances.min()),
               max_sideways_m=float(np.abs(positions[:,1]).max()),task_id=tid)
        (temp/'obstacle_positions.json').write_text(json.dumps(positions.tolist()))
        rpc('POST','/land',body());rpc('POST','/v21/session/release',{'session_id':sid})
        lease_stop.set();monitor_stop.set()
        print('PASS: real EGO avoids injected wall, minimum measured clearance '+str(distances.min()),flush=True)
        raise SystemExit(0)
    def ego_pid(generation):
        needle='__ns:=/owl_ego/planners/g_'+generation
        for line in subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines():
            if needle in line:return int(line.split()[0])
        return None
    def restart_fixture():
        global server,controller,bridge,xyz,yaw,target,target_yaw,mode,armed,sid,obs,lease_stop,landed_override,ext_enabled,cloud_enabled,lio_offset
        server.shutdown();server.server_close();controller.close()
        if bridge.poll() is None:
            os.killpg(bridge.pid,signal.SIGTERM);bridge.wait(timeout=8)
        with mock_lock:
            xyz=np.zeros(3);yaw=0.;target=np.zeros(3);target_yaw=0.;mode='OFFBOARD';armed=True
            landed_override=None;ext_enabled=True;cloud_enabled=True;lio_offset=0.
        bridge=subprocess.Popen(['rosrun','owl_nav','owl_nav_node.py','_config:='+str(config)],
            stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(bridge)
        rospy.wait_for_service(topics['command'],timeout=10)
        # Service disappearance/reappearance is not a cached-connection guarantee.
        time.sleep(.3)
        controller=OwlEgoController(config=cfg)
        server=run_http_server(controller,NullKeepalive(),'127.0.0.1',int(url.rsplit(':',1)[1]))
        obs=wait(lambda:rpc('GET','/v21/observation'))
        wait(lambda:rpc('GET','/health')['health'].get('landed_state_fresh'))
        sid=rpc('POST','/v21/session',{'request_id':str(uuid.uuid4())})['session_id']
        lease_stop=threading.Event()
        worker=threading.Thread(target=heartbeats,daemon=True);worker.start()
        # Wait for audit/cloud initialization without relaxing actual flight authorization.
        wait(lambda:rpc('POST','/init',body()))
        rpc('POST','/takeoff',body())
        return worker
    def blocking_track(wait_executing=True):
        result=[]
        def call():
            try:result.append(rpc('POST','/move_relative_xyz_yaw',body(x=200,y=0,z=0,yaw=0,timeout_s=15),timeout=18))
            except Exception as e:result.append(str(e))
        worker=threading.Thread(target=call);worker.start()
        tid=wait(lambda:rpc('GET','/health')['health'].get('active_task_id'))
        if wait_executing:wait(lambda:status(tid)['status']=='executing')
        return tid,worker,result
    def exec_faults():
        global phase,mode,armed,landed_override,ext_enabled,cloud_enabled,lio_offset
        from traj_utils.msg import PolyTraj
        reset_pub=rospy.Publisher(topics['localization_reset'],Empty,queue_size=1)
        for case in ['startup-hold','startup-timeout','execution-heartbeat-loss','lease-expiry','epoch-reset','manual-takeover','cloud-loss','frame-mismatch','landing-confirmation']:
            if case=='frame-mismatch' and not a.vendor_frames:continue
            phase='fault'
            hb_worker=restart_fixture()
            begin=time.monotonic()
            if case=='startup-hold':
                # Freeze this fixture's idle planner so teardown takes its real 1 s
                # escalation window. The new planner must start while hold persists.
                time.sleep(.5)
                master_api=rosgraph.Master('/owl_mock_fcu')
                def live_planner():
                    pubs,_,_=master_api.getSystemState()
                    nodes={n for topic,names in pubs for n in names if n.startswith('/owl_ego/planners/g_')}
                    # Exited roscpp nodes can briefly retain master registrations.
                    live=[ego_pid(n.split('/g_',1)[1].split('/')[0]) for n in nodes]
                    live=[pid for pid in live if pid is not None]
                    return live[0] if len(live)==1 else None
                pid=wait(live_planner,timeout=8)
                os.kill(pid,signal.SIGSTOP)
                sample_start=len(setpoint_times)
                tid=navigate(dict(x=180,y=0,z=100,yaw=0))
                states=[]
                while status(tid)['status']=='planning':
                    states.append(rpc('GET','/health')['health']);time.sleep(.05)
                task=status(tid)
                assert task['timing_s']['first_heartbeat']>1,task
                assert all(h['hold_ready'] and h['control_ready'] for h in states)
                assert any(h['planner_state']=='starting' for h in states)
                assert max(np.diff(setpoint_times[sample_start:]))<.2
                oldgen=task['generation']
                rpc('POST','/v21/navigation/cancel',body(task_id=tid));terminal(tid,'cancelled')
                newer=navigate(dict(x=-50,y=0,z=100,yaw=0))
                stale=rospy.Publisher('/owl_ego/planners/g_'+oldgen+'/trajectory',PolyTraj,queue_size=1)
                delayed=PolyTraj(order=5,traj_id=999,duration=[5.],coef_x=[0,0,0,0,0,20.],
                                coef_y=[0.]*6,coef_z=[0,0,0,0,0,1.])
                for _ in range(5):
                    delayed.start_time=rospy.Time.now();stale.publish(delayed)
                    rpc('POST','/v21/navigation/cancel',body(task_id=tid));time.sleep(.05)
                reached(dict(x=-50,y=0,z=100,yaw=0),newer)
                stale.unregister()
                evidence=dict(task=task,late_generation=oldgen,new_task=newer,
                              max_hold_gap_s=max(np.diff(setpoint_times[sample_start:])))
            else:
                tid,track_worker,track_result=blocking_track(case!='startup-timeout')
                generation=status(tid)['generation']
                if case in ('execution-heartbeat-loss','startup-timeout'):
                    pid=wait(lambda:ego_pid(generation));os.kill(pid,signal.SIGSTOP)
                    if case=='startup-timeout':assert status(tid)['status']=='planning'
                elif case=='lease-expiry':
                    lease_stop.set();hb_worker.join(2)
                elif case=='epoch-reset':
                    reset_pub.publish(Empty())
                elif case=='manual-takeover':
                    with mock_lock:mode='POSCTL'
                elif case=='cloud-loss':
                    cloud_enabled=False
                elif case=='frame-mismatch':
                    assert a.vendor_frames, 'frame mismatch scenario requires --vendor-frames'
                    lio_offset=1.
                else:
                    # An old ON_GROUND sample while still armed cannot later
                    # combine with disarm after it has expired.
                    with mock_lock:landed_override=1
                    time.sleep(.12)
                    ext_enabled=False
                    cloud_enabled=False  # landing must not fail when obstacle feed disappears
                    landing_result=[]
                    def land_call():
                        try:landing_result.append(rpc('POST','/land',body()))
                        except Exception as e:landing_result.append(str(e))
                    land_worker=threading.Thread(target=land_call);land_worker.start()
                    wait(lambda:not armed,timeout=10)
                    assert land_worker.is_alive(),landing_result
                    ext_enabled=True
                    for state in [0,4]:
                        with mock_lock:landed_override=state
                        time.sleep(.2)
                        assert land_worker.is_alive(),landing_result
                    with mock_lock:landed_override=1
                    land_worker.join(3)
                    assert not land_worker.is_alive() and isinstance(landing_result[0],dict),landing_result
                failed=wait(lambda:status(tid) if status(tid)['status']=='failed' else None,timeout=12)
                track_worker.join(3)
                assert not track_worker.is_alive() and track_result and isinstance(track_result[0],str),track_result
                time.sleep(.25)
                output_count=len(setpoint_times)
                if case in ('manual-takeover','epoch-reset','frame-mismatch'):
                    if case=='manual-takeover':
                        with mock_lock:mode='OFFBOARD'
                    time.sleep(.4)
                    assert len(setpoint_times)==output_count
                elif case in ('startup-timeout','execution-heartbeat-loss','lease-expiry','cloud-loss'):
                    wait(lambda:rpc('GET','/health')['health']['stopped'])
                    with mock_lock:held=target.copy()
                    time.sleep(.2)
                    with mock_lock:np.testing.assert_allclose(target,held,atol=1e-8)
                    assert len(setpoint_times)>output_count
                if case=='lease-expiry':
                    try:rpc('POST','/v21/heartbeat',{'session_id':sid});raise AssertionError('expired lease revived')
                    except RuntimeError:pass
                if case=='epoch-reset':
                    h=rpc('GET','/health')['health']
                    assert h['localization_epoch']!=obs['localization_epoch'] and not h['control_ready']
                evidence=dict(task=failed,blocking_result=track_result,output_count_after_fault=output_count,
                              output_count_final=len(setpoint_times),health=rpc('GET','/health')['health'])
            lease_stop.set();hb_worker.join(2)
            result=dict(case=case,elapsed_s=time.monotonic()-begin,evidence=evidence)
            fault_results.append(result);record('fault_pass',**result)
            print('PASS fault: '+case,flush=True)
        reset_pub.unregister()

    completed_targets=[]
    round_results=[]
    waypoints=[dict(x=250,y=0,z=100,yaw=0),dict(x=250,y=250,z=100,yaw=30),
               dict(x=0,y=250,z=100,yaw=-20)]
    for index in range(0 if a.faults_only else a.rounds):
        B=waypoints[index%len(waypoints)]
        phase='round-%d-outbound'%(index+1)
        tid=navigate(B)
        def moving():
            task=status(tid)
            # A vertical tracking offset is not outbound motion toward B.
            with mock_lock:speed=float(np.linalg.norm((target-xyz)[:2]*4))
            return task['status']=='executing' and speed>.18 and distance(current(),B)>100
        wait(moving,timeout=20)
        exposure=rpc('GET','/v21/observation');P=exposure['pose']
        time.sleep(.9)  # synthetic detector inference; vehicle continues flying
        detection=current()
        assert distance(detection,P)>12,(detection,P)
        assert status(tid)['status']=='executing'
        phase='round-%d-cancel'%(index+1)
        cancel_body=body(task_id=tid)
        begin=time.monotonic();rpc('POST','/v21/navigation/cancel',cancel_body)
        cancel_latency=time.monotonic()-begin
        assert cancel_latency<3
        assert rpc('POST','/v21/navigation/cancel',cancel_body)['task_id']==tid
        stopped=terminal(tid,'cancelled')
        stop_pose=current()
        assert distance(stop_pose,P)>12
        phase='round-%d-return-exposure'%(index+1)
        back=navigate(P);reached(P,back)
        # Repeated cancellation of the previous terminal task cannot affect this task.
        rpc('POST','/v21/navigation/cancel',body(task_id=tid))
        assert status(back)['status']=='arrived'
        phase='round-%d-track'%(index+1)
        tracks=[]
        for x,y,z,dyaw in [(35,0,0,0),(0,30,0,0),(-35,0,0,0),(0,0,0,30),(25,-20,0,-20)]:
            before=current();angle=math.radians(before['yaw'])
            expected=dict(x=before['x']+math.cos(angle)*x-math.sin(angle)*y,
                          y=before['y']+math.sin(angle)*x+math.cos(angle)*y,
                          z=P['z'],yaw=(before['yaw']+dyaw+180)%360-180)
            begin=time.monotonic()
            result=rpc('POST','/move_relative_xyz_yaw',body(x=x,y=y,z=z,yaw=dyaw,timeout_s=15),timeout=18)
            duration=time.monotonic()-begin
            assert duration<15
            task=reached(expected,result['task_id'])
            tracks.append(dict(duration_s=duration,task=task,relative=[x,y,z,dyaw],start=before))
        # A blocking TRACK call is independently cancellable while monitors/lease run.
        partial=[]
        def long_track():
            try:partial.append(rpc('POST','/move_relative_xyz_yaw',body(x=150,y=0,z=0,yaw=0,timeout_s=15),timeout=18))
            except Exception as e:partial.append(str(e))
        worker=threading.Thread(target=long_track);worker.start()
        track_id=wait(lambda:rpc('GET','/health')['health'].get('active_task_id'))
        wait(lambda:status(track_id)['status']=='executing')
        rpc('POST','/v21/navigation/cancel',body(task_id=track_id))
        terminal(track_id,'cancelled');worker.join(3)
        assert not worker.is_alive() and partial and isinstance(partial[0],str)
        phase='round-%d-return-after-track'%(index+1)
        back2=navigate(P);reached(P,back2)
        completed_targets.append(exposure['frame_id'])  # injected scan-skip/dedup decision only
        phase='round-%d-resume-original-waypoint'%(index+1)
        resumed=navigate(B)
        assert resumed!=tid
        reached(B,resumed)
        result=dict(round=index+1,original_task=tid,exposure={k:v for k,v in exposure.items() if k!='rgb_jpeg_base64'},
                    detection_pose=detection,stop_pose=stop_pose,stop_distance_from_exposure_cm=distance(stop_pose,P),
                    cancellation=stopped,cancel_latency_s=cancel_latency,return_task=back,tracks=tracks,
                    return_after_track=back2,resumed_task=resumed,waypoint=B)
        round_results.append(result);record('round_complete',**result)
        print('PASS: round %d moving cancel/stop, return P, 5 TRACK actions + TRACK cancel, return P, resume waypoint'%(index+1),flush=True)
    phase='return-home'
    reached(home,navigate(home))
    phase='landing'
    rpc('POST','/land',body())
    h=rpc('GET','/health')['health']
    assert not h['airborne'] and h['landed_state']==1 and h['landed_state_fresh']
    phase='released'
    lease_stop.set()
    workers[0].join(2)
    rpc('POST','/v21/session/release',{'session_id':sid})
    monitor_stop.set()
    for worker in workers[1:]:worker.join(2)
    assert not monitor_errors,monitor_errors[:10]
    fault_results=[]
    if a.faults_only:
        exec_faults()
    for path,limit in [('/v21/heartbeat',2),('/health',2),('/v21/observation',2),('/v21/navigation/cancel',3)]:
        if path in latencies:assert max(latencies[path])<limit,(path,max(latencies[path]))
    summary=dict(ok=True,rounds=round_results,faults=fault_results,monitor_errors=monitor_errors,
                 latencies={k:dict(count=len(v),max_s=max(v),p95_s=float(np.percentile(v,95))) for k,v in latencies.items()},
                 planner_commit=cfg['planner']['commit'],planner_sha256=cfg['planner']['sha256'],
                 planning_timeout_s=cfg['control']['planning_timeout_s'],track_timeout_s=15,
                 duration_s=time.monotonic()-started,simulation='real EGO + mock FCU; NOT PX4 SITL',
                 max_setpoint_gap_s=max(np.diff(setpoint_times)))
    (temp/'result.json').write_text(json.dumps(summary,indent=2))
    print('PASS: '+('fault suite' if a.faults_only else 'all three route rounds')+', explicit landing confirmation; logs: '+str(temp),flush=True)
    server.shutdown();server.server_close();controller.close()
finally:
    if 'lease_stop' in globals():lease_stop.set()
    if 'monitor_stop' in globals():monitor_stop.set()
    if 'server' in globals():server.shutdown();server.server_close()
    if 'controller' in globals():controller.close()
    stop.set()
    record('shutdown',monitor_errors=monitor_errors)
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
