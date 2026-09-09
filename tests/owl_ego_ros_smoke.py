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
a=p.parse_args()
if a.port==11311:raise SystemExit('Refusing live ROS port')
os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(a.port)
os.environ['ROS_IP']='127.0.0.1'
os.environ.pop('ROS_HOSTNAME',None)
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
processes=[]
stop=threading.Event()
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
    from geometry_msgs.msg import TransformStamped
    from mavros_msgs.msg import State,ExtendedState,PositionTarget
    from mavros_msgs.srv import SetMode,SetModeResponse
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header
    import tf2_ros
    rospy.init_node('owl_mock_fcu',disable_signals=True)
    cfg=yaml.safe_load(Path(a.config).read_text())
    cfg['control'].update(flight_enabled=True,failsafe_validated=True,sensors_validated=True,
                          planning_timeout_s=20.,task_timeout_s=90.)
    cfg['topics'].update(cloud='/mock/cloud')
    cfg['hardware'].update(intrinsics_mode='camera_info',extrinsics_mode='tf')
    cfg['hardware']['camera_optical_frame']='mock_camera_optical'
    temp=Path(tempfile.mkdtemp(prefix='owl-ego-smoke-'))
    config=temp/'config.yaml';config.write_text(yaml.safe_dump(cfg))
    output=(temp/'bridge.log').open('w')
    bridge=subprocess.Popen(['rosrun','owl_nav','owl_nav_node.py','_config:='+str(config)],stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(bridge)
    topics=cfg['topics']
    odom_pub=rospy.Publisher(topics['odom'],Odometry,queue_size=5)
    state_pub=rospy.Publisher(topics['state'],State,queue_size=5)
    ext_pub=rospy.Publisher(topics['extended_state'],ExtendedState,queue_size=5)
    cloud_pub=rospy.Publisher(topics['cloud'],PointCloud2,queue_size=1)
    image_pub=rospy.Publisher(topics['rgb'],Image,queue_size=1)
    info_pub=rospy.Publisher(topics['camera_info'],CameraInfo,queue_size=1)
    mock_lock=threading.Lock()
    xyz=np.zeros(3);yaw=0.;target=np.zeros(3);target_yaw=0.;mode='OFFBOARD';armed=True
    def setpoint(m):
        global target,target_yaw
        with mock_lock:
            target=np.array([m.position.x,m.position.y,m.position.z]);target_yaw=m.yaw
    rospy.Subscriber(topics['setpoint'],PositionTarget,setpoint,queue_size=1)
    def setmode(req):
        global mode
        with mock_lock:mode=req.custom_mode
        return SetModeResponse(mode_sent=True)
    rospy.Service('/mavros/set_mode',SetMode,setmode)
    static=tf2_ros.StaticTransformBroadcaster()
    tf=TransformStamped();tf.header.stamp=rospy.Time.now();tf.header.frame_id='base_link'
    tf.child_frame_id='mock_camera_optical';tf.transform.translation.x=.10
    # ROS body FLU <- camera optical RDF.
    tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5
    tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5
    static.sendTransform(tf)
    def sensors():
        global xyz,yaw,armed
        seq=0
        while not stop.wait(.02):
            with mock_lock:
                vel=np.clip((target-xyz)*4,-.6,.6)
                if mode=='AUTO.LAND':vel=np.array([0,0,-.4 if xyz[2]>.01 else 0])
                xyz+=vel*.02
                xyz[2]=max(0.,xyz[2])
                if mode=='AUTO.LAND' and xyz[2]<.02:armed=False
                dy=(target_yaw-yaw+math.pi)%(2*math.pi)-math.pi
                yaw+=np.clip(dy,-.6*.02,.6*.02)
                pos=xyz.copy();angle=yaw;current_mode=mode;is_armed=armed
            stamp=rospy.Time.now();header=Header(seq=seq,stamp=stamp,frame_id='world')
            odom=Odometry(header=header,child_frame_id='base_link')
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=pos
            odom.pose.pose.orientation.z=math.sin(angle/2);odom.pose.pose.orientation.w=math.cos(angle/2)
            # Convert synthetic ENU velocity to body FLU, matching nav_msgs contract.
            odom.twist.twist.linear.x=math.cos(angle)*vel[0]+math.sin(angle)*vel[1]
            odom.twist.twist.linear.y=-math.sin(angle)*vel[0]+math.cos(angle)*vel[1]
            odom.twist.twist.linear.z=vel[2]
            odom_pub.publish(odom)
            state_pub.publish(State(header=header,connected=True,armed=is_armed,mode=current_mode))
            ext_pub.publish(ExtendedState(header=header,landed_state=2 if pos[2]>.05 else 1))
            if seq%5==0:
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(header,[(3.,3.,1.),(-3.,-3.,1.),(3.,-3.,1.)]))
                h=Header(seq=seq,stamp=stamp,frame_id='mock_camera_optical')
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
        with opener.open(request,timeout=timeout) as r:return json.load(r)
    def wait(predicate,timeout=25):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            try:
                result=predicate()
                if result:return result
            except Exception:pass
            time.sleep(.1)
        raise RuntimeError('condition timed out; logs: '+str(temp))
    wait(lambda:rpc('GET','/health')['health'].get('planner_ok'))
    obs=wait(lambda:rpc('GET','/v21/observation'))
    assert obs['rectified'] and obs['image_size']==[320,240]
    sid=rpc('POST','/v21/session',{'request_id':str(uuid.uuid4())})['session_id']
    lease_stop=threading.Event()
    def heartbeats():
        while not lease_stop.wait(.3):rpc('POST','/v21/heartbeat',{'session_id':sid})
    threading.Thread(target=heartbeats,daemon=True).start()
    def body(**kw):return dict(session_id=sid,request_id=str(uuid.uuid4()),**kw)
    rpc('POST','/init',body())
    rpc('POST','/takeoff',body())
    print('PASS: synchronized observation and takeoff',flush=True)
    def navigate(x,y,yaw=0):
        wait(lambda:rpc('GET','/health')['health'].get('planner_ok'))
        return rpc('POST','/v21/navigation',body(localization_epoch=obs['localization_epoch'],pose=dict(x=x,y=y,z=100,yaw=yaw)))['task_id']
    def status(tid):return rpc('GET','/v21/navigation/status?task_id='+tid)
    tid=navigate(150,0)
    wait(lambda:status(tid)['status']=='executing')
    rpc('POST','/v21/navigation/cancel',body(task_id=tid))
    wait(lambda:status(tid)['status']=='cancelled')
    assert status(tid)['stopped']
    print('PASS: real EGO planning, asynchronous cancel and measured stop',flush=True)
    tid2=navigate(-50,30,45)
    wait(lambda:status(tid2)['status']=='arrived',timeout=45)
    rpc('POST','/v21/navigation/cancel',body(task_id=tid))
    assert status(tid2)['status']=='arrived'
    print('PASS: new EGO process resumes backward navigation and independent yaw',flush=True)
    rpc('POST','/move_relative_xyz_yaw',body(x=0,y=0,z=0,yaw=30,timeout_s=15))
    rpc('POST','/land',body())
    assert not rpc('GET','/health')['health']['airborne']
    rpc('POST','/v21/session/release',{'session_id':sid})
    lease_stop.set()
    print('PASS: relative yaw, landing confirmation, session release; logs: '+str(temp),flush=True)
    server.shutdown();server.server_close();controller.close()
finally:
    stop.set()
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
