#!/usr/bin/env python3
"""Read-only flight diagnostics. Never calls a flight service or publishes setpoints."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import yaml


def recording_topics(config):
    topics=config['topics']
    return list(dict.fromkeys([topics[k] for k in
        ('odom','state','extended_state','setpoint','bridge_status','localization_reset','vision_pose_reset')]
        +['/mavros/local_position/pose','/mavros/vision_pose/pose',
          '/mavros/local_position/velocity_local','/mavros/local_position/velocity_body']))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    parser.add_argument('--output')
    parser.add_argument('--duration',type=float,default=0,help='0: record until Ctrl-C; otherwise seconds, up to 3600')
    args=parser.parse_args()
    if not 0<=args.duration<=3600:parser.error('duration must be 0..3600')
    cfg=yaml.safe_load(Path(args.config).read_text())
    import roslib.packages
    binaries=roslib.packages.find_node('rosbag','record')
    if not binaries:parser.error('rosbag record executable not found')
    folder=Path(args.output or 'logs/owl_diagnostics/'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]).resolve()
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'config.yaml').write_text(yaml.safe_dump(cfg))
    (folder/'planner.yaml').write_text(Path(cfg['planner']['parameters']).read_text())
    root=Path(__file__).resolve().parents[2]
    files=['scripts/owl_ego/record.py','scripts/owl_ego/analyze_recording.py','scripts/owl_ego/live_sequence.py',
           'scripts/owl_ego/console.py','ros/owl_nav/scripts/owl_nav_node.py',
           'ros/owl_nav/src/owl_nav/core.py','ros/owl_nav/src/owl_nav/frames.py']
    topics=recording_topics(cfg)
    manifest=dict(start_wall_time_s=time.time(),ros_master=os.environ.get('ROS_MASTER_URI','http://localhost:11311'),
        config_source=str(Path(args.config).resolve()),topics=topics,
        files={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files},
        scope='Subscribed messages only; bag receipt time is not sensor acquisition time. No flight control.')
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print('只读录制位置/速度、map/LIO、setpoint、任务和飞控状态：'+str(folder),flush=True)
    print('保持此终端运行；完成降落后 Ctrl-C 停止并封存 bag。相机与点云不录制。',flush=True)
    # Own the actual recorder, not the Python rosbag CLI wrapper, whose SIGINT
    # handler can return before the native child has finished indexing the bag.
    command=[binaries[0],'--buffsize=64','-O',str(folder/'flight.bag'),*topics]
    started=time.monotonic()
    announced=False
    with (folder/'recorder.log').open('w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while process.poll() is None:
                if not announced and (folder/'flight.bag.active').exists():
                    print('录制已启动。飞行前确认本终端没有退出；录制错误见 recorder.log。',flush=True)
                    announced=True
                if args.duration and time.monotonic()-started>=args.duration:break
                time.sleep(.1)
        except KeyboardInterrupt:
            pass
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    process.wait(timeout=5)
    result=dict(exit_code=process.returncode,duration_s=time.monotonic()-started,
                finalized=(folder/'flight.bag').exists() and not (folder/'flight.bag.active').exists())
    (folder/'recording_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if result['finalized'] and process.returncode==0 else 1


if __name__=='__main__':raise SystemExit(main())
