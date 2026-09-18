#!/usr/bin/env python3
"""Keep ROS camera/LiDAR subscriptions active and report stream liveness.

Receives full serialized messages but retains only header/timing counters. Does
not decode/store images or point arrays, publish controls, or restart drivers.
"""
import argparse
import fcntl
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import struct
import threading
import time


class Stream:
    def __init__(self, topic, expected_type, now):
        self.topic, self.expected_type = topic, expected_type
        self.started = self.checked = now
        self.received = self.previous = 0
        self.last = self.advanced = None
        self.stamp = None
        self.error = None
        self.rate = 0.
        self.lock = threading.Lock()

    def receive(self, raw, message_type, now):
        with self.lock:
            self.received += 1
            self.last = now
            self.error = None
            # Both supported ROS1 message definitions begin with std_msgs/Header:
            # uint32 seq, uint32 sec/nsec, uint32 frame_id byte length, frame_id.
            if message_type != self.expected_type:
                self.error = 'unexpected message type: ' + str(message_type)
                return
            if len(raw) < 16:
                self.error = 'truncated header'
                return
            _, sec, nsec, size = struct.unpack_from('<IIII', raw)
            if nsec >= 1000000000 or len(raw) <= 16+size:
                self.error = 'invalid header or missing payload'
                return
            stamp = sec+nsec*1e-9
            if self.stamp is None or stamp != self.stamp:
                self.advanced = now
            self.stamp = stamp

    def snapshot(self, now, ros_now, timeout, min_hz, grace):
        with self.lock:
            elapsed = now-self.checked
            if elapsed > 0:
                self.rate = (self.received-self.previous)/elapsed
                self.previous, self.checked = self.received, now
            receive_age = None if self.last is None else now-self.last
            stamp_age = None if self.stamp is None else ros_now-self.stamp
            if self.last is None:
                state = 'WAITING' if now-self.started < grace else 'NO_DATA'
            elif self.error:
                state = 'INVALID'
            elif receive_age > timeout:
                state = 'NO_DATA'
            elif self.stamp is None or self.stamp <= 0 or stamp_age < -.1 or stamp_age > timeout:
                state = 'STALE_STAMP'
            elif self.advanced is None or now-self.advanced > timeout:
                state = 'FROZEN_STAMP'
            elif self.rate < min_hz and now-self.started >= grace:
                state = 'LOW_RATE'
            else:
                state = 'OK'
            return dict(topic=self.topic,state=state,hz=round(self.rate,2),
                        messages=self.received,receive_age_s=receive_age,
                        source_age_s=stamp_age,error=self.error)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camera-topic',default='/visbot_media_g/gimbal_camera/image_raw')
    p.add_argument('--lidar-topic',default='/livox/lidar')
    p.add_argument('--timeout',type=float,default=2.,help='maximum silence/source age, seconds')
    p.add_argument('--min-hz',type=float,default=2.)
    p.add_argument('--startup-grace',type=float,default=10.)
    p.add_argument('--log-interval',type=float,default=10.)
    p.add_argument('--duration',type=float,default=0.,help='0: run until Ctrl-C; otherwise bounded check')
    p.add_argument('--output',default=str(Path(__file__).resolve().parents[2]/'logs/sensor_heartbeat'))
    a = p.parse_args(argv)
    for key in ('timeout','min_hz','startup_grace','log_interval','duration'):
        value = getattr(a,key)
        if not math.isfinite(value) or value < 0 or (key in ('timeout','log_interval') and value == 0):
            p.error(key+' must be finite and nonnegative (timeout/log-interval must be positive)')
    return a


def write_status(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    temp.replace(path)


def main(argv=None):
    a = parse_args(argv)
    output = Path(a.output).expanduser().resolve()
    output.mkdir(parents=True,exist_ok=True)
    # A host-wide per-user lock prevents duplicate subscriptions even with a
    # different output directory. Keep the descriptor open for process lifetime.
    lock_path = Path(os.environ.get('XDG_RUNTIME_DIR','/tmp')) / ('owl_sensor_heartbeat_%s.lock' % os.getuid())
    with lock_path.open('a+') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('sensor heartbeat already running for this user')
        log = logging.getLogger('sensor_heartbeat')
        log.setLevel(logging.INFO)
        handlers = [logging.StreamHandler(),RotatingFileHandler(output/'heartbeat.log',maxBytes=1024*1024,backupCount=2)]
        for handler in handlers:
            handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
            log.addHandler(handler)
        import rospy
        rospy.init_node('owl_sensor_heartbeat',anonymous=True)
        started = time.monotonic()
        streams = [Stream(a.camera_topic,'sensor_msgs/Image',started),
                   Stream(a.lidar_topic,'livox_ros_driver2/CustomMsg',started)]
        def callback(msg, stream):
            stream.receive(msg._buff,msg._connection_header.get('type'),time.monotonic())
        # AnyMsg consumes the full TCPROS payload without building 20k Python
        # point objects per scan. Only bounded counters survive each callback.
        subs = [rospy.Subscriber(s.topic,rospy.AnyMsg,callback,callback_args=s,
                                queue_size=1,buff_size=16*1024*1024) for s in streams]
        log.info('Started pid=%s node=%s topics=%s',os.getpid(),rospy.get_name(),[s.topic for s in streams])
        state, next_log, result = None, 0., None
        exit_code = 0
        try:
            while not rospy.is_shutdown():
                now = time.monotonic()
                rows = [s.snapshot(now,rospy.Time.now().to_sec(),a.timeout,a.min_hz,a.startup_grace) for s in streams]
                healthy = all(r['state']=='OK' for r in rows)
                result = dict(running=True,healthy=healthy,pid=os.getpid(),node=rospy.get_name(),
                              checked_at_unix_s=time.time(),streams=rows)
                write_status(output/'status.json',result)
                current = tuple(r['state'] for r in rows)
                if current != state or now >= next_log:
                    message = ' | '.join('%s %s %.1f Hz age=%s' % (r['topic'],r['state'],r['hz'],r['receive_age_s']) for r in rows)
                    (log.info if healthy else log.warning)(message)
                    state, next_log = current, now+a.log_interval
                if a.duration and now-started >= a.duration:
                    exit_code = 0 if healthy else 2
                    break
                time.sleep(1.)  # Wall clock: monitoring continues if ROS time stalls.
        except KeyboardInterrupt:
            pass
        finally:
            for sub in subs:
                sub.unregister()
            if result:
                result.update(running=False,healthy=False,stopped_at_unix_s=time.time())
                write_status(output/'status.json',result)
            rospy.signal_shutdown('sensor heartbeat stopped')
            log.info('Stopped; no drivers restarted')
        return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
