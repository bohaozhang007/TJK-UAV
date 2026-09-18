"""Passive local console logger for all Robot tasks, independent of control lease."""
import csv
import datetime
import json
import math
from pathlib import Path
import threading
import time
import urllib.request

FIELDS = ['started_at','finished_at','action','action_xyz_yaw','before','after','error']

def public(p):
    return [p[0]*100,-p[1]*100,p[2]*100,-math.degrees(p[3])]

def packed(p):
    return '' if p is None else '('+', '.join(f'{0.0 if abs(v)<0.005 else v:.2f}' for v in p)+')'

def row(task):
    before=public(task['before_pose']); after=public(task['after_pose']) if task.get('after_pose') else None
    goal=public(task['goal']); relative=task.get('relative_command')
    action=task['kind']; command=public(relative) if relative is not None else goal
    if relative is not None: action='rotate' if all(v==0 for v in relative[:3]) else 'move_rel_xyz_yaw'
    error=None
    if after is not None and task.get('before_epoch')==task.get('after_epoch') and action!='land':
        error=[a-b for a,b in zip(after,goal)]
        if relative is not None:
            angle=math.radians(before[3]);c,s=math.cos(angle),math.sin(angle)
            error[0],error[1]=c*error[0]+s*error[1],-s*error[0]+c*error[1]
        error[3]=(error[3]+180)%360-180
    stamp=lambda k:datetime.datetime.fromtimestamp(task[k]).isoformat(timespec='milliseconds') if task.get(k) else ''
    return dict(started_at=stamp('started_wall_s'),finished_at=stamp('finished_wall_s'),action=action,
                action_xyz_yaw=packed(command),before=packed(before),after=packed(after),error=packed(error))

class MotionLog:
    def __init__(self, folder, url):
        self.folder=Path(folder);self.url=url;self.since=time.time();self.stop=threading.Event()
        self.tasks={};self.last_error=None
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.write()
        self.thread=threading.Thread(target=self.run,name='robot-motion-log',daemon=True)
        self.thread.start()

    def write(self):
        temp=self.folder/'motions.csv.tmp'
        with temp.open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=FIELDS);w.writeheader()
            for t in sorted(self.tasks.values(),key=lambda t:t['started_wall_s']):w.writerow(row(t))
        temp.replace(self.folder/'motions.csv')

    def poll(self):
        with self.opener.open(self.url+'/v21/motion_log',timeout=2) as response:data=json.load(response)
        changed=False
        for t in data['tasks']:
            if t.get('started_wall_s',0)<self.since:continue
            key=(data.get('bridge_id'),t['task_id'])
            if self.tasks.get(key)!=t:
                self.tasks[key]=t;changed=True
                with (self.folder/'motions.jsonl').open('a') as f:f.write(json.dumps(t)+'\n')
        if changed:self.write()

    def run(self):
        while not self.stop.is_set():
            try:self.poll();self.last_error=None
            except Exception as e:
                if str(e)!=self.last_error:
                    print('运动CSV记录暂不可用：'+str(e),flush=True);self.last_error=str(e)
            self.stop.wait(.5)

    def close(self):
        self.stop.set();self.thread.join(3)
        if not self.thread.is_alive():
            try:self.poll()
            except Exception:pass
