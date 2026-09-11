#!/usr/bin/env python3
"""Session-owning HTTP console for owl_ego. Startup never submits flight commands."""
import argparse
import json
from pathlib import Path
import threading
import time
import uuid
from live_sequence import Runner, Failure, parse_args


class ConsoleRunner(Runner):
    def __init__(self,args):
        super().__init__(args)
        self.abort = threading.Event()

    def rpc(self,method,path,data=None,timeout=2):
        if self.abort.is_set() and method=='POST' and path in (
                '/init','/takeoff','/v21/navigation','/move_relative_xyz_yaw'):
            raise Failure('Console operation cancelled')
        return super().rpc(method,path,data,timeout)

    def health(self,flight=True,require_localization=True):
        if self.abort.is_set():
            raise Failure('Console operation cancelled')
        return super().health(flight,require_localization=require_localization)


class Console:
    def __init__(self,args):
        self.r=ConsoleRunner(args)
        self.worker=None
        self.failed=False
        self.operator_token=None
        self.operator_land_request=None
        self.operator_stop_request=None

    def initialize(self):
        r=self.r
        c=r.rpc('GET','/v21/capabilities')
        if c.get('backend')!='owl_ego' or c.get('software_takeoff') is not True:
            raise Failure('Robot 缺少 software_takeoff 能力，请更新并重启 bridge/server')
        r.abort=threading.Event()
        h=r.health(flight=False)
        deadline=time.monotonic()+3
        while (not h.get('stopped') and not h.get('active_task_id') and not h.get('airborne')
               and h.get('landed_state')==1 and h.get('landed_state_fresh') and time.monotonic()<deadline):
            time.sleep(.1)
            h=r.health(flight=False)
        if not h.get('stopped') or h.get('active_task_id'):
            raise Failure('需要静止且无活动任务')
        if r.sid:
            if r.delegated or h.get('control_owner') == 'agent' and self.operator_token:
                raise Failure('Agent正在控制；本窗口可直接stop悬停或land降落抢占，无需重复init。')
            if h.get('initialized'):
                print('已经初始化，当前会话继续保持。',flush=True)
                return
            if h.get('airborne') or h.get('landing') or h.get('landed_state')!=1 or not h.get('landed_state_fresh'):
                raise Failure('需要确认落地后才能重新初始化')
            r.abort.set()
            if not r.cleanup():
                raise Failure('旧会话释放未确认，不能重新初始化')
            r.sid=None
            r.abort=threading.Event()
        r.epoch=h['localization_epoch']
        r.tolerances=r.rpc('GET','/motion_tolerances')['motion_tolerances']
        session=r.rpc('POST','/v21/session',dict(request_id=str(uuid.uuid4()),
            **({'operator':True} if c.get('operator_override') else {})))
        r.sid=session['session_id']
        self.operator_token=session.get('operator_token')
        self.operator_land_request=None
        self.operator_stop_request=None
        r.delegated=False
        r.stop.clear();r.hb_error=None
        r.hb_thread=threading.Thread(target=r.heartbeat,daemon=True);r.hb_thread.start()
        try:r.post('/init')
        except Exception:
            r.cleanup();r.sid=None
            raise
        print('初始化完成，心跳持续。输入 takeoff 请求 OFFBOARD、解锁和起飞。',flush=True)
        if self.operator_token:
            print('起飞停稳后Agent可直接接入；保持本窗口，stop悬停、land降落均可抢占Agent运动。',flush=True)

    def launch(self,name,operation):
        if self.worker and self.worker.is_alive():
            raise Failure('任务运行中；使用 stop 或 land 抢占')
        if not self.r.sid:
            raise Failure('请先 init')
        if self.r.delegated:
            raise Failure('运控已交给Agent；本窗口可用stop或land抢占。')
        self.r.abort=threading.Event()
        self.r.phase=name
        def work():
            try:
                result=operation()
                self.r.record('console_completed',command=name,result=result)
                print('\n完成 '+name,flush=True)
            except Exception as e:
                self.failed=True
                self.r.abort.set()
                self.r.record('console_failed',command=name,error=str(e))
                print('\n失败 '+name+': '+str(e)+'；未继续后续动作，可用 stop 或 land。',flush=True)
                # No automatic reacquisition, arming or landing after failure.
                try:
                    h=self.r.rpc('GET','/health')['health']
                    if h.get('active_task_id') and not h.get('manual_takeover') and not h.get('landing'):
                        self.r.post('/v21/navigation/cancel',task_id=h['active_task_id'])
                except Exception as cleanup_error:
                    self.r.record('cancel_unconfirmed',error=str(cleanup_error))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def interrupt(self):
        self.r.abort.set()
        if self.worker:
            self.worker.join(3)
            if self.worker.is_alive():
                raise Failure('客户端任务尚未退出，请遥控接管')

    def operator_land(self):
        r=self.r
        self.interrupt()
        # Stop the old operator heartbeat before installing the landing session.
        r.stop.set()
        if r.hb_thread:
            r.hb_thread.join(2.5)
            if r.hb_thread.is_alive():
                raise Failure('旧心跳尚未退出，未提交降落抢占')
        r.abort=threading.Event()
        r.phase='land'
        if self.operator_land_request is None:
            self.operator_land_request=dict(operator_token=self.operator_token,request_id=str(uuid.uuid4()))
        result=r.rpc('POST','/v21/operator/land',self.operator_land_request)
        r.sid=result['session_id'];r.delegated=False
        r.stop.clear();r.hb_error=None
        r.hb_thread=threading.Thread(target=r.heartbeat,daemon=True);r.hb_thread.start()
        def wait_landing():
            deadline=time.monotonic()+95
            while time.monotonic()<deadline:
                r.health(flight=False,require_localization=False)
                task=r.task(result['task_id'])
                if task['status'] in ('arrived','failed','cancelled'):
                    if task['status']=='arrived' and task.get('stopped'):
                        r.epoch=None
                        self.operator_land_request=None
                        return task
                    self.operator_land_request=None
                    raise Failure('降落未完成：'+str(task))
                time.sleep(.1)
            raise Failure('降落确认超时；未自动取消AUTO.LAND')
        self.launch('land',wait_landing)

    def operator_stop(self):
        r=self.r
        self.interrupt()
        r.stop.set()
        if r.hb_thread:
            r.hb_thread.join(2.5)
            if r.hb_thread.is_alive():
                raise Failure('旧心跳尚未退出，未提交悬停抢占')
        if self.operator_stop_request is None:
            self.operator_stop_request=dict(operator_token=self.operator_token,request_id=str(uuid.uuid4()))
        result=r.rpc('POST','/v21/operator/stop',self.operator_stop_request)
        r.sid=result['session_id'];r.delegated=False
        r.stop.clear();r.hb_error=None
        r.hb_thread=threading.Thread(target=r.heartbeat,daemon=True);r.hb_thread.start()
        def wait_stopped():
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                h=r.health()
                if h.get('stopped') and not h.get('active_task_id'):
                    self.operator_stop_request=None
                    return h
                time.sleep(.1)
            raise Failure('悬停已请求，但8秒内未确认实测停稳；心跳继续，可用land')
        self.launch('stop',wait_stopped)

    def move_relative(self, values):
        r=self.r
        r.wait_motion_ready()
        start=r.pose()
        samples=[start]
        next_sample=next_report=0.
        completed=False
        r.record('relative_motion_start',relative=values,start=start)
        print('相对运动（cm/°）：%s；起始实测Z %.2f cm；z=0保持既有高度参考。'%(values,start['z']),flush=True)

        def observe(pose):
            samples.append(pose)
            r.record('relative_motion_sample',pose=pose,delta_z_cm=pose['z']-start['z'])

        def sample():
            nonlocal next_sample,next_report
            now=time.monotonic()
            if now<next_sample:return
            pose=r.pose();observe(pose)
            next_sample=now+.2
            if now>=next_report:
                print('实测Z %.2f cm，较起点 %+.2f cm，采样最高 %+.2f cm。'%(
                    pose['z'],pose['z']-start['z'],max(p['z'] for p in samples)-start['z']),flush=True)
                next_report=now+2.
        try:
            value=r.blocking('/move_relative_xyz_yaw',18,on_poll=sample,
                **dict(zip(('x','y','z','yaw'),values)),timeout_s=15)
            task=r.wait_task(value['task_id'])
            end=r.pose();observe(end)
            completed=True
            return dict(task=task,start=start,end=end)
        finally:
            last=samples[-1]
            summary=dict(completed=completed,relative=values,start=start,last_sample=last,
                sample_count=len(samples),delta_z_cm=last['z']-start['z'],
                max_rise_cm=max(p['z'] for p in samples)-start['z'],
                min_delta_z_cm=min(p['z'] for p in samples)-start['z'])
            r.record('relative_motion_summary',**summary)
            print('高度采样统计：起始 %.2f cm → 最后 %.2f cm（%+.2f cm），采样最高抬升 %.2f cm。'%(
                start['z'],last['z'],summary['delta_z_cm'],summary['max_rise_cm']),flush=True)

    def command(self,line):
        cmd=line.strip().lower()
        if not cmd:return True
        self.r.record('console_command',command=cmd)
        if cmd=='help':
            print('init | takeoff | move_rel_xyz_yaw X Y Z YAW | move_rel_xyz X Y Z | test | land | stop | wait | health | pose | quit',flush=True)
            print('相对运动参数为整数：XYZ厘米、yaw度；正方向为前/右/上/顺时针。z=0保持既有高度参考。',flush=True)
        elif cmd in ('health','pose'):
            print(json.dumps(self.r.rpc('GET','/health' if cmd=='health' else '/get_pose'),ensure_ascii=False,indent=2),flush=True)
        elif cmd=='init':
            if self.worker and self.worker.is_alive():raise Failure('任务运行中')
            self.initialize()
        elif cmd=='takeoff':
            self.launch(cmd,lambda:self.r.blocking('/takeoff',65,flight=False,auto_arm=True))
        elif cmd=='test':
            self.launch(cmd,self.r.sequence)
        elif cmd.split()[0] in ('move_rel_xyz_yaw','move_relative_xyz_yaw','move_rel_xyz','move_relative_xyz'):
            parts=cmd.split()
            count=4 if parts[0].endswith('_yaw') else 3
            usage='用法：move_rel_xyz_yaw X_CM Y_CM Z_CM YAW_DEG（整数）；或 move_rel_xyz X_CM Y_CM Z_CM'
            if len(parts)!=count+1:raise Failure(usage)
            try:values=[int(v) for v in parts[1:]]
            except ValueError:raise Failure(usage) from None
            if count==3:values.append(0)
            self.launch(cmd,lambda:self.move_relative(values))
        elif cmd=='land':
            if self.operator_token:
                self.operator_land()
            else:
                self.interrupt()
                self.launch(cmd,lambda:self.r.blocking('/land',95,flight=False))
        elif cmd=='stop' and self.operator_token:
            self.operator_stop()
        elif cmd=='wait':
            if self.worker:self.worker.join()
        elif cmd in ('stop','quit','exit'):
            if self.operator_token and (self.r.delegated or self.r.rpc('GET','/health')['health'].get('control_owner')=='agent'):
                if cmd=='stop':
                    raise Failure('Agent正在控制；如需结束飞行请用land抢占。')
                self.r.stop.set()
                if self.r.hb_thread:self.r.hb_thread.join(2.5)
                self.r.sid=None
                print('退出本窗口不停止Agent；需要本窗口降落抢占时请保持打开。',flush=True)
                return False
            self.interrupt()
            confirmed=self.r.cleanup()
            self.r.sid=None
            if not confirmed:self.failed=True
            print('会话已结束；空中退出不会自动降落。再次操作需 init。',flush=True)
            return cmd=='stop'
        else:raise Failure('未知命令，输入 help')
        return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:8765')
    parser.add_argument('--output',default=None)
    parser.add_argument('--commands',help='Read explicit console commands from a file (including wait where required)')
    args=parser.parse_args()
    argv=['--execute','--url',args.url]
    if args.output:argv+=['--output',args.output]
    flight_args=parse_args(argv)
    console=Console(flight_args)
    from motion_log import MotionLog
    motion_log=MotionLog(Path(console.r.log.name).parent,args.url.rstrip('/'))
    console.command('help')
    print('位姿/动作日志：'+str(console.r.log.name),flush=True)
    print('运动表（含Agent动作）：'+str(Path(console.r.log.name).with_name('motions.csv')),flush=True)
    print('启动仅连接客户端，不控制飞机。test: B=前方%gm，P=移动%gm后的实测位置；记录P后约%gs取消。'%(
        flight_args.b_forward_cm/100,flight_args.capture_after_cm/100,flight_args.delay_s),flush=True)
    print('返回P → P左侧1m → P右侧1m（横移约2m）→ 顺时针转90° → 返回P并恢复原航向 → 前往B。',flush=True)
    try:
        if args.commands:
            for line in Path(args.commands).read_text().splitlines():
                if not console.command(line):break
        else:
            # Importing readline enables input() editing and in-session history.
            # Keep command-file execution independent of terminal support.
            try:
                import readline
            except ImportError:
                print('当前Python缺少readline，方向键编辑和历史命令不可用。',flush=True)
            while True:
                try:line=input('owl_ego> ')
                except (EOFError,KeyboardInterrupt):break
                try:
                    if not console.command(line):break
                except KeyboardInterrupt:
                    console.command('stop')
                except Exception as e:print(str(e),flush=True)
    finally:
        if console.r.sid:
            if console.operator_token and console.r.delegated:
                console.r.stop.set()
            else:
                console.interrupt();console.r.cleanup()
            console.r.sid=None
        motion_log.close()
        console.r.record('console_exit',had_errors=console.failed)
    return 1 if console.failed else 0


if __name__=='__main__':
    raise SystemExit(main())
