"""Bounded ground-only recovery of failed stack startup, never an airborne watchdog."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def recovery_plan(report):
    errors = report.get('errors', [])
    if not errors:
        raise RuntimeError('No failed startup check was recorded; refusing an unclassified restart')
    keys = {error.split(':', 1)[0] for error in errors}
    allowed = {'camera_baseline', 'rgb', 'odom', 'cloud', 'source_odometry',
               'lio_px4_alignment', 'state', 'component'}
    if keys-allowed:
        raise RuntimeError('Restart cannot fix configuration or ownership checks: '+str(sorted(keys-allowed)))
    components = {e.split(':', 1)[1].strip() for e in errors if e.startswith('component:')}
    known = {'00_roscore.sh', '10_mavros.sh', '20_livox.sh', '30_fastlio.sh',
             '40_camera_relay.sh', 'sensors', 'bridge', 'server'}
    if components-known:
        raise RuntimeError('Unknown failed component: '+str(components-known))
    actions = []
    if keys & {'odom', 'cloud', 'source_odometry', 'lio_px4_alignment'} or components & {'20_livox.sh', '30_fastlio.sh'}:
        actions.append('stop_lio')
    if keys & {'odom', 'cloud', 'source_odometry'} or '20_livox.sh' in components:
        actions.append('stop_livox')
    if 'lio_px4_alignment' in keys:
        actions.append('reboot_px4')
    if 'state' in keys or '10_mavros.sh' in components:
        actions.append('stop_mavros')
    # Owned camera relay, sensors, bridge and server were cleaned up by the
    # child stack. The next attempt creates fresh processes and resets baseline.
    return actions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempts', type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.attempts <= 5:
        parser.error('--attempts must be between 1 and 5')
    folder = Path('logs/i7_startup')/(time.strftime('%Y%m%d_%H%M%S')+'-'+str(os.getpid()))
    folder.mkdir(parents=True)
    stopping = []
    active = [None]
    def stop(number, frame):
        if not stopping:
            stopping.append(number)
        if active[0] is not None and active[0].poll() is None:
            try:
                os.killpg(active[0].pid, number)
            except ProcessLookupError:
                pass
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, stop)
    def run(command, env=None):
        if stopping:
            return 128+stopping[0]
        child = subprocess.Popen(command, env=env, start_new_session=True)
        active[0] = child
        if stopping:
            stop(stopping[0], None)
        try:
            return child.wait()
        finally:
            active[0] = None
    with open('/tmp/tjk-i7-supervisor-'+str(os.getuid())+'.lock', 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('Another i7 startup supervisor is already running.', file=sys.stderr)
            return 1
        print('Startup reports: '+str(folder), flush=True)
        for attempt in range(1, args.attempts+1):
            report = folder/f'attempt_{attempt}.json'
            ready = folder/f'attempt_{attempt}.ready'
            env = dict(os.environ, I7_STARTUP_REPORT=str(report.resolve()), I7_STARTUP_READY=str(ready.resolve()))
            print(f'I7 stack startup attempt {attempt}/{args.attempts}', flush=True)
            status = run(['bash', 'app/robot/i7/bringup/run_i7_hardware.sh', '--stack'], env)
            if stopping:
                return 128+stopping[0]
            # Once deployed, never restart sensors or PX4 automatically.
            if ready.exists() or status in (0, 129, 130, 143, -2, -15) or attempt == args.attempts:
                if status != 0 and not ready.exists() and attempt == args.attempts:
                    print('Startup recovery attempts exhausted; see '+str(folder), file=sys.stderr, flush=True)
                return status
            try:
                actions = recovery_plan(json.loads(report.read_text()))
            except (OSError, ValueError, RuntimeError) as exc:
                print('Startup recovery stopped: '+str(exc), file=sys.stderr, flush=True)
                return 1
            print('Ground recovery before retry: '+str(actions or ['restart stack and reset camera']), flush=True)
            command = ['bash', '-c', 'source /opt/ros/noetic/setup.bash; exec python3 -m app.robot.i7.bringup.recover_startup "$@"',
                       'recover-startup', '--log', str(folder/'recovery.jsonl'), *actions]
            if run(command) != 0:
                return 128+stopping[0] if stopping else 1
        return 1


if __name__ == '__main__':
    sys.exit(main())
