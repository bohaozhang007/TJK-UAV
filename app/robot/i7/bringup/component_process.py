"""Own and reap a component's descendants, including detached ROS children."""
import ctypes
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from owned_resources import OwnedResources


def descendants():
    processes = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            processes[int(entry.name)] = (int(fields[1]), fields[19], fields[0])
        except (FileNotFoundError, ProcessLookupError):
            continue
    owned = {os.getpid()}
    while True:
        added = {pid for pid, (parent, _, _) in processes.items() if parent in owned}-owned
        if not added:
            break
        owned.update(added)
    return {pid: processes[pid] for pid in owned if pid != os.getpid()}


def main():
    os.setsid()
    # Adopt grandchildren if a launcher exits before its ROS nodes.
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(36, 1, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'cannot enable child subreaper')
    stopping = []
    def stop(number, frame):
        if not stopping:
            stopping.append(number)
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, stop)
    # Restore signals ignored by an asynchronous shell before executing Python.
    command = [sys.executable, '-c',
        'import os,signal,sys; '
        'signal.signal(signal.SIGINT,signal.SIG_DFL); '
        'signal.signal(signal.SIGTERM,signal.SIG_DFL); '
        'signal.signal(signal.SIGHUP,signal.SIG_DFL); '
        'os.execvp(sys.argv[1],sys.argv[1:])', *sys.argv[1:]]
    child = subprocess.Popen(command, start_new_session=True, close_fds=True)
    resources = OwnedResources(descendants)
    resources.start()
    while child.poll() is None and not stopping:
        time.sleep(.05)
    status = child.poll()
    sent = set()
    started = time.monotonic()
    while True:
        child.poll()
        try:
            while os.waitpid(-1, os.WNOHANG)[0]:
                pass
        except ChildProcessError:
            pass
        owned = descendants()
        if not owned:
            break
        elapsed = time.monotonic()-started
        number = signal.SIGINT if elapsed < 4. else signal.SIGTERM if elapsed < 5. else signal.SIGKILL
        for pid, (_, birth, state) in owned.items():
            key = (pid, birth, number)
            if key in sent or state == 'Z':
                continue
            try:
                fields = Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()
                if fields[19] == birth:
                    os.kill(pid, number)
                sent.add(key)
            except (FileNotFoundError, ProcessLookupError):
                pass
        if elapsed > 7.:
            print('Component cleanup incomplete: '+str(sorted(owned)), file=sys.stderr, flush=True)
            return 1
        time.sleep(.05)
    if not resources.cleanup():
        return 1
    return 128+stopping[0] if stopping else (status or 0)


if __name__ == '__main__':
    sys.exit(main())
