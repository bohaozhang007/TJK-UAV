"""Wait for an owned launcher to finish its normal interrupt cleanup."""
import signal
import subprocess


def run_launcher(command):
    child = subprocess.Popen(command)
    try:
        return child.wait()
    except KeyboardInterrupt:
        # Popen.wait does not kill on interruption, unlike subprocess.call.
        # Let roslaunch reap its nodes before removing its temporary config.
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            status = child.wait()
            return 130 if status in (0, -signal.SIGINT, 130) else status
        finally:
            signal.signal(signal.SIGINT, previous)
