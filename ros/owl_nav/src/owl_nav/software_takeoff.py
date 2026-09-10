"""One-shot software takeoff preparation. No ROS dependencies; caller fences every step."""
import time


def prepare(guard, mode_request, arm_request, warm, clock=time.monotonic, sleep=time.sleep):
    """guard returns current mode/armed, raises when task ownership is invalid.

    No retries of FCU mutations. Acknowledgment alone never authorizes ascent.
    """
    deadline = clock()+10
    def wait(predicate):
        while True:
            state = guard()
            if predicate(state):
                return state
            if clock() >= deadline:
                raise RuntimeError('software takeoff preparation timed out')
            sleep(.05)
    wait(lambda state: warm())
    guard()
    mode_request()
    wait(lambda state: state['mode']=='OFFBOARD')
    guard()
    arm_request()
    wait(lambda state: state['mode']=='OFFBOARD' and state['armed'])
    guard()
