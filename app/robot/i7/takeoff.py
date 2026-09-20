"""Legacy i7 ordering: stream hold, wait for RC OFFBOARD, then arm once."""
import time


def prepare_manual(guard, arm_request, warm, phase, timeout, clock=time.monotonic, sleep=time.sleep):
    deadline = clock()+timeout
    def wait(predicate, until):
        while True:
            state = guard()
            if clock() >= until:
                raise RuntimeError('manual OFFBOARD takeoff preparation timed out')
            if predicate(state):
                return state
            sleep(.05)
    wait(lambda state: warm(), deadline)
    phase('WAIT_OFFBOARD')
    state = wait(lambda state: state['mode'] == 'OFFBOARD', deadline)
    if not state['armed']:
        guard()
        arm_request()
    wait(lambda state: state['mode'] == 'OFFBOARD' and state['armed'], min(deadline, clock()+5.))
    guard()
