"""PX4-owned ascent followed by a measured, guarded OFFBOARD handoff."""
import time


def prepare_px4(guard, mode_request, arm_request, warm, phase, *, timeout=55.,
                clock=time.monotonic, sleep=time.sleep):
    deadline = clock()+timeout

    def wait(predicate):
        while True:
            state = guard()
            if predicate(state):
                return state
            if clock() >= deadline:
                raise RuntimeError('PX4 takeoff/handoff timed out')
            sleep(.05)

    state = guard()
    if state['mode'] != 'POSCTL' or state['armed']:
        raise RuntimeError('PX4 takeoff requires disarmed POSITION')
    mode_request('AUTO.TAKEOFF')
    wait(lambda s: s['mode'] == 'AUTO.TAKEOFF' and not s['armed'])
    phase('PX4_TAKEOFF')
    guard()
    arm_request()
    wait(lambda s: s['armed'] and s['mode'] in ('AUTO.TAKEOFF', 'AUTO.LOITER'))
    wait(lambda s: s['mode'] == 'AUTO.LOITER' and s['armed'] and s['airborne']
         and s['stopped'] and s['height_ok'] and warm())
    phase('HANDOFF')
    guard()
    mode_request('OFFBOARD')
    wait(lambda s: s['mode'] == 'OFFBOARD' and s['armed'] and s['airborne'])
    guard()
