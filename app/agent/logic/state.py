"""Shared mission states and allowed transitions."""
import enum


class State(enum.Enum):
    WAIT_OPERATOR = 'wait_operator'
    PATROL_PLAN = 'patrol_plan'
    PATROL_MOVE = 'patrol_move'
    DETECT = 'detect'
    LOCALIZE = 'localize'
    ORBIT_PLAN = 'orbit_plan'
    ORBIT_MOVE = 'orbit_move'
    PHOTO = 'photo'
    AUTOFOCUS = 'autofocus'
    RETURN_CAPTURE = 'return_capture'
    RETURN_HOME = 'return_home'
    LANDING = 'landing'
    COMPLETE = 'complete'
    ERROR_HOLD = 'error_hold'
    ERROR_LANDING = 'error_landing'
    FAILED = 'failed'
    CONTROL_LOST = 'control_lost'


NEXT = {
    State.WAIT_OPERATOR: {State.PATROL_PLAN},
    State.PATROL_PLAN: {State.PATROL_MOVE, State.RETURN_HOME},
    State.PATROL_MOVE: {State.DETECT},
    State.DETECT: {State.LOCALIZE},
    State.LOCALIZE: {State.PHOTO, State.AUTOFOCUS, State.ORBIT_PLAN, State.PATROL_PLAN, State.RETURN_HOME},
    State.AUTOFOCUS: {State.AUTOFOCUS, State.PATROL_PLAN, State.RETURN_HOME},
    State.ORBIT_PLAN: {State.ORBIT_MOVE, State.RETURN_CAPTURE},
    State.ORBIT_MOVE: {State.PHOTO, State.ORBIT_PLAN, State.RETURN_CAPTURE},
    State.PHOTO: {State.ORBIT_MOVE, State.ORBIT_PLAN, State.RETURN_CAPTURE, State.PATROL_PLAN, State.RETURN_HOME},
    State.RETURN_CAPTURE: {State.ORBIT_PLAN, State.PATROL_PLAN, State.RETURN_HOME},
    State.RETURN_HOME: {State.LANDING},
    State.LANDING: {State.COMPLETE},
    State.ERROR_LANDING: {State.FAILED},
}

