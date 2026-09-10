# OWL EGO / Agent v21 HTTP contract — protocol version 1

This document and `src/robot_client/owl_ego.py` are the shared source of truth.
The client is implemented on the Windows Agent computer. The Robot backend is
implemented in this checkout and runs on the OWL computer and must not route navigation through Captain. Keep existing `owl`
and all v20 behavior available as a separate backend.

## Coordinates and timestamps

- Public poses everywhere (including navigation goals and capture poses):
  `{x,y,z,yaw}` in centimetres/degrees, with the same fixed origin as MAVROS
  local odometry. Public `x=ROS x*100`, `y=-ROS y*100`, `z=ROS z*100`,
  `yaw=-ROS yaw radians*180/pi`, normalized to [-180,180).
- Relative movement: x forward, y right, z up, positive yaw clockwise,
  relative to the body yaw **at command acceptance**. Use the same convention
  as v20. MAVROS ENU/NED conversion is Robot-owned; never convert twice.
  For owl_ego, z=0 preserves the existing hold-altitude reference; a nonzero z
  remains an offset from measured altitude at acceptance (see altitude fix below).
- Agent route coordinates are relative to its first connected hover pose;
  Agent converts them to the public fixed world before sending navigation.
- `world_frame` is the odometry ENU frame name. `localization_epoch` is an opaque
  nonempty string which changes on localization reset, origin change, odometry
  discontinuity or Robot restart. Never silently rebase an active mission.
- Camera optical axes: x right, y down, z forward. The observation transform
  below maps into **right-handed ROS ENU**, not the reflected public frame.
  Its translation is in cm. Agent reflects Y only after transforming points.
- Timestamp and age are measured on Robot, using image acquisition time;
  no assumption of synchronized Windows/Robot wall clocks. Epoch timestamps
  in seconds are acceptable; age must use the matching Robot time domain.

## Common response and ownership rules

JSON, HTTP port 8765 by default. Success always has `ok:true`; failures have
`ok:false,error:string` and an appropriate non-2xx HTTP status. Do not return
HTTP success when a command was rejected or failed. All v21 POST endpoints
must respond within 3 seconds; they accept work rather than waiting for flight.
Legacy blocking takeoff/land/relative-motion exceptions are listed below.

Only one control session and one active flight task at a time. Every mutation
below requires its session token, except creating the session itself. The
session token is for ownership, not internet-facing authentication.
`request_id` is a client-generated UUID. For the duration of a session, repeat
requests with the same ID and identical body must return the original result,
never repeat a flight action; mismatched bodies with the same ID must fail.
Reject foreign-session mutations and concurrent task submissions. Unknown task
IDs must fail, not query/modify the current task. Keep terminal task records for
the entire session. Do not use HTTP connection lifetime to control a flight.

## Read-only preflight

`GET /v21/capabilities`:

```json
{"ok":true,"backend":"owl_ego","protocol_version":1,
 "async_navigation":true,"cancel_and_hold":true,
 "synchronized_observation":true,"control_lease":true,"relative_xyz_yaw":true}
```

Capabilities and observations must be available before init/takeoff/session.
Start passive sensor subscriptions at server startup. Missing/stale sensor data
returns an error; do not invent calibration, identity transforms, or poses.

`GET /v21/observation`:

```json
{"ok":true,"frame_id":"camera-12345","timestamp_s":1234.5,
 "age_s":0.02,"sync_error_s":0.008,
 "localization_epoch":"odom-epoch-abc","world_frame":"odom",
 "pose":{"x":0.0,"y":0.0,"z":100.0,"yaw":0.0},
 "image_size":[640,360],"rectified":true,
 "rgb_jpeg_base64":"<base64 encoded JPEG>",
 "intrinsics":[[500,0,320],[0,500,180],[0,0,1]],
 "world_from_camera_optical_cm":[[0,0,1,0],[-1,0,0,0],[0,-1,0,100],[0,0,0,1]]}
```

The numbers above are illustrative, not calibration values. Requirements:

- Unique `frame_id` for each actual exposure; rereading cached RGB retains its ID.
- `pose` is the body pose **at exposure**, obtained from stamped odometry history.
- `sync_error_s` bounds temporal misalignment of odometry/TF with exposure.
- RGB must be rectified. K must correspond exactly to returned RGB dimensions
  after rectification, crop and resize; depth estimates are optical Z.
- Compose full odometry attitude, calibrated body-to-camera extrinsic, and
  time-varying gimbal orientation when applicable to obtain the rigid transform.
  CameraInfo alone does not provide body-camera extrinsics. Use TF/calibration.
- Include camera translation/lever arm and full roll/pitch/yaw, not yaw only.
- Default Agent rejects age including network transit >0.5 s or sync error >0.05 s.
- Atomically snapshot image plus metadata; never attach a later current pose.
- JPEG should remain modest (default long edge 640). No shared-filesystem paths.
- Keep returned RGB dimensions fixed throughout the control session.

`GET /health`: compatible envelope, with required fields:

```json
{"ok":true,"health":{"initialized":true,"airborne":true,
 "control_ready":true,"odom_ok":true,"rgb_ok":true,"planner_ok":true,
 "localization_epoch":"odom-epoch-abc"}}
```

Before init/takeoff the appropriate fields are false. `planner_ok` requires a
fresh planner heartbeat, not merely an existing topic. Health must reflect
manual takeover, odometry loss, localization reset, control failure and lease
expiry. Do not populate irrelevant Captain link flags as false.

`GET /get_pose`: `{ok:true,pose:{x,y,z,yaw},localization_epoch:"..."}`.

`GET /motion_tolerances`: existing envelope
`{ok:true,motion_tolerances:{position_tolerance_cm:15,yaw_tolerance_deg:5,
position_error_metric:"euclidean_3d",source:"owl_ego"}}`.
Actual tolerances are Robot configuration; example numbers match current OWL.

## Control lease

- `POST /v21/session` body `{request_id}` returns `{ok:true,session_id:"..."}`.
  Acquire exclusive control authority, but do not arm/take off merely on acquire.
  Reject acquisition if another live owner exists.
- `POST /v21/heartbeat` body `{session_id}` returns `{ok:true}`.
  Agent sends every 0.5 s, with a 2 s HTTP timeout. Robot lease timeout: **5 s**.
- On expiry, invalidate all active/queued tasks and stop/hold on Robot without
  relying on Agent. Preserve pilot takeover and PX4 failsafe authority. If hold
  capability is unavailable, use the configured validated failsafe behavior.
- `POST /v21/session/release` body `{session_id}` returns `{ok:true}`. Begin
  stop/hold if airborne; release must never disarm in flight or resume old work.
  No delayed packet may revive an expired/released session.
- The lease also governs blocking relative movement and takeoff; heartbeat
  handling must never wait behind a flight-duration lock.

## Async absolute navigation

`POST /v21/navigation`:

```json
{"session_id":"...","request_id":"uuid","localization_epoch":"odom-epoch-abc",
 "pose":{"x":0,"y":100,"z":100,"yaw":0}}
```

Returns `{ok:true,task_id:"nav-123"}` after accepting the task. Reject epoch
mismatch, invalid/nonfinite/out-of-range coordinates, missing flight readiness,
or another active task. EGO plans XYZ. The execution bridge enforces requested
yaw with bounded rate, including backward translation and pure rotation.

`GET /v21/navigation/status?task_id=nav-123`:

```json
{"ok":true,"task_id":"nav-123","status":"executing","stopped":false}
```

Statuses: `accepted`, `planning`, `executing`, `stopping`, `arrived`, `cancelled`,
`failed`. Include `error` for failure and preferably pose/error/timing for logs.
`arrived` requires XYZ Euclidean error and yaw error within Robot tolerances,
plus the fresh pose-window stability confirmation specified below; return `stopped:true`.
Do not equate planner trajectory completion with actual arrival.

`POST /v21/navigation/cancel` body `{session_id,request_id,task_id}` returns
`{ok:true,task_id}` immediately after accepting cancellation. State progresses
to `stopping` then `cancelled,stopped:true` only after confirmed stop/hold.
If the task already arrived, preserve `arrived,stopped:true` (arrival/cancel race).
Repeated cancellation of the same terminal task is harmless and must not affect
any newer task. `failed` remains failed; never hide an execution failure.

Cancellation must invalidate old trajectory ownership and queued planner output.
Fence stale output by trajectory/task generation and timing/acknowledgment;
"received after new goal" alone is insufficient because old trajectories can
still publish after the new goal is sent. Resume is a **new** navigation request,
not replaying or unpausing the old timed trajectory. Define a tested stop/resume
mechanism; do not assume upstream mandatory_stop is a reversible pause.

## Existing commands required by reused v20 TRACK

Session and request ID are appended to these POST bodies by the local Client:

- `POST /init` initializes navigation readiness; no automatic takeoff.
- `POST /takeoff` returns only after airborne/ready or failure. Already airborne
  should not take off again. May block within the 180 s Agent HTTP timeout.
- `POST /move_relative_xyz_yaw` body
  `{x,y,z,yaw,timeout_s?,session_id,request_id}`; integer cm/deg. Blocking until
  reached/stable or explicit failure. Combined axes in one task, no Captain.
  z=0 preserves the established height reference, including during yaw-only moves;
  it does not rebase that reference on the current measured hover error.
  v21 passes its TRACK timeout (currently 15 s). Must remain cancellable by
  landing, lease expiry, pilot takeover or onboard safety; no global long lock.
- `POST /land` preempts any active work, clears trajectory authority and returns
  after landing confirmation, or explicit failure. Preserve manual control and
  failsafe semantics. Agent may call it after another operation failed.

For legacy responses preserve `{ok:true,message:...}` and optional diagnostics.
Agent uses `/v21/observation` for all RGB and local DA3 for depth; it does not
require Robot-side DA3/SAM3/SAM2 or `/get_depth_np` on this backend.

## Acceptance checks on Robot

Use mocked ROS/SITL before controlled hardware verification. Verify: async
acceptance during long motion; concurrent telemetry/heartbeat/cancel; actual
stop confirmation; stale output exclusion after cancel and resume; arrival vs
cancel races; duplicated requests; unknown task/session rejection; lease loss;
manual takeover; low-speed/yaw/XYZ convergence; pure yaw/backward motion;
image/pose/TF alignment and resized K; odometry reset invalidation; takeoff and
landing preemption. Never let Captain/another bridge simultaneously publish
competing MAVROS setpoints. Keep camera/VIO/MAVROS launch dependencies alive.


## 2026-09-09 user-authorized approximate camera profile

Status: Robot implemented; Agent opt-in handling pending. See
[`collab/messages/robot-001.md`](collab/messages/robot-001.md). Protocol version
remains 1; existing flight endpoints, ownership, timing and coordinate fields do
not change. This optional profile is an explicit exception to the calibrated
observation requirements above, not a claim that approximate data is calibrated.

The user accepts a camera center coincident with the body position and confirms
a fixed gimbal angle. By the latest user instruction, Robot models body-relative
pitch as 0 degrees (horizontal forward), superseding the earlier +20-degree
model. This is a geometry approximation, not a physical gimbal command. The optical-to-
body axes conversion is still applied; the transform is not an identity rotation.
Body attitude remains the full exposure-time interpolated attitude.

In `hardware.intrinsics_mode: approximate_fov`, K is computed as:

```
fx = fy = width / (2 * tan(horizontal_fov / 2))
cx = width / 2
cy = height / 2
```

Square pixels and a centered principal point are assumptions. Robot's initial
`assumed_horizontal_fov_deg: 90.0` is an explicit trial value, NOT a measured
OWL camera specification. K scales with the returned image dimensions. For
1280x720 source -> 640x360 output, this setting gives fx=fy=320, cx=320, cy=180.
Width and height alone cannot determine focal length. No D is invented and no
real lens correction is claimed: raw resized RGB has `rectified:false`.

Observation adds these metadata fields (existing calibrated fields remain):

```json
{"rectified":false,"calibration_quality":"approximate",
 "geometry_assumptions":{
   "intrinsics":"approximate_fov","assumed_horizontal_fov_deg":90.0,
   "principal_point":"image_center","square_pixels_assumed":true,
   "distortion":"unknown_not_corrected","extrinsics":"body_coincident_fixed",
   "camera_translation":"body_coincident_assumption"}}
```

Calibrated CameraInfo + TF returns `rectified:true`,
`calibration_quality:"calibrated"`. Calibrated CameraInfo + approximate extrinsics
returns `rectified:true`, `calibration_quality:"approximate"`; rectification and
extrinsic quality are independent. All modes still require fresh image/odometry,
valid rotation, finite positive focal length, matching epoch and stable output size.

Agent MUST retain its strict default. To use the user-authorized approximation,
Agent needs an explicit opt-in (suggested config `allow_approximate_geometry`),
validation of this profile/metadata, and preservation of assumptions in mission
logs. It may then accept `rectified:false` for this profile only. Do not globally
remove rectification/synchronization validation or relabel raw RGB as rectified.
The existing Agent client currently rejects this profile before session/takeoff;
this is expected until the Agent-side adaptation is implemented and verified.
Target world positions and the 100 cm deduplication rule are approximate under
this profile; onboard FAST-LIO collision avoidance is unchanged.

## 2026-09-09 Robot round 2: planning transitions and observation availability

Status: Robot implementation and mock-FCU validation are recorded in
[`collab/messages/robot-002.md`](collab/messages/robot-002.md). Agent adaptation
is pending; protocol_version remains 1. These additive fields do not change
coordinates, session ownership, the 5 s lease, or the relative-motion 15 s
budget. Older clients that require planner_ok in every phase must be adapted
before using this Robot version for a mission.

### Health and task semantics

Previously planner heartbeat freshness also gated Node authorization during
process replacement, conflicting with the Core's 10 s planning startup window.
Now control authority and planner readiness are separate:

- `hold_ready`: fresh telemetry/extended state and validated cloud/config/control
  publisher audit, bridge enabled, armed OFFBOARD, no manual takeover or landing.
  This can remain true while stopping after lease loss; it does not grant a new
  session or authorize a new navigation task.
- `control_ready`: the existing initialized/airborne/armed/OFFBOARD/session
  conditions plus the same external authorization checks. It no longer requires
  a planner heartbeat. A true value alone does not mean an active task succeeded;
  always inspect task status and health error/manual_takeover/epoch.
- `planner_state`: `starting`, `ready`, `lost`, or `not_required`.
  `starting` means an active XYZ task is planning, lacks a fresh heartbeat from
  its own generation, and remains within the 10 s startup budget. `ready` means
  that generation has a heartbeat no older than 1 s; it does not prove a first
  trajectory or a ready obstacle map. `lost` means a required planner is outside
  those conditions. `not_required` covers hold, stopping, terminal tasks, takeoff
  and pure yaw. An idle process heartbeat cannot make a new generation ready.
- `planner_ok` is true only for `planner_state:ready`. It is deliberately false
  in `starting` and `not_required`; never make it unconditionally true.
- `active_task_id` is the current task ID or null. Blocking TRACK can be located
  through this field and cancelled with the existing cancel endpoint.
- `landed_state` is the MAVROS enum (0 UNKNOWN, 1 ON_GROUND, 2 IN_AIR,
  3 TAKEOFF, 4 LANDING); `landed_state_fresh` indicates receipt freshness.
  Landing succeeds only with fresh explicit ON_GROUND and fresh connected,
  disarmed State. UNKNOWN/LANDING/expired messages are not landing confirmation.

During bounded startup, the bridge continues hold setpoints. If no trajectory
arrives within 10 s, the task fails and holds. Once executing, heartbeat loss
exceeding 1 s fails/holds immediately, even during the first 10 s of the task.
Telemetry/control loss still revokes output as appropriate; this change does
not bypass the watchdog or pilot takeover. A failed task stays failed; a safe
hold is not automatic mission recovery.

Agent must check control/hold authority, odometry, epoch, manual takeover and
errors in all airborne phases; allow `starting` only for the matching planning
task, `ready` for a planner-backed task, and `not_required` for hold/stopping or
bridge-controlled pure yaw/takeoff. Do not require planner_ok for those latter
phases. Poll terminal status and wait for stopped:true before submitting new
motion. The existing Agent does not yet implement these phase-aware checks.

Task status responses may additionally include:

- `generation`: immutable planner ownership ID, retained in the task record
  after cancellation/arrival for diagnostics, not an instruction to resume it.
- `timing_s`: monotonic durations from task acceptance: process_setup_started,
  process_spawned, first_odom_forwarded, fsm_initialized, first_map_output,
  first_heartbeat, goal_sent, first_trajectory and terminal,
  when observed. Fields are absent until observed or if not applicable.
- `diagnostics`: measured position_error_cm, yaw_error_deg, speed_m_s,
  yaw_rate_deg_s and elapsed_s, from odometry. Terminal records retain the last
  sample. These do not replace status/stopped checks.

For the pinned upstream EGO, Robot connects to its DataDisp output before
forwarding odometry through a private per-process topic. The first DataDisp
before any goal acknowledges INIT → WAIT_TARGET; Robot then waits for a
inflated-map output before sending the goal. This avoids an upstream
initialization race where an early goal blocks waiting for an unset trigger.
`first_map_output` measures the first observed (possibly empty, heading-filtered) inflated-map output after FSM
initialization, not exact map computation CPU time, cloud completeness or
physical obstacle-map correctness. No upstream source changes are required.

### Temporarily unavailable observations

Robot keeps a bounded RGB cache and selects the newest fresh exposure supported
by bracketing odometry, or waits up to 80 ms for sensor callbacks while releasing
the cache lock. Repeated reads retain that exposure's frame_id. The 50 ms sync
bound and acquisition-age limit are unchanged, including an age recheck after
JPEG assembly. No current pose substitution is permitted.

When no fresh synchronized exposure is available, HTTP 503 returns:

```json
{"ok":false,"error":"no fresh exposure bracketed by odometry within 80 ms wait",
 "error_code":"observation_unavailable","retryable":true}
```

A localization epoch change during snapshot wait or assembly returns HTTP 409,
`error_code:localization_epoch_changed`, `retryable:false`; invalidate the
mission's old world coordinates rather than retrying them in a new frame.
Invalid geometry/calibration returns HTTP 422, `error_code:invalid_observation`,
`retryable:false`; it is not temporary absence.
`/health` keeps `rgb_ok:false` on observation failure and adds
`observation_error_code` and `observation_retryable` to distinguish the reason.

Agent should retry only explicit observation_unavailable/retryable:true with a
bounded overall time budget (initial recommendation 0.5 s, 50–100 ms spacing),
while heartbeats and safety checks remain independent. Persistent absence ends
that attempt and invokes existing failure handling. Do not retry epoch changes,
invalid geometry or arbitrary HTTP errors as if they were missing frames.
The explicit approximate-geometry opt-in and both rectified/calibration_quality
checks from the previous section remain required and are still Agent-owned.

## 2026-09-10 Robot console software takeoff (optional extension)

Robot advertises `software_takeoff:true` in `/v21/capabilities`. An explicit
`POST /takeoff` with the existing session/request IDs and `auto_arm:true` requests
software OFFBOARD selection and ordinary MAVROS arming. The field must be boolean;
absent/false preserves the existing pilot-controlled startup. Agent is unchanged
and need not opt in for existing calls. Both bridge and server must be updated.

On the ground, require fresh disarmed ON_GROUND state and existing flight gates.
Stream the current hold for at least 1.2 s, request OFFBOARD once, observe State
OFFBOARD, request `/mavros/cmd/arming` once with `value:true`, observe armed State,
and recheck task/session/epoch/authority before allowing ascent. No forced arm,
parameter edits, automatic retry or automatic reacquisition. Software preparation
has a 10 s budget; service calls have a 1.5 s response bound. Cancellation, lease
loss, epoch reset and manual takeover invalidate preparation. An already sent FCU
request cannot be recalled: transport failure/timeout latches uncertain outcome
and blocks further software takeoff on that bridge; late acknowledgments cannot
resume the old ascent. Operator recovery is required, with RC for emergency.
These software checks do not certify physical FCU failsafe behavior.

`run_owl_ego_console.sh` is a separate HTTP client. Startup submits no control.
`init` acquires one session and starts heartbeats; `takeoff` explicitly opts into
software startup; `test` runs B/cancel/P/TRACK/P/B in that same session; `land`
preempts local work and requests AUTO.LAND. `stop` cancels, attempts measured stop
confirmation and releases the session; `quit` does the same and exits. Neither
means motor emergency stop or automatic landing. Failed/unconfirmed cleanup is
reported. After session release, use `init` again before new operations.

## 2026-09-10 OWL vendor MAVROS frame correction

The installed vendor MAVROS differs from upstream: local_position.cpp rotates
map positions/orientation by -90 degrees around Z into `world`, and publishes
world linear velocity despite `child_frame_id:base_link`. Its raw setpoint input
expects map vectors. Previously the Robot bridge treated these as identical ENU
frames; this caused a physical clockwise 90-degree turn on takeoff and also
misdirected horizontal setpoints. This supersedes earlier claims that the bridge
could forward world setpoints directly into this vendor MAVROS.

`control.mavros_frame_profile: owl_vendor_world` now converts outgoing position,
velocity and acceleration as `(x,y,z) -> (-y,x,z)` and yaw as `wrap(yaw+pi/2)`.
Incoming vendor linear velocity is already world-relative and is not rotated
again. No manufacturer source or PX4 parameter is modified. The verified local
MAVROS initial x/y/z/R/P/Y offsets are zero; this profile is not a general adapter
for arbitrary vendor initial orientation settings.

`standard_enu` retains ordinary MAVROS behavior (body odometry twist, unchanged
ENU setpoints). Missing field preserves that historical standard behavior;
repository OWL defaults and this robot's deployment configs explicitly select
`owl_vendor_world`. Unknown profiles reject startup. Health/preflight report the
selected profile. Planner, cloud, public pose, relative commands and observation
remain in the same world as before; Agent should not add its own 90-degree offset.
Robot init also resets its yaw limiter to current measured yaw, and localization
reset discards the previous limiter heading. Source-level and simulated transport
validation does not replace the pending physical verification of the repair.

## 2026-09-10 follow-up safety audit

Robot-only changes; Agent implementation remains unchanged. World is a local
right-handed, gravity-aligned frame, not necessarily geographic east/north.
The public reflection and camera optical transform remain unchanged.

In owl_vendor_world mode the bridge now continuously compares timestamp-matched
references: map pose after -90-degree conversion versus control odom uses
the same acquisition timestamp (1 microsecond numerical tolerance, 3 cm /
2 degrees spatial limits); FAST-LIO vision pose versus control odom permits
50 ms (25 cm / 10 degrees spatial limits). Both arrival orders are supported;
replayed samples do not refresh the original matched odometry receipt time. Both need a successful comparison in the last 0.5 s.
Nonzero vendor initial x/y/z/R/P/Y parameters are unsupported and block control.
References are `/mavros/local_position/pose` and `/mavros/vision_pose/pose`.
A mismatch invalidates localization and output, rather than silently learning an
offset. Missing references revoke output while enabled. Health adds
`frame_alignment_ok`, `frame_alignment_error`, and `landing`. These checks bound
consistency; they do not certify true physical scale or obstacle visibility.

Incoming point clouds require valid XYZ float storage and at least one finite
point; infinity/malformed buffers/all-NaN clouds invalidate cloud freshness.
Repeated acquisition timestamps do not renew freshness. NaN points are permitted
only when finite points also exist (upstream skips NaNs). Cloud coordinates are
not rotated: they already belong to the same FAST-LIO world as planner odometry.
Vendor angular-rate XY swapping is undone and Euler yaw rate is derived with
roll/pitch, instead of assuming body Z angular rate always equals yaw derivative.

New motion goals must be at or above world Z=0 and below the planner virtual
ceiling minus obstacle inflation (currently 2.7 m). This is an absolute world
height, not height above current ground. Landing remains available outside the
navigation world limit. `control.max_tracking_error_m` defaults to 0.5 m and
bounds reference-to-measured position error; exceeding it fails/holds the task.
Takeoff remains a direct vertical ramp, not an EGO obstacle-planned maneuver.

On land acceptance, measured hold continues while OFFBOARD is still active and
AUTO.LAND is pending. It stops when AUTO.LAND State is observed. Mode takeover
before or after that confirmation is latched; a late FCU acknowledgment cannot
re-enable the bridge. Navigation cancel rejects active landing. Cloud loss does
not fail an ongoing landing; explicit fresh ON_GROUND/disarmed confirmation is
still required. Successful landing clears initialized; another flight requires
an explicit new init. After confirmed landing and measured stop, console init
releases the old session and acquires a new one before initializing. It waits
up to 3 s for ground stop settling; uncertain release prevents reacquisition. In-flight FCU service requests
remain non-recallable and physical emergency recovery still belongs to the pilot.

Invalid odometry/attitude immediately revokes output; cancel resets the yaw hold
to measured heading. Takeoff and pure yaw no longer create task-owned EGO
processes or accept unsolicited planner trajectories. An idle EGO can remain
running for readiness, but its callbacks do not own direct motion.


## 2026-09-10 takeoff lag and retry correction

Previously the direct takeoff ramp advanced independently of actual ascent; FCU
spool-up could accumulate reference error and trigger the 0.5 m tracking limit.
Robot now caps the vertical lead at `control.takeoff_max_lead_m` (default 0.2 m,
also capped to half `max_tracking_error_m`). The 0.3 m/s reference ramp and +1 m
target remain unchanged. Once armed/OFFBOARD preparation is complete, absence
of 3 cm measured upward progress for `control.takeoff_progress_timeout_s`
(default 10 s), while outside goal tolerance, fails/holds the task. The global
0.5 m tracking limit remains active; there is no automatic landing or retry.

Task status may add `takeoff_reference` (`z_m`, `measured_z_m`, `lead_m`) and
`execution_error` on execution/tracking rejection. The latter contains reference
and measured arrays `[x_m,y_m,z_m,yaw_rad]` in internal ROS world,
`tracking_error_m`, `tracking_limit_m`, and reference derivative norms
`velocity_m_s`, `acceleration_m_s2`. These diagnostics do not change public
cm/degree motion commands. Nonfinite diagnostic values become JSON null. Blocking task failure
also includes these diagnostics in its error string when available. Agent may
log optional fields; no new request or mandatory field is required.

The hash-pinned, per-generation EGO process may confirm its INIT→WAIT_TARGET
transition using its own private stdout log if its one-shot DataDisp message is
lost. This requires prior odometry forwarding and the exact transition emitted
by that process; process elapsed time alone cannot satisfy initialization. Goal
submission still requires subsequent map output and the goal subscriber. Optional
`timing_s.fsm_log_confirmation` identifies this path. Process teardown now excludes
late odometry callbacks from already unregistered publishers. This is Robot-only;
Agent adaptation and physical flight acceptance remain pending as stated above.


## 2026-09-10 current-stop gating between motions

A terminal task's `stopped:true` records confirmation at completion, not a lasting
promise about subsequent hover. New navigation (including relative XYZ/yaw)
continues to require current Robot stop confirmation at command acceptance.
Current health `stopped` can become false again after arrival if measured position
or heading variation exceeds the window limits; this does not rewrite completed task history.

Robot's standalone console/test client now waits up to 8 s for current
`health.stopped:true` with no active task before capturing its test origin,
submitting each waypoint or submitting each TRACK relative motion. Heartbeat,
epoch and flight-authority checks continue during the wait. Active tasks are
rejected rather than queued. Timeout submits no new motion and reports measured
stop diagnostics; it does not relax thresholds, cancel landing, reacquire authority
or retry a request with uncertain outcome. A change between the health response
and command acceptance can still produce a safe 409 rejection at the Robot.

Health optionally adds `stop_diagnostics`: `speed_m_s`, `yaw_rate_deg_s`,
`speed_limit_m_s`, `yaw_rate_limit_deg_s`, `stable_samples`, `required_samples`,
`stable_duration_s`, `required_duration_s`. Duration is measured through the last
accepted odometry receipt, not extended by repeated health reads. Unavailable
speeds are null. These diagnostic fields do not replace odom freshness and
control/hold checks. The original twist-speed gate is superseded by the pose-window
rule below; raw speed and rate fields remain available for diagnosis.

Agent code is unchanged. Agent should apply the same bounded current-stop gate
between motions and retain server-side rejection handling; historical terminal
status alone does not guarantee that the next command can be admitted.


## 2026-09-10 diagnostic capture for physical stop investigation

No flight-control or stopping-policy change. `run_owl_ego.sh record --output DIR`
starts an independent read-only ROS bag recorder using the selected config. It
subscribes to odometry, map/LIO poses, local/body velocity, raw local setpoints,
bridge status, FCU state/landed state and localization reset topics. It captures
no RGB or point cloud. Ctrl-C stops the owned recorder and waits for bag indexing;
`recording_result.json.finalized` confirms the final bag, not that every requested
topic had messages. Config/planner snapshots and source hashes accompany the bag.

`run_owl_ego.sh analyze DIR` exports odometry, reference poses, velocity sources,
full bridge-status snapshots and per-stopping-task statistics. Raw acquisition
and bag receipt ROS timestamps are separate; nearest preceding status/setpoint
ages are explicit. Odometry raw twist and bridge-profile world velocity are both
preserved. Position-difference estimates use 0.2/0.5/1 s windows and reset on epoch,
time reversal, duplicate stamp or data gap. They average displacement, so can hide
oscillation and must not replace stopping checks. Setpoint position error is only
calculated for fresh unmasked LOCAL_NED-profile position targets. Missing required
odom/state/status/setpoint topics are reported; no setpoints while grounded is
expected but cannot verify the output side of the chain.

The HTTP console log now adds non-heartbeat POST `http_request` events with request
body and `console_command` events. Request event time is before sending; existing
HTTP response time is after completion, neither is FCU execution time. Current
stop wait diagnostics print and log at most once every 2 s during waiting. The
pre-submission and cancellation waits now both use 8 s, as specified below.
These are Robot tools; no new Agent request fields are required.

## 2026-09-10 simplified pose-window stop confirmation

Robot now uses one stability rule for cancellation, arrival and admission of the
next motion. It reuses cancellation's existing trajectory invalidation and measured
position/yaw hold. It does not add another EGO stop/restart state machine.

Previously raw odometry speed had to remain below 0.1 m/s. The captured stationary
hover reported about -0.124 m/s vertical twist despite only centimetres of position
variation. Robot now examines all accepted poses over at least `stable_duration_s`
(0.5 s), keeping the sample at or just before the window boundary and requiring at
least `stable_samples` (5) distinct timestamps. The norm of the XYZ component ranges
must be at most `stop_speed_m_s * stable_duration_s` (0.05 m); unwrapped yaw range
must be at most `stop_yaw_rate_rad_s * stable_duration_s` (2.5 degrees). This includes
intermediate excursions, not just displacement between the two endpoints.

`stopped:true` therefore means bounded pose stability, allowing small hover motion;
it is not a claim of physically zero velocity. Arrival additionally requires the
existing current goal tolerances (0.15 m / 5 degrees). The separate arrival stability
counter is removed. Cancellation clears the window before collecting confirmation.
Duplicate samples cannot fill it; resets, time gaps and invalid odometry invalidate
confirmation. Lease, controller ownership, epoch and old-trajectory isolation remain
required. This change does not repair the FCU estimator or alter EGO velocity input.

Config field names are retained for existing YAML files; their products above now
define pose-range limits. Raw speed/rate remain logged, but do not gate `stopped`.
Optional health diagnostics add `method: "pose_window"`, `position_span_m`,
`position_limit_m`, `yaw_span_deg`, `yaw_span_limit_deg`. `stable_duration_s` now
describes the accepted pose window; repeated health reads cannot extend it.

Robot console waits at most 8 s both before a new motion and after cancelling.
Timeout submits no next motion. HTTP requests and terminal task fields are unchanged.
Agent implementation is unchanged and has not confirmed this revised stop semantics;
it should consume Robot's `stopped` result rather than reconstruct a raw-speed gate.
The additional diagnostic fields are optional for compatibility with older Robot.
Physical flight acceptance of this revision remains pending.


## 2026-09-10 physical validation finding: relative altitude accumulation

The physical console run at 17:34 completed its task sequence but failed the intended
level-flight behavior. Five relative commands with z=0 accumulated 0.5005 m of measured
world-Z rise. Current Robot rebases each relative goal on measured pose; approximately
0.10 m of positive altitude tracking error fits within the 0.15 m arrival tolerance
and becomes the next command's altitude reference, including a yaw-only command.
An offline FlightCore reproduction with a constant +0.10 m position offset confirms
this accumulation mechanism. Prior mock tests injected twist bias but did not model
this persistent position offset. They do not validate physical level flight.

No interface or control-code change is made by this investigation. A revision must
specify reference preservation for uncommanded Z, explicit vertical movements and
cancellation, and test multi-action altitude bounds before physical revalidation.
Agent must not interpret the current route's successful task statuses as flight
acceptance. Detailed evidence is in robot-002's latest altitude investigation section.


## 2026-09-10 relative zero-Z altitude reference fix

Robot implementation now corrects the accumulation identified above. For relative
z=0, the target Z is the existing `hold` reference Z. It remains unchanged during
planner startup as well as at the goal. No new persistent anchor or configuration
mode is introduced. XYZ/yaw hold and trajectory invalidation are reused. A yaw-only
command preserves height and continues through the existing direct yaw path without
requesting a vertical EGO trajectory just to correct the measured hover offset.

XY offsets and yaw still use measured position/heading at command acceptance.
Nonzero z remains measured-Z plus the requested displacement; on arrival this new
altitude goal becomes the reference for subsequent z=0 moves. Absolute navigation
sets its supplied goal, including Z. Actual cancellation/failure still captures the
measured stop pose for hold; this intentionally establishes a new hold altitude,
not a return to the cancelled destination. Localization reset discards the hold,
and reinitialization establishes a new one. Rejected commands cannot modify it.

This is a semantic correction to z=0 with no request/response shape change. Agent
must not predict z=0's target by adding zero to a measured height with tracking
error. Its requests and units remain compatible; explicit acceptance of this revised
reference semantics is pending. Robot console's level TRACK checks now all use
the returned P altitude as their expected Z, so a per-step height error cannot silently
change the next validation target. Existing 0.15 m goal tolerance and stop window are
unchanged. This prevents reference accumulation; it does not guarantee perfectly
level EGO trajectories or fix the FCU's underlying altitude tracking offset.


## 2026-09-10 standalone console route update

Only the Robot standalone test client's default route changes; HTTP units and
control semantics are unchanged. B is 400 cm forward from the initial hover pose.
P remains the measured pose captured once displacement from that origin reaches
100 cm while the outbound navigation is executing. It is not an exact surveyed
point, nor a request to stop immediately upon reaching 100 cm.

The post-capture delay changes from 0.8 s to 1.0 s. The client then requests cancel,
waits for confirmed stopping, and only then navigates back to P. Polling and HTTP
latency mean the delay is approximate and the actual stopping location is beyond P.
After returning, the latest user correction makes the first two TRACK destinations
P-left 100 cm and P-right 100 cm, both rotated by P's saved heading and both using
P's saved Z/yaw. The client computes these absolute poses and reuses `/v21/navigation`;
the trip from left to right is approximately 200 cm. The earlier implementation
measured the rightward metre from the left endpoint and has been superseded.
The third command remains `/move_relative_xyz_yaw` `[0,0,0,90]`, clockwise at the
right endpoint, with Z=0 retaining the P altitude reference. The client next navigates
to P's full saved pose, restoring its heading, then submits a new navigation to B.
At B it holds until the operator requests landing. Preview and `track_complete`
logs distinguish `saved_P` from `current_body`; `relative` values in these diagnostic
events must be interpreted using their reference, not assumed to be HTTP payloads.

Agent code is unchanged. Existing generic five-action multi-round regression
scenarios remain separate from these new console defaults. The previously identified
FCU altitude offset and transient vertical motion are still pending physical resolution.


## 2026-09-10 position/yaw completion deadline correction

Previously an EGO task expired at polynomial end plus `trajectory_grace_s` (2 s),
regardless of the separately rate-limited yaw. A short XYZ path combined with a
roughly 90-degree heading restoration could therefore fail before actual yaw settled.

Robot now budgets XYZ and yaw concurrently. On the first accepted polynomial it
computes the remaining shortest wrapped yaw angle from the current command reference,
divides it by `yaw_rate_rad_s` (30 deg/s), and adds that duration to the later of
polynomial start and receipt ROS time. This fixed expected yaw-reference completion
time is retained across subsequent replans. Each polynomial's expiration deadline is:
`max(polynomial_end, yaw_reference_end) + trajectory_grace_s + stable_duration_s`.
The current extra terms are 2 s for tracking lag plus 0.5 s for measured stability.
This is an expected command-reference schedule with a bounded allowance, not a claim
that the physical vehicle necessarily turns at the configured maximum rate.

After polynomial end and before this deadline, Robot holds the sampled polynomial
endpoint with zero velocity/acceleration feedforward and continues rate-limited yaw.
It does not extrapolate the polynomial or jump directly to an unverified destination.
Actual position error <=15 cm, yaw error <=5 degrees and the fresh stop window remain
required; successful arrival can complete earlier than the deadline. Failure to settle
still fails at the deadline. Overall task timeout, planner heartbeat, lease, odometry,
tracking-error checks, cancel/land/manual takeover and generation isolation still apply.
Direct yaw and takeoff retain their existing paths/timeouts.

Optional `timing_s.yaw_reference_ready` logs the expected yaw-reference completion
relative to command acceptance. Requests and terminal statuses are unchanged; Agent
should continue polling actual completion and handling failure rather than deriving
total completion from position-trajectory duration. Agent implementation is unchanged.
Robot console now reports a populated Robot execution error as `Robot motion failed`
instead of labelling every such error as loss of flight authority. This change does
not resolve the separately documented FCU altitude offset or establish physical-flight
acceptance.

## 2026-09-10 navigation startup altitude continuity

Previously only relative Z=0 retained the established altitude while the next EGO
process started. Absolute navigation reset its waiting hold to measured altitude,
creating upward reference steps when the vehicle already hovered above the target.
Robot now preserves the previous hold Z during startup for every accepted navigation,
including explicit vertical requests. The requested goal is unchanged and the EGO
polynomial is followed normally when available. Waiting XY/yaw still capture the
measured stopped pose. Cancel, failure, initialization and localization reset retain
their existing reference semantics. This introduces no additional mode, tolerance
change or endpoint. Agent requests remain compatible; no Agent code was changed.

This removes a startup reference reset, not the FCU's measured vertical-velocity
bias or EGO's normal use of measured starting position. It does not force a horizontal
plane through an obstacle-avoiding 3D trajectory. Latest physical log analysis still
shows an approximately 11 cm steady altitude offset; physical resolution is pending.

## 2026-09-10 reboot preparation and controller handover

User-authorized Robot startup now handles the vendor controllers started at boot.
`run_owl_ego.sh prepare` runs a guarded handover followed by preflight with a nonzero
exit status if errors remain. `bridge` also runs the handover before roslaunch;
handover failure prevents launch. `check` remains read-only; server/console retain
their existing behavior. No HTTP or Agent interface changes are involved.

Handover requires fresh received and stamped MAVROS State/ExtendedState (<=2 s),
connected, disarmed and explicit ON_GROUND. It stops only `/captain`, followed by
`/mavros_controller`, using ROS node shutdown. State is rechecked before each stop
and during verification. Other motion publishers, including an existing owl bridge,
cause refusal rather than automatic termination. Both known nodes must disappear
and MAVROS motion publishers must remain absent for 1 s within a 5 s verification
budget. Reappearing or unreachable nodes cause failure, not repeated forced kills.

The script does not modify vendor boot files or stop rc-local, MAVROS, localization,
camera or other nodes, nor set flight flags, modes or arm the FCU. The original v20
boot path remains available on reboot. This startup check does not replace the
bridge's continuous ownership checks or resolve the documented altitude issue.

## 2026-09-10 standalone console manual relative commands

The Robot console now accepts `move_rel_xyz_yaw X Y Z YAW` and `move_rel_xyz X Y Z`
(yaw defaults to zero), also spelled `move_relative_xyz_yaw` / `move_relative_xyz`.
Arguments must be integer cm/degrees. These commands reuse the existing blocking
HTTP relative endpoint, the 15 s motion timeout, session heartbeat, current-stop
gate and console cancellation/landing behavior; they do not introduce a new flight
controller or bypass EGO. Unlike the scripted P-left/P-right destinations, manual
XY/yaw commands refer to the current body pose at Robot acceptance. Z=0 retains
the established altitude reference under the existing contract.

The console logs start pose, approximately 5 Hz GET-pose samples while the request
is running, and a final or failure summary. `relative_motion_summary` includes
`completed`, `sample_count`, `delta_z_cm`, `max_rise_cm`, and `min_delta_z_cm`, all
relative to the measured start height. This measures sampled motion, not exact
continuous extrema or goal-height tracking error. It does not change acceptance
tolerances or add altitude compensation. Existing ROS recording remains necessary
to inspect setpoints and faster transients. Agent requests/implementation are unchanged.

## 2026-09-10 explicit landing survives localization reset

Previously localization reset failed every active task, including a land awaiting
AUTO.LAND service confirmation. Console also rejected an epoch change while waiting
for landing. A physical log now confirms FCU AUTO.LAND and ground/disarm despite
the Robot reporting failure during a LIO/world mismatch.

Robot retains an active explicit land across localization reset, revokes all world
output and initialization, changes epoch and records optional task `localization_error`.
Navigation status exposes this field even after successful landing. Navigation and
takeoff still fail on reset. The FCU mode worker may finish the already requested
AUTO.LAND. Arrival requires fresh connected/disarmed State and fresh ON_GROUND;
mode-service failure/timeout, session expiry and manual takeover remain failures.

Console/Runner specifically omit odometry/epoch requirements while waiting for
`/land`; heartbeat, manual-takeover checks and bounded HTTP waiting remain. After
success they clear the old epoch for the next ground init. Agent should likewise
observe an already requested landing across reset, log `localization_error`, and
require a fresh initialized epoch before further motion. Agent is unchanged and
joint acceptance is pending. This corrects landing task handling, not FCU altitude
bias or the physical yaw motion observed during AUTO.LAND.
