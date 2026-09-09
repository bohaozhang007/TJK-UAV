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
`arrived` requires XYZ Euclidean error, yaw error and low speed within Robot
tolerances for consecutive fresh odometry samples; return `stopped:true`.
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
