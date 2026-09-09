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
