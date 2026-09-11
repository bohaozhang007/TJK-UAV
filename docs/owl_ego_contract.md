# OWL EGO / Agent v21 HTTP contract — protocol version 1

Updated: 2026-09-11. This is the shared interface specification for the current
Robot implementation. It consolidates the earlier amendments; implementation
history and test evidence remain in [robot-002](collab/messages/robot-002.md).
The Agent client in `src/robot_client/owl_ego.py` still needs the adaptations
listed below. Protocol version 1 and the existing endpoint names are retained;
Robot availability does not mean the current Agent is already compatible.

Robot runs on OWL; Agent runs on the Windows computer and calls Robot over Wi-Fi.
Agent owns route decisions, detection/TRACK and DA3 depth. Robot owns FAST-LIO
obstacle input, EGO, flight execution, cancellation and control authority.
Only one motion session is valid at a time. Optional local operator supervision
allows automatic initial transfer to Agent and a separate landing override,
without sharing or extending the Agent's lease. See the operator section below.

## Endpoint inventory

HTTP JSON, port 8765 by default. Base URL is the Robot's reachable Wi-Fi address,
for example `http://192.168.2.20:8765` if that remains its configured address.

| Method | Path | Request | Successful response / completion |
| --- | --- | --- | --- |
| GET | /v21/capabilities | None | Capability flags below |
| GET | /health | None | `{ok:true,health:{...}}` |
| GET | /get_pose | None | `{ok:true,pose:{x,y,z,yaw},localization_epoch}` |
| GET | /motion_tolerances | None | `{ok:true,motion_tolerances:{...}}` |
| GET | /v21/observation | None | Synchronized JPEG, exposure pose and geometry |
| POST | /v21/session | `{request_id,operator?}` | `{ok:true,session_id}`; optional local operator_token; does not arm |
| POST | /v21/heartbeat | `{session_id}` | `{ok:true}` |
| POST | /v21/session/release | `{session_id}` | `{ok:true}`; revoke session, request hold |
| POST | /init | `{session_id,request_id}` | `{ok:true,message}`; initialize without arming |
| POST | /takeoff | `{session_id,request_id,auto_arm?}` | Blocking flight completion or error |
| POST | /v21/navigation | `{session_id,request_id,localization_epoch,pose}` | Immediate acceptance: `{ok:true,task_id}` |
| GET | /v21/navigation/status | Query `?task_id=...` | Task state, stop confirmation, optional diagnostics |
| POST | /v21/navigation/cancel | `{session_id,request_id,task_id}` | Immediate acceptance: `{ok:true,task_id}` |
| POST | /move_relative_xyz_yaw | `{session_id,request_id,x,y,z,yaw,timeout_s?}` | Blocking flight completion or error |
| POST | /land | `{session_id,request_id}` | Preempt motion; block for confirmed landing or error |
| POST | /v21/operator/land | Local only: `{operator_token,request_id}` | Immediate `{ok:true,session_id,task_id}`; retire Agent authority and monitor landing under new operator lease |

Capabilities:
~~~json
{"ok":true,"backend":"owl_ego","protocol_version":1,
 "async_navigation":true,"cancel_and_hold":true,
 "synchronized_observation":true,"control_lease":true,
 "relative_xyz_yaw":true,"software_takeoff":true,"operator_override":true}
~~~

No owl_ego HTTP aliases exist for `/move_relative_xyz`, `/rotate`, `/stop`,
`/abort`, `/close` or Robot depth/image-file APIs. Use the combined relative
endpoint, task cancellation, session release, and synchronized observation.

## Ownership, retries and time budgets

- Every mutation except session acquisition requires the current session.
  Acquisition rejects another live Agent owner, active tasks or landing; when
  airborne it also requires measured stopping. The operator-to-Agent transition
  below is the only exception to rejecting an existing live owner.
- `request_id` must be a UUID. For init/takeoff/navigation/cancel/relative/land,
  keep the same ID and identical body when retrying the same uncertain request.
  Robot caches results/errors within the session. Reusing an ID with a different
  path/body is rejected; a new ID can create a new action.
- Session acquisition is also deduplicated. Expired/released sessions cannot be
  revived by replaying their acquisition request. Heartbeat and release do not
  use request_id; repeat release after retirement may be rejected.
- Send heartbeat every 0.5 s with a 2 s HTTP timeout, independently of inference,
  capture and blocking motion. The onboard lease expires after 5 s.
- Expiry/release invalidates active work and requests measured hold where output
  authority remains valid. It does not land or disarm. Telemetry/control loss
  may revoke setpoints and leave recovery to validated PX4 failsafe/pilot.
  Agent must not automatically reacquire and resume an interrupted mission.
- v21 mutation endpoints acknowledge within a 3 s client budget; they do not wait
  for flight. Legacy relative/takeoff/land are blocking exceptions. Success is
  normally `{ok:true,message:"motion completed",task_id:"nav-..."}`.
  Already-airborne takeoff may return success without a task_id.
- Current server relative wait defaults to 15 s; optional `timeout_s` must be
  finite, numeric, non-boolean and in (0,170]. Takeoff/landing waits default to
  60/90 s. Transport timeout must exceed the selected server wait; existing
  legacy client default is 180 s. Heartbeat/cancellation must remain concurrent.
- Relative/takeoff server wait expiration requests cancellation and returns
  HTTP 504; landing confirmation timeout returns 504 without cancelling landing.
  HTTP timeout/disconnection alone never proves that movement has stopped.
- Robot task timeout is 120 s; EGO startup has 10 s, executing planner heartbeat
  loss has 1 s. Increasing relative `timeout_s` does not override these limits.
- For an EGO polynomial, completion deadline is
  `max(polynomial_end, yaw_reference_end) + trajectory_grace_s + stable_duration_s`.
  The deployed live YAML now specifies 5 s + 0.5 s; repository default remains
  2 s + 0.5 s. A running bridge must load the changed configuration on restart;
  no HTTP endpoint reports the loaded grace value. After polynomial end, hold
  its endpoint with zero velocity/acceleration feedforward while yaw settles.
  Complete as soon as arrival is confirmed; never extrapolate or wait a fixed
  number of seconds to declare success.

## 2026-09-11 local operator supervision and automatic Agent access

The user-selected workflow is local operator initialization/takeoff, followed by
Agent navigation/TRACK; the operator can request landing at any time. There is
no extra user handoff command and no shared heartbeat/session between clients.

When operator_override is advertised, a local client may acquire with
`{request_id,operator:true}`. Robot returns an additional opaque operator_token.
Operator acquisition and /v21/operator/land require an actual loopback connection
(127.0.0.1 or ::1), not a forwarded header. The override also requires the token;
an arbitrary local client cannot invoke it without that token. Normal Wi-Fi
Agent requests omit operator and never receive the operator token.

The operator initially owns the ordinary motion session and heartbeats it. Once
initialized, armed OFFBOARD, airborne, fresh, stopped, without an active task or
landing/manual takeover, Agent's ordinary POST /v21/session atomically acquires
a new session and retires the operator's motion session. Hold height, yaw,
initialization, epoch and stopping evidence are preserved. A second Agent is
rejected. A concurrent operator movement and Agent acquisition are serialized;
they cannot both obtain motion authority.

The old operator heartbeat returns `{ok:true,delegated:true}` without renewing
the Agent's lease; the local client stops that heartbeat. Agent must send its
own 0.5 s heartbeats. Its loss still expires after 5 s and stops motion even if
the operator application remains open. Expiry/release does not automatically
return navigation authority or reopen acquisition; operator recovery is required.

After acquiring, Agent may call /init to acknowledge existing initialization;
in supervised mode this preserves the established references. Agent must not
initiate takeoff: it is already airborne, and supervised Agent takeoff requests
are rejected. If initialization has been lost, Agent cannot reinitialize it by
itself. Agent may use its owned /land for normal mission completion or explicit
recovery. The user confirmed that takeoff belongs to the operator, but both
operator and Agent may initiate landing. Keep Agent heartbeat active until
landing confirmation, then release its session; do not release before requesting
Agent-owned landing. This supersedes the earlier recommendation to skip Agent's
normal final landing. The operator's landing override remains available.

POST /v21/operator/land atomically retires the Agent session, fails its active
movement with `preempted by landing`, fences old trajectory ownership, and reuses
the ordinary AUTO.LAND execution. It returns a new operator session_id and the
landing task_id immediately. The operator heartbeats that new session and polls
the landing task; new requests from the retired Agent cannot move, cancel the
landing or release its lease. If landing was already underway, adopt its task
without issuing a second AUTO.LAND request. Repeating the same request_id/body
returns the original result, including after an uncertain transport outcome.

The token grants only local landing override, not concurrent navigation or a
pilot bypass. Fresh telemetry and the existing manual takeover checks apply.
Accepted landing still survives localization reset and requires fresh actual
ON_GROUND/disarmed confirmation. Robot server/bridge restart revokes the token;
release of the operator's own session revokes supervision. Closing the local
application after delegation does not cancel Agent motion or keep its lease alive.

Optional health fields: `operator_supervised` (boolean) and `control_owner`
(`operator`, `agent`, `none`). They describe current ownership, not a new grant.
Agent should identify operator landing/preemption using these fields plus task
status/landing, end its current mission, and tolerate rejection of old-session
cleanup. Do not reacquire, send another takeoff, or fight the operator's landing.
No operator endpoint or token needs to be implemented in the Agent client.

Absent operator:true, existing exclusive sessions and Agent-operated takeoff
retain their earlier behavior. This is an optional protocol-1 extension, with
Robot tests recorded in robot-002; actual Agent adaptation is still pending.

## Coordinates and altitude reference

Public absolute poses have exactly `x,y,z,yaw`, all finite numbers:
XYZ centimetres in the fixed localization world, yaw degrees in [-180,180).
The origin is not rebased at takeoff. Agent converts mission-relative waypoints
to this world before submitting them. Current bounds include absolute XYZ
magnitudes <=30 m and navigation height >=0 and <2.7 m (Robot configuration,
world Z, not height above terrain).

Public mapping: x=ROS world x*100, y=-ROS world y*100, z=ROS world z*100,
yaw=-ROS yaw radians*180/pi, wrapped to [-180,180).
World is local, gravity aligned and right handed internally; it need not be
geographic east/north. The public Y reflection is not a camera rigid transform.

Relative inputs are integer centimetres/degrees: positive x forward, y right,
z up, yaw clockwise. XY and yaw use measured body heading at Robot acceptance.
Send a combined XYZ/yaw action in one request; do not rotate axes twice.

**Zero-Z correction:** relative z=0 retains the existing hold-height reference,
including yaw-only moves; it is not measured altitude +0. Nonzero z uses measured
altitude at acceptance plus the requested displacement. Absolute navigation uses
the entire supplied goal, including Z and yaw. Successful arrival adopts that
goal as hold reference. All navigation startup waits preserve the previous hold
Z until EGO supplies the trajectory. This prevents successive hover errors from
becoming higher targets, but does not constrain a 3D obstacle-avoiding path to a
horizontal plane or guarantee zero actual altitude variation.

Cancellation/failure captures measured position/yaw for hold and establishes a
new height reference. Localization reset discards old references. Agent must
not calculate z=0's expected goal by adding zero to a biased measured height,
or silently replace a saved exposure P with the later stopping pose.

`localization_epoch` changes on reset, origin discontinuity or restart. Never
reuse a saved waypoint/exposure in a new epoch. Public navigation requires the
saved epoch; relative requests use Robot's current epoch internally, so Agent
must also monitor epoch throughout every relative action.

Robot's `owl_vendor_world` adapter handles the vendor MAVROS map/world 90-degree
rotation for position, velocity, acceleration and yaw. Agent must not add its own
90-degree correction. FAST-LIO cloud and planning stay in the same Robot world.

## Navigation, cancellation and current stopping

Example absolute navigation:
~~~json
{"session_id":"SESSION","request_id":"UUID","localization_epoch":"EPOCH",
 "pose":{"x":200,"y":0,"z":100,"yaw":0}}
~~~
Coordinates here are illustrative fixed-world values, not a safe predefined route.

Statuses: `accepted`, `planning`, `executing`, `stopping`, `arrived`,
`cancelled`, `failed`. Poll the returned task_id, including for takeoff,
relative movement and landing when their task_id is available. Unknown task IDs
return HTTP 404. Task GET `ok:true` means the query succeeded; `status:failed`
is still a motion failure.

- `arrived,stopped:true`: measured 3D distance and yaw meet configured tolerances,
  plus current pose stability. Trajectory end alone is not arrival.
- Cancel acceptance revokes old trajectory/generation ownership and starts hold.
  Wait for `cancelled,stopped:true` before continuing. The legitimate race
  `arrived,stopped:true` is also acceptable.
- Cancel a terminal task safely without disturbing a newer task. A failed task
  stays failed. Active landing cannot be cancelled with navigation cancel.
- During blocking relative movement, its task_id can be found in
  `health.active_task_id`; cancellation may run in another thread. Associate
  the ID with the owned motion and poll that ID, not whichever task appears later.
- A terminal task's stopped flag records its completion time. Before **every**
  new navigation/relative command, require current `health.stopped:true`,
  `active_task_id:null`, matching epoch and flight authority. Use a bounded
  wait (initial Agent recommendation 8 s), keeping heartbeat/health active.
  A later change can still cause HTTP 409 `previous motion not confirmed stopped`.
  Do not queue the next action or regard this rejection as an accepted movement.
- Resume B with a **new** navigation and request_id after returning to P.
  Never replay/unpause the cancelled time trajectory.

Current tolerance response:
~~~json
{"ok":true,"motion_tolerances":{"position_tolerance_cm":15,
 "yaw_tolerance_deg":5,"position_error_metric":"euclidean_3d","source":"owl_ego"}}
~~~
Read the endpoint rather than hardcoding arrival tolerances.

Current stop confirmation uses >=0.5 s of accepted poses and >=5 distinct samples:
the norm of XYZ component ranges <=5 cm and unwrapped yaw range <=2.5 degrees.
It includes intermediate excursions. Raw twist speed is diagnostic only; Agent
must not reintroduce a speed <=0.1 m/s gate. `stopped` means bounded stability,
not physically zero velocity. Repeated polls do not accumulate stable time.

## Health and task semantics

<a id="2026-09-09-robot-round-2-planning-transitions-and-observation-availability"></a>

`GET /health` returns `{ok:true,health:{...}}`; outer ok does not certify flight.

| Field | Current meaning / Agent handling |
| --- | --- |
| initialized | Explicit Robot initialization; cleared after successful land/reset |
| airborne | Freshness must also be checked; IN_AIR enum drives the flag |
| control_ready | Initialized, owned, fresh, enabled, armed OFFBOARD airborne authority plus Robot sensor/publisher checks; independent of planner heartbeat |
| hold_ready | Valid OFFBOARD hold output authority; can survive loss of the old lease, so does not authorize a new owner |
| odom_ok | Fresh control odometry |
| rgb_ok | A valid synchronized observation can currently be assembled |
| active_task_id | Current active task or null |
| operator_supervised / control_owner | Optional local supervision / current owner operator, agent or none |
| planner_state | starting / ready / lost / not_required, as below |
| planner_ok | True only for ready; false is expected for starting and not_required |
| stopped | Current measured pose-window stability, distinct from historical task stopped |
| manual_takeover | Latched pilot/mode takeover; no automatic resume |
| landing | Explicit landing in progress; normal airborne authority checks do not apply |
| landed_state / landed_state_fresh | 0 UNKNOWN, 1 ON_GROUND, 2 IN_AIR, 3 TAKEOFF, 4 LANDING / receipt freshness |
| localization_epoch | Must match the mission's saved world identity |
| error | Current failure reason or null; holding after failure is not mission success |
| conflicting_publishers | Competing controllers; expected empty |
| mavros_frame_profile | Current Robot adapter, normally owl_vendor_world |
| frame_alignment_ok / frame_alignment_error | Robot world/map/LIO consistency result |
| observation_error_code / observation_retryable | Present when observation assembly fails |
| stop_diagnostics | Optional pose-window ranges, samples, duration and raw speed/rate |

Planner phase handling:
- `starting`: matching planning task is within the 10 s startup budget without
  its own fresh heartbeat. Continue checking control/hold/epoch; do not abort
  solely for planner_ok=false.
- `ready`: current task generation heartbeat <=1 s. This alone does not mean
  a usable first trajectory/map has arrived; task may still be planning.
- `not_required`: hold, stopping, terminal work, direct takeoff or pure yaw.
  Do not require a planner heartbeat for these phases.
- `lost`: required planner has lost readiness; fail the mission phase.

During airborne navigation/TRACK, check current ownership, control/hold authority,
odom, epoch, manual takeover, conflicts and errors. Observe phase/task together;
the separate HTTP snapshots are not atomic, so terminal transitions need
reconciliation with the same task_id. Handle temporary observation absence by the
bounded retry rule below, not by dropping all flight checks.

Task status always includes `ok,task_id,status,stopped`; optional fields are:
`error,generation,timing_s,diagnostics,execution_error,takeoff_reference,localization_error`.
No goal or full measured pose is currently returned by task GET. Agent should
log its submitted goal and sample /get_pose if XYZ error components are needed.
`diagnostics` contains position_error_cm, yaw_error_deg, speed_m_s,
yaw_rate_deg_s and elapsed_s. Terminal diagnostics retain the last sample.

`timing_s` records observed monotonic durations since acceptance: process setup/
spawn, odom forwarding, FSM/map/heartbeat, goal/first trajectory, expected
yaw_reference_ready and terminal, with optional fsm_log_confirmation.
Missing fields are normal for direct motion or events not yet observed.
`execution_error` reference/measured arrays use internal ROS world metres/radians;
`takeoff_reference` uses metres. These optional diagnostics are not public
centimetre/degree motion payloads.

## Initialization, software takeoff and landing

Acquire session and start heartbeat before POST /init. Initialization does not
arm or select OFFBOARD; control_ready/hold_ready can still be false on the ground.
Robot enforces configured flight flags, current telemetry, cloud, frame alignment
and exclusive publishers. Agent does not edit these deployment gates.

For Agent-operated ground takeoff, explicitly opt in:
~~~json
{"session_id":"SESSION","request_id":"UUID","auto_arm":true}
~~~
Check `software_takeoff:true` before using it. Absent/false `auto_arm` preserves
pilot-controlled mode/arming and cannot provide unattended ground startup.
Current Agent BaseClient._takeoff sends an empty body before lease injection;
it must be adapted if software takeoff is wanted.
This ground-start option does not apply to the user's operator-supervised flow:
Agent attaches after takeoff and must skip its own takeoff; normal final landing
by Agent remains supported and authorized by the user.

Robot performs bounded hold preparation, OFFBOARD selection and ordinary arming,
then direct +1 m takeoff at configured reference speed/lead limits. Preserve
measured heading. No forced arm or automatic retry after uncertain FCU outcome.
Takeoff is not an EGO obstacle-planned maneuver. Already airborne with valid
authority does not trigger another ascent.

POST /land preempts current motion and requests AUTO.LAND. It remains subject
to ownership, fresh telemetry at acceptance and manual-takeover rules; failure
of another motion does not by itself disable landing. During an already accepted
landing, do not require control_ready/hold_ready/planner_ok/airborne to remain true.
Success requires fresh connected/disarmed State and fresh explicit ON_GROUND;
UNKNOWN/LANDING or merely airborne=false is insufficient.

**Landing reset correction:** an already accepted land survives a localization
reset. Robot revokes world outputs, clears initialization, changes epoch and
records optional task `localization_error`, while the FCU can finish the requested
AUTO.LAND. Agent must continue observing this landing within its deadline rather
than treating the epoch change as proof of failed landing. Navigation/takeoff
still fail on reset. Lease loss, manual takeover and FCU service failure are not
bypassed. This is not permission to initiate a new motion in an invalid frame.

After successful landing, clear old mission coordinates and explicitly initialize
a fresh mission before further flight. Never auto-retry an uncertain takeoff or
infer physical landing success from an HTTP error.

## Synchronized observations and geometry

<a id="2026-09-09-user-authorized-approximate-camera-profile"></a>

GET /v21/observation is available before acquiring control. Its current payload
is illustrated below. The image is rgb_jpeg_base64, not a shared file path.

~~~json
{"ok":true,"frame_id":"camera-123","timestamp_s":1234.5,
 "age_s":0.02,"sync_error_s":0.01,"localization_epoch":"EPOCH","world_frame":"world",
 "pose":{"x":0,"y":0,"z":100,"yaw":0},"image_size":[640,360],
 "rectified":false,"calibration_quality":"approximate",
 "intrinsics":[[320,0,320],[0,320,180],[0,0,1]],
 "world_from_camera_optical_cm":[[0,0,1,0],[-1,0,0,0],[0,-1,0,100],[0,0,0,1]],
 "rgb_jpeg_base64":"BASE64_JPEG",
 "geometry_assumptions":{"intrinsics":"approximate_fov",
 "assumed_horizontal_fov_deg":90.0,"principal_point":"image_center",
 "square_pixels_assumed":true,"distortion":"unknown_not_corrected",
 "extrinsics":"body_coincident_fixed","camera_translation":"body_coincident_assumption"}}
~~~
Values are illustrative. P is the interpolated **exposure** pose, not HTTP receipt
pose or detection-completion pose. Duplicate reads of one exposure retain frame_id.
Preserve matching JPEG/K/transform/epoch atomically; output dimensions remain fixed
during a session. Initial long edge is 640 px. K applies to that returned resolution.

Optical axes are right/down/forward. The 4x4 transform maps optical coordinates
to right-handed ROS world, with translation in cm and full exposure body attitude.
Reflect world Y only when converting a projected point into public navigation
coordinates. DA3 depth is Agent-side; no image/depth path is shared over Wi-Fi.

Current user-approved profile assumes camera/body centres coincide, fixed camera
horizontal forward (pitch=0), unknown distortion, centred principal point and
square pixels. K uses fx=fy=W/(2*tan(HFOV/2)), cx=W/2, cy=H/2, with HFOV=90°
an explicit assumption. This does not claim calibration or command the gimbal.
Raw resized RGB has rectified=false and calibration_quality=approximate.

Agent must keep strict geometry as its default and explicitly opt into this
profile (suggested allow_approximate_geometry), validate/log assumptions and
check **both** rectified and calibration_quality. Calibrated CameraInfo with
approximate extrinsics may have rectified=true but calibration_quality=approximate.
Fully calibrated CameraInfo/TF yields rectified=true/calibration_quality=calibrated.
All modes still require valid positive K, rigid rotation, fresh synchronized
exposure, epoch consistency and JPEG/image-size agreement.

Robot selects the newest cached fresh image bracketed by odometry, waiting at
most 80 ms for callbacks. Source age limit is currently 0.4 s, sync bound 50 ms;
age is rechecked after JPEG assembly. Agent applies age plus transfer elapsed
time <=0.5 s and sync <=50 ms. Robot/Agent wall clocks need not be synchronized.

## Errors and observation retry

Non-2xx JSON has `{ok:false,error:"..."}`; machine-readable observation errors
also contain error_code and retryable. Preserve these fields in the Client rather
than reducing them to an opaque exception string.

| HTTP | Meaning | Action |
| --- | --- | --- |
| 400 | Invalid request types/numbers/UUID | Correct request; do not retry as sensor absence |
| 403 | Nonlocal operator request or invalid operator token | Operator-only facility; Agent must not use it |
| 404 / 405 | Unknown endpoint/task or wrong method | Correct caller |
| 409 | Ownership, readiness, epoch, busy, motion failure or request-ID conflict | Inspect error/task/health; not blanket retryable |
| 503 | Bridge unavailable, pending duplicate or temporary observation absence | Retry observations only with the explicit code below |
| 504 | Server motion/landing wait expired | Reconcile task/stop/landing; not proof of stopped |

Observation-specific responses:
- HTTP 503: `error_code:"observation_unavailable",retryable:true`.
  Bounded Agent retry: initially 0.5 s total, 50–100 ms spacing.
- HTTP 409: `error_code:"localization_epoch_changed",retryable:false`.
  Discard old mission coordinates.
- HTTP 422: `error_code:"invalid_observation",retryable:false`.
  Reject invalid geometry/calibration.

Retry must not block heartbeat, cancellation or flight checks. Persistent absence
ends the attempt; epoch changes and invalid geometry are not missing-frame retries.

## Agent integration sequence and outstanding adaptation

1. Validate capabilities, motion tolerances, geometry opt-in and DA3 on observation
   before acquiring authority. In supervised mode wait for the operator's completed
   takeoff and current stop; do not compete with another Agent session.
2. Acquire a session and heartbeat independently. For supervised attachment,
   retain/acknowledge initialization and skip takeoff. For a separately configured
   Agent-operated ground start, init and explicitly select software takeoff.
3. Convert B into public world coordinates, submit navigation with saved epoch,
   retain task_id, and capture/infer concurrently with health/task polling.
4. Save exposure P including yaw/epoch/frame_id. On detection, cancel B and wait
   for cancelled/arrived with stopped:true, then recheck current stop/authority.
5. Navigate to full P, wait for arrival and current stop, execute TRACK actions via
   the combined relative endpoint, waiting for each completion/current stop.
6. Return to the same full P, then submit a new navigation to the unfinished B.
   Route index, target identity/deduplication and detection-result age belong to Agent.
7. Agent may land on mission completion: keep its lease, POST /land, confirm
   completion, then release. If leaving landing to the operator, finish/cancel
   work and confirm stopping before release. If the operator preempts, stop the
   mission without reacquiring or attempting another landing with the retired lease.

The current Agent checkout still requires:
- explicit approximate-geometry decoding/metadata logging;
- phase-aware health checks and bounded current-stop waits between movements;
- structured observation error handling and bounded retry;
- software takeoff opt-in if Agent will arm/take off from the ground;
- accepted-landing handling across epoch changes and fresh mission initialization;
- retry IDs preserved for uncertain requests; use only supported motion endpoints;
- acceptance of zero-Z reference semantics and logging sufficient for XYZ diagnosis.
- supervised airborne attachment and recognition of operator landing/retired lease,
  without automatic takeoff or reacquisition; retain Agent's normal final landing.

Compared with the original contract, endpoint shapes and units are retained.
Additive capabilities/status/error metadata and software takeoff are backward
compatible only when clients handle them by phase. Zero-Z and pose-window stopping
are explicit semantic corrections; unconditional planner_ok/rectified checks in
the old Agent are incompatible with the current deployed profile.

Robot-side mock-FCU/real-EGO and physical operator tests are recorded in robot-002.
On 2026-09-10 the user reported the latest cancellation test passed and requested
moving on to Agent integration. Earlier logs still show altitude tracking offsets;
a successful API action is not a guarantee of zero altitude error. Agent/Robot
joint model-driven mission validation remains pending. Do not substitute the
Robot's standalone test route for an actual Agent integration run.
