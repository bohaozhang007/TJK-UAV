# I7 camera controls

`rotate` controls the aircraft. `gimbal_yaw`, `gimbal_pitch`, `zoom` and
`autofocus` control only the camera/gimbal.

## Automatic target framing

```python
result = controller.autofocus(img, box=(x1, y1, x2, y2))
```

`img` is a recent `uint8` BGR image with shape `(height, width, 3)` from this
camera. `box` is in that image's pixel coordinates, not normalized coordinates.
Use the camera's native resolution. The client sends a lossless full-size PNG
and stops if live frame dimensions change. It no longer uses ORB features or
requires a 24-pixel minimum box. SAM2 still performs its own model preprocessing
inside the tracker service and returns boxes in the original image coordinates.

This function centers and enlarges/reduces the target; it is not a lens-sharpness
autofocus command. It calls the existing relative `gimbal_yaw`, `gimbal_pitch`
and absolute `zoom` functions. Fresh joint feedback is used for angle limits.
No aircraft motion commands are issued.

Defaults:

- `target_ratio=0.4`: `max(box_width/image_width, box_height/image_height)`.
- `center_tolerance=0.06`: center error up to 6% of each image dimension.
- `size_tolerance=0.08`: acceptable occupancy is 0.32–0.48, allowing tracking-box jitter.
- `max_steps=30`, `timeout_s=180`, `settle_s=0.5`.
- `hold_s=1.0`: after successful framing, hold for one second, then restore
  the yaw, pitch and zoom measured before this call's tracker initialization.
- `tracker_url=http://192.168.31.66:8791`, `tracker_timeout_s=30`.

The tracker API is the one in `app/tracker/server.py`:

1. `POST /init` once with `{image: base64_png, box: [x1,y1,x2,y2]}`.
2. After each adjustment/settling interval, capture a newly received frame and
   `POST /track` with `{image: base64_png}`. No box or reinitialization is sent.
3. Use the returned `box` to calculate center error and occupancy. Stop on
   `found=false`, invalid results or request failures. There is no ORB fallback.
4. After two consecutive tracked frames satisfy both tolerances, hold for
   `hold_s`, restore the initial zoom and angles, and verify their feedback.
   Pose verification tolerance is 0.5 degrees; zoom tolerance is 0.05x.
   `timeout_s` limits tracking/framing; the hold and bounded camera restoration
   requests happen afterwards. A restoration failure returns `ok=false` even
   when framing succeeded.

Control parameters live in `ros/i7_nav/config/i7_nav.yaml` under `autofocus`.
Following v21, convert pixel error using reference dimensions (640x360):

```text
yaw_delta   = (cx - width/2) * yaw_deg_per_pixel * reference_width/width
pitch_delta = (height/2 - cy) * pitch_deg_per_pixel * reference_height/height
```

Both deltas are multiplied by `damping * reference_zoom/current_zoom` and
clamped to separate per-axis maximum steps (3 degrees by default). These are
initial tuning gains, not measured camera intrinsics; zoom compensation is an
approximation. Pitch is in degrees/pixel, not the vertical cm/pixel used for
aircraft movement in v21. Adjust the axis with the larger normalized error,
then track again. Once centered, request `zoom * target_ratio/current_ratio`,
bounded to at most 25% up / 20% down per step and the camera's 1–160 range.

The service currently maintains one global target; dedicate this tracker
instance to this operation and do not let another client call `/init` while
autofocus is running. Tracking requests are not automatically retried because
they advance model state. Frame freshness refers to reception time; increase
`settle_s` if the video stream has significant buffering. Moving targets and
occlusion may prevent convergence within the configured limits.

The returned dictionary includes `ok`, `message`, the latest `box` in input
image coordinates, `box_confirmed`, `center_error`, `occupancy`, `zoom` and
`track_count`, `tracker_url`, and `actions`. `initial_camera` records this call's
starting pose/zoom. `focus_reached` and `focus` retain the framing result before
returning; `restoration` contains the restore target, final readings and errors.
The top-level `zoom` is the final measured zoom after restoration. The box,
occupancy and center error describe the focused view and `box_confirmed` is
false after restoration because that box no longer describes the current view.
A failed camera movement command is not resent. On confirmation timeout, the
client performs one bounded read-only verification phase on a fresh UDP socket
(up to another `camera_control.timeout_s`). It retains the original absolute
target: both gimbal axes must be within 0.5 degrees for three consecutive
samples, or zoom must equal the requested value and have stopped. A successful
verification allows framing to continue even if the movement ACK was lost;
camera command results include `verified_after_timeout` and
`command_acknowledged`. Failed verification stops framing and reports the
last measured status and target, when available. Explicit rejection or camera
faults stop immediately. If `box_confirmed` is false,
the box predates the last unconfirmed adjustment and must not be treated as a
current detection. Out-of-range targets, tracking loss and exhausted limits
stop further adjustment. Restoration runs only after successful framing;
failed/interrupted framing does not automatically move back.

In the console, after `init`:

```text
autofocus X1 Y1 X2 Y2
autofocus /path/to/image.png X1 Y1 X2 Y2
autofocus "/path/with spaces/image.png" X1 Y1 X2 Y2
```

With four arguments, the console captures the current native BGR frame directly,
without JPEG encoding. With five arguments, the first is an image file path
(relative paths and `~` are supported); that image and the four box coordinates
are sent to tracker init. Coordinates refer to the selected image's native pixels.
Neither mode resizes the image; tracker transport uses lossless PNG. A JPEG input
file already contains its original compression loss. Subsequent tracking uses
live camera frames, whose resolution must match the initialization image.
The Python function still accepts `controller.autofocus(img, box)` with a BGR array.
Ctrl-C during automatic
framing stops further camera commands and returns a failure result. An already
sent absolute gimbal movement may still complete.

Restart the console after updating code. Flight or OFFBOARD is not required
for camera adjustment.

I7 camera UDP diagnostics are written automatically to `logs/k40t_udp.log`
under the repository (JSON lines, 10 MiB per file, three rotated backups).
Each request has a request ID, wall timestamp, elapsed time, PID, peer, command
and target. Events record the local UDP endpoint, transmitted/received packet
hex and sequence numbers, ACK acceptance/rejection, ignored-packet reasons,
measured pose errors, zoom status, timeout and final outcome. Read-only
verification requests have `verification: true`. These logs observe packets
delivered to the client socket; they cannot prove whether packets were sent
to a closed/other port or dropped before reaching it.

```bash
tail -f /home/jkhk/TJK-UAV/logs/k40t_udp.log
```
