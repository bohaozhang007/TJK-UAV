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
- `center_tolerance=0.04`: center error relative to each image dimension.
- `size_tolerance=0.05`: acceptable occupancy is 0.35–0.45.
- `max_steps=30`, `timeout_s=180`, `settle_s=0.5`.
- `tracker_url=http://192.168.31.66:8791`, `tracker_timeout_s=30`.

The tracker API is the one in `app/tracker/server.py`:

1. `POST /init` once with `{image: base64_png, box: [x1,y1,x2,y2]}`.
2. After each adjustment/settling interval, capture a newly received frame and
   `POST /track` with `{image: base64_png}`. No box or reinitialization is sent.
3. Use the returned `box` to calculate center error and occupancy. Stop on
   `found=false`, invalid results or request failures. There is no ORB fallback.
4. Finish after two consecutive tracked frames satisfy both tolerances.

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
`track_count`, `tracker_url`, and `actions`. A failed camera command is not retried. If `box_confirmed` is false,
the box predates the last unconfirmed adjustment and must not be treated as a
current detection. Out-of-range targets, tracking loss and exhausted limits
stop further adjustment; no automatic return-to-start movement is attempted.

In the console, after `init`:

```text
autofocus X1 Y1 X2 Y2
```

The console captures the current frame, so these coordinates must refer to its
native resolution and a target still at that position. Use the Python function
when passing a specific detection image and its box. Ctrl-C during automatic
framing stops further camera commands and returns a failure result. An already
sent absolute gimbal movement may still complete.

Restart the console after updating code. Flight or OFFBOARD is not required
for camera adjustment.
