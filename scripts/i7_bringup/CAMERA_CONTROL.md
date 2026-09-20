# I7 camera controls

`rotate` controls the aircraft. `gimbal_yaw`, `gimbal_pitch`, `zoom` and
`autofocus` control only the camera/gimbal.

## Automatic target framing

```python
result = controller.autofocus(img, box=(x1, y1, x2, y2))
```

`img` is a recent `uint8` BGR image with shape `(height, width, 3)` from this
camera. `box` is in that image's pixel coordinates, not normalized coordinates.
Use the camera's native resolution; tracking does not resize images and stops
if the live frame dimensions differ from the input. Use a box at least
24 pixels wide and high with enough distinct visual texture.

This function centers and enlarges/reduces the target; it is not a lens-sharpness
autofocus command. It calls the existing relative `gimbal_yaw`, `gimbal_pitch`
and absolute `zoom` functions. Fresh joint feedback is used for angle limits.
No aircraft motion commands are issued.

Defaults:

- `target_ratio=0.4`: `max(box_width/image_width, box_height/image_height)`.
- `center_tolerance=0.04`: center error relative to each image dimension.
- `size_tolerance=0.05`: acceptable occupancy is 0.35–0.45.
- `max_steps=30`, `timeout_s=120`, `settle_s=0.5`.

The method matches target features between frames, starts with small angle
steps to estimate visual response without calibrated intrinsics, centers the
target, and changes zoom in steps of at most 25% up / 20% down. It confirms
framing on two observations. It assumes an approximately stationary target
and camera platform; featureless, repetitive, occluded, or rapidly moving
targets may fail to track. Frame freshness refers to reception time; increase
`settle_s` if the video stream has significant buffering.

The returned dictionary includes `ok`, `message`, the latest `box` in input
image coordinates, `box_confirmed`, `center_error`, `occupancy`, `zoom` and
`actions`. A failed camera command is not retried. If `box_confirmed` is false,
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
