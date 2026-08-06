# 1. search 高度扩展：h, h+50, h-50
# 2. 增大 gd 置信度

import argparse
import os
import sys
from typing import Optional

import cv2
import numpy as np
from PIL import Image

SRC_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, SRC_ROOT)

GROUNDING_SAM_ROOT = r'C:\Users\colab999\Desktop\project\Grounded_SAM_2_main'
sys.path.insert(0, os.path.join(GROUNDING_SAM_ROOT, "grounding_dino"))

import groundingdino.datasets.transforms as T
from groundingdino.util.inference import load_model, predict
from third_party.sam2_stream import Sam2VideoPredictor, SAM2Config
from robot_client.base import BaseClient
from utils import show_fig


STABLE = 'stable'
FORWARD = 'forward'
BACKWARD = 'backward'
UP = 'up'
DOWN = 'down'
RIGHT = 'right_rotate'
LEFT = 'left_rotate'

class TJKAgent:
    def __init__(
        self,
        client: BaseClient,
        vis_dir: str = './tjk_vis',
        save_vis: bool = True,
        save_depth: bool = False,
    ):
        self.client = client
        
        self.vis_dir = vis_dir
        self.save_vis = save_vis
        self.save_depth = save_depth

        # max step distance
        self.max_fb_step_cm = 150 # 60 #150.0
        self.max_z_step_cm =  80 # 40 #80.0

        # min step distance
        self.min_fb_step_cm = 50 #20 #50.0
        self.min_z_step_cm = 20.0
        self.min_rotate_deg = 5.0

        # fixed action
        self.backward_ratio = 0.25
        self.rotate_30 = 30

        # safe thresh
        self.safe_z_cm = 10.0
        self.depth_safe_distance_cm = 100.0

        # img setting
        self.h_fov = 70 # degrees
        self.img_height = 480
        self.img_width = 640
        self.horizontal_center = self.img_width / 2
        self.vertical_center = self.img_height / 2

        # stop thresh
        self.target_stop_ratio = 0.40

        # max exec iters
        self.max_exec_iters = 200

        sam_cfg = SAM2Config()
        sam_cfg.VIDEO_HEIGHT = self.img_height
        sam_cfg.VIDEO_WIDTH = self.img_width
        self.tracker = Sam2VideoPredictor(sam_cfg)
        self.tracker.set_vis_mode(self.save_vis)
        self.tracker.set_vis_dir(self.vis_dir)

        self.grounding_device = "cuda"
        self.grounding_model = load_model(
            os.path.join(
                GROUNDING_SAM_ROOT,
                "grounding_dino",
                "groundingdino",
                "config",
                "GroundingDINO_SwinB_cfg.py",
            ),
            os.path.join(GROUNDING_SAM_ROOT, "gdino_checkpoints", "groundingdino_swinb_cogcoor.pth"),
            device=self.grounding_device,
        )
        self.grounding_transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        self.gd_box_threshold = 0.5 #0.35
        self.gd_text_threshold = 0.25

    def detect_with_grounding_dino(self, image_rgb, text_prompt):
        image_tensor, _ = self.grounding_transform(Image.fromarray(image_rgb), None)
        boxes, confidences, labels = predict(
            model=self.grounding_model,
            image=image_tensor,
            caption=text_prompt,
            box_threshold=self.gd_box_threshold,
            text_threshold=self.gd_text_threshold,
            device=self.grounding_device,
        )
        if len(boxes) == 0:
            return None

        best_idx = int(confidences.argmax().item())
        cx, cy, box_w, box_h = boxes[best_idx].numpy()
        image_h, image_w = image_rgb.shape[:2]
        box_xyxy = np.array(
            [
                (cx - box_w / 2) * image_w,
                (cy - box_h / 2) * image_h,
                (cx + box_w / 2) * image_w,
                (cy + box_h / 2) * image_h,
            ],
            dtype=np.float32,
        )
        box_xyxy[[0, 2]] = np.clip(box_xyxy[[0, 2]], 0, image_w - 1)
        box_xyxy[[1, 3]] = np.clip(box_xyxy[[1, 3]], 0, image_h - 1)
        return box_xyxy, float(confidences[best_idx]), labels[best_idx]

    def connect(self):
        self.client.start()
        frame = self.client.capture(include_depth=False)
        height, width = frame.shape[:2]

        self.img_height = height
        self.img_width = width
        self.horizontal_center = width / 2
        self.vertical_center = height / 2

        self.tracker.set_img_size(height, width)
        print(f"[ROBOT-INFO] RGB resolution: {width}x{height}")

        pose = self.client.get_pose()

        x = float(pose["x"])
        y = float(pose["y"])
        z = float(pose["z"])
        yaw = float(pose["yaw"])
        pose_str = f"x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}"

        print(f"[ROBOT-INFO] Connected to {self.client.base_url}; pose: {pose_str}")

    def save_track_depth(self, depth_raw, frame_idx):
        depth_path = os.path.join(self.vis_dir, f"sam2_trak_{frame_idx}.npy")
        np.save(depth_path, depth_raw)
        print(f"[DEPTH] Saved raw centimeter depth in {depth_path}")

    def get_nearest_path_depth_cm(self, depth_raw, mask):
        mask = np.asarray(mask, dtype=bool).squeeze()
        if mask.ndim != 2:
            raise RuntimeError(f"Expected a 2D target mask, got shape={mask.shape}")
        if depth_raw.shape != mask.shape:
            depth_raw = cv2.resize(
                depth_raw,
                (mask.shape[1], mask.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        ys, xs = np.where(mask)
        if xs.size == 0 or ys.size == 0:
            raise RuntimeError("Target mask is empty")

        height, width = mask.shape
        roi_width = max(int(xs.max() - xs.min() + 1), width // 2)
        roi_height = max(int(ys.max() - ys.min() + 1), height // 2)
        x1 = (width - roi_width) // 2
        y1 = (height - roi_height) // 2
        roi_depth = depth_raw[y1:y1 + roi_height, x1:x1 + roi_width]

        valid_path_depths = roi_depth[np.isfinite(roi_depth) & (roi_depth > 0)]
        if valid_path_depths.size == 0:
            raise RuntimeError("No valid positive depth values inside the forward-path ROI")
        return float(np.percentile(valid_path_depths, 1))

    def check_box_offset(self, bbox):
        # bbox: [[x1, y1], [x2, y2]]
        box_center = (bbox[0, :] + bbox[1, :]) / 2
        horizontal_offset = box_center[0] - self.horizontal_center
        # Image y grows downward, so positive vertical_offset means the object is above center.
        vertical_offset = self.vertical_center - box_center[1]
        return horizontal_offset, vertical_offset

    def get_bbox_state(self, bbox):
        w = bbox[1, 0] - bbox[0, 0]
        h = bbox[1, 1] - bbox[0, 1]
        box_ratio = max(w / self.img_width, h / self.img_height)
        horizontal_offset, vertical_offset = self.check_box_offset(bbox)
        return {
            "box_ratio": float(box_ratio),
            "horizontal_offset": float(horizontal_offset),
            "vertical_offset": float(vertical_offset),
        }

    def prepare_rotate_action_deg(self, horizontal_offset):
        angle_to_rotate = (horizontal_offset / self.img_width) * self.h_fov
        if angle_to_rotate > 0:
            action = RIGHT
        else:
            action = LEFT
        return angle_to_rotate, action
    
    def exec_rotate_action_deg(self, angle_to_rotate, action):
        dyaw = float(angle_to_rotate)
        print(f"[ACTION] Robot {action} {abs(dyaw):.2f} deg.")
        result = self.client.move_relative(dx=0.0, dy=0.0, dz=0.0, dyaw=dyaw)
        return result
    
    def prepare_z_action_cm(self, vertical_offset):
        dz_cm = (vertical_offset / self.img_height) * (2.0 * self.max_z_step_cm)
        dz_cm = float(np.clip(dz_cm, -self.max_z_step_cm, self.max_z_step_cm))
        if dz_cm > 0:
            action = UP
        else:
            action = DOWN
        return dz_cm, action
    
    def exec_z_action_cm(self, dz_cm, action):
        print(f"[ACTION] Robot move {action} {abs(dz_cm):.2f} cm.")
        result = self.client.move_relative(dx=0.0, dy=0.0, dz=dz_cm, dyaw=0.0)
        return result

    def get_backward_step_cm(self):
        return -self.backward_ratio * self.max_fb_step_cm
    
    def prepare_fb_action_cm(self, box_ratio):
        target = self.target_stop_ratio
        deadband = self.min_fb_step_cm / self.max_fb_step_cm * target

        if box_ratio > target + deadband:
            step_cm = self.get_backward_step_cm()
            action = BACKWARD
        elif box_ratio < target - deadband:
            progress = box_ratio / target
            step_cm = self.max_fb_step_cm * (1.0 - progress)
            action = FORWARD
        else:
            step_cm = 0.0
            action = STABLE

        return step_cm, action
    
    def exec_fb_action_cm(self, step_cm, action):
        print(f"[ACTION] Robot move {action} {abs(step_cm):.2f} cm.")
        result = self.client.move_relative(dx=step_cm, dy=0.0, dz=0.0, dyaw=0.0)
        return result

    def get_current_z_cm(self) -> Optional[float]:
        pose = self.client.get_pose()
        return float(pose["z"])

    def move_to_target(self):
        success = False
        iterations = 0
        while True:
            iterations += 1
            if iterations > self.max_exec_iters:
                print(f"[WARN] Stop target loop after {self.max_exec_iters} iterations to avoid infinite motion.")
                break

            frame_rgb, depth_raw = self.client.capture(include_depth=True)  
            track_frame_idx = self.tracker.frame_idx
            bbox, mask = self.tracker.track_with_mask(frame_rgb)
            if self.save_depth:
                self.save_track_depth(depth_raw, track_frame_idx)

            # get box state
            state = self.get_bbox_state(bbox)
            box_ratio = state["box_ratio"]
            horizontal_offset = state["horizontal_offset"]
            vertical_offset = state["vertical_offset"]

            # 1. rotate
            angle_to_rotate, action = self.prepare_rotate_action_deg(horizontal_offset)
            if abs(angle_to_rotate) > self.min_rotate_deg:
                self.exec_rotate_action_deg(angle_to_rotate, action)
                continue
            else:
                print(f"[STALL] angle is {angle_to_rotate} and Stalled.")

            # 2. z
            dz_cm, action = self.prepare_z_action_cm(vertical_offset)
            z_blocked_by_safety  = False
            if action == DOWN:
                cur_z_cm = self.get_current_z_cm()
                z_blocked_by_safety = cur_z_cm <= self.safe_z_cm
                if not z_blocked_by_safety:
                    dz_cm = max(dz_cm, self.safe_z_cm - cur_z_cm)

            if not z_blocked_by_safety:
                if abs(dz_cm) > self.min_z_step_cm:
                    self.exec_z_action_cm(dz_cm, action)
                    continue            
                else:
                    print(f"[STALL] dz_cm is {dz_cm} and Stalled.")

            # 3. fb
            step_cm, action = self.prepare_fb_action_cm(box_ratio)
            if abs(step_cm) > self.min_fb_step_cm:
                if action == FORWARD:
                    x1 = step_cm
                    path_depth_cm = self.get_nearest_path_depth_cm(depth_raw, mask)
                    x2 = max(0.0, path_depth_cm - self.depth_safe_distance_cm)
                    step_cm = min(x1, x2)
                    print(
                        f"[DEPTH] path_p01={path_depth_cm:.2f} cm, safe={self.depth_safe_distance_cm:.2f} cm, "
                        f"x1={x1:.2f} cm, x2={x2:.2f} cm, forward={step_cm:.2f} cm."
                    )
                    if step_cm <= self.min_fb_step_cm:
                        print(
                            f"[STALL] Depth-limited forward step {step_cm:.2f} cm is not executable "
                            f"(minimum {self.min_fb_step_cm:.2f} cm); stop at the safe distance."
                        )
                    else:
                        self.exec_fb_action_cm(step_cm, action)
                        continue
                else:
                    self.exec_fb_action_cm(step_cm, action)
                    continue
            else:
                print(f"[STALL] dx is {step_cm} cm and Stalled.")

            # 4. all stall
            success = True
            break

        if success:
            print("[RES] Successfully moved to target.")
        else:
            print("[RES] Stopped before confirmed target because motion appears stalled or loop limit was reached.")

        self.client.end()

    def run(self, text_prompt):
        self.init_tracker(text_prompt)
        self.move_to_target()

    def init_tracker(self, text_prompt):
        # Search at h, h+50 cm, and h-50 cm, one full turn at each height.
        cur_frame = None
        target_box = None
        for height_idx, height_offset_cm in enumerate((0, 50, -50)):
            if height_idx == 1:
                self.exec_z_action_cm(50, UP)
            elif height_idx == 2:
                self.exec_z_action_cm(-100, DOWN)

            for i in range(12):
                # Search only needs RGB. Depth estimation starts in move_to_target().
                cur_frame = self.client.capture(include_depth=False)
                detection = self.detect_with_grounding_dino(cur_frame, text_prompt)
                candidate_idx = height_idx * 12 + i

                if detection is not None:
                    target_box, confidence, label = detection
                    show_fig(cur_frame, f"{self.vis_dir}/candidate_{candidate_idx}.png", box_coords=target_box)
                    print(
                        f"[SEARCH] Found {label} with confidence={confidence:.3f} "
                        f"at height offset={height_offset_cm:+d} cm, heading={i * 30} deg."
                    )
                    break

                show_fig(cur_frame, f"{self.vis_dir}/candidate_{candidate_idx}.png")
                self.exec_rotate_action_deg(self.rotate_30, RIGHT)

            if target_box is not None:
                break

        if target_box is None:
            self.client.land()
            raise RuntimeError("没有找到目标")

        # Initialize SAM2 tracking directly with GroundingDINO's xyxy box.
        bbox = self.tracker.track(cur_frame, box=target_box)

        state = self.get_bbox_state(bbox)
        horizontal_offset = state["horizontal_offset"]
        angle_to_rotate, action = self.prepare_rotate_action_deg(horizontal_offset)
        self.exec_rotate_action_deg(angle_to_rotate, action)


def build_client(
    client_name: str,
    server_host: str,
    server_port: int,
    http_timeout_s: float,
) -> BaseClient:
    if client_name == "tello":
        from robot_client.tello import TelloClient

        return TelloClient(server_host, server_port, http_timeout_s)
    if client_name == "ue":
        try:
            from robot_client.ue import UEClient
        except ModuleNotFoundError as exc:
            if exc.name == "robot_client.ue":
                raise NotImplementedError("UEClient is not implemented yet") from exc
            raise
        return UEClient(server_host, server_port, http_timeout_s)
    raise ValueError(f"unsupported client: {client_name}")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vdir",
        type=str,
        default="./tjk_vis",
        help="Directory for visualization images and depth data (default: ./tjk_vis)",
    )
    parser.add_argument(
        "--obj",
        type=str,
        default="street lamp",
        help="Object description used by GroundingDINO (default: white ball)",
    )
    parser.add_argument(
        "--client",
        choices=("tello", "ue"),
        default="ue",
        help="Robot client to use: tello or ue (default: ue)",
    )
    return parser


def main():
    args = _build_arg_parser().parse_args()
    os.makedirs(args.vdir, exist_ok=True)
    print(f"[MODE] GroundingDINO search: {args.obj!r}")

    client = build_client(
        args.client,
        server_host="127.0.0.1",
        server_port=8765,
        http_timeout_s=180.0,
    )
    tjkAgent = TJKAgent(client=client, vis_dir=args.vdir)
    tjkAgent.connect()
    tjkAgent.run(args.obj)

if __name__ == "__main__":
    main()
