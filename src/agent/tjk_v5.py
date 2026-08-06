# 1. 抽象出无人机状态
# 2. 各阶段抽象为 search，track，return 等单独的函数
# 3. 添加了 scan 操作
# 4. yaml 配置参数

import argparse
import datetime as dt
import logging
import os
import sys
import time
from enum import Enum
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml
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


def _load_config(config_path: str | Path) -> dict:
    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Agent config file not found: {path}")
    try:
        with path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Failed to load required agent config: {path}") from exc
    if not isinstance(config, dict):
        raise ValueError(f"Agent config must contain a mapping: {path}")
    return config


def _config_section(config: dict, section_name: str) -> dict:
    section = config.get(section_name)
    if not isinstance(section, dict):
        raise ValueError(f"Missing or invalid config section: {section_name}")
    return section


def _config_number(
    section: dict,
    key: str,
    *,
    integer: bool = False,
    minimum: float | None = None,
    maximum: float | None = None,
):
    if key not in section:
        raise ValueError(f"Missing config value: {key}")
    value = section[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Config value {key} must be numeric, got {value!r}")
    if integer and float(value) != int(value):
        raise ValueError(f"Config value {key} must be an integer, got {value!r}")
    result = int(value) if integer else float(value)
    if minimum is not None and result < minimum:
        raise ValueError(f"Config value {key} must be >= {minimum}, got {result}")
    if maximum is not None and result > maximum:
        raise ValueError(f"Config value {key} must be <= {maximum}, got {result}")
    return result


class AgentAction(str, Enum):
    STABLE = "stable"
    FORWARD = "forward"
    BACKWARD = "backward"
    UP = "up"
    DOWN = "down"
    RIGHT = "right_rotate"
    LEFT = "left_rotate"
    XYZ_YAW_HYBRID = "xyz_yaw_hybrid"


class AgentState(str, Enum):
    SEARCH = "SEARCH"
    TRACK = "TRACK"
    SCAN = "SCAN"
    RETURN_WAYPOINT = "RETURN_WAYPOINT"


def _timestamped_dir(prefix: str, timestamp: str) -> Path:
    return Path(f"{Path(prefix).expanduser()}_{timestamp}")


def _timestamped_log_path(prefix: str, timestamp: str) -> Path:
    path = Path(prefix).expanduser()
    if path.suffix:
        return path.with_name(f"{path.stem}_{timestamp}{path.suffix}")
    return Path(f"{path}_{timestamp}.log")


def _configure_logger(log_path: Path) -> logging.Logger:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("tjk_v5")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)
    logger.addHandler(file_handler)
    return logger

class TJKAgent:
    def __init__(
        self,
        client: BaseClient,
        config_path: str | Path,
        vis_dir: str = './tjk_vis',
        save_vis: bool = True,
        save_depth: bool = False,
        logger: logging.Logger | None = None,
    ):
        self.client = client
        self.logger = logger
        self._command_idx = 0
        self.vis_dir = vis_dir
        self.save_vis = save_vis
        self.save_depth = save_depth
        self.state = AgentState.SEARCH
        self.waypoint_pose = None
        self.last_track_observation = None
        self.config_path = Path(config_path).expanduser().resolve()
        config = _load_config(self.config_path)
        motion_config = _config_section(config, "motion")
        search_config = _config_section(config, "search")
        safety_config = _config_section(config, "safety")
        track_config = _config_section(config, "track")
        scan_config = _config_section(config, "scan")
        detection_config = _config_section(config, "detection")

        # max step distance
        self.max_fb_step_cm = _config_number(
            motion_config, "max_fb_step_cm", minimum=1e-6
        )
        self.max_z_step_cm = _config_number(
            motion_config, "max_z_step_cm", minimum=1e-6
        )
        self.max_rotate_deg = _config_number(
            motion_config, "max_rotate_deg", minimum=1e-6
        )

        # min step distance
        self.min_fb_step_cm = _config_number(
            motion_config, "min_fb_step_cm", minimum=0.0
        )
        self.min_z_step_cm = _config_number(
            motion_config, "min_z_step_cm", minimum=0.0
        )
        self.min_rotate_deg = _config_number(
            motion_config, "min_rotate_deg", minimum=0.0
        )
        if self.min_fb_step_cm > self.max_fb_step_cm:
            raise ValueError("motion.min_fb_step_cm cannot exceed max_fb_step_cm")
        if self.min_z_step_cm > self.max_z_step_cm:
            raise ValueError("motion.min_z_step_cm cannot exceed max_z_step_cm")
        if self.min_rotate_deg > self.max_rotate_deg:
            raise ValueError("motion.min_rotate_deg cannot exceed max_rotate_deg")

        # fixed action
        self.backward_ratio = _config_number(
            motion_config,
            "backward_ratio",
            minimum=0.0,
            maximum=1.0,
        )
        self.search_rotation_step_deg = _config_number(
            search_config,
            "rotation_step_deg",
            minimum=1.0,
            maximum=360.0,
        )
        search_rotation_count = 360.0 / self.search_rotation_step_deg
        if abs(search_rotation_count - round(search_rotation_count)) > 1e-6:
            raise ValueError(
                "search.rotation_step_deg must divide 360 exactly, got "
                f"{self.search_rotation_step_deg}"
            )
        self.search_rotation_count = int(round(search_rotation_count))
        self.search_height_offset_cm = _config_number(
            search_config,
            "height_offset_cm",
            integer=True,
            minimum=0.0,
        )

        # safe thresh
        self.safe_z_cm = _config_number(
            safety_config, "safe_z_cm", minimum=0.0
        )
        self.depth_safe_distance_cm = _config_number(
            safety_config,
            "depth_safe_distance_cm",
            minimum=0.0,
        )

        # img setting
        self.img_height = 480
        self.img_width = 640
        self.horizontal_center = self.img_width / 2
        self.vertical_center = self.img_height / 2

        # stop thresh
        self.target_stop_ratio = _config_number(
            track_config,
            "target_stop_ratio",
            minimum=1e-6,
            maximum=1.0,
        )

        # max exec iters
        self.max_exec_iters = _config_number(
            track_config,
            "max_exec_iters",
            integer=True,
            minimum=1.0,
        )

        # scan settings
        self.scan_num_points = _config_number(
            scan_config,
            "num_points",
            integer=True,
            minimum=3.0,
        )
        self.scan_radius_cm = _config_number(
            scan_config,
            "radius_cm",
            minimum=1e-6,
        )

        sam_cfg = SAM2Config()
        sam_cfg.VIDEO_HEIGHT = self.img_height
        sam_cfg.VIDEO_WIDTH = self.img_width
        self.tracker = Sam2VideoPredictor(sam_cfg)
        self.tracker.set_vis_mode(self.save_vis)
        self.tracker.set_vis_dir(self.vis_dir)

        if "device" not in detection_config:
            raise ValueError("Missing config value: detection.device")
        self.grounding_device = detection_config["device"]
        if not isinstance(self.grounding_device, str) or not self.grounding_device:
            raise ValueError("Config value detection.device must be a non-empty string")
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
        self.gd_box_threshold = _config_number(
            detection_config,
            "box_threshold",
            minimum=0.0,
            maximum=1.0,
        )
        self.gd_text_threshold = _config_number(
            detection_config,
            "text_threshold",
            minimum=0.0,
            maximum=1.0,
        )

    def _log(self, message: str) -> None:
        if self.logger is None:
            print(message)
        else:
            self.logger.info(message)

    def _log_sam2_timing(self, phase: str, frame_idx: int, measured_total_s: float) -> None:
        timing = getattr(self.tracker, "last_timing", {})
        self._log(
            f"[TIMING] phase={phase} frame={frame_idx} "
            f"sam2_inference_s={float(timing.get('inference_s', measured_total_s)):.4f} "
            f"sam2_postprocess_s={float(timing.get('postprocess_s', 0.0)):.4f} "
            f"sam2_visualization_s={float(timing.get('visualization_s', 0.0)):.4f} "
            f"sam2_total_s={float(timing.get('total_s', measured_total_s)):.4f}"
        )

    def _log_decision_timing(
        self,
        iteration: int | str,
        started_at: float,
        action: AgentAction,
        command: str,
    ) -> None:
        self._log(
            f"[TIMING] phase=track iteration={iteration} "
            f"box_to_command_s={time.perf_counter() - started_at:.4f} "
            f"selected_action={action.value} command={command}"
        )

    def _execute_motion(self, action: AgentAction, command: str, operation):
        self._command_idx += 1
        command_idx = self._command_idx
        pose = self.client.get_pose()
        self._log(
            f"[COMMAND] id={command_idx} action={action.value} command={command} "
            f"pose=(x={float(pose['x']):.2f}, y={float(pose['y']):.2f}, "
            f"z={float(pose['z']):.2f}, yaw={float(pose['yaw']):.2f})"
        )
        started_at = time.perf_counter()
        try:
            result = operation()
        except Exception:
            self._log(
                f"[TIMING] command_id={command_idx} action={action.value} "
                f"drone_execution_s={time.perf_counter() - started_at:.4f} status=error"
            )
            raise
        self._log(
            f"[TIMING] command_id={command_idx} action={action.value} "
            f"drone_execution_s={time.perf_counter() - started_at:.4f} status=ok"
        )
        return result

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
        angle_to_rotate = (
            horizontal_offset / self.img_width
        ) * self.max_rotate_deg
        if angle_to_rotate > 0:
            action = AgentAction.RIGHT
        else:
            action = AgentAction.LEFT
        return angle_to_rotate, action
    
    def exec_rotate_action_deg(self, angle_to_rotate, action):
        dyaw = float(angle_to_rotate)
        return self._execute_motion(
            action,
            f"dyaw={dyaw:.2f}deg",
            lambda: self.client.move_relative(dx=0.0, dy=0.0, dz=0.0, dyaw=dyaw),
        )
    
    def prepare_z_action_cm(self, vertical_offset):
        dz_cm = (vertical_offset / self.img_height) * (2.0 * self.max_z_step_cm)
        dz_cm = float(np.clip(dz_cm, -self.max_z_step_cm, self.max_z_step_cm))
        if dz_cm > 0:
            action = AgentAction.UP
        else:
            action = AgentAction.DOWN
        return dz_cm, action
    
    def exec_z_action_cm(self, dz_cm, action):
        return self._execute_motion(
            action,
            f"dz={float(dz_cm):.2f}cm",
            lambda: self.client.move_relative(dx=0.0, dy=0.0, dz=dz_cm, dyaw=0.0),
        )

    def get_backward_step_cm(self):
        return -self.backward_ratio * self.max_fb_step_cm
    
    def prepare_fb_action_cm(self, box_ratio):
        target = self.target_stop_ratio
        deadband = self.min_fb_step_cm / self.max_fb_step_cm * target

        if box_ratio > target + deadband:
            step_cm = self.get_backward_step_cm()
            action = AgentAction.BACKWARD
        elif box_ratio < target - deadband:
            progress = box_ratio / target
            step_cm = self.max_fb_step_cm * (1.0 - progress)
            action = AgentAction.FORWARD
        else:
            step_cm = 0.0
            action = AgentAction.STABLE

        return step_cm, action
    
    def exec_fb_action_cm(self, step_cm, action):
        return self._execute_motion(
            action,
            f"dx={float(step_cm):.2f}cm",
            lambda: self.client.move_relative(dx=step_cm, dy=0.0, dz=0.0, dyaw=0.0),
        )

    def get_current_z_cm(self) -> Optional[float]:
        pose = self.client.get_pose()
        return float(pose["z"])

    def track(self):
        """Track the initialized target and adjust the drone pose toward it."""
        success = False
        iterations = 0
        while True:
            iterations += 1
            if iterations > self.max_exec_iters:
                print(f"[WARN] Stop target loop after {self.max_exec_iters} iterations to avoid infinite motion.")
                break

            capture_started = time.perf_counter()
            frame_rgb, depth_raw = self.client.capture(include_depth=True)
            self._log(
                f"[TIMING] phase=track iteration={iterations} "
                f"capture_rgb_depth_s={time.perf_counter() - capture_started:.4f}"
            )
            track_frame_idx = self.tracker.frame_idx
            sam2_started = time.perf_counter()
            bbox, mask = self.tracker.track_with_mask(frame_rgb)
            self._log_sam2_timing(
                "track",
                track_frame_idx,
                time.perf_counter() - sam2_started,
            )
            if self.save_depth:
                self.save_track_depth(depth_raw, track_frame_idx)

            # get box state
            decision_started = time.perf_counter()
            state = self.get_bbox_state(bbox)
            box_ratio = state["box_ratio"]
            horizontal_offset = state["horizontal_offset"]
            vertical_offset = state["vertical_offset"]
            self.last_track_observation = {
                "frame_rgb": frame_rgb,
                "depth_raw": depth_raw,
                "bbox": np.asarray(bbox).copy(),
                "mask": np.asarray(mask).copy(),
                "pose": self.client.get_pose(),
            }

            # 1. rotate
            angle_to_rotate, action = self.prepare_rotate_action_deg(horizontal_offset)
            if abs(angle_to_rotate) > self.min_rotate_deg:
                self._log_decision_timing(
                    iterations,
                    decision_started,
                    action,
                    f"dyaw={float(angle_to_rotate):.2f}deg",
                )
                self.exec_rotate_action_deg(angle_to_rotate, action)
                continue
            else:
                print(f"[STALL] angle is {angle_to_rotate} and Stalled.")

            # 2. z
            dz_cm, action = self.prepare_z_action_cm(vertical_offset)
            z_blocked_by_safety  = False
            if action == AgentAction.DOWN:
                cur_z_cm = self.get_current_z_cm()
                z_blocked_by_safety = cur_z_cm <= self.safe_z_cm
                if not z_blocked_by_safety:
                    dz_cm = max(dz_cm, self.safe_z_cm - cur_z_cm)

            if not z_blocked_by_safety:
                if abs(dz_cm) > self.min_z_step_cm:
                    self._log_decision_timing(
                        iterations,
                        decision_started,
                        action,
                        f"dz={float(dz_cm):.2f}cm",
                    )
                    self.exec_z_action_cm(dz_cm, action)
                    continue            
                else:
                    print(f"[STALL] dz_cm is {dz_cm} and Stalled.")

            # 3. fb
            step_cm, action = self.prepare_fb_action_cm(box_ratio)
            if abs(step_cm) > self.min_fb_step_cm:
                if action == AgentAction.FORWARD:
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
                        self._log_decision_timing(
                            iterations,
                            decision_started,
                            action,
                            f"dx={float(step_cm):.2f}cm",
                        )
                        self.exec_fb_action_cm(step_cm, action)
                        continue
                else:
                    self._log_decision_timing(
                        iterations,
                        decision_started,
                        action,
                        f"dx={float(step_cm):.2f}cm",
                    )
                    self.exec_fb_action_cm(step_cm, action)
                    continue
            else:
                print(f"[STALL] dx is {step_cm} cm and Stalled.")

            # 4. all stall
            self._log_decision_timing(
                iterations,
                decision_started,
                AgentAction.STABLE,
                "none",
            )
            success = True
            break

        if success:
            print("[RES] Successfully moved to target.")
        else:
            print("[RES] Stopped before confirmed target because motion appears stalled or loop limit was reached.")

        return success

    def _relative_pose_delta(self, target_pose, current_pose=None):
        """Convert a world-frame target pose to one body-relative command."""
        pose = self.client.get_pose() if current_pose is None else current_pose
        world_dx_cm = float(target_pose["x"]) - float(pose["x"])
        world_dy_cm = float(target_pose["y"]) - float(pose["y"])
        yaw_rad = np.radians(float(pose["yaw"]))
        body_dx_cm = float(
            np.cos(yaw_rad) * world_dx_cm + np.sin(yaw_rad) * world_dy_cm
        )
        body_dy_cm = float(
            -np.sin(yaw_rad) * world_dx_cm + np.cos(yaw_rad) * world_dy_cm
        )
        dz_cm = float(target_pose["z"]) - float(pose["z"])
        dyaw_deg = self._normalize_angle_deg(
            float(target_pose["yaw"]) - float(pose["yaw"])
        )
        return body_dx_cm, body_dy_cm, dz_cm, dyaw_deg

    def exec_xyz_yaw_hybrid(
        self,
        dx_cm,
        dy_cm,
        dz_cm,
        dyaw_deg,
        context="",
    ):
        """Send XYZ translation and yaw through one move_relative call."""
        context_text = f" context={context}" if context else ""
        return self._execute_motion(
            AgentAction.XYZ_YAW_HYBRID,
            f"dx={float(dx_cm):.2f}cm dy={float(dy_cm):.2f}cm "
            f"dz={float(dz_cm):.2f}cm dyaw={float(dyaw_deg):.2f}deg"
            f"{context_text}",
            lambda: self.client.move_relative(
                dx=dx_cm,
                dy=dy_cm,
                dz=dz_cm,
                dyaw=dyaw_deg,
            ),
        )

    def _move_to_pose_hybrid(self, target_pose, context):
        delta = self._relative_pose_delta(target_pose)
        return self.exec_xyz_yaw_hybrid(*delta, context=context)

    def _get_target_depth_cm(self, depth_raw, mask) -> Optional[float]:
        mask = np.asarray(mask, dtype=bool).squeeze()
        if mask.ndim != 2:
            return None
        if depth_raw.shape != mask.shape:
            depth_raw = cv2.resize(
                depth_raw,
                (mask.shape[1], mask.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        target_depths = depth_raw[
            mask & np.isfinite(depth_raw) & (depth_raw > 0)
        ]
        if target_depths.size == 0:
            return None
        return float(np.median(target_depths))

    def _estimate_scan_circle(self):
        """Plan a local vertical circle around the track-end drone position."""
        observation = self.last_track_observation
        if observation is None:
            raise RuntimeError("Last track observation is unavailable for scan")

        pose = observation["pose"]
        bbox_state = self.get_bbox_state(observation["bbox"])
        target_distance_cm = self._get_target_depth_cm(
            observation["depth_raw"],
            observation["mask"],
        )
        if target_distance_cm is None:
            target_distance_cm = self.depth_safe_distance_cm
            self._log(
                "[SCAN] Target depth unavailable; use "
                f"fallback_distance={target_distance_cm:.2f}cm"
            )

        horizontal_angle_deg = (
            bbox_state["horizontal_offset"] / self.img_width
        ) * self.max_rotate_deg
        vertical_span_deg = self.max_rotate_deg * self.img_height / self.img_width
        vertical_angle_deg = (
            bbox_state["vertical_offset"] / self.img_height
        ) * vertical_span_deg
        horizontal_distance_cm = max(
            1.0,
            float(
                target_distance_cm
                * np.cos(np.radians(vertical_angle_deg))
            ),
        )
        target_bearing_deg = self._normalize_angle_deg(
            float(pose["yaw"]) + horizontal_angle_deg
        )
        target_bearing_rad = np.radians(target_bearing_deg)
        target_world_x = float(pose["x"]) + float(
            np.cos(target_bearing_rad) * horizontal_distance_cm
        )
        target_world_y = float(pose["y"]) + float(
            np.sin(target_bearing_rad) * horizontal_distance_cm
        )
        target_world_z = float(pose["z"]) + float(
            target_distance_cm * np.sin(np.radians(vertical_angle_deg))
        )

        origin_yaw_rad = np.radians(float(pose["yaw"]))
        trajectory = []
        for point_idx in range(self.scan_num_points):
            circle_angle_rad = 2.0 * np.pi * point_idx / self.scan_num_points
            right_offset_cm = float(
                np.cos(circle_angle_rad) * self.scan_radius_cm
            )
            up_offset_cm = float(
                np.sin(circle_angle_rad) * self.scan_radius_cm
            )
            point_x = float(pose["x"]) - float(
                np.sin(origin_yaw_rad) * right_offset_cm
            )
            point_y = float(pose["y"]) + float(
                np.cos(origin_yaw_rad) * right_offset_cm
            )
            point_z = max(
                self.safe_z_cm,
                float(pose["z"]) + up_offset_cm,
            )
            point_yaw = float(
                np.degrees(
                    np.arctan2(
                        target_world_y - point_y,
                        target_world_x - point_x,
                    )
                )
            )
            trajectory.append(
                {
                    "x": point_x,
                    "y": point_y,
                    "z": point_z,
                    "yaw": self._normalize_angle_deg(point_yaw),
                }
            )

        self._log(
            "[SCAN] Planned local vertical circle: "
            f"points={self.scan_num_points} radius={self.scan_radius_cm:.2f}cm "
            f"estimated_target_distance={horizontal_distance_cm:.2f}cm "
            f"center=(x={float(pose['x']):.2f}, y={float(pose['y']):.2f}, "
            f"z={float(pose['z']):.2f}) "
            f"target=(x={target_world_x:.2f}, y={target_world_y:.2f}, "
            f"z={target_world_z:.2f})"
        )
        return trajectory

    def _capture_scan_point(self, point_idx):
        capture_started = time.perf_counter()
        frame_rgb = self.client.capture(include_depth=False)
        self._log(
            f"[TIMING] phase=scan point={point_idx} "
            f"capture_rgb_s={time.perf_counter() - capture_started:.4f}"
        )
        image_path = Path(self.vis_dir) / f"scan_{point_idx:02d}.png"
        Image.fromarray(frame_rgb).save(image_path)
        pose = self.client.get_pose()
        self._log(
            f"[SCAN] Captured point={point_idx} image={image_path} "
            f"pose=(x={pose['x']:.2f}, y={pose['y']:.2f}, "
            f"z={pose['z']:.2f}, yaw={pose['yaw']:.2f})"
        )
        return pose

    def scan(self):
        """Follow a small local circle and capture one RGB image at each point."""
        trajectory = self._estimate_scan_circle()
        for point_idx, target_pose in enumerate(trajectory):
            self._move_to_pose_hybrid(
                target_pose,
                context=f"scan_point={point_idx}",
            )
            self._capture_scan_point(point_idx)

        # Close the final 30-degree arc without taking a duplicate image.
        self._move_to_pose_hybrid(
            trajectory[0],
            context="scan_close_circle",
        )

        self._log(
            f"[SCAN] Completed local circular scan: points={len(trajectory)}."
        )
        return True

    def run(self, text_prompt):
        """Run one waypoint task without ending the client or simulation."""
        self.waypoint_pose = self.client.get_pose()
        self._log(
            "[WAYPOINT] Saved initial pose: "
            f"x={self.waypoint_pose['x']:.2f}, y={self.waypoint_pose['y']:.2f}, "
            f"z={self.waypoint_pose['z']:.2f}, yaw={self.waypoint_pose['yaw']:.2f}"
        )

        search_succeeded = False
        track_succeeded = False
        scan_succeeded = False
        self.state = AgentState.SEARCH

        try:
            while True:
                self._log(f"[STATE] Enter {self.state.value}")

                if self.state == AgentState.SEARCH:
                    search_succeeded = self.search(text_prompt)
                    self.state = (
                        AgentState.TRACK
                        if search_succeeded
                        else AgentState.RETURN_WAYPOINT
                    )
                elif self.state == AgentState.TRACK:
                    track_succeeded = self.track()
                    self.state = (
                        AgentState.SCAN
                        if track_succeeded
                        else AgentState.RETURN_WAYPOINT
                    )
                elif self.state == AgentState.SCAN:
                    scan_succeeded = self.scan()
                    self.state = AgentState.RETURN_WAYPOINT
                elif self.state == AgentState.RETURN_WAYPOINT:
                    self.return_waypoint()
                    break
        except Exception:
            # An inference or tracking failure must not strand the drone away
            # from the waypoint. Preserve the original exception after return.
            if self.state != AgentState.RETURN_WAYPOINT:
                self._log(
                    f"[STATE] {self.state.value} failed; enter "
                    f"{AgentState.RETURN_WAYPOINT.value}"
                )
                self.state = AgentState.RETURN_WAYPOINT
                self.return_waypoint()
            raise

        succeeded = search_succeeded and track_succeeded and scan_succeeded
        self._log(
            f"[RES] Waypoint task completed; target_reached={track_succeeded} "
            f"scan_completed={scan_succeeded}."
        )
        return succeeded

    def search(self, text_prompt):
        """Search three height levels and initialize the tracker on detection."""
        # Each waypoint is an independent tracking segment.
        self.tracker.reset()
        self.last_track_observation = None

        # Search at h, h+offset, and h-offset, one full turn at each height.
        cur_frame = None
        target_box = None
        search_height_offsets_cm = (
            0,
            self.search_height_offset_cm,
            -self.search_height_offset_cm,
        )
        for height_idx, height_offset_cm in enumerate(search_height_offsets_cm):
            if height_idx == 1:
                self.exec_z_action_cm(
                    self.search_height_offset_cm,
                    AgentAction.UP,
                )
            elif height_idx == 2:
                self.exec_z_action_cm(
                    -2 * self.search_height_offset_cm,
                    AgentAction.DOWN,
                )

            for i in range(self.search_rotation_count):
                candidate_idx = height_idx * self.search_rotation_count + i
                # Search only needs RGB. Depth estimation starts in track().
                capture_started = time.perf_counter()
                cur_frame = self.client.capture(include_depth=False)
                self._log(
                    f"[TIMING] phase=search candidate={candidate_idx} "
                    f"capture_rgb_s={time.perf_counter() - capture_started:.4f}"
                )
                detection_started = time.perf_counter()
                detection = self.detect_with_grounding_dino(cur_frame, text_prompt)
                self._log(
                    f"[TIMING] phase=search candidate={candidate_idx} "
                    f"grounding_dino_s={time.perf_counter() - detection_started:.4f}"
                )

                if detection is not None:
                    target_box, confidence, label = detection
                    show_fig(cur_frame, f"{self.vis_dir}/candidate_{candidate_idx}.png", box_coords=target_box)
                    print(
                        f"[SEARCH] Found {label} with confidence={confidence:.3f} "
                        f"at height offset={height_offset_cm:+d} cm, "
                        f"heading={i * self.search_rotation_step_deg:.1f} deg."
                    )
                    break

                show_fig(cur_frame, f"{self.vis_dir}/candidate_{candidate_idx}.png")
                self.exec_rotate_action_deg(
                    self.search_rotation_step_deg,
                    AgentAction.RIGHT,
                )

            if target_box is not None:
                break

        if target_box is None:
            self._log("[SEARCH] Target was not found; return to the waypoint.")
            return False

        # Initialize SAM2 tracking directly with GroundingDINO's xyxy box.
        track_frame_idx = self.tracker.frame_idx
        sam2_started = time.perf_counter()
        bbox = self.tracker.track(cur_frame, box=target_box)
        self._log_sam2_timing(
            "track_init",
            track_frame_idx,
            time.perf_counter() - sam2_started,
        )

        decision_started = time.perf_counter()
        state = self.get_bbox_state(bbox)
        horizontal_offset = state["horizontal_offset"]
        angle_to_rotate, action = self.prepare_rotate_action_deg(horizontal_offset)
        self._log_decision_timing(
            "init",
            decision_started,
            action,
            f"dyaw={float(angle_to_rotate):.2f}deg",
        )
        self.exec_rotate_action_deg(angle_to_rotate, action)

        return True

    @staticmethod
    def _normalize_angle_deg(angle_deg: float) -> float:
        return (float(angle_deg) + 180.0) % 360.0 - 180.0

    def return_waypoint(self):
        """Return to the saved pose with one relative XYZ/yaw command."""
        if self.waypoint_pose is None:
            raise RuntimeError("Waypoint pose is unavailable; call run() first")

        target = self.waypoint_pose
        self._move_to_pose_hybrid(
            target,
            context="return_waypoint",
        )

        final_pose = self.client.get_pose()
        position_error_cm = float(
            np.linalg.norm(
                [
                    float(target["x"]) - float(final_pose["x"]),
                    float(target["y"]) - float(final_pose["y"]),
                    float(target["z"]) - float(final_pose["z"]),
                ]
            )
        )
        yaw_error_deg = self._normalize_angle_deg(
            float(target["yaw"]) - float(final_pose["yaw"])
        )
        self._log(
            "[WAYPOINT] Return completed: "
            f"x={final_pose['x']:.2f}, y={final_pose['y']:.2f}, "
            f"z={final_pose['z']:.2f}, yaw={final_pose['yaw']:.2f}, "
            f"position_error={position_error_cm:.2f}cm, "
            f"yaw_error={yaw_error_deg:.2f}deg"
        )
        return final_pose


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
        "--config",
        type=str,
        default="config.yaml",
        help="Required agent YAML config path",
    )
    parser.add_argument(
        "--vdir",
        type=str,
        default="./tjk_vis",
        help="Prefix for the timestamped visualization directory (default: ./tjk_vis)",
    )
    parser.add_argument(
        "--log",
        type=str,
        default="./.log",
        help="Prefix or .log path for the timestamped run log (default: ./.log)",
    )
    parser.add_argument(
        "--obj",
        type=str,
        default="white ball",
        help="Object description used by GroundingDINO (default: street lamp)",
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
    # Validate the required config before creating logs, clients, or models.
    _load_config(args.config)
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    vis_dir = _timestamped_dir(args.vdir, timestamp)
    log_path = _timestamped_log_path(args.log, timestamp)
    vis_dir.mkdir(parents=True, exist_ok=True)
    logger = _configure_logger(log_path)
    logger.info(
        f"[RUN] client={args.client} object={args.obj!r} "
        f"depth_source={'ue_native' if args.client == 'ue' else 'client_da3'} "
        f"config={Path(args.config).expanduser().resolve()} "
        f"vdir={vis_dir} log={log_path}"
    )

    client = build_client(
        args.client,
        server_host="127.0.0.1",
        server_port=8765,
        http_timeout_s=180.0,
    )
    tjkAgent = TJKAgent(
        client=client,
        vis_dir=str(vis_dir),
        logger=logger,
        config_path=args.config,
    )
    tjkAgent.connect()
    tjkAgent.run(args.obj)

if __name__ == "__main__":
    main()
