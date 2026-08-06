# 1. 单航点处 select 最近的目标

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
    ROTATE = "rotate"
    XYZ_YAW_HYBRID = "xyz_yaw_hybrid"


class AgentState(str, Enum):
    SEARCH = "SEARCH"
    SELECT = "SELECT"
    TRACK = "TRACK"
    SCAN = "SCAN"
    RETURN_WAYPOINT = "RETURN_WAYPOINT"


def _timestamped_experiment_dir(exp_name: str, timestamp: str) -> Path:
    return Path(f"{Path(exp_name).expanduser()}_{timestamp}")


def _configure_logger(log_path: Path) -> logging.Logger:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("tjk_v9")
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
        self.last_motion_error = None
        self._last_motion_timing = {}
        self._last_motion_command_id = None
        self.vis_dir = vis_dir
        self.save_vis = save_vis
        self.save_depth = save_depth
        self.state = AgentState.SEARCH
        self.waypoint_pose = None
        self.search_candidates = []
        self.selected_candidate = None
        self.last_track_observation = None
        self.config_path = Path(config_path).expanduser().resolve()
        config = _load_config(self.config_path)
        motion_config = _config_section(config, "motion")
        search_config = _config_section(config, "search")
        safety_config = _config_section(config, "safety")
        track_config = _config_section(config, "track")
        scan_config = _config_section(config, "scan")
        detection_config = _config_section(config, "detection")

        # Forward/backward action.
        self.max_fb_step_cm = _config_number(
            motion_config, "max_fb_step_cm", minimum=1e-6
        )
        self.min_reliable_fb_step_cm = _config_number(
            motion_config, "min_reliable_fb_step_cm", minimum=0.0
        )
        self.backward_ratio = _config_number(
            motion_config,
            "backward_ratio",
            minimum=0.0,
            maximum=1.0,
        )

        # Vertical action.
        self.z_cm_per_pixel = _config_number(
            motion_config, "z_cm_per_pixel", minimum=1e-6
        )
        self.max_z_step_cm = _config_number(
            motion_config, "max_z_step_cm", minimum=1e-6
        )
        self.min_reliable_z_step_cm = _config_number(
            motion_config, "min_reliable_z_step_cm", minimum=0.0
        )

        # Yaw action.
        self.rotate_deg_per_pixel = _config_number(
            motion_config, "rotate_deg_per_pixel", minimum=1e-6
        )
        self.max_rotate_deg = _config_number(
            motion_config, "max_rotate_deg", minimum=1e-6
        )
        self.min_reliable_rotate_deg = _config_number(
            motion_config, "min_reliable_rotate_deg", minimum=0.0
        )

        if self.min_reliable_fb_step_cm > self.max_fb_step_cm:
            raise ValueError(
                "motion.min_reliable_fb_step_cm cannot exceed max_fb_step_cm"
            )
        if self.min_reliable_z_step_cm > self.max_z_step_cm:
            raise ValueError(
                "motion.min_reliable_z_step_cm cannot exceed max_z_step_cm"
            )
        if self.min_reliable_rotate_deg > self.max_rotate_deg:
            raise ValueError(
                "motion.min_reliable_rotate_deg cannot exceed max_rotate_deg"
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
        self.yaw_only_threshold_deg = _config_number(
            track_config,
            "yaw_only_threshold_deg",
            minimum=self.min_reliable_rotate_deg,
            maximum=self.max_rotate_deg,
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
            minimum=12.0,
            maximum=12.0,
        )
        self.scan_radius_cm = _config_number(
            scan_config,
            "radius_cm",
            minimum=1e-6,
        )
        self.scan_yaw_unit_deg = _config_number(
            scan_config,
            "yaw_unit_deg",
            minimum=1e-6,
        )

        sam_cfg = SAM2Config()
        sam_cfg.VIDEO_HEIGHT = self.img_height
        sam_cfg.VIDEO_WIDTH = self.img_width
        self.tracker = Sam2VideoPredictor(sam_cfg)
        self.tracker.set_vis_mode(self.save_vis)
        self.tracker.set_vis_dir(self.vis_dir)
        self.tracker.set_track_vis_naming("track", index_width=2)

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

    def _log_timing(self, total_s: float, command_id=None, **fields) -> None:
        command_id_text = "none" if command_id is None else str(command_id)
        parts = [
            f"[TIMING] command_id={command_id_text}",
            f"total_s={float(total_s):.4f}",
        ]
        for name, value in fields.items():
            time_name = name if name.endswith("_s") else f"{name}_s"
            if isinstance(value, dict):
                details = ", ".join(
                    f"{detail_name if detail_name.endswith('_s') else f'{detail_name}_s'}="
                    f"{float(detail_value):.4f}"
                    for detail_name, detail_value in value.items()
                )
                parts.append(f"{name}:{{{details}}}")
            elif isinstance(value, (int, float)):
                parts.append(f"{time_name}={float(value):.4f}")
            else:
                parts.append(f"{time_name}={value}")
        self._log(" ".join(parts))

    @staticmethod
    def _sam2_timing_fields(tracker, measured_total_s: float) -> dict:
        timing = getattr(tracker, "last_timing", {})
        return {
            "total": float(measured_total_s),
            "infer": float(
                timing.get("inference_s", measured_total_s)
            ),
            "vis": float(
                timing.get("visualization_s", 0.0)
            ),
        }

    def _log_track_iteration_timing(
        self,
        capture_s,
        save_depth_s,
        sam2_fields,
        decision_s,
        motion_timing=None,
    ) -> None:
        motion = motion_timing or {
            "drone_execution_s": 0.0,
        }
        total_s = (
            capture_s
            + save_depth_s
            + sam2_fields["total"]
            + decision_s
            + motion["drone_execution_s"]
        )
        self._log_timing(
            total_s,
            command_id=(
                self._last_motion_command_id
                if motion_timing is not None
                else None
            ),
            capture=capture_s,
            save_depth=save_depth_s,
            tracker=sam2_fields,
            decision=decision_s,
            execution=motion["drone_execution_s"],
        )

    @staticmethod
    def _format_motion_command(dx_cm, dy_cm, dz_cm, dyaw_deg) -> str:
        return (
            f"(dx={float(dx_cm):.2f}cm, dy={float(dy_cm):.2f}cm, "
            f"dz={float(dz_cm):.2f}cm, dyaw={float(dyaw_deg):.2f}deg)"
        )

    @staticmethod
    def _format_motion_error(error) -> str:
        if error is None:
            return "(ex=N/A, ey=N/A, ez=N/A, eyaw=N/A, epos=N/A)"
        return (
            f"(ex={error['ex']:.2f}cm, ey={error['ey']:.2f}cm, "
            f"ez={error['ez']:.2f}cm, eyaw={error['eyaw']:.2f}deg, "
            f"epos={error['epos']:.2f}cm)"
        )

    def _calculate_motion_error(
        self,
        start_pose,
        end_pose,
        dx_cm,
        dy_cm,
        dz_cm,
        dyaw_deg,
    ) -> dict:
        """Return requested-minus-actual error in the command's body frame."""
        world_dx_cm = float(end_pose["x"]) - float(start_pose["x"])
        world_dy_cm = float(end_pose["y"]) - float(start_pose["y"])
        start_yaw_rad = np.radians(float(start_pose["yaw"]))
        actual_dx_cm = float(
            np.cos(start_yaw_rad) * world_dx_cm
            + np.sin(start_yaw_rad) * world_dy_cm
        )
        actual_dy_cm = float(
            -np.sin(start_yaw_rad) * world_dx_cm
            + np.cos(start_yaw_rad) * world_dy_cm
        )
        actual_dz_cm = float(end_pose["z"]) - float(start_pose["z"])
        actual_dyaw_deg = self._normalize_angle_deg(
            float(end_pose["yaw"]) - float(start_pose["yaw"])
        )
        ex_cm = float(dx_cm) - actual_dx_cm
        ey_cm = float(dy_cm) - actual_dy_cm
        ez_cm = float(dz_cm) - actual_dz_cm
        eyaw_deg = self._normalize_angle_deg(
            float(dyaw_deg) - actual_dyaw_deg
        )
        return {
            "ex": ex_cm,
            "ey": ey_cm,
            "ez": ez_cm,
            "eyaw": eyaw_deg,
            "epos": float(np.linalg.norm([ex_cm, ey_cm, ez_cm])),
        }

    def _execute_motion(
        self,
        action: AgentAction,
        dx_cm,
        dy_cm,
        dz_cm,
        dyaw_deg,
        operation,
        context="",
        log_timing=True,
    ):
        self._command_idx += 1
        command_idx = self._command_idx
        self._last_motion_command_id = command_idx
        pose = self.client.get_pose()
        command = self._format_motion_command(dx_cm, dy_cm, dz_cm, dyaw_deg)
        last_error = self._format_motion_error(
            getattr(self, "last_motion_error", None)
        )
        context_text = f" context={context}" if context else ""
        self._log(
            f"[COMMAND] id={command_idx} action={action.value} "
            f"command={command} "
            f"pose=(x={float(pose['x']):.2f}, y={float(pose['y']):.2f}, "
            f"z={float(pose['z']):.2f}, yaw={float(pose['yaw']):.2f}) "
            f"last_error={last_error}{context_text}"
        )
        execution_started = time.perf_counter()
        try:
            result = operation()
        except Exception:
            drone_execution_s = time.perf_counter() - execution_started
            self._last_motion_timing = {
                "drone_execution_s": drone_execution_s,
            }
            if log_timing:
                self._log_timing(
                    drone_execution_s,
                    command_id=command_idx,
                    execution=drone_execution_s,
                )
            raise
        drone_execution_s = time.perf_counter() - execution_started
        end_pose = self.client.get_pose()
        self.last_motion_error = self._calculate_motion_error(
            pose,
            end_pose,
            dx_cm,
            dy_cm,
            dz_cm,
            dyaw_deg,
        )
        self._last_motion_timing = {
            "drone_execution_s": drone_execution_s,
        }
        if log_timing:
            self._log_timing(
                drone_execution_s,
                command_id=command_idx,
                execution=drone_execution_s,
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

    @staticmethod
    def _copy_pose(pose):
        return {
            axis: float(pose[axis])
            for axis in ("x", "y", "z", "yaw")
        }

    @staticmethod
    def _mask_median_depth_cm(depth_raw, mask) -> Optional[float]:
        mask = np.asarray(mask, dtype=bool).squeeze()
        if mask.ndim != 2:
            return None
        depth = np.asarray(depth_raw)
        if depth.shape != mask.shape:
            depth = cv2.resize(
                depth,
                (mask.shape[1], mask.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        valid_depths = depth[
            mask & np.isfinite(depth) & (depth > 0)
        ]
        if valid_depths.size == 0:
            return None
        return float(np.median(valid_depths))

    def _segment_search_candidate(self, frame_rgb, target_box):
        """Run an independent box-prompted SAM segmentation for one candidate."""
        self.tracker.reset()
        self.tracker.set_vis_mode(False)
        try:
            tracker_started = time.perf_counter()
            bbox, mask = self.tracker.track_with_mask(
                frame_rgb,
                box=target_box,
            )
            tracker_fields = self._sam2_timing_fields(
                self.tracker,
                time.perf_counter() - tracker_started,
            )
            return bbox, mask, tracker_fields
        finally:
            # Search candidates are independent. Do not let one candidate's
            # temporal state influence the next candidate or final tracking.
            self.tracker.set_vis_mode(self.save_vis)
            self.tracker.reset()

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
        depth_path = os.path.join(self.vis_dir, f"track_{frame_idx:02d}.npy")
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
        angle_to_rotate = horizontal_offset * self.rotate_deg_per_pixel
        angle_to_rotate = np.clip(
            angle_to_rotate,
            -self.max_rotate_deg,
            self.max_rotate_deg,
        )
        return float(angle_to_rotate)
    
    def exec_rotate_action_deg(
        self,
        angle_to_rotate,
        context="",
        log_timing=True,
    ):
        dyaw = float(angle_to_rotate)
        return self._execute_motion(
            AgentAction.ROTATE,
            0.0,
            0.0,
            0.0,
            dyaw,
            lambda: self.client.move_relative(dx=0.0, dy=0.0, dz=0.0, dyaw=dyaw),
            context=context,
            log_timing=log_timing,
        )
    
    def prepare_z_action_cm(self, vertical_offset):
        dz_cm = vertical_offset * self.z_cm_per_pixel
        dz_cm = float(np.clip(dz_cm, -self.max_z_step_cm, self.max_z_step_cm))
        return dz_cm

    def get_backward_step_cm(self):
        return -self.backward_ratio * self.max_fb_step_cm
    
    def prepare_fb_action_cm(self, box_ratio):
        target = self.target_stop_ratio
        deadband = self.min_reliable_fb_step_cm / self.max_fb_step_cm * target

        if box_ratio > target + deadband:
            step_cm = self.get_backward_step_cm()
        elif box_ratio < target - deadband:
            progress = box_ratio / target
            step_cm = self.max_fb_step_cm * (1.0 - progress)
        else:
            step_cm = 0.0

        return float(step_cm)

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
            capture_s = time.perf_counter() - capture_started
            track_frame_idx = self.tracker.frame_idx
            sam2_started = time.perf_counter()
            bbox, mask = self.tracker.track_with_mask(frame_rgb)
            sam2_fields = self._sam2_timing_fields(
                self.tracker,
                time.perf_counter() - sam2_started,
            )
            save_depth_started = time.perf_counter()
            if self.save_depth:
                self.save_track_depth(depth_raw, track_frame_idx)
            save_depth_s = time.perf_counter() - save_depth_started

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

            angle_to_rotate = self.prepare_rotate_action_deg(horizontal_offset)

            # Large yaw errors are corrected without translation. Once yaw is
            # approximately aligned, all remaining axes share one command.
            if abs(angle_to_rotate) > self.yaw_only_threshold_deg:
                decision_s = time.perf_counter() - decision_started
                self.exec_rotate_action_deg(
                    angle_to_rotate,
                    context=f"track iteration={iterations} yaw_only",
                    log_timing=False,
                )
                self._log_track_iteration_timing(
                    capture_s,
                    save_depth_s,
                    sam2_fields,
                    decision_s,
                    self._last_motion_timing,
                )
                continue
            if abs(angle_to_rotate) > self.min_reliable_rotate_deg:
                dyaw_deg = angle_to_rotate
            else:
                dyaw_deg = 0.0
                print(f"[STALL] angle is {angle_to_rotate} and Stalled.")

            dz_cm = self.prepare_z_action_cm(vertical_offset)
            if dz_cm < 0.0:
                cur_z_cm = self.get_current_z_cm()
                if cur_z_cm <= self.safe_z_cm:
                    dz_cm = 0.0
                    print(
                        f"[STALL] Downward motion is blocked at safe_z="
                        f"{self.safe_z_cm:.2f} cm."
                    )
                else:
                    dz_cm = max(dz_cm, self.safe_z_cm - cur_z_cm)

            if abs(dz_cm) <= self.min_reliable_z_step_cm:
                print(f"[STALL] dz_cm is {dz_cm} and Stalled.")
                dz_cm = 0.0

            dx_cm = self.prepare_fb_action_cm(box_ratio)
            if abs(dx_cm) <= self.min_reliable_fb_step_cm:
                print(f"[STALL] dx is {dx_cm} cm and Stalled.")
                dx_cm = 0.0
            elif dx_cm > 0.0:
                requested_dx_cm = dx_cm
                path_depth_cm = self.get_nearest_path_depth_cm(depth_raw, mask)
                available_dx_cm = max(
                    0.0,
                    path_depth_cm - self.depth_safe_distance_cm,
                )
                dx_cm = min(requested_dx_cm, available_dx_cm)
                print(
                    f"[DEPTH] path_p01={path_depth_cm:.2f} cm, "
                    f"safe={self.depth_safe_distance_cm:.2f} cm, "
                    f"requested={requested_dx_cm:.2f} cm, "
                    f"available={available_dx_cm:.2f} cm, "
                    f"forward={dx_cm:.2f} cm."
                )
                if dx_cm <= self.min_reliable_fb_step_cm:
                    print(
                        f"[STALL] Depth-limited forward step {dx_cm:.2f} cm "
                        f"is not executable (minimum "
                        f"{self.min_reliable_fb_step_cm:.2f} cm); stop at the safe "
                        "distance."
                    )
                    dx_cm = 0.0

            if any(value != 0.0 for value in (dx_cm, dz_cm, dyaw_deg)):
                decision_s = time.perf_counter() - decision_started
                self.exec_xyz_yaw_hybrid(
                    dx_cm,
                    0.0,
                    dz_cm,
                    dyaw_deg,
                    context=f"track iteration={iterations}",
                    log_timing=False,
                )
                self._log_track_iteration_timing(
                    capture_s,
                    save_depth_s,
                    sam2_fields,
                    decision_s,
                    self._last_motion_timing,
                )
                continue

            # All axes are inside their stop thresholds.
            decision_s = time.perf_counter() - decision_started
            self._log_track_iteration_timing(
                capture_s,
                save_depth_s,
                sam2_fields,
                decision_s,
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
        log_timing=True,
    ):
        """Send XYZ translation and yaw through one move_relative call."""
        return self._execute_motion(
            AgentAction.XYZ_YAW_HYBRID,
            dx_cm,
            dy_cm,
            dz_cm,
            dyaw_deg,
            lambda: self.client.move_relative(
                dx=dx_cm,
                dy=dy_cm,
                dz=dz_cm,
                dyaw=dyaw_deg,
            ),
            context=context,
            log_timing=log_timing,
        )

    def _move_to_pose_hybrid(self, target_pose, context, log_timing=True):
        delta = self._relative_pose_delta(target_pose)
        return self.exec_xyz_yaw_hybrid(
            *delta,
            context=context,
            log_timing=log_timing,
        )

    def _estimate_scan_circle(self):
        """Plan a 12-point clock-face circle around the track-end pose."""
        observation = self.last_track_observation
        if observation is None:
            raise RuntimeError("Last track observation is unavailable for scan")

        pose = observation["pose"]
        origin_yaw_rad = np.radians(float(pose["yaw"]))
        # Positive dyaw is a right turn in the agent command convention.
        yaw_steps_by_hour = {
            1: -1,
            2: -2,
            3: -3,
            4: -2,
            5: -1,
            6: 0,
            7: 1,
            8: 2,
            9: 3,
            10: 2,
            11: 1,
            12: 0,
        }
        trajectory = []
        clock_hours = [12, *range(1, self.scan_num_points)]
        for clock_hour in clock_hours:
            clock_angle_rad = 2.0 * np.pi * clock_hour / self.scan_num_points
            right_offset_cm = float(
                np.sin(clock_angle_rad) * self.scan_radius_cm
            )
            up_offset_cm = float(
                np.cos(clock_angle_rad) * self.scan_radius_cm
            )
            if clock_hour in (6, 12):
                right_offset_cm = 0.0
            if clock_hour in (3, 9):
                up_offset_cm = 0.0
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
            point_yaw = self._normalize_angle_deg(
                float(pose["yaw"])
                + yaw_steps_by_hour[clock_hour] * self.scan_yaw_unit_deg
            )
            trajectory.append(
                {
                    "x": point_x,
                    "y": point_y,
                    "z": point_z,
                    "yaw": point_yaw,
                    "clock_hour": clock_hour,
                }
            )

        yaw_offsets = ", ".join(
            f"{point['clock_hour']}="
            f"{self._normalize_angle_deg(point['yaw'] - float(pose['yaw'])):.2f}"
            for point in trajectory
        )
        self._log(
            "[SCAN] Planned simple clock trajectory: "
            f"points={self.scan_num_points} radius={self.scan_radius_cm:.2f}cm "
            f"yaw_unit={self.scan_yaw_unit_deg:.2f}deg "
            f"center=(x={float(pose['x']):.2f}, y={float(pose['y']):.2f}, "
            f"z={float(pose['z']):.2f}, yaw={float(pose['yaw']):.2f}) "
            f"yaw_offsets_deg=({yaw_offsets})"
        )
        return trajectory

    def _capture_scan_point(self, clock_hour, motion_timing):
        capture_started = time.perf_counter()
        frame_rgb = self.client.capture(include_depth=False)
        capture_rgb_s = time.perf_counter() - capture_started
        image_path = Path(self.vis_dir) / f"scan_{clock_hour:02d}.png"
        save_image_started = time.perf_counter()
        Image.fromarray(frame_rgb).save(image_path)
        save_image_s = time.perf_counter() - save_image_started
        pose = self.client.get_pose()
        self._log(
            f"[SCAN] Captured clock_hour={clock_hour} image={image_path} "
            f"pose=(x={pose['x']:.2f}, y={pose['y']:.2f}, "
            f"z={pose['z']:.2f}, yaw={pose['yaw']:.2f})"
        )
        total_s = (
            motion_timing["drone_execution_s"]
            + capture_rgb_s
            + save_image_s
        )
        self._log_timing(
            total_s,
            command_id=self._last_motion_command_id,
            capture=capture_rgb_s,
            save_image=save_image_s,
            execution=motion_timing["drone_execution_s"],
        )
        return pose

    def scan(self):
        """Follow a small local circle and capture one RGB image at each point."""
        trajectory = self._estimate_scan_circle()
        for target_pose in trajectory:
            clock_hour = target_pose["clock_hour"]
            self._move_to_pose_hybrid(
                target_pose,
                context=f"scan_clock_hour={clock_hour}",
                log_timing=False,
            )
            self._capture_scan_point(clock_hour, self._last_motion_timing)

        # Close the 30-degree arc from 11 o'clock to 12 o'clock without
        # taking a duplicate image.
        self._move_to_pose_hybrid(
            trajectory[0],
            context="scan_close_circle clock_hour=12",
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
        select_succeeded = False
        track_succeeded = False
        scan_succeeded = False
        self.state = AgentState.SEARCH

        try:
            while True:
                self._log(f"[STATE] Enter {self.state.value}")

                if self.state == AgentState.SEARCH:
                    search_succeeded = self.search(text_prompt)
                    self.state = (
                        AgentState.SELECT
                        if search_succeeded
                        else AgentState.RETURN_WAYPOINT
                    )
                elif self.state == AgentState.SELECT:
                    select_succeeded = self.select()
                    self.state = (
                        AgentState.TRACK
                        if select_succeeded
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

        succeeded = (
            search_succeeded
            and select_succeeded
            and track_succeeded
            and scan_succeeded
        )
        self._log(
            f"[RES] Waypoint task completed; candidates="
            f"{len(self.search_candidates)} target_selected={select_succeeded} "
            f"target_reached={track_succeeded} "
            f"scan_completed={scan_succeeded}."
        )
        return succeeded

    def search(self, text_prompt):
        """Scan all three height levels and collect every detected candidate."""
        self.tracker.reset()
        self.search_candidates = []
        self.selected_candidate = None
        self.last_track_observation = None

        # Search at h, h+offset, and h-offset, one full turn at each height.
        search_height_offsets_cm = (
            0,
            self.search_height_offset_cm,
            -self.search_height_offset_cm,
        )
        for height_idx, height_offset_cm in enumerate(search_height_offsets_cm):
            if height_idx == 1:
                self.exec_xyz_yaw_hybrid(
                    0.0,
                    0.0,
                    self.search_height_offset_cm,
                    0.0,
                    context="search_height_up",
                )
            elif height_idx == 2:
                self.exec_xyz_yaw_hybrid(
                    0.0,
                    0.0,
                    -2 * self.search_height_offset_cm,
                    0.0,
                    context="search_height_down",
                )

            for i in range(self.search_rotation_count):
                candidate_idx = height_idx * self.search_rotation_count + i
                capture_started = time.perf_counter()
                cur_frame, depth_raw = self.client.capture(include_depth=True)
                capture_rgb_s = time.perf_counter() - capture_started
                detection_started = time.perf_counter()
                detection = self.detect_with_grounding_dino(cur_frame, text_prompt)
                grounding_dino_s = time.perf_counter() - detection_started
                tracker_fields = None

                if detection is not None:
                    target_box, confidence, label = detection
                    sam_bbox, mask, tracker_fields = (
                        self._segment_search_candidate(
                            cur_frame,
                            target_box,
                        )
                    )
                    distance_cm = self._mask_median_depth_cm(
                        depth_raw,
                        mask,
                    )
                    if distance_cm is None:
                        self._log(
                            f"[SEARCH] Skip candidate={candidate_idx}: "
                            "SAM mask contains no valid positive depth."
                        )
                    else:
                        candidate_pose = self._copy_pose(
                            self.client.get_pose()
                        )
                        candidate = {
                            "candidate_idx": candidate_idx,
                            "height_offset_cm": height_offset_cm,
                            "heading_deg": (
                                i * self.search_rotation_step_deg
                            ),
                            "pose": candidate_pose,
                            "box": np.asarray(target_box).copy(),
                            "sam_bbox": np.asarray(sam_bbox).copy(),
                            "mask": np.asarray(mask).copy(),
                            "distance_cm": distance_cm,
                            "confidence": float(confidence),
                            "label": str(label),
                        }
                        self.search_candidates.append(candidate)
                        self._log(
                            f"[SEARCH] Queued candidate={candidate_idx} "
                            f"label={label} confidence={confidence:.3f} "
                            f"distance={distance_cm:.2f}cm "
                            f"height_offset={height_offset_cm:+d}cm "
                            f"heading={candidate['heading_deg']:.1f}deg "
                            f"pose=(x={candidate_pose['x']:.2f}, "
                            f"y={candidate_pose['y']:.2f}, "
                            f"z={candidate_pose['z']:.2f}, "
                            f"yaw={candidate_pose['yaw']:.2f})"
                        )
                    visualization_started = time.perf_counter()
                    show_fig(
                        cur_frame,
                        f"{self.vis_dir}/candidate_{candidate_idx:02d}.png",
                        masks=[mask],
                        box_coords=target_box,
                    )
                    visualization_s = time.perf_counter() - visualization_started
                else:
                    visualization_started = time.perf_counter()
                    show_fig(
                        cur_frame,
                        f"{self.vis_dir}/candidate_{candidate_idx:02d}.png",
                    )
                    visualization_s = (
                        time.perf_counter() - visualization_started
                    )

                # Always complete the full rotation, even after a detection.
                self.exec_rotate_action_deg(
                    self.search_rotation_step_deg,
                    context=f"search candidate={candidate_idx}",
                    log_timing=False,
                )
                motion_timing = self._last_motion_timing
                total_s = (
                    capture_rgb_s
                    + grounding_dino_s
                    + visualization_s
                    + (
                        tracker_fields["total"]
                        if tracker_fields is not None
                        else 0.0
                    )
                    + motion_timing["drone_execution_s"]
                )
                timing_fields = {
                    "capture": capture_rgb_s,
                    "detector": {
                        "total": grounding_dino_s + visualization_s,
                        "infer": grounding_dino_s,
                        "vis": visualization_s,
                    },
                }
                if tracker_fields is not None:
                    timing_fields["tracker"] = tracker_fields
                timing_fields["execution"] = (
                    motion_timing["drone_execution_s"]
                )
                self._log_timing(
                    total_s,
                    command_id=self._last_motion_command_id,
                    **timing_fields,
                )

        if not self.search_candidates:
            self._log(
                "[SEARCH] Completed all height levels; candidate queue is "
                "empty, return to the waypoint."
            )
            return False

        nearest = min(
            self.search_candidates,
            key=lambda candidate: candidate["distance_cm"],
        )
        self._log(
            f"[SEARCH] Completed all height levels; queued="
            f"{len(self.search_candidates)} nearest_candidate="
            f"{nearest['candidate_idx']} "
            f"distance={nearest['distance_cm']:.2f}cm."
        )
        return True

    def select(self):
        """Return to the nearest candidate pose and initialize SAM tracking."""
        if not self.search_candidates:
            self._log("[SELECT] Candidate queue is empty.")
            return False

        selected = min(
            self.search_candidates,
            key=lambda candidate: candidate["distance_cm"],
        )
        self.selected_candidate = selected
        selected_pose = selected["pose"]
        self._log(
            f"[SELECT] Selected candidate={selected['candidate_idx']} "
            f"distance={selected['distance_cm']:.2f}cm "
            f"confidence={selected['confidence']:.3f} "
            f"pose=(x={selected_pose['x']:.2f}, "
            f"y={selected_pose['y']:.2f}, z={selected_pose['z']:.2f}, "
            f"yaw={selected_pose['yaw']:.2f})"
        )
        self._move_to_pose_hybrid(
            selected_pose,
            context=f"select candidate={selected['candidate_idx']}",
        )

        # Re-capture at the recorded pose and initialize SAM directly from
        # the saved detector box. GroundingDINO is intentionally not called.
        capture_started = time.perf_counter()
        cur_frame = self.client.capture(include_depth=False)
        capture_s = time.perf_counter() - capture_started
        self.tracker.reset()
        sam2_started = time.perf_counter()
        bbox, _mask = self.tracker.track_with_mask(
            cur_frame,
            box=selected["box"],
        )
        sam2_fields = self._sam2_timing_fields(
            self.tracker,
            time.perf_counter() - sam2_started,
        )

        decision_started = time.perf_counter()
        state = self.get_bbox_state(bbox)
        horizontal_offset = state["horizontal_offset"]
        angle_to_rotate = self.prepare_rotate_action_deg(horizontal_offset)
        decision_s = time.perf_counter() - decision_started
        self.exec_rotate_action_deg(
            angle_to_rotate,
            context=f"track_init candidate={selected['candidate_idx']}",
            log_timing=False,
        )
        motion_timing = self._last_motion_timing
        self._log_timing(
            capture_s
            + sam2_fields["total"]
            + decision_s
            + motion_timing["drone_execution_s"],
            command_id=self._last_motion_command_id,
            capture=capture_s,
            tracker=sam2_fields,
            decision=decision_s,
            execution=motion_timing["drone_execution_s"],
        )

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
        default="config_v9.yaml",
        help="Required agent YAML config path",
    )
    parser.add_argument(
        "--exp_name",
        type=str,
        default="./test",
        help=(
            "Experiment directory prefix; a timestamp suffix is added "
            "automatically (default: ./test)"
        ),
    )
    parser.add_argument(
        "--obj",
        type=str,
        default="white light bulb",
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
    exp_dir = _timestamped_experiment_dir(args.exp_name, timestamp)
    vis_dir = exp_dir / "vis"
    log_path = exp_dir / "log.txt"
    vis_dir.mkdir(parents=True, exist_ok=True)
    logger = _configure_logger(log_path)
    logger.info(
        f"[RUN] client={args.client} object={args.obj!r} "
        f"depth_source={'ue_native' if args.client == 'ue' else 'client_da3'} "
        f"config={Path(args.config).expanduser().resolve()} "
        f"exp_dir={exp_dir} vis={vis_dir} log={log_path}"
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
