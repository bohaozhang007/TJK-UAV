"""Streaming single-object adapter for the official SAM 2 video predictor."""

from __future__ import annotations

from collections import OrderedDict
from typing import Any

import numpy as np
import torch
import torch.nn.functional as F

from sam2.build_sam import build_sam2_video_predictor


class OneObjVideoPredictor:
    """Process one RGB frame at a time while retaining SAM 2 video memory."""

    def __init__(self, predictor: Any) -> None:
        self.predictor = predictor
        self.video_height = 0
        self.video_width = 0
        self._state: dict[str, Any] | None = None

    def init_inference_state(self) -> dict[str, Any]:
        """Reset all prompts, frame features, and temporal memory."""
        device = self.predictor.device
        self._state = {
            "images": {},
            "num_frames": 0,
            "offload_video_to_cpu": False,
            "offload_state_to_cpu": False,
            "video_height": int(self.video_height),
            "video_width": int(self.video_width),
            "device": device,
            "storage_device": device,
            "point_inputs_per_obj": {},
            "mask_inputs_per_obj": {},
            "cached_features": {},
            "constants": {},
            "obj_id_to_idx": OrderedDict(),
            "obj_idx_to_id": OrderedDict(),
            "obj_ids": [],
            "output_dict_per_obj": {},
            "temp_output_dict_per_obj": {},
            "frames_tracked_per_obj": {},
            "tracking_has_started": False,
        }
        return self._state

    def _preprocess_frame(self, image_rgb: np.ndarray) -> torch.Tensor:
        image = np.asarray(image_rgb)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("SAM2 input must be an HxWx3 RGB image")
        if image.dtype != np.uint8:
            raise ValueError("SAM2 input must use uint8 RGB pixels")

        image_tensor = torch.from_numpy(np.ascontiguousarray(image)).permute(2, 0, 1)
        image_tensor = image_tensor.unsqueeze(0).float().div_(255.0)
        image_tensor = F.interpolate(
            image_tensor,
            size=(self.predictor.image_size, self.predictor.image_size),
            mode="bilinear",
            align_corners=False,
        ).squeeze(0)
        mean = image_tensor.new_tensor((0.485, 0.456, 0.406))[:, None, None]
        std = image_tensor.new_tensor((0.229, 0.224, 0.225))[:, None, None]
        return (image_tensor - mean) / std

    @staticmethod
    def _has_temporary_output(state: dict[str, Any]) -> bool:
        return any(
            outputs[storage_key]
            for outputs in state["temp_output_dict_per_obj"].values()
            for storage_key in ("cond_frame_outputs", "non_cond_frame_outputs")
        )

    @torch.inference_mode()
    def gen_mask(
        self,
        img_np: np.ndarray,
        frame_idx: int,
        points: np.ndarray | None = None,
        labels: np.ndarray | None = None,
    ) -> torch.Tensor:
        """Consume one frame and return its single-object mask logits."""
        frame_idx = int(frame_idx)
        if frame_idx < 0:
            raise ValueError("frame_idx must be non-negative")
        if (points is None) != (labels is None):
            raise ValueError("points and labels must be provided together")

        state = self._state or self.init_inference_state()
        height, width = np.asarray(img_np).shape[:2]
        state["video_height"] = int(self.video_height or height)
        state["video_width"] = int(self.video_width or width)
        state["images"][frame_idx] = self._preprocess_frame(img_np)
        state["num_frames"] = max(int(state["num_frames"]), frame_idx + 1)

        if points is not None:
            _, _, video_masks = self.predictor.add_new_points_or_box(
                inference_state=state,
                frame_idx=frame_idx,
                obj_id=1,
                points=np.asarray(points, dtype=np.float32),
                labels=np.asarray(labels, dtype=np.int32),
            )
            self.predictor.propagate_in_video_preflight(state)
            obj_idx = state["obj_id_to_idx"][1]
            state["frames_tracked_per_obj"][obj_idx][frame_idx] = {
                "reverse": False
            }
            state["tracking_has_started"] = True
            return video_masks[0]

        if not state["obj_ids"]:
            raise RuntimeError("SAM2 requires points or a box on the first frame")
        if self._has_temporary_output(state):
            self.predictor.propagate_in_video_preflight(state)

        obj_idx = state["obj_id_to_idx"][1]
        output_dict = state["output_dict_per_obj"][obj_idx]
        current_out, pred_masks = self.predictor._run_single_frame_inference(
            inference_state=state,
            output_dict=output_dict,
            frame_idx=frame_idx,
            batch_size=1,
            is_init_cond_frame=False,
            point_inputs=None,
            mask_inputs=None,
            reverse=False,
            run_mem_encoder=True,
        )
        output_dict["non_cond_frame_outputs"][frame_idx] = current_out
        state["frames_tracked_per_obj"][obj_idx][frame_idx] = {"reverse": False}
        state["tracking_has_started"] = True
        _, video_masks = self.predictor._get_orig_video_res_output(state, pred_masks)
        return video_masks[0]


def build_one_obj_video_predictor(
    config_file: str,
    checkpoint: str,
    *,
    device: str = "cuda",
    **kwargs: Any,
) -> OneObjVideoPredictor:
    """Build the official SAM 2 model behind the streaming compatibility API."""
    predictor = build_sam2_video_predictor(
        config_file,
        checkpoint,
        device=device,
        **kwargs,
    )
    return OneObjVideoPredictor(predictor)


__all__ = ["OneObjVideoPredictor", "build_one_obj_video_predictor"]
