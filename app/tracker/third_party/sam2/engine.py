"""Single-object streaming adapter for the local SAM2 video predictor."""
from collections import OrderedDict
from contextlib import nullcontext
from pathlib import Path
import sys

import numpy as np
from PIL import Image


class Sam2Tracker:
    def __init__(self, model_root, checkpoint, device="cuda"):
        root, checkpoint = Path(model_root).resolve(), Path(checkpoint).resolve()
        if not (root / "sam2").is_dir() or not checkpoint.is_file():
            raise FileNotFoundError("SAM2 source directory or checkpoint not found")
        sys.path.insert(0, str(root))
        import torch
        from sam2.build_sam import build_sam2_video_predictor

        self.torch = torch
        self.predictor = build_sam2_video_predictor(
            "configs/sam2.1/sam2.1_hiera_s.yaml", str(checkpoint), device=device,
        )
        # Hole filling requires an optional compiled extension absent on this Windows setup.
        self.predictor.fill_hole_area = 0
        self.device = self.predictor.device
        self.dtype = (torch.bfloat16 if self.device.type == "cuda"
                      and torch.cuda.is_bf16_supported() else torch.float32)
        self.keep_frames = max(self.predictor.max_obj_ptrs_in_encoder,
                               self.predictor.num_maskmem * self.predictor.memory_temporal_stride_for_eval) + 1
        self.state = None
        self.frame_index = 0

    def _precision(self):
        return (self.torch.autocast("cuda", dtype=self.dtype)
                if self.dtype == self.torch.bfloat16 else nullcontext())

    def reset(self):
        if self.state is not None:
            self.predictor.reset_state(self.state)
        self.state = None
        self.frame_index = 0

    def warmup(self):
        image = np.random.default_rng().integers(0, 256, (960, 1280, 3), dtype=np.uint8)
        with self.torch.inference_mode(), self._precision():
            self.predictor.forward_image(self._image(image).unsqueeze(0))
            if self.device.type == "cuda":
                self.torch.cuda.synchronize(self.device)

    def _image(self, image):
        size = self.predictor.image_size
        pixels = np.array(Image.fromarray(image).resize((size, size)))
        tensor = self.torch.from_numpy(pixels).permute(2, 0, 1).to(self.device).float() / 255.
        mean = tensor.new_tensor([.485, .456, .406])[:, None, None]
        std = tensor.new_tensor([.229, .224, .225])[:, None, None]
        return (tensor - mean) / std

    @staticmethod
    def _result(masks):
        mask = (masks[0, 0] > 0).detach().cpu().numpy()
        y, x = np.nonzero(mask)
        if not len(x):
            return {"box": None, "mask": mask}
        return {"box": [int(x.min()), int(y.min()), int(x.max()) + 1, int(y.max()) + 1], "mask": mask}

    def init(self, image, box):
        self.reset()
        try:
            with self.torch.inference_mode(), self._precision():
                # Match the upstream init_state schema without loading a video from disk.
                self.state = dict(
                    images={0: self._image(image)}, num_frames=1,
                    offload_video_to_cpu=False, offload_state_to_cpu=False,
                    video_height=image.shape[0], video_width=image.shape[1],
                    device=self.device, storage_device=self.device,
                    point_inputs_per_obj={}, mask_inputs_per_obj={}, cached_features={}, constants={},
                    obj_id_to_idx=OrderedDict(), obj_idx_to_id=OrderedDict(), obj_ids=[],
                    output_dict_per_obj={}, temp_output_dict_per_obj={}, frames_tracked_per_obj={},
                )
                _, _, masks = self.predictor.add_new_points_or_box(
                    self.state, frame_idx=0, obj_id=1, box=np.asarray(box, dtype=np.float32))
                self.predictor.propagate_in_video_preflight(self.state)
                return self._result(masks)
        except Exception:
            self.reset()
            raise

    def track(self, image):
        if self.state is None:
            raise RuntimeError("tracker is not initialized; call /init first")
        if image.shape[:2] != (self.state['video_height'], self.state['video_width']):
            raise ValueError("image dimensions changed; call /init again")
        try:
            with self.torch.inference_mode(), self._precision():
                self.frame_index += 1
                index = self.frame_index
                self.state['images'] = {index: self._image(image)}
                self.state['num_frames'] = index + 1
                result = None
                for _, _, masks in self.predictor.propagate_in_video(
                        self.state, start_frame_idx=index, max_frame_num_to_track=0):
                    result = self._result(masks)
                # Retain the prompt frame and every recent frame the model can attend to.
                for outputs in self.state['output_dict_per_obj'].values():
                    history = outputs['non_cond_frame_outputs']
                    for old in list(history):
                        if old <= index - self.keep_frames:
                            del history[old]
                for history in self.state['frames_tracked_per_obj'].values():
                    for old in list(history):
                        if old <= index - self.keep_frames:
                            del history[old]
                return result
        except Exception:
            self.reset()
            raise
