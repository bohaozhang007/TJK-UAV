"""Reuse the detector's asynchronous box/mask renderer."""
from app.detector.image_log import ImageLog as OverlayLog


class ImageLog(OverlayLog):
    def submit(self, image, result, target_id):
        items = [] if result['box'] is None else [
            {**result, 'label': f'target {target_id}'}]
        super().submit(image, items, prefix=f'{target_id}_')
