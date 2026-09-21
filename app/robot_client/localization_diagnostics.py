"""Save exposure/mask/occupancy projection evidence without changing localization."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw


def save_projection(prefix, obs, mask, diagnostics):
    prefix = Path(prefix)
    summary = {k:v for k,v in diagnostics.items() if k not in ('uv', 'depth_m', 'hits')}
    summary['stage'] = ('map_empty' if not summary.get('occupied_voxels') else
        'no_voxels_in_front' if not summary.get('camera_front_voxels') else
        'no_voxels_in_image' if not summary.get('image_voxels') else
        'no_mask_overlap' if not summary.get('mask_hit_voxels') else
        'localized' if 'target_position_cm' in summary else 'insufficient_surface_support')
    uv = diagnostics.get('uv', np.empty((0, 2), int))
    depth = diagnostics.get('depth_m', np.empty(0))
    hits = diagnostics.get('hits', np.empty(0, bool))
    height, width = mask.shape
    ys, xs = np.nonzero(mask)
    bbox = [int(xs.min()), int(ys.min()), int(xs.max()+1), int(ys.max()+1)] if len(xs) else None
    if len(uv) and mask.any():
        distance = cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 5)
        nearest = float(distance[uv[:, 1], uv[:, 0]].min())
    else:
        nearest = None
    exposure = prefix.parent / ('exposure_'+hashlib.sha256(obs.frame_id.encode()).hexdigest()[:16]+'.png')
    if not exposure.exists():
        Image.fromarray(obs.rgb).save(exposure, compress_level=1)
    mask_file = prefix.with_name(prefix.name+'_mask.png')
    Image.fromarray(mask.astype(np.uint8)*255).save(mask_file)
    base = np.asarray(obs.rgb).copy()
    tinted = base.copy()
    tinted[mask] = (base[mask]*.55+np.array([0, 255, 255])*.45).astype(np.uint8)
    projected = Image.fromarray(base.copy())
    draw = ImageDraw.Draw(projected)
    ceiling = max(.1, float(np.percentile(depth, 95))) if len(depth) else 1.
    if len(depth):
        colors = cv2.applyColorMap(np.uint8(np.clip(depth/ceiling, 0, 1)*255).reshape(-1, 1),
                                  cv2.COLORMAP_TURBO)[:, 0, ::-1]
        # Draw distant points first; near points remain visible at overlaps.
        for i in np.argsort(depth)[::-1]:
            x, y = map(int, uv[i]); color = tuple(map(int, colors[i]))
            draw.ellipse((x-2, y-2, x+2, y+2), fill=color)
        for x, y in uv[hits]:
            x,y=int(x),int(y)
            draw.ellipse((x-4,y-4,x+4,y+4), outline=(255,0,255), width=2)
    contour, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    overlay = np.asarray(projected).copy()
    cv2.drawContours(overlay, contour, -1, (0,255,255), 2)
    panels = [Image.fromarray(base), Image.fromarray(tinted), Image.fromarray(overlay)]
    if 'target_position_cm' in summary:
        point = np.array(summary['target_position_cm'])*[1,-1,1]
        camera = (point-obs.world_from_camera_cm[:3,3]) @ obs.world_from_camera_cm[:3,:3]
        pixel = obs.intrinsics @ camera
        x,y = map(int, np.rint(pixel[:2]/pixel[2]))
        draw = ImageDraw.Draw(panels[2]); draw.line((x-10,y,x+10,y),fill=(0,255,0),width=3)
        draw.line((x,y-10,x,y+10),fill=(0,255,0),width=3)
    panel_width = min(width, 960); panel_height = round(height*panel_width/width)
    canvas = Image.new('RGB', (3*panel_width, panel_height+80), (24,24,24))
    draw = ImageDraw.Draw(canvas)
    titles = ['Rectified exposure', 'Mask: cyan', 'Occupied voxel centers: depth colors; mask hits: magenta']
    for i,panel in enumerate(panels):
        canvas.paste(panel.resize((panel_width,panel_height)),(i*panel_width,40))
        draw.text((i*panel_width+8,12),titles[i],fill='white')
    line = 'occupied={}  front={}  image={}  mask_hits={}  depth scale=0..{:.2f} m (blue..red)'.format(
        summary.get('occupied_voxels',0),summary.get('camera_front_voxels',0),
        summary.get('image_voxels',0),summary.get('mask_hit_voxels',0),ceiling)
    draw.text((8,panel_height+50),line,fill='white')
    overlay_file = prefix.with_name(prefix.name+'_projection.jpg')
    canvas.save(overlay_file, quality=95)
    summary.update(frame_id=obs.frame_id, localization_epoch=obs.epoch, timestamp_s=obs.timestamp_s,
        exposure_pose=obs.pose, intrinsics=obs.intrinsics.tolist(),
        world_from_camera_optical_cm=obs.world_from_camera_cm.tolist(), exposure_metadata=obs.metadata,
        image_size=[width,height], mask_bbox_xyxy=bbox, nearest_projection_to_mask_px=nearest,
        projected_depth_range_m=[float(depth.min()),float(depth.max())] if len(depth) else None,
        exposure_file=exposure.name, mask_file=mask_file.name, overlay_file=overlay_file.name,
        limitations=['Planner occupied voxel centers, not raw lidar returns.',
                     'Zero mask hits alone cannot distinguish missing lidar returns from map filtering or projection error.'])
    report = prefix.with_suffix('.json')
    report.write_text(json.dumps(summary, indent=2, allow_nan=False), encoding='utf-8')
    result = {k:summary.get(k) for k in ('stage','occupied_voxels','camera_front_voxels','image_voxels',
        'mask_hit_voxels','nearest_projection_to_mask_px','projected_depth_range_m','overlay_file')}
    result['report_file'] = report.name
    return result
