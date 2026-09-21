"""Save mask/occupancy projection evidence without changing localization."""
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
from app.robot.mapping import project_camera


def save_projection(prefix, obs, mask, diagnostics, cloud=None):
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
    base = np.asarray(obs.rgb).copy()
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
    panel = Image.fromarray(overlay)
    if 'target_position_cm' in summary:
        point = np.array(summary['target_position_cm'])*[1,-1,1]
        camera = (point-obs.world_from_camera_cm[:3,3]) @ obs.world_from_camera_cm[:3,:3]
        pixel = project_camera(camera.reshape(1, 3), obs.intrinsics, obs.distortion)[0]
        x,y = map(int, np.rint(pixel))
        draw = ImageDraw.Draw(panel); draw.line((x-10,y,x+10,y),fill=(0,255,0),width=3)
        draw.line((x,y-10,x,y+10),fill=(0,255,0),width=3)
    panel_width = min(width, 960); panel_height = round(height*panel_width/width)
    cloud_panel = None
    cloud_summary = None
    if cloud is not None:
        cloud_panel = Image.fromarray(base.copy())
        cloud_summary = dict(cloud.get('metadata', {}))
        if 'error' in cloud:
            cloud_summary['error'] = cloud['error']
        else:
            camera = (cloud['points_world_cm']-obs.world_from_camera_cm[:3,3]) @ obs.world_from_camera_cm[:3,:3]
            camera = camera[camera[:,2] > 1e-3]
            pixels = np.rint(project_camera(camera, obs.intrinsics, obs.distortion))
            inside = (np.isfinite(pixels).all(axis=1) & (pixels[:,0] >= 0) & (pixels[:,0] < width)
                      & (pixels[:,1] >= 0) & (pixels[:,1] < height))
            pixels, depths = pixels[inside].astype(int), camera[inside,2]/100
            cloud_hits = mask[pixels[:,1],pixels[:,0]]
            cloud_summary.update(camera_front_points=len(camera), image_points=len(pixels),
                                 mask_hit_points=int(cloud_hits.sum()))
            draw_cloud = ImageDraw.Draw(cloud_panel)
            if len(depths):
                colors = cv2.applyColorMap(np.uint8(np.clip(depths/ceiling,0,1)*255).reshape(-1,1),
                                           cv2.COLORMAP_TURBO)[:,0,::-1]
                for i in np.argsort(depths)[::-1]:
                    x,y = map(int,pixels[i])
                    draw_cloud.ellipse((x-1,y-1,x+1,y+1),fill=tuple(map(int,colors[i])))
                for x,y in pixels[cloud_hits]:
                    x,y = int(x),int(y)
                    draw_cloud.ellipse((x-3,y-3,x+3,y+3),outline=(255,0,255),width=1)
        cloud_overlay = np.asarray(cloud_panel).copy()
        cv2.drawContours(cloud_overlay,contour,-1,(0,255,255),2)
        cloud_panel = Image.fromarray(cloud_overlay)
    canvas = Image.new('RGB', (panel_width*(2 if cloud_panel is not None else 1), panel_height+100), (24,24,24))
    draw = ImageDraw.Draw(canvas)
    canvas.paste(panel.resize((panel_width,panel_height)),(0,40))
    draw.text((8,12),'Occupancy: mask cyan; depth colors; hits magenta; target green',fill='white')
    if cloud_panel is not None:
        canvas.paste(cloud_panel.resize((panel_width,panel_height)),(panel_width,40))
        draw.text((panel_width+8,12),'Registered point cloud: mask cyan; depth colors; hits magenta',fill='white')
        if 'error' in cloud_summary:
            cloud_line = 'POINT CLOUD UNAVAILABLE - see JSON error'
        else:
            cloud_line = 'points={} image={} mask_hits={} image delta={:+.3f}s'.format(
                cloud_summary['point_count'],cloud_summary['image_points'],
                cloud_summary['mask_hit_points'],cloud_summary['image_delta_s'])
        draw.text((panel_width+8,panel_height+50),cloud_line,fill='white')
    line = 'occupied={}  front={}  image={}  mask_hits={}  depth scale=0..{:.2f} m (blue..red)'.format(
        summary.get('occupied_voxels',0),summary.get('camera_front_voxels',0),
        summary.get('image_voxels',0),summary.get('mask_hit_voxels',0),ceiling)
    draw.text((8,panel_height+50),line,fill='white')
    draw.text((8,panel_height+75),'Shared depth scale; no occlusion removal. Cloud is one scan, occupancy accumulates observations.',fill='white')
    overlay_file = prefix.with_name(prefix.name+'_projection.jpg')
    canvas.save(overlay_file, quality=95)
    summary.update(frame_id=obs.frame_id, localization_epoch=obs.epoch, timestamp_s=obs.timestamp_s,
        exposure_pose=obs.pose, intrinsics=obs.intrinsics.tolist(),
        pixel_geometry='raw_distorted' if obs.distortion is not None else 'rectified',
        distortion_coefficients=obs.distortion.tolist() if obs.distortion is not None else None,
        world_from_camera_optical_cm=obs.world_from_camera_cm.tolist(), exposure_metadata=obs.metadata,
        image_size=[width,height], mask_bbox_xyxy=bbox, nearest_projection_to_mask_px=nearest,
        projected_depth_range_m=[float(depth.min()),float(depth.max())] if len(depth) else None,
        overlay_file=overlay_file.name,
        point_cloud=cloud_summary,
        limitations=['Planner occupied voxel centers, not raw lidar returns.',
                     'Zero mask hits alone cannot distinguish missing lidar returns from map filtering or projection error.'])
    report = prefix.with_suffix('.json')
    report.write_text(json.dumps(summary, indent=2, allow_nan=False), encoding='utf-8')
    result = {k:summary.get(k) for k in ('stage','occupied_voxels','camera_front_voxels','image_voxels',
        'mask_hit_voxels','nearest_projection_to_mask_px','projected_depth_range_m','overlay_file')}
    result['report_file'] = report.name
    return result
