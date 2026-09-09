"""Calibrated observation assembly, independent of ROS subscription lifetimes."""
import base64
import math
import numpy as np

def rotation(q):
    q = np.asarray(q,dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .02:
        raise ValueError('invalid attitude quaternion')
    x,y,z,w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def interpolate_pose(history, stamp, max_sync):
    before = [p for p in history if p['stamp'] <= stamp]
    after = [p for p in history if p['stamp'] >= stamp]
    if not before or not after:
        raise ValueError('exposure not bracketed by odometry')
    a,b = before[-1],after[0]
    sync = max(stamp-a['stamp'],b['stamp']-stamp)
    if sync > max_sync or a['frame'] != b['frame'] or a['body'] != b['body']:
        raise ValueError('odometry/exposure synchronization failed')
    f = 0 if b['stamp']==a['stamp'] else (stamp-a['stamp'])/(b['stamp']-a['stamp'])
    qa,qb = np.array(a['q']),np.array(b['q'])
    rotation(qa); rotation(qb)
    dot = float(qa@qb)
    if dot < 0:
        qb, dot = -qb,-dot
    if dot > .9995:
        q = qa + f*(qb-qa)
    else:
        angle = math.acos(np.clip(dot,-1,1))
        q = (math.sin((1-f)*angle)*qa+math.sin(f*angle)*qb)/math.sin(angle)
    q /= np.linalg.norm(q)
    transform = np.eye(4)
    transform[:3,:3] = rotation(q)
    transform[:3,3] = (1-f)*np.array(a['xyz'])+f*np.array(b['xyz'])
    return transform,sync,a['frame'],a['body']


def transform_sync_error(edges, source, target, stamp, maximum):
    """Bound support-sample distance on every dynamic edge of the TF chain.

    tf2's interpolated result is stamped at the requested time, which alone does
    not bound the age of gimbal samples used for interpolation.
    """
    pending = [(source, 0., set())]
    while pending:
        frame,error,seen = pending.pop()
        if frame == target:
            return error
        for (parent,child),samples in edges.items():
            if frame not in (parent,child):
                continue
            other = child if frame == parent else parent
            if other in seen:
                continue
            edge_error = 0.
            if samples is not None:
                before = [t for t in samples if t <= stamp]
                after = [t for t in samples if t >= stamp]
                if not before or not after:
                    continue
                edge_error = max(stamp-max(before),min(after)-stamp)
                if edge_error > maximum:
                    continue
            pending.append((other,max(error,edge_error),seen|{frame}))
    raise ValueError('camera TF chain lacks sufficiently synchronized support samples')


def camera_intrinsics(hardware, image, info):
    """Return K, D, rectification truth and explicit source metadata."""
    mode = hardware.get('intrinsics_mode','camera_info')
    if mode == 'approximate_fov':
        fov = hardware.get('assumed_horizontal_fov_deg')
        if isinstance(fov,bool) or not isinstance(fov,(int,float)) or not math.isfinite(fov) or not 1 < fov < 179:
            raise ValueError('approximate intrinsics require horizontal FOV in (1,179) degrees')
        if image.width <= 0 or image.height <= 0:
            raise ValueError('invalid image dimensions')
        f = image.width/(2*math.tan(math.radians(fov)/2))
        k = np.array([[f,0,image.width/2],[0,f,image.height/2],[0,0,1.]])
        return k,None,False,dict(intrinsics='approximate_fov',
            assumed_horizontal_fov_deg=fov,principal_point='image_center',
            square_pixels_assumed=True,distortion='unknown_not_corrected')
    if mode != 'camera_info':
        raise ValueError('unknown intrinsics_mode: '+str(mode))
    if info is None:
        raise ValueError('CameraInfo unavailable')
    if (info.width,info.height)!=(image.width,image.height) or info.header.frame_id != image.header.frame_id:
        raise ValueError('CameraInfo/image dimensions or frames mismatch')
    if info.binning_x > 1 or info.binning_y > 1 or info.roi.width or info.roi.height:
        raise ValueError('cropped/binned CameraInfo requires normalized calibration')
    k = np.asarray(info.K,dtype=float).reshape(3,3)
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]):
        raise ValueError('missing/invalid camera calibration')
    if info.distortion_model not in ('plumb_bob','rational_polynomial') or len(info.D) not in (4,5,8,12,14):
        raise ValueError('unsupported or missing lens distortion calibration')
    d = np.asarray(info.D,dtype=float)
    if not np.isfinite(d).all():
        raise ValueError('nonfinite distortion calibration')
    return k,d,True,dict(intrinsics='camera_info',distortion='corrected')


def fixed_optical_rotation(hardware):
    """ROS body FLU <- optical RDF, with explicit fixed mount pitch about body Y."""
    r = hardware.get('body_from_camera_optical_rotation')
    if r is None:
        pitch = hardware.get('fixed_camera_pitch_deg')
        if isinstance(pitch,bool) or not isinstance(pitch,(int,float)) or not math.isfinite(pitch) or not -90 <= pitch <= 90:
            raise ValueError('body_coincident_fixed requires an explicit proper camera optical rotation or fixed pitch')
        a = math.radians(pitch)
        r = np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]]) @ np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    r = np.asarray(r,dtype=float)
    if (r.shape != (3,3) or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(r),1.,atol=1e-6)):
        raise ValueError('body_coincident_fixed requires an explicit proper camera optical rotation')
    return r


def build_observation(hw):
    import cv2
    from .owl_ego import public_pose
    m,info,hist,epoch,edges = hw.camera_snapshot()
    config = hw.c
    if m is None:
        raise ValueError('RGB unavailable')
    stamp = m.header.stamp.to_sec()
    age = hw.now_s()-stamp
    if stamp <= 0 or not 0 <= age <= config['hardware']['rgb_max_age_s']:
        raise ValueError('stale RGB acquisition timestamp')
    k,d,rectified,geometry = camera_intrinsics(config['hardware'],m,info)
    world_body,sync,world,body = interpolate_pose(hist,stamp,config['hardware']['sync_max_s'])
    mode = config['hardware'].get('extrinsics_mode','tf')
    body_camera = np.eye(4)
    if mode == 'body_coincident_fixed':
        # User-authorized approximation: zero camera lever arm. Orientation is
        # independent of that approximation and must be explicitly supplied.
        body_camera[:3,:3] = fixed_optical_rotation(config['hardware'])
    elif mode == 'tf':
        optical = config['hardware']['camera_optical_frame']
        if not optical or m.header.frame_id != optical:
            raise ValueError('camera optical frame must be explicitly calibrated and match RGB frame')
        sync = max(sync,transform_sync_error(edges,body,optical,stamp,config['hardware']['sync_max_s']))
        tf = hw.camera_transform(body,optical,m.header.stamp)
        tr,qr = tf.transform.translation,tf.transform.rotation
        body_camera[:3,:3] = rotation([qr.x,qr.y,qr.z,qr.w])
        body_camera[:3,3] = [tr.x,tr.y,tr.z]
        tf_stamp = tf.header.stamp.to_sec()
        if tf_stamp:
            sync = max(sync,abs(tf_stamp-stamp))
        if sync > config['hardware']['sync_max_s'] or not np.isfinite(body_camera).all():
            raise ValueError('invalid/stale camera TF')
    else:
        raise ValueError('unknown camera extrinsics_mode: '+str(mode))
    bgr = hw.rgb_array(m)
    if d is not None:
        bgr = cv2.undistort(bgr,k,d,None,k)
    scale = min(1.,config['hardware']['long_edge_px']/max(m.width,m.height))
    w,h = round(m.width*scale),round(m.height*scale)
    bgr = cv2.resize(bgr,(w,h))
    k = np.diag([w/m.width,h/m.height,1.])@k
    success,encoded = cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,80])
    if not success:
        raise ValueError('JPEG encoding failed')
    transform = world_body@body_camera
    transform[:3,3] *= 100
    yaw = math.atan2(world_body[1,0],world_body[0,0])
    if epoch != hw.current_epoch():
        raise ValueError('localization changed during observation')
    geometry['extrinsics'] = mode
    geometry['camera_translation'] = 'body_coincident_assumption' if mode == 'body_coincident_fixed' else 'tf'
    quality = 'approximate' if not rectified or mode != 'tf' else 'calibrated'
    return dict(ok=True,calibration_quality=quality,geometry_assumptions=geometry,frame_id=epoch+':'+str(m.header.stamp.to_nsec()),timestamp_s=stamp,
                age_s=hw.now_s()-stamp,sync_error_s=sync,
                localization_epoch=epoch,world_frame=world,pose=public_pose([*world_body[:3,3],yaw]),
                image_size=[w,h],rectified=rectified,rgb_jpeg_base64=base64.b64encode(encoded).decode('ascii'),
                intrinsics=k.tolist(),world_from_camera_optical_cm=transform.tolist())

