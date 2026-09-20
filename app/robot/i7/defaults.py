"""i7 wiring; shared v22 flight and planner protocol."""
from copy import deepcopy
from app.robot.owl_ego.defaults import DEFAULTS as EGO_DEFAULTS

DEFAULTS = deepcopy(EGO_DEFAULTS)
DEFAULTS['namespace'] = '/i7_ego_v22'
DEFAULTS['control'].update(mavros_frame_profile='standard_enu', world_frame='camera_init')
DEFAULTS['hardware'].update(camera_optical_frame='i7_camera_optical')
DEFAULTS['topics'].update(odom='/i7_v22/odometry', rgb='/i7_v22/image_raw',
                         camera_info='/i7_v22/camera_info', cloud='/laserMapping/cloud_registered',
                         bridge_status='/i7_ego_v22/status', command='/i7_ego_v22/command',
                         localization_reset='/i7_ego_v22/localization_reset')
DEFAULTS['camera_preflight'] = {'profile': None, 'max_age_s': .5}
DEFAULTS['sensor_geometry']['frames'].update(camera_optical='i7_camera_optical',
    lidar='livox_frame', imu='i7_imu', body='base_link')
DEFAULTS['camera'] = dict(host='192.168.144.64', port=1030, timeout_s=3.,
    rtsp_url='rtsp://127.0.0.1:8554/k40t', baseline_yaw_deg=0., baseline_pitch_deg=0.,
    baseline_zoom=1., settle_s=.5, observation_settle_s=2.)
DEFAULTS['source'] = dict(odom='/laserMapping/odometry', world_frame='camera_init',
                         body_frame='body')
DEFAULTS['autofocus'] = dict(tracker_timeout_s=20., reference_width_px=640,
    reference_height_px=360, reference_zoom=1., yaw_deg_per_pixel=.1, pitch_deg_per_pixel=.1,
    max_yaw_step_deg=3., max_pitch_step_deg=3., damping=.7, zoom_step_up=1.25,
    zoom_step_down=.8, target_ratio=.4, center_tolerance=.06, size_tolerance=.08,
    max_steps=30, timeout_s=180., settle_s=.5, hold_s=1.)

DEFAULTS['bringup'] = dict(startup_grace_s=2., shutdown_poll_interval_s=.1,
    interrupt_shutdown_timeout_s=8., terminate_shutdown_timeout_s=3.,
    mavros_fcu_url='/dev/ttyTHS0:921600',
    mavros_gcs_url='udp://0.0.0.0:14555@192.168.31.240:14550',
    mediamtx_binary='/home/jkhk/workspace/jetson-core/libs/mediamtx/arm64/mediamtx',
    mediamtx_rtsp_address=':8554', camera_source_url='rtsp://192.168.144.64:558/live/single',
    camera_source_on_demand='yes')
