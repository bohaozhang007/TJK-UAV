"""Fixed OWL interfaces and runtime defaults; field tuning lives in config/owl_ego.yaml."""

DEFAULTS = {
    "namespace": "/owl_ego_v22",
    "queries": {
        "preview_timeout_s": 12
    },
    "controller": {
        "motion_timeout_s": 15,
        "takeoff_timeout_s": 60,
        "landing_timeout_s": 90
    },
    "hardware": {
        "bridge_timeout_s": 1,
        "rgb_max_age_s": 0.4,
        "sync_max_s": 0.05,
        "intrinsics_mode": "configured_calibration",
        "camera_optical_frame": "camera_link",
        "extrinsics_mode": "sensor_geometry"
    },
    "control": {
        "mavros_frame_profile": "owl_vendor_world",
        "control_hz": 50,
        "sensor_timeout_s": 1.5,
        "state_timeout_s": 2,
        "planner_timeout_s": 1,
        "planning_timeout_s": 10,
        "task_timeout_s": 120,
        "trajectory_grace_s": 5,
        "stable_samples": 5,
        "takeoff_max_lead_m": 0.2,
        "takeoff_progress_timeout_s": 10,
        "cloud_sync_max_s": 0.1,
        "world_frame": "world"
    },
    "planner": {
        "commit": "9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010"
    },
    "topics": {
        "odom": "/mavros/local_position/odom",
        "rgb": "/visbot_media_g/gimbal_camera/image_raw",
        "gimbal_imu": "/gimbal_controller/imu",
        "camera_info": "/visbot_media_g/gimbal_camera/camera_info",
        "state": "/mavros/state",
        "extended_state": "/mavros/extended_state",
        "cloud": "/ego_planner_node/grid_map/cloud",
        "localization_reset": "/owl_ego_v22/localization_reset",
        "vision_pose_reset": "/mavros/vision_pose/pose_reset",
        "bridge_status": "/owl_ego_v22/status",
        "command": "/owl_ego_v22/command",
        "setpoint": "/mavros/setpoint_raw/local"
    },
    "camera_preflight": {
        "profile": "owl_fixed_gimbal",
        "max_age_s": 0.5
    },
    "sensor_geometry": {
        "units": "m",
        "frames": {
            "camera_optical": "camera_link",
            "lidar": "livox_frame",
            "imu": "flight_controller_imu_flu",
            "body": "base_link"
        },
        "axes": {
            "camera_optical": "right, down, forward",
            "imu": "forward, left, up",
            "body": "forward, left, up"
        }
    }
}
