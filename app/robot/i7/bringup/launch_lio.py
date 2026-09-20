"""Launch the vendor binary with an isolated v22 fixed-geometry configuration."""
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import rosgraph
import rospy
import yaml

from app.robot.config_loader import load_robot_config
from app.robot.i7.mapping import parameters, fingerprint, require_running
from app.robot.i7.bringup.reuse_ros_component import run


def main():
    config = load_robot_config('i7', os.environ.get('I7_V22_CONFIG'))
    nodes = ['/laserMapping', '/lio_to_mavros']
    state = rosgraph.Master('/i7_lio_start_check').getSystemState()
    registered = {name for group in state for _, names in group for name in names}
    if set(nodes) & registered:
        require_running(config, rospy)
        run(nodes, [])
        return
    params = yaml.safe_load(Path(__file__).with_name('lio.yaml').read_text())
    params['mapping'].update(parameters(config))
    root = Path(__file__).resolve().parents[4]
    logs = root/'logs/i7_v22/trajectories'
    logs.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='i7_v22_lio_') as directory:
        folder = Path(directory)
        param_file = folder/'lio.yaml'
        param_file.write_text(yaml.safe_dump(params))
        launch = ET.Element('launch')
        ET.SubElement(launch, 'rosparam', command='load', file=str(param_file))
        for key, value in dict(localization_mode_en='false', savePCD='false').items():
            ET.SubElement(launch, 'param', name=key, value=value, type='bool')
        node = ET.SubElement(launch, 'node', pkg='faster_lio', type='run_mapping_online',
                             name='laserMapping', output='screen', required='true',
                             args='--traj_log_dir='+str(logs))
        ET.SubElement(node, 'param', name='i7_geometry_fingerprint', value=fingerprint(config))
        ET.SubElement(launch, 'node', pkg='faster_lio', type='lio_to_mavros', name='lio_to_mavros',
                      output='screen', required='true')
        launch_file = folder/'mapping.launch'
        ET.ElementTree(launch).write(str(launch_file), encoding='unicode')
        raise SystemExit(subprocess.call(['roslaunch', str(launch_file)]))


if __name__ == '__main__':
    main()
