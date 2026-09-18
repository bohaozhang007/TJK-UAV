"""Apply the small v22 read-only map extension to its independent pinned EGO checkout."""
import argparse
import re
from pathlib import Path


def patch(root):
    env = root / 'swarm-playground/main_ws/src/planner/plan_env'
    header = env / 'include/plan_env/grid_map.h'
    source = env / 'src/grid_map.cpp'
    h, s = header.read_text(), source.read_text()
    if 'v22Query' in h:
        if ('v22_cloud_stamp_ = img->header.stamp' not in s
                or not s.endswith(Path(__file__).with_name('map_query.cpp').read_text())):
            raise RuntimeError('incomplete existing v22 patch')
        return
    h = h.replace('#include <plan_env/raycast.h>',
        '#include <plan_env/raycast.h>\n#include <std_srvs/Trigger.h>\n#include <sstream>\n#include <iomanip>\n#include <cstdint>')
    h = h.replace('  ros::Subscriber indep_cloud_sub_',
        '  bool v22Query(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &);\n'
        '  ros::ServiceServer v22_query_;\n  ros::Time v22_cloud_stamp_;\n  uint64_t v22_map_version_ = 0;\n'
        '  ros::Subscriber indep_cloud_sub_')
    s = s.replace('  node_ = nh;', '  node_ = nh;\n  v22_query_ = node_.advertiseService("grid_map/query", &GridMap::v22Query, this);', 1)
    s, count = re.subn(r'\n}(\s*\nvoid GridMap::extrinsicCallback)',
                      r'\n  v22_cloud_stamp_ = img->header.stamp;\n  ++v22_map_version_;\n}\1', s)
    if count != 1:
        raise RuntimeError('pinned cloud callback layout changed')
    s += '\n' + Path(__file__).with_name('map_query.cpp').read_text()
    if 'std_srvs/Trigger.h' not in h or 'bool v22Query' not in h:
        raise RuntimeError('pinned header layout changed')
    header.write_text(h)
    source.write_text(s)
    cmake = env / 'CMakeLists.txt'
    cmake.write_text(cmake.read_text().replace('  std_msgs\n', '  std_msgs\n  std_srvs\n', 1)
                    .replace(' CATKIN_DEPENDS roscpp std_msgs', ' CATKIN_DEPENDS roscpp std_msgs std_srvs'))
    package = env / 'package.xml'
    package.write_text(package.read_text().replace('</package>', '  <depend>std_srvs</depend>\n</package>'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    patch(parser.parse_args().upstream)
