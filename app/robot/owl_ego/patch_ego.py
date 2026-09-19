"""Patch the independent v22 EGO checkout with map queries and persistent planning."""
import argparse
import re
from pathlib import Path


def patch_map(root):
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


def patch_planner(root):
    folder = root / 'swarm-playground/main_ws/src/planner/plan_manage'
    header = folder / 'include/plan_manage/ego_replan_fsm.h'
    source = folder / 'src/ego_replan_fsm.cpp'
    h, s = header.read_text(), source.read_text()
    extension = Path(__file__).with_name('planner_control.cpp').read_text()
    marker = '// v22 planner control:'
    if marker in s:
        if 'bool v22Plan' not in h or 'v22Publish(poly_msg);' not in s:
            raise RuntimeError('incomplete existing v22 planner patch')
        updated = s[:s.index(marker)] + extension
        if updated != s:
            source.write_text(updated)
        return

    def replace(text, old, new, count=1):
        if text.count(old) != count:
            raise RuntimeError('pinned planner layout changed: ' + old)
        return text.replace(old, new)

    h = replace(h, '#include <traj_utils/PolyTraj.h>',
        '#include <traj_utils/PolyTraj.h>\n#include <owl_nav_v22/Plan.h>\n'
        '#include <owl_nav_v22/PlannerTrajectory.h>\n#include <std_msgs/String.h>\n#include <stdexcept>')
    h = replace(h, '    /* ROS utils */',
        '    bool v22Plan(owl_nav_v22::Plan::Request &, owl_nav_v22::Plan::Response &);\n'
        '    void v22Idle();\n    void v22Publish(const traj_utils::PolyTraj &);\n'
        '    ros::ServiceServer v22_plan_;\n    ros::Publisher v22_trajectory_pub_;\n'
        '    std::string v22_generation_;\n    uint64_t v22_sequence_ = 0;\n'
        '    bool v22_preview_ = false;\n\n    /* ROS utils */')
    s = replace(s, '    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);',
        '    v22_plan_ = nh.advertiseService("planning/command", &EGOReplanFSM::v22Plan, this);\n'
        '    v22_trajectory_pub_ = nh.advertise<owl_nav_v22::PlannerTrajectory>("planning/trajectory", 10);')
    s = replace(s, 'nh.advertise<std_msgs::Empty>("planning/heartbeat", 10)',
                  'nh.advertise<std_msgs::String>("planning/heartbeat", 10)')
    s = replace(s, '    std_msgs::Empty heartbeat_msg;',
                  '    std_msgs::String heartbeat_msg;\n    heartbeat_msg.data = v22_generation_;')
    s = replace(s, 'poly_traj_pub_.publish(poly_msg);', 'v22Publish(poly_msg);', 2)
    s = replace(s, 'broadcast_ploytraj_pub_.publish(MINCO_msg);',
                  'if (!v22_preview_ && !v22_generation_.empty()) broadcast_ploytraj_pub_.publish(MINCO_msg);', 2)
    s = replace(s, '      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);',
                  '      // Goals enter only through the fenced v22 service.')
    cmake = folder / 'CMakeLists.txt'
    c = replace(cmake.read_text(), '  message_generation\n', '  message_generation\n  owl_nav_v22\n')
    c = replace(c, '#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})',
                   'add_dependencies(ego_planner_node ${catkin_EXPORTED_TARGETS})')
    package = folder / 'package.xml'
    p = replace(package.read_text(), '</package>', '  <depend>owl_nav_v22</depend>\n</package>')
    header.write_text(h)
    source.write_text(s + '\n' + extension)
    cmake.write_text(c)
    package.write_text(p)


def patch(root):
    patch_map(root)
    patch_planner(root)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    patch(parser.parse_args().upstream)
