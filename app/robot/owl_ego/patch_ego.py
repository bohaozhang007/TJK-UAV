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
        marker = '// Included in the pinned grid_map.cpp'
        if 'v22_cloud_stamp_ = img->header.stamp' not in s or marker not in s:
            raise RuntimeError('incomplete existing v22 patch')
        updated = s[:s.index(marker)] + Path(__file__).with_name('map_query.cpp').read_text()
        if updated != s:
            source.write_text(updated)
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


def patch_diagnostics(root):
    planner = root / 'swarm-playground/main_ws/src/planner'
    header = planner / 'plan_env/include/plan_env/v22_diagnostics.h'
    content = Path(__file__).with_name('planning_diagnostics.h').read_text()
    if not header.exists() or header.read_text() != content:
        header.write_text(content)

    def replace(text, old, new, count=1):
        if text.count(old) != count:
            raise RuntimeError('pinned diagnostics layout changed: ' + old)
        return text.replace(old, new)

    def update(relative, transform):
        path = planner / relative
        original = path.read_text()
        marker = '// v22 planning diagnostics\n'
        if marker not in original:
            path.write_text(marker + transform(original))

    def grid_header(s):
        s = replace(s, '#include <plan_env/raycast.h>',
                    '#include <plan_env/raycast.h>\n#include <plan_env/v22_diagnostics.h>')
        return replace(s, '  void initMap(ros::NodeHandle &nh);', '''  V22PlanningDiagnostics v22_diagnostics;
  std::string v22PointState(const Eigen::Vector3d &point);
  std::string v22PointInfo(const Eigen::Vector3d &point);
  std::string v22MapInfo();
  void initMap(ros::NodeHandle &nh);''')

    def manager_header(s):
        return replace(s, '    /* main planning interface */', '''    void v22BeginAttempt(int failures) { continous_failures_count_ = failures; }
    void v22Reset()
    {
      continous_failures_count_ = 0;
      traj_.global_traj = GlobalTrajData{};
      traj_.local_traj = LocalTrajData{};
      traj_.local_traj.drone_id = -1;
      traj_.local_traj.start_pos.setZero();
    }

    /* main planning interface */''')

    def manager_source(s):
        begin = s.index('  bool EGOPlannerManager::reboundReplan(')
        end = s.index('  bool EGOPlannerManager::computeInitState(', begin)
        part = s[begin:end]
        part = replace(part, '    poly_traj::MinJerkOpt initMJO;', '''    auto &v22_diag = grid_map_->v22_diagnostics;
    ros::WallTime v22_stage = ros::WallTime::now();
    poly_traj::MinJerkOpt initMJO;''')
        part = replace(part, '      return false;\n    }\n\n    Eigen::MatrixXd cstr_pts', '''      v22_diag.add("initialization", "failed", (ros::WallTime::now()-v22_stage).toSec());
      return false;
    }
    v22_diag.add("initialization", "success", (ros::WallTime::now()-v22_stage).toSec());
    v22_stage = ros::WallTime::now();

    Eigen::MatrixXd cstr_pts''')
        part = replace(part, '      return false;\n    }\n\n    t_init', '''      v22_diag.add("initial_constraints", "failed", (ros::WallTime::now()-v22_stage).toSec());
      return false;
    }
    v22_diag.add("initial_constraints", "success", (ros::WallTime::now()-v22_stage).toSec());
    v22_stage = ros::WallTime::now();

    t_init''')
        part = replace(part, '    /*** STEP 3: Store and display results ***/', '''    v22_diag.add("optimization", flag_success ? "success" : "failed", (ros::WallTime::now()-v22_stage).toSec());
    /*** STEP 3: Store and display results ***/''')
        return s[:begin]+part+s[end:]

    def astar_source(s):
        s = replace(s, '    ros::Time time_1 = ros::Time::now();', '''    const ros::WallTime v22_started = ros::WallTime::now();
    auto v22_result = [&](ASTAR_RET code, const std::string &reason) {
        grid_map_->v22_diagnostics.add("astar", reason, (ros::WallTime::now()-v22_started).toSec(),
            ",\\\"start\\\":" + grid_map_->v22PointInfo(start_pt) + ",\\\"end\\\":" + grid_map_->v22PointInfo(end_pt));
        return code;
    };
    ros::Time time_1 = ros::Time::now();''')
        s = replace(s, 'return ASTAR_RET::INIT_ERR;', 'return v22_result(ASTAR_RET::INIT_ERR, "endpoint_adjustment_failed");')
        s = replace(s, 'return ASTAR_RET::SUCCESS;', 'return v22_result(ASTAR_RET::SUCCESS, "success");')
        s = replace(s, '            return ASTAR_RET::SEARCH_ERR;', '            return v22_result(ASTAR_RET::SEARCH_ERR, "search_timeout");')
        s = replace(s, '    return ASTAR_RET::SEARCH_ERR;', '    return v22_result(ASTAR_RET::SEARCH_ERR, "no_path");')
        return s

    def optimizer_source(s):
        s = replace(s, '      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");', '''      grid_map_->v22_diagnostics.add("optimization", "invalid_initial_dimensions");
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");''')
        s = replace(s, '    } while ((flag_still_unsafe && restart_nums < 3)', '''      grid_map_->v22_diagnostics.add("optimizer_iteration",
          flag_success ? "success" : (flag_still_unsafe ? "collision_or_clearance" : (flag_force_return ? (force_stop_type_ == STOP_FOR_ERROR ? "constraint_error" : "rebound") : "solver_error")),
          time_ms / 1000., ",\\\"solver_code\\\":" + std::to_string(result) + ",\\\"iterations\\\":" + std::to_string(iter_num_)
          + ",\\\"restart_count\\\":" + std::to_string(restart_nums) + ",\\\"rebound_count\\\":" + std::to_string(rebound_times));
    } while ((flag_still_unsafe && restart_nums < 3)''')
        s = replace(s, '    if (!computePointsToCheck(traj, i_end, pts_check))\n    {', '''    if (!computePointsToCheck(traj, i_end, pts_check))
    {
      grid_map_->v22_diagnostics.add("constraint_check", "sampling_failed");''')
        s = replace(s, '            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);', '''            grid_map_->v22_diagnostics.add("constraint_check", "invalid_collision_segment");
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);''')
        s = replace(s, '          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");', '''          grid_map_->v22_diagnostics.add("constraint_check", "no_free_point_before_collision", 0,
              ",\\\"trajectory_start\\\":" + grid_map_->v22PointInfo(cps_.points.col(0)));
          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");''')
        s = replace(s, '          ROS_WARN("Local target in collision, skip this planning.");', '''          grid_map_->v22_diagnostics.add("constraint_check", "no_free_point_after_collision", 0,
              ",\\\"trajectory_end\\\":" + grid_map_->v22PointInfo(cps_.points.col(cps_.cp_size-1)));
          ROS_WARN("Local target in collision, skip this planning.");''')
        return s

    update('plan_env/include/plan_env/grid_map.h', grid_header)
    update('plan_manage/include/plan_manage/planner_manager.h', manager_header)
    update('plan_manage/src/planner_manager.cpp', manager_source)
    update('path_searching/src/dyn_a_star.cpp', astar_source)
    update('traj_opt/src/poly_traj_optimizer.cpp', optimizer_source)


def patch(root):
    patch_map(root)
    patch_planner(root)
    patch_diagnostics(root)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    patch(parser.parse_args().upstream)
