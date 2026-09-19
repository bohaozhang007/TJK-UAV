// v22 planner control: preserve the map, fence execution, never publish previews.
namespace ego_planner
{
  void EGOReplanFSM::v22Idle()
  {
    v22_generation_.clear();
    v22_preview_ = false;
    have_target_ = have_trigger_ = have_new_target_ = false;
    mandatory_stop_ = false;
    flag_escape_emergency_ = true;
    planner_manager_->v22Reset();
    changeFSMExecState(WAIT_TARGET, "V22");
    continously_called_times_ = 1;
  }

  void EGOReplanFSM::v22Publish(const traj_utils::PolyTraj &trajectory)
  {
    if (v22_preview_ || v22_generation_.empty())
      return;
    owl_nav_v22::PlannerTrajectory message;
    message.generation = v22_generation_;
    message.trajectory = trajectory;
    v22_trajectory_pub_.publish(message);
  }

  bool EGOReplanFSM::v22Plan(owl_nav_v22::Plan::Request &req,
                           owl_nav_v22::Plan::Response &res)
  {
    res.success = false;
    if (req.sequence <= v22_sequence_ || req.deadline < ros::Time::now())
    {
      res.error = "expired planner command";
      return true;
    }
    v22_sequence_ = req.sequence;
    if (req.mode == "idle")
    {
      v22Idle();
      res.success = true;
      return true;
    }
    if ((req.mode != "preview" && req.mode != "execute") ||
        (req.mode == "execute" && req.generation.empty()) ||
        !std::isfinite(req.goal.x) || !std::isfinite(req.goal.y) || !std::isfinite(req.goal.z))
    {
      res.error = "invalid planner command";
      return true;
    }
    if (!have_odom_ || !v22_generation_.empty() || have_target_)
    {
      res.error = "planner requires initialized idle state";
      return true;
    }
    // ros::spin serializes this call with map, odometry and FSM callbacks.
    v22Idle();
    auto map = planner_manager_->grid_map_;
    auto &diagnostics = map->v22_diagnostics;
    diagnostics.active = true;
    diagnostics.clear();
    const ros::WallTime started = ros::WallTime::now();
    const Eigen::Vector3d goal(req.goal.x, req.goal.y, req.goal.z);
    std::ostringstream report, attempts;
    report << std::setprecision(12) << "{\"sequence\":" << req.sequence
           << ",\"start\":" << map->v22PointInfo(odom_pos_)
           << ",\"start_velocity_world_m_s\":" << v22Vector(odom_vel_)
           << ",\"goal\":" << map->v22PointInfo(goal)
           << ",\"map\":" << map->v22MapInfo();
    int attempted = 0;
    try
    {
      // Keep execution output fenced until generation and deadline checks finish.
      v22_preview_ = true;
      const std::string start_state = map->v22PointState(odom_pos_);
      if (start_state != "free")
      {
        res.error_code = "planning_failed";
        throw std::runtime_error("EGO start check failed: " + start_state);
      }
      const ros::WallTime global_started = ros::WallTime::now();
      const bool global_ok = planNextWaypoint(goal);
      report << ",\"global_planning_s\":" << (ros::WallTime::now()-global_started).toSec();
      if (!global_ok)
      {
        res.error_code = "planning_failed";
        throw std::runtime_error("EGO global planning failed");
      }
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      bool planned = false;
      for (int attempt = 1; attempt <= 10; ++attempt)
      {
        if (req.deadline < ros::Time::now())
          throw std::runtime_error("planner command expired during planning");
        diagnostics.clear();
        planner_manager_->v22BeginAttempt(attempt-1);
        const ros::WallTime attempt_started = ros::WallTime::now();
        planned = callReboundReplan(true, attempt > 1);
        if (attempted++) attempts << ',';
        attempts << std::setprecision(12) << "{\"attempt\":" << attempt
                 << ",\"initialization\":\"" << (attempt == 1 ? "normal" : "random")
                 << "\",\"success\":" << (planned ? "true" : "false")
                 << ",\"elapsed_s\":" << (ros::WallTime::now()-attempt_started).toSec()
                 << ",\"stages\":" << diagnostics.json()
                 << ",\"omitted_events\":" << diagnostics.omitted << '}';
        if (planned) break;
      }
      if (!planned)
      {
        res.error_code = "planning_failed";
        throw std::runtime_error("EGO initial trajectory generation failed");
      }
      traj_utils::MINCOTraj broadcast;
      polyTraj2ROSMsg(res.trajectory, broadcast);
      if (req.deadline < ros::Time::now())
        throw std::runtime_error("planner command expired during planning");
      if (req.mode == "preview")
      {
        v22Idle();
      }
      else
      {
        v22_preview_ = false;
        v22_generation_ = req.generation;
        have_trigger_ = true;
        changeFSMExecState(EXEC_TRAJ, "V22");
        v22Publish(res.trajectory);
        broadcast_ploytraj_pub_.publish(broadcast);
      }
      res.success = true;
    }
    catch (const std::exception &error)
    {
      v22Idle();
      res.error = error.what();
    }
    diagnostics.active = false;
    report << ",\"attempts\":[" << attempts.str() << "],\"elapsed_s\":"
           << (ros::WallTime::now()-started).toSec() << '}';
    res.diagnostics = report.str();
    return true;
  }
}
