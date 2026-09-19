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
    planner_manager_->traj_.local_traj.traj_id = 0;
    planner_manager_->traj_.local_traj.pts_chk.clear();
    changeFSMExecState(WAIT_TARGET, "V22");
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
    try
    {
      // Keep execution output fenced until generation and deadline checks finish.
      v22_preview_ = true;
      if (!planNextWaypoint(Eigen::Vector3d(req.goal.x, req.goal.y, req.goal.z)))
      {
        res.error_code = "planning_failed";
        throw std::runtime_error("EGO global planning failed");
      }
      if (!planFromGlobalTraj(10))
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
    return true;
  }
}
