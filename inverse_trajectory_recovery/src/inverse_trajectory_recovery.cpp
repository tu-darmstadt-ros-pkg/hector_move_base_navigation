//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
#include <inverse_trajectory_recovery/inverse_trajectory_recovery.h>
#include <pluginlib/class_list_macros.h>

#include <hector_nav_msgs/GetRecoveryInfo.h>
#include <hector_move_base_msgs/move_base_action.h>

#include <actionlib_msgs/GoalID.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(inverse_trajectory_recovery, InverseTrajectoryRecovery, inverse_trajectory_recovery::InverseTrajectoryRecovery, nav_core::RecoveryBehavior)

namespace inverse_trajectory_recovery {
InverseTrajectoryRecovery::InverseTrajectoryRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false), world_model_(NULL) {}

void InverseTrajectoryRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    //ros::NodeHandle private_nh("~/" + name_);

    ros::NodeHandle nh;
    inverse_trajectory_service_client_ = nh.serviceClient<hector_nav_msgs::GetRecoveryInfo>("trajectory_recovery_info");
    inv_traj_back_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("inv_traj_back_pose",1);

    ros::NodeHandle controller_nh("controller");

    path_pub_ = controller_nh.advertise<hector_move_base_msgs::MoveBaseActionGeneric>("generic",1);
    path_drive_feedback_sub_ = controller_nh.subscribe("result", 5, &InverseTrajectoryRecovery::resultCallback, this);

    /*
    //we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    blp_nh.param("acc_lim_th", acc_lim_th_, 1.0);
    blp_nh.param("acc_lim_x", acc_lim_x_, 0.4);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("max_vel_x", max_vel_x_, 0.4);
    blp_nh.param("escape_vel", escape_vel_, -0.4);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);
    blp_nh.param("min_radius", min_radius_, 0.30);
    */

    local_costmap_->getCostmapCopy(costmap_);
    world_model_ = new base_local_planner::CostmapModel(costmap_);

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

InverseTrajectoryRecovery::~InverseTrajectoryRecovery(){
  delete world_model_;
}

void InverseTrajectoryRecovery::runBehavior(){

  ROS_INFO("Running inverse trajectory recovery");

  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  hector_nav_msgs::GetRecoveryInfo trajectory_info;

  ros::Time curr_time = ros::Time::now();

  trajectory_info.request.request_time = curr_time;
  trajectory_info.request.request_radius = 0.5;

  bool service_success = inverse_trajectory_service_client_.call(trajectory_info);

  if (!service_success){
    ROS_INFO("Call to inverse trajectory service failed, aborting recovery behavior");
    return;
  }else{
    ROS_INFO("Received inverse path with %lu poses", trajectory_info.response.trajectory_radius_entry_pose_to_req_pose.poses.size() );
  }

  boost::mutex::scoped_lock lock(condition_mutex_);

  target_path_goal_.header.stamp = curr_time;
  target_path_goal_.goal_id.id = "inverse trajectory trajectory";
  target_path_goal_.goal_id.stamp = curr_time;

  if (false){
    hector_move_base_msgs::MoveBasePath target_path;
    target_path.target_path = trajectory_info.response.trajectory_radius_entry_pose_to_req_pose;
    hector_move_base_msgs::setAction(target_path_goal_, target_path);
  }else{
    hector_move_base_msgs::MoveBaseGoal target_goal;
    target_goal.target_pose = trajectory_info.response.radius_entry_pose;
    target_goal.target_pose.header.stamp = trajectory_info.response.trajectory_radius_entry_pose_to_req_pose.header.stamp;
    inv_traj_back_pose_pub_.publish(target_goal.target_pose);
    hector_move_base_msgs::setAction(target_path_goal_, target_goal);
  }

  path_pub_.publish(target_path_goal_);


  while (1){

    while (!condition_path_ready_.timed_wait(lock, ros::Duration(1.0).toBoost())){
      if ((ros::Time::now() - curr_time) > ros::Duration(30.0)){
        ROS_WARN("Failed inverse trajectory recovery after X seconds");
        return;
      }
    }

    if (target_path_result_.status.status == actionlib_msgs::GoalStatus::SUCCEEDED){
      ROS_INFO("Recovery successful");
      return;
    }else if (target_path_result_.status.status != actionlib_msgs::GoalStatus::ACTIVE){
      ROS_INFO("Recovery aborted, cause: %s",target_path_result_.status.text.c_str() );
      return;
    }
  }

  ROS_INFO("This should never be reached");

  return;

}

void InverseTrajectoryRecovery::resultCallback(const hector_move_base_msgs::MoveBaseActionResult& move_base_result)
{
  if ((move_base_result.status.goal_id.id == target_path_goal_.goal_id.id) && (move_base_result.status.goal_id.stamp == target_path_goal_.goal_id.stamp)) {
    target_path_result_ = move_base_result;
    condition_path_ready_.notify_one();
  }
}


}
