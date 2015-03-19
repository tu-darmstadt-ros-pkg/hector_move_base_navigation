/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <turn_recovery/turn_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(turn_recovery, TurnRecovery, turn_recovery::TurnRecovery, nav_core::RecoveryBehavior)

namespace turn_recovery {
TurnRecovery::TurnRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void TurnRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
//TODO should be unbound from TrajectoryPlannerROS
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

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

    local_costmap_->getCostmapCopy(costmap_);
    world_model_ = new base_local_planner::CostmapModel(costmap_);

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

TurnRecovery::~TurnRecovery(){
  delete world_model_;
}

void TurnRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_angle = 0.0;

  bool direction = true;

  double start_offset = 0.0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  ros::Time time_counter = ros::Time().now();
  ros::Duration time_difference;
  unsigned int counter = 0;
  unsigned int safety_counter = 0;
  while(n.ok()){
    if (!local_costmap_->getRobotPose(global_pose)) {
      ROS_ERROR("getting RobotPose FAILED");
    }

    double norm_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
    current_angle = angles::normalize_angle(norm_angle + start_offset);

    //update the costmap copy that the world model holds
    local_costmap_->getCostmapCopy(costmap_);

    time_difference = ros::Time().now() - time_counter;
    if (time_difference.sec > 4.0f) {
        direction = !direction;
        safety_counter++;
        time_counter = ros::Time().now();
    }

    if (direction) {
      forwardSimulation(global_pose, vel_pub);
    }
    else {
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = escape_vel_;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = -max_rotational_vel_;

      vel_pub.publish(cmd_vel);
    }

    //makes sure that we won't decide we're done right after we start
    if(fabs(current_angle) > M_PI_2)
      return;

    if(safety_counter > 10)
      return; //RUECKMELDUNG ABBRUCH

    r.sleep();
  }
}

void TurnRecovery::forwardSimulation(tf::Stamped<tf::Pose> global_pose, ros::Publisher vel_pub) {

  //setting maximum distance for one iteration
  //and maximum kappa for turning counterclockwise
  double max_dist = max_vel_x_ / frequency_;
  double sim_kappa = -1.0 / min_radius_;

  //check if that velocity is legal by forward simulating
  double best_pair[2] = {0.0, 0.0};
  while(sim_kappa < 0){
    double sim_alpha = 0.0;
    while( (sim_alpha / sim_kappa) <= max_dist ) {
      std::vector<geometry_msgs::Point> oriented_footprint;
      double alpha = (M_PI - sim_alpha)/2;
      double theta = tf::getYaw(global_pose.getRotation()) + alpha;
      double reach = 2 / sim_kappa * sin(alpha);

      geometry_msgs::Point position;
      position.x = global_pose.getOrigin().x() + reach * sin(theta);
      position.y = global_pose.getOrigin().y() + reach * cos(theta);

      local_costmap_->getOrientedFootprint(position.x, position.y, theta, oriented_footprint);

      //make sure that the point is legal, if it isn't... we'll abort
      double footprint_cost = world_model_->footprintCost(position, oriented_footprint, local_costmap_->getInscribedRadius(), local_costmap_->getCircumscribedRadius());
      if(footprint_cost < 0.0){
        best_pair[0] = sim_kappa;
        best_pair[1] = sim_alpha;
        break;
      }
      sim_alpha -= sim_granularity_;
    }

    if (sim_alpha / sim_kappa >= max_dist) {
      best_pair[0] = sim_kappa;
      best_pair[1] = max_dist * sim_kappa;
      break;
    }
    else {
      //ranking of best turn can be tweaked
      if ( (sim_alpha / sim_kappa) > (best_pair[1] / best_pair[0]) ) {
        best_pair[0] = sim_kappa;
        best_pair[1] = sim_alpha;
      }
    }

    sim_kappa += sim_granularity_;
  }

  //compute the velocity that will let us stop by the time we reach the goal
  double vel = (best_pair[1] / best_pair[0]) * frequency_;

  //make sure that this velocity falls within the specified limits
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = best_pair[0]/vel;

  vel_pub.publish(cmd_vel);
}
};
