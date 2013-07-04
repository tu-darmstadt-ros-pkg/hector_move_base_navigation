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


#include "ros/ros.h"
#include <hector_exploration_planner/hector_exploration_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>


#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

class PoseWithCovToTf
{
public:
  PoseWithCovToTf()
    : transform_changed_(false)
  {
    ros::NodeHandle nh("");

    pub_tf_timer_ = nh.createTimer(ros::Duration(0.05), &PoseWithCovToTf::pubTfTimerCallback, this, false);

    pose_with_cov_sub_ = nh.subscribe("/initialpose", 2, &PoseWithCovToTf::poseWithCovCallback, this);

    pose_changed_pub_ = nh.advertise<std_msgs::Empty>("/replan",2,false);

    transform_.frame_id_ = "map";
    transform_.child_frame_id_ = "base_link";
    transform_.setIdentity();
  }

  void poseWithCovCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    tf::poseMsgToTF(msg->pose.pose, transform_);
    transform_changed_ = true;
  }

  void pubTfTimerCallback(const ros::TimerEvent& event)
  {
    transform_.stamp_ = ros::Time::now();
    tfb_.sendTransform(transform_);

    if (transform_changed_){

      std_msgs::Empty tmp;
      ros::WallDuration(0.1).sleep();
      pose_changed_pub_.publish(tmp);
      transform_changed_ = false;
    }
  }


protected:

  ros::Subscriber pose_with_cov_sub_;
  ros::Publisher pose_changed_pub_;

  tf::TransformBroadcaster tfb_;

  tf::StampedTransform transform_;

  ros::Timer pub_tf_timer_;

  bool transform_changed_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  PoseWithCovToTf ep;

  ros::spin();

  return 0;
}
