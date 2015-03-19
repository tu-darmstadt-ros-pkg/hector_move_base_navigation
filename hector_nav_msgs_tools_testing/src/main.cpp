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

#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <hector_nav_msgs_tools/hector_nav_msgs_tools.h>

ros::Subscriber test_pose_sub_;
ros::Subscriber plan_sub_;

ros::Publisher closest_pose_pub_;

nav_msgs::Path path_;
geometry_msgs::PoseStamped pose_;

void testPoseCallback(const geometry_msgs::PoseStamped pose)
{
  pose_ = pose;

  if (path_.poses.size() > 0){
    Eigen::Vector2d position (pose.pose.position.x,pose.pose.position.y);

    double dist;
    size_t idx;

    if (hector_nav_msgs_tools::getClosestPoseOnPath(path_.poses,position,dist, idx))
    {
      ROS_INFO("Distance to path is: %f", dist);

      double angle;
      hector_nav_msgs_tools::getPathYawAngle(path_.poses,idx, angle);

      geometry_msgs::PoseStamped pose = path_.poses[idx];

      pose.pose.orientation.w = cos(angle*0.5f);
      pose.pose.orientation.z = sin(angle*0.5f);
      ROS_INFO("Angle: %f", angle);

      closest_pose_pub_.publish(pose);

    }else{

    }




  }
}

void pathSubCallback(const nav_msgs::Path& path)
{
  path_ = path;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_nav_msgs_tools_test");

  ros::NodeHandle nh;

  closest_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("nav_msgs_tools_closest_pose",1);
  ros::Subscriber plan_sub_ = nh.subscribe("/drivepath",5, pathSubCallback);
  ros::Subscriber test_pose_sub_ = nh.subscribe("/test_pose",5, testPoseCallback);

  ros::spin();

  return 0;
}
