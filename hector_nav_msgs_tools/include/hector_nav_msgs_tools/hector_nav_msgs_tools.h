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

#ifndef HECTOR_NAV_MSGS_TOOLS_H
#define HECTOR_NAV_MSGS_TOOLS_H

#include <nav_msgs/Path.h>
#include <Eigen/Core>

namespace hector_nav_msgs_tools{

static inline bool getClosestPoseOnPath(const std::vector<geometry_msgs::PoseStamped>& path, const Eigen::Vector2d& position, double& dist_to_robot, size_t& path_pose_index)
{

  //If the path is empty, we cań't give a reasonable answer
  if (path.size() < 1){
    return false;
  }

  double robotX = position.x();
  double robotY = position.y();

  double diffX = path[0].pose.position.x - robotX;
  double diffY = path[0].pose.position.y - robotY;


  double last_dist_sqr = 0.0;
  double curr_dist_sqr = diffX * diffX + diffY * diffY;

  //If we have only one pose, this has to be closest to the robot
  if (path.size() < 2){
    dist_to_robot = sqrt(curr_dist_sqr);
    path_pose_index = 0;
    return true;
  }


  for (std::size_t i = 1 ; i < path.size(); ++i){
    last_dist_sqr = curr_dist_sqr;

    double curr_x = path[i].pose.position.x - robotX;
    double curr_y = path[i].pose.position.y - robotY;

    curr_dist_sqr = curr_x * curr_x + curr_y * curr_y;

    // When the first distance to a plan point is farther away from the robot position than the preceding one,
    // we assume to be past the point perpendicular to the path
    if (last_dist_sqr < curr_dist_sqr){
      dist_to_robot = sqrt(last_dist_sqr);
      path_pose_index = i-1;

      return true;
    }
  }

  //If we reach this point, the path has at least two poses and the latest distance has to be returned
  dist_to_robot = sqrt(curr_dist_sqr);
  path_pose_index = path.size()-1;
  return true;

}

static inline bool getPathYawAngle(const std::vector<geometry_msgs::PoseStamped>& poses, size_t path_pose_index, double& angle)
{

  if ( (path_pose_index < poses.size()) && (poses.size() > 1) ){

    if (path_pose_index == (poses.size()-1) ){
      --path_pose_index;
    }

    double diff_x = poses[path_pose_index+1].pose.position.x - poses[path_pose_index].pose.position.x;
    double diff_y = poses[path_pose_index+1].pose.position.y - poses[path_pose_index].pose.position.y;
    angle = atan2(diff_y, diff_x);

    return true;
  }else{
    return false;
  }

}



static inline bool getPoseInPlanForRadiusAroundRobotPose(const nav_msgs::Path& path, double radius, Eigen::Vector3f& pose_at_radius)
{
  return false;
}

}

#endif
