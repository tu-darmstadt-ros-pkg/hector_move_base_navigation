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
#ifndef INVERSE_TRAJECTORY_RECOVERY_H_
#define INVERSE_TRAJECTORY_RECOVERY_H_

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
//#include <angles/angles.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include <hector_move_base_msgs/MoveBaseActionResult.h>
#include <hector_move_base_msgs/MoveBaseActionGeneric.h>

namespace inverse_trajectory_recovery{
  /**
   * @class InverseTrajectoryRecovery
   * @brief A recovery behavior that attempts to travel back the path the robot came from
   */
  class InverseTrajectoryRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param
       * @return
       */
      InverseTrajectoryRecovery();

      /**
       * @brief  Initialization function for the InverseTrajectoryRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack
       * @param local_costmap A pointer to the local_costmap used by the navigation stack
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief  Run the InverseTrajectoryRecovery recovery behavior.
       */
      void runBehavior();

      /**
       * @brief  Destructor for the turn recovery behavior
       */
      ~InverseTrajectoryRecovery();


      void resultCallback(const hector_move_base_msgs::MoveBaseActionResult& move_base_result);


    private:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      std::string name_;

      tf2_ros::Buffer* tf_;
      ros::ServiceClient inverse_trajectory_service_client_;

      ros::Publisher path_pub_;
      ros::Publisher inv_traj_back_pose_pub_;
      ros::Subscriber path_drive_feedback_sub_;

      hector_move_base_msgs::MoveBaseActionGeneric target_path_goal_;
      hector_move_base_msgs::MoveBaseActionResult target_path_result_;

      boost::condition condition_path_ready_;
      boost::mutex condition_mutex_;

      bool initialized_;
      double frequency_;
      //double sim_granularity_, min_rotational_vel_, max_rotational_vel_, max_vel_x_, escape_vel_, acc_lim_th_, acc_lim_x_, min_radius_, tolerance_, frequency_;
      base_local_planner::CostmapModel* world_model_;
  };
};
#endif
