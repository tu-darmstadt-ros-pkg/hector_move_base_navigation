//=================================================================================================
// Copyright (c) 2012, Mark Sollweck, Stefan Kohlbrecher, TU Darmstadt
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

#include <hector_nav_core_exploration_plugin/hector_nav_core_exploration_plugin.h>
#include <hector_nav_core_exploration_plugin/exploration_planner_loader.h>

namespace hector_nav_core{

HectorNavCoreExplorationPlugin::HectorNavCoreExplorationPlugin()
{
}

HectorNavCoreExplorationPlugin::~HectorNavCoreExplorationPlugin()
{
}

bool HectorNavCoreExplorationPlugin::makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, const float& distance)
{
  if (!exploration_planner) return false;

    if (distance <= 0) {
      return exploration_planner->makePlan(start, goal, plan);
    } else {
        ROS_DEBUG("[exploration_plugin]: starting getObservationPose. distance = %f; (%f,%f,%f) (%f,%f,%f,%f)", distance, goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

        geometry_msgs::PoseStamped observation_goal;

        if (!exploration_planner->getObservationPose(goal, distance, observation_goal)){
          return false;
        }

        return exploration_planner->makePlan(start, observation_goal, plan);
    }
}

bool HectorNavCoreExplorationPlugin::doExploration(const geometry_msgs::PoseStamped &start,
    std::vector<geometry_msgs::PoseStamped> &plan)
{
  if (!exploration_planner) return false;
  return exploration_planner->doExploration(start, plan);
}

void HectorNavCoreExplorationPlugin::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  exploration_planner = ExplorationPlannerLoader::GetInstance(name);

  if (exploration_planner.use_count() == 1) {
    // initialize only on first use
    exploration_planner->initialize(name, costmap_ros);
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_nav_core::HectorNavCoreExplorationPlugin, hector_nav_core::ExplorationPlanner)
