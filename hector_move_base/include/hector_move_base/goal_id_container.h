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
#ifndef GOAL_ID_CONTAINER_H_
#define GOAL_ID_CONTAINER_H_

class GoalIdContainer
{
public:
  void setStampAndString(const std::string string_in, const ros::Time& stamp_in )
  {
    controller_goal_id_.id = string_in;
    controller_goal_id_.stamp = stamp_in;
  }

  void invalidate()
  {
    controller_goal_id_.id = "invalidated";
    controller_goal_id_.stamp = ros::Time();
  }

  bool isEqual(const actionlib_msgs::GoalID& goal_id_in)const
  {
    return isEqual (goal_id_in.id, goal_id_in.stamp);
  }

  bool isEqual(const std::string string_in, const ros::Time& stamp_in) const
  {
    return ((controller_goal_id_.stamp == stamp_in) && (controller_goal_id_.id == string_in));
  }

  const actionlib_msgs::GoalID& getGoalID() const
  {
    return this->controller_goal_id_;
  }

protected:
  actionlib_msgs::GoalID controller_goal_id_;
};

#endif
