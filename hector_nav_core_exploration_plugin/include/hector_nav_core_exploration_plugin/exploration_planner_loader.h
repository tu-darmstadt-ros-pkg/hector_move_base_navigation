//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
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


#ifndef HECTOR_MOVE_BASE_EXPLORATION_PLANNER_LOADER_H
#define HECTOR_MOVE_BASE_EXPLORATION_PLANNER_LOADER_H

#include <hector_exploration_planner/hector_exploration_planner.h>
//#include <pluginlib/class_loader.h>

#include <boost/weak_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <map>

namespace hector_nav_core {

template<typename T, typename _Key = std::string>
class SingletonLoader {
private:
//  pluginlib::ClassLoader<T> expl_loader_;

  static boost::shared_ptr<T> createInstance()
  {
    return boost::shared_ptr<T>(new T());
  }

public:
  typedef _Key KeyType;

  static boost::shared_ptr<T> GetInstance(const KeyType& name = std::string())
  {
    static std::map<KeyType,boost::weak_ptr<T> > s_singleton;
    boost::shared_ptr<T> instance;

    if (s_singleton[name].expired()) {
      instance = createInstance();
      s_singleton[name] = instance;
    } else {
      instance = s_singleton[name].lock();
    }

    return instance;
  }
};

typedef SingletonLoader<hector_exploration_planner::HectorExplorationPlanner> ExplorationPlannerLoader;

} // namespace

#endif // HECTOR_MOVE_BASE_EXPLORATION_PLANNER_LOADER_H
