#ifndef HECTOR_NAV_CORE_EXPLORATION_PLANNER_
#define HECTOR_NAV_CORE_EXPLORATION_PLANNER_

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace hector_nav_core {
  /**
   * @class ExplorationPlanner
   * @brief Provides an interface for exploration planners used in navigation. All exploration planners written as plugins for the hector_navigation stack must adhere to this interface.
   */
  class ExplorationPlanner{
    public:
      
     /**
      * @brief Given a goal pose in the world, compute a plan
      * @param start The start pose
      * @param goal The goal pose
      * @param plan The plan... filled by the planner
      * @param distance The distance the observation pose should have to the goal
      * @return True if a valid plan was found, false otherwise
      */
     virtual bool makePlan(const geometry_msgs::PoseStamped& start,
         const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, const float distance = 0.0f) = 0;

      /**
       * @brief Given a start pose in the world, compute a plan to a frontier
       * @param start The start pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool doExploration(const geometry_msgs::PoseStamped &start,
          std::vector<geometry_msgs::PoseStamped> &plan) = 0;

      /**
       * @brief  Initialization function for the ExplorationPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~ExplorationPlanner(){}

    protected:
      ExplorationPlanner(){}
  };
}

#endif
