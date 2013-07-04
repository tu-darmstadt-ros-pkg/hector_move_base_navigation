#ifndef HECTOR_NAV_CORE_MOVE_BASE_INTERFACE_H_
#define HECTOR_NAV_CORE_MOVE_BASE_INTERFACE_H_

#include <hector_move_base_msgs/MoveBaseActionGoal.h>
#include <hector_move_base_msgs/MoveBaseActionPath.h>

#include <actionlib_msgs/GoalID.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>

#include <boost/shared_ptr.hpp>

namespace hector_move_base_handler {

    class HectorMoveBaseHandler;

}

namespace hector_move_base {

enum RESULT {SUCCESS,
             FAIL,
             NEXT,
             ALTERNATIVE,
             WAIT};

struct handlerActionGoal {

    handlerActionGoal() :
        goal_id(),
        target_pose(),
        speed(100),
        distance(0.5),
        do_exploration(false) {}

    actionlib_msgs::GoalID goal_id;
    geometry_msgs::PoseStamped target_pose;
    float speed, distance;
    bool do_exploration;};

/**
   * @interface IHectorMoveBase
   * @brief Provides an interface for the use of functions of hector_move_base
   */
class IHectorMoveBase{
public:
    /*
      / GOALS
      */
    //return the original goal move base was called last with
    virtual handlerActionGoal getGlobalGoal() = 0;

    //return the goal point last added to the stack of goal points
    virtual handlerActionGoal getCurrentGoal() = 0;

    virtual void popCurrentGoal() = 0;

    //add new goal as waypoint
    virtual void pushCurrentGoal(const handlerActionGoal&) = 0;

    //send a goal point to the controller
    virtual void sendActionGoal(const handlerActionGoal&) = 0;

    /*
      / PATHS
      */
    virtual hector_move_base_msgs::MoveBaseActionPath getCurrentActionPath() = 0;
    virtual void setActionPath (hector_move_base_msgs::MoveBaseActionPath) = 0;

    //send a path to the controller
    virtual void sendActionPath(const hector_move_base_msgs::MoveBaseActionPath&) = 0;

    /*
      / MOVE_BASE SEQUENCE MANIPULATION
      */

    virtual void setNextState(boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>) = 0;

    /*
      / GETTERS AND SERVICES
      */
    virtual costmap_2d::Costmap2DROS* getCostmap() = 0;

    virtual tf::TransformListener& getTransformListener() = 0;

};
}

#endif
