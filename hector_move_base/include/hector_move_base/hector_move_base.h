#ifndef HECTOR_MOVE_BASE_H_
#define HECTOR_MOVE_BASE_H_

#include <hector_move_base/handler_hector_exploration.hpp>
#include <hector_move_base/handler_hector_planning.hpp>
#include <hector_move_base/handler_hector_refine_plan.hpp>
#include <hector_move_base/handler_hector_publish_path.hpp>
#include <hector_move_base/handler_hector_wait_for_replanning.hpp>
#include <hector_move_base/handler_hector_wait_for_reexploring.hpp>
#include <hector_move_base/handler_hector_publish_feedback.hpp>
#include <hector_move_base/handler_hector_publish_abort.hpp>
#include <hector_move_base/handler_hector_publish_preempted.hpp>
#include <hector_move_base/handler_hector_publish_rejected.hpp>
#include <hector_move_base/handler_hector_publish_success.hpp>
#include <hector_move_base/handler_hector_stuck_recovery.hpp>
#include <hector_move_base/handler_hector_idle.hpp>
#include <hector_move_base/hector_move_base_state_machine.h>

#include <hector_move_base_msgs/MoveBaseActionExplore.h>
#include <hector_move_base_msgs/MoveBaseActionGoal.h>
#include <hector_move_base_msgs/MoveBaseActionPath.h>
#include <hector_move_base_msgs/MoveBaseActionResult.h>
#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_core/hector_move_base_interface.h>

#include <monstertruck_msgs/SetAlternativeTolerance.h>

#include <visualization_msgs/Marker.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace hector_move_base {

/**
 * @class HectorMoveBase
 * @brief A class that moves the robot base to a goal location.
 */
class HectorMoveBase : public IHectorMoveBase {

private:
    costmap_2d::Costmap2DROS* costmap_;
    ros::NodeHandle private_nh_;
    boost::shared_ptr<HectorMoveBaseStateMachine> statemachine_;
    tf::TransformListener& tf_;
    boost::thread* main_loop_thread_;

    boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> idleState_;
    boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> exploringState_, planningState_, refinePlanState_, publishPathState_, publishFeedbackState_, waitForReplanningState_, waitForReexploringState_;
    boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> publishSuccessState_, publishAbortState_, publishPreemptedState_, publishRejectedState_;
    boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> stuckExplorationRecoveryState_, stuckPlanningRecoveryState_;
    boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> currentState_, nextState_, startState_;

    double circumscribedRadius_, goalReachedRadius_, timeToTriggerReplannning_, timeToTriggerExploration_, goalReachchedAngularVariance_, goalReachedSquaredLinearVariance_, observeLinearTolerance_, observeAngularTolerance_;
    std::string controller_namespace_;
    bool use_alternate_planner_;
    std::vector<handlerActionGoal> goals_;
    hector_move_base_msgs::MoveBaseActionPath path_;
    ros::Publisher current_goal_pub_, drivepath_pub_, feedback_pub_, result_pub_, goalmarker_pub_, state_name_pub_;
    ros::Subscriber cancel_sub_, controller_result_sub_, explore_sub_, goal_sub_, observation_sub_, syscommand_sub_, simple_goal_sub_;
    ros::ServiceClient tolerance_client_;
    pluginlib::ClassLoader<nav_core::RecoveryBehavior> move_base_plugin_loader_;
    std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > move_base_plugins_;

    boost::recursive_mutex currentStatMutex_;

public:
    /**
   * @brief  Constructor for the actions
   * @param name The name of the action
   * @param tf A reference to a TransformListener
   */
    HectorMoveBase(std::string name, tf::TransformListener& tf);

    /**
   * @brief  Destructor - Cleans up
   */
    virtual ~HectorMoveBase();

    handlerActionGoal getGlobalGoal();
    handlerActionGoal getCurrentGoal();
    void popCurrentGoal();
    void pushCurrentGoal(const handlerActionGoal&);
    void sendActionGoal(const handlerActionGoal&);

    hector_move_base_msgs::MoveBaseActionPath getCurrentActionPath();
    void setActionPath (hector_move_base_msgs::MoveBaseActionPath);
    void sendActionPath(const hector_move_base_msgs::MoveBaseActionPath&);

    void setNextState(boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>);

    costmap_2d::Costmap2DROS* getCostmap();
    tf::TransformListener& getTransformListener();

    void moveBaseStep();

private:
    bool loadMoveBasePlugins(ros::NodeHandle node);
    void loadDefaultMoveBasePlugins();

    /**
   * callback methods
   */
    void exploreCB(const hector_move_base_msgs::MoveBaseActionExplore::ConstPtr& goal);
    void goalCB(const hector_move_base_msgs::MoveBaseActionGoal::ConstPtr& goal);
    void observationCB(const hector_move_base_msgs::MoveBaseActionGoal::ConstPtr& goal);
    void simple_goalCB(const geometry_msgs::PoseStamped::ConstPtr& simpleGoal);
    void cmd_velCB(const ros::MessageEvent<geometry_msgs::Twist>& event);
    void cancelCB(const std_msgs::Empty::ConstPtr& empty);
    void syscommandCB(const std_msgs::String::ConstPtr& string);
    void controllerResultCB(const hector_move_base_msgs::MoveBaseActionResult::ConstPtr& result);

    void moveBaseLoop(ros::NodeHandle&, ros::Rate);
    void abortedGoal();
    void preemptedGoal();
    void rejectedGoal();
    void recoveryGoal();
    void successGoal();
    void clearGoal();
    geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped&);
    bool isGoalIDEqual(const actionlib_msgs::GoalID&,const actionlib_msgs::GoalID&);
};
}
#endif

