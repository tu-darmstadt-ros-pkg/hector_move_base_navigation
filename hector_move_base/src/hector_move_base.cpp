#include <hector_move_base/hector_move_base.h>

namespace hector_move_base
{

HectorMoveBase::HectorMoveBase(std::string name, tf::TransformListener& tf) :
    costmap_(NULL),
    nh_(""),
    private_nh_("~"),
    tf_(tf),
    statemachine_(new HectorMoveBaseStateMachine),
    move_base_plugin_loader_("nav_core", "nav_core::RecoveryBehavior"),
    action_server_(nh_, name, false)
{
    private_nh_.param("goal_reached_radius", goalReachedRadius_, 0.2);
    private_nh_.param("controller_namespace", controller_namespace_, std::string("/controller"));
    private_nh_.param("use_alternate_planner", use_alternate_planner_, true);
    private_nh_.param("observe_linear_tolerance", observeLinearTolerance_, 0.2);
    private_nh_.param("observe_angular_tolerance", observeAngularTolerance_, M_PI_2);
    private_nh_.param("use_exploring", use_exploring_, true);

    costmap_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);

    ROS_DEBUG("[hector_move_base] Costmap loaded");

    setupStateMachine();

    goal_id_counter_ = 0;

    path_ = hector_move_base_msgs::MoveBaseActionPath();

    ros::NodeHandle controller_nh(controller_namespace_);

    drivepath_pub_ = controller_nh.advertise<hector_move_base_msgs::MoveBaseActionPath>("path", 0 );
    goalmarker_pub_ = private_nh_.advertise<visualization_msgs::Marker>("goal_marker", 0);

    explore_sub_ = private_nh_.subscribe<hector_move_base_msgs::MoveBaseActionExplore>("explore", 1, boost::bind(&HectorMoveBase::exploreCB, this, _1));
    simple_goal_sub_ = private_nh_.subscribe<geometry_msgs::PoseStamped>("simple_goal", 1, boost::bind(&HectorMoveBase::simple_goalCB, this, _1));
    syscommand_sub_ = nh_.subscribe<std_msgs::String>("syscommand", 1, boost::bind(&HectorMoveBase::syscommandCB, this, _1));

    controller_result_sub_ = controller_nh.subscribe<hector_move_base_msgs::MoveBaseActionResult>("result", 1, boost::bind(&HectorMoveBase::controllerResultCB, this, _1));

    tolerance_client_ = controller_nh.serviceClient<monstertruck_msgs::SetAlternativeTolerance>("set_alternative_tolerances");

    action_server_.registerGoalCallback(boost::bind(&HectorMoveBase::asGoalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&HectorMoveBase::asCancelCB, this));
    action_server_.start();
}

HectorMoveBase::~HectorMoveBase()
{
    if(costmap_ != NULL)
        delete costmap_;
}

void HectorMoveBase::setupStateMachine()
{
    exploringState_.reset(new hector_move_base_handler::HectorExplorationHandler(this));
    planningState_.reset(new hector_move_base_handler::HectorPlanningHandler(this));
    refinePlanState_.reset(new hector_move_base_handler::HectorRefinePlanHandler(this));
    publishPathState_.reset(new hector_move_base_handler::HectorPublishPathHandler(this));
    publishFeedbackState_.reset(new hector_move_base_handler::HectorPublishFeedbackHandler(this));
    publishSuccessState_.reset(new hector_move_base_handler::HectorPublishSuccessHandler(this));
    publishAbortState_.reset(new hector_move_base_handler::HectorPublishAbortHandler(this));
    publishPreemptedState_.reset(new hector_move_base_handler::HectorPublishPreemptedHandler(this));
    publishRejectedState_.reset(new hector_move_base_handler::HectorPublishRejectedHandler(this));
    waitForReplanningState_.reset(new hector_move_base_handler::HectorWaitForReplanningHandler(this));
    waitForReexploringState_.reset(new hector_move_base_handler::HectorWaitForReexploringHandler(this));
    stuckExplorationRecoveryState_.reset(new hector_move_base_handler::HectorStuckRecoveryHandler(this));
    stuckPlanningRecoveryState_.reset(new hector_move_base_handler::HectorStuckRecoveryHandler(this));
    idleState_.reset(new hector_move_base_handler::HectorIdleHandler(this));

    ROS_DEBUG("[hector_move_base] All states created");

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForExploration;
    mappingForExploration.insert(std::make_pair(NEXT, planningState_));
    if (use_alternate_planner_)
        mappingForExploration.insert(std::make_pair(ALTERNATIVE, refinePlanState_));
    else
        mappingForExploration.insert(std::make_pair(ALTERNATIVE, publishPathState_));
    mappingForExploration.insert(std::make_pair(FAIL, stuckExplorationRecoveryState_));
    statemachine_->addHandlerMapping(exploringState_, mappingForExploration);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPlanning;
    if (use_alternate_planner_)
        mappingForPlanning.insert(std::make_pair(NEXT, refinePlanState_));
    else
        mappingForPlanning.insert(std::make_pair(NEXT, publishPathState_));
    mappingForPlanning.insert(std::make_pair(ALTERNATIVE, exploringState_));
    mappingForPlanning.insert(std::make_pair(FAIL, stuckPlanningRecoveryState_));
    statemachine_->addHandlerMapping(planningState_, mappingForPlanning);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForRefinePlan;
    mappingForRefinePlan.insert(std::make_pair(NEXT, publishPathState_));
    mappingForRefinePlan.insert(std::make_pair(ALTERNATIVE, planningState_));
    mappingForRefinePlan.insert(std::make_pair(FAIL, stuckPlanningRecoveryState_));
    statemachine_->addHandlerMapping(refinePlanState_, mappingForRefinePlan);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishPath;
    mappingForPublishPath.insert(std::make_pair(NEXT, publishFeedbackState_));
    statemachine_->addHandlerMapping(publishPathState_, mappingForPublishPath);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishFeedback;
    if(use_exploring_)
        mappingForPublishFeedback.insert(std::make_pair(NEXT, waitForReplanningState_));
    else
        mappingForPublishFeedback.insert(std::make_pair(NEXT, waitForReplanningState_));
    statemachine_->addHandlerMapping(publishFeedbackState_, mappingForPublishFeedback);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForWaitForReplanning;
    mappingForWaitForReplanning.insert(std::make_pair(NEXT, waitForReexploringState_));
    mappingForWaitForReplanning.insert(std::make_pair(ALTERNATIVE, planningState_));
    statemachine_->addHandlerMapping(waitForReplanningState_, mappingForWaitForReplanning);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForWaitForReexploring;
    mappingForWaitForReexploring.insert(std::make_pair(NEXT, publishFeedbackState_));
    mappingForWaitForReexploring.insert(std::make_pair(ALTERNATIVE, exploringState_));
    statemachine_->addHandlerMapping(waitForReexploringState_, mappingForWaitForReexploring);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishSuccess;
    mappingForPublishSuccess.insert(std::make_pair(NEXT, idleState_));
    statemachine_->addHandlerMapping(publishSuccessState_, mappingForPublishSuccess);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishAbort;
    mappingForPublishAbort.insert(std::make_pair(NEXT, idleState_));
    statemachine_->addHandlerMapping(publishAbortState_, mappingForPublishAbort);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishPreempted;
    mappingForPublishPreempted.insert(std::make_pair(NEXT, idleState_));
    statemachine_->addHandlerMapping(publishPreemptedState_, mappingForPublishPreempted);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishRejected;
    mappingForPublishRejected.insert(std::make_pair(NEXT, idleState_));
    statemachine_->addHandlerMapping(publishRejectedState_, mappingForPublishRejected);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForExplorationStuckRecovery;
    mappingForExplorationStuckRecovery.insert(std::make_pair(NEXT, exploringState_));
    mappingForExplorationStuckRecovery.insert(std::make_pair(FAIL, publishAbortState_));
    statemachine_->addHandlerMapping(stuckExplorationRecoveryState_, mappingForExplorationStuckRecovery);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPlanningStuckRecovery;
    mappingForPlanningStuckRecovery.insert(std::make_pair(NEXT, planningState_));
    mappingForPlanningStuckRecovery.insert(std::make_pair(FAIL, publishAbortState_));
    statemachine_->addHandlerMapping(stuckPlanningRecoveryState_, mappingForPlanningStuckRecovery);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForIdleState;
    mappingForIdleState.insert(std::make_pair(NEXT, idleState_));
    statemachine_->addHandlerMapping(idleState_, mappingForIdleState);

    ROS_DEBUG("[hector_move_base] State machine mapping created");

    startState_ = exploringState_;
    currentState_ = idleState_;
    nextState_ = currentState_;

    ROS_DEBUG("[hector_move_base] Statemachine initialized");
}

handlerActionGoal HectorMoveBase::getGlobalGoal() {
    if (goals_.empty()) {
        return handlerActionGoal();
    }
    return goals_.front();
}

handlerActionGoal HectorMoveBase::getCurrentGoal() {
    if (goals_.empty()) {
        return handlerActionGoal();
    }
    return goals_.back();
}

void HectorMoveBase::popCurrentGoal()
{
    if (!goals_.empty())
        goals_.pop_back();
}

void HectorMoveBase::pushCurrentGoal(const handlerActionGoal & goal)
{
    goals_.push_back(goal);

    visualization_msgs::Marker marker;
    marker.header.frame_id = goal.target_pose.header.frame_id;
    marker.header.stamp = goal.goal_id.stamp;
    marker.ns = "hector_move_base";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = goal.target_pose.pose;

    marker.scale.x = 0.15;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.6;

    marker.lifetime = ros::Duration();
    goalmarker_pub_.publish(marker);
}

void HectorMoveBase::sendActionGoal(const handlerActionGoal& goal)
{
    hector_move_base_msgs::MoveBaseActionPath path;

    path.goal_id = goal.goal_id;
    path.goal.speed = goal.speed;
    path.goal.target_path.header = goal.target_pose.header;
    path.reverse_allowed = goal.reverse_allowed;

    geometry_msgs::PoseStamped targetPose;
    targetPose.header = goal.target_pose.header;
    targetPose.pose = goal.target_pose.pose;
    path.goal.target_path.poses.push_back(targetPose);

    drivepath_pub_.publish(path);
}

hector_move_base_msgs::MoveBaseActionPath HectorMoveBase::getCurrentActionPath() {
    return path_;
}

void HectorMoveBase::setActionPath (hector_move_base_msgs::MoveBaseActionPath path) {
    if (path.goal.target_path.header.frame_id.empty()) {
        path.goal.target_path.header.frame_id = costmap_->getGlobalFrameID();
        ROS_WARN("[hector_move_base]: planner returned a path with empty frame_id. Assuming %s frame.", path.goal.target_path.header.frame_id.c_str());
    }

    path_ = path;
}

void HectorMoveBase::sendActionPath(const hector_move_base_msgs::MoveBaseActionPath& pathToSend) {
    drivepath_pub_.publish(pathToSend);
}

void HectorMoveBase::setActionServerCanceled() {
  abortedGoal();
}

void HectorMoveBase::setNextState(boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> nextState) {
    ROS_DEBUG("[hector_move_base]: setNextState()");
    nextState_ = nextState;
}


costmap_2d::Costmap2DROS* HectorMoveBase::getCostmap() {
    return costmap_;
}

tf::TransformListener& HectorMoveBase::getTransformListener() {
    return tf_;
}

//we'll load our default recovery behaviors here
void HectorMoveBase::loadDefaultMoveBasePlugins() {
  move_base_plugins_.clear();
  try{
    //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
    ros::NodeHandle n("~");
    n.setParam("conservative_reset/reset_distance", 0.2);
    n.setParam("aggressive_reset/reset_distance", 0.3 * 4);

    //first, we'll load a recovery behavior to clear the costmap
    boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(move_base_plugin_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
    cons_clear->initialize("conservative_reset", &tf_, costmap_, costmap_);
    move_base_plugins_.push_back(cons_clear);

    //next, we'll load a recovery behavior to rotate in place
    boost::shared_ptr<nav_core::RecoveryBehavior> rotate(move_base_plugin_loader_.createInstance("rotate_recovery/RotateRecovery"));
    if(false){
      rotate->initialize("rotate_recovery", &tf_, costmap_, costmap_);
      move_base_plugins_.push_back(rotate);
    }

    //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
    boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(move_base_plugin_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
    ags_clear->initialize("aggressive_reset", &tf_, costmap_, costmap_);
    move_base_plugins_.push_back(ags_clear);

    //we'll rotate in-place one more time
    if(false)
      move_base_plugins_.push_back(rotate);
  }
  catch(pluginlib::PluginlibException& ex){
    ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
  }

  return;
}

void HectorMoveBase::exploreCB(const hector_move_base_msgs::MoveBaseActionExploreConstPtr &goal) {
    ROS_DEBUG("[hector_move_base]: In explore callback");
    abortedGoal();
    handlerActionGoal newGoal = handlerActionGoal();
    newGoal.goal_id = goal->goal_id;
    newGoal.speed = goal->goal.speed;
    newGoal.do_exploration = true;
    pushCurrentGoal(newGoal);
    setNextState(exploringState_);
    return;
}

  void HectorMoveBase::asGoalCB() {
    ROS_INFO("[hector_move_base]: In ActionServer goal callback");
    const hector_move_base_msgs::MoveBaseGoalConstPtr goal = action_server_.acceptNewGoal();

    clearGoal();

    handlerActionGoal newGoal = handlerActionGoal();
    newGoal.goal_id.id = goal_id_counter_++;
    newGoal.goal_id.stamp = ros::Time::now();
    newGoal.speed = goal->speed;
    newGoal.reverse_allowed = goal->reverse_allowed;
    if (goal->exploration == true) {
        newGoal.do_exploration = true;
    }
    else {
        newGoal.do_exploration = false;
    }
   

    //make sure goal could be transformed to costmap frame
    if (goal->exploration == false) {
    	newGoal.target_pose = goalToGlobalFrame(goal->target_pose);
    	if (newGoal.target_pose.header.frame_id != costmap_->getGlobalFrameID()) {
        	ROS_ERROR("[hector_move_base]: tf transformation into global frame failed. goal will be canceled");
        	//new Goal has to be set in order to publish goal aborted result
        	pushCurrentGoal(newGoal);
        	abortedGoal();
        	return;
    	}
    }
    pushCurrentGoal(newGoal);
    setNextState(planningState_);
    return;
}

void HectorMoveBase::observationCB(const hector_move_base_msgs::MoveBaseActionGoalConstPtr &goal) {
    ROS_WARN("[hector_move_base] Observation callback");
    abortedGoal();

    handlerActionGoal newGoal = handlerActionGoal();
    newGoal.goal_id = goal->goal_id;
    newGoal.speed = goal->goal.speed;
    newGoal.do_exploration = false;
    newGoal.distance = goal->goal.distance;

    monstertruck_msgs::SetAlternativeTolerance tolerance_srv;
    tolerance_srv.request.goalID = goal->goal_id;
    tolerance_srv.request.linearTolerance = observeLinearTolerance_;
    tolerance_srv.request.angularTolerance = observeAngularTolerance_;
    if (tolerance_client_.call(tolerance_srv)) {
        ROS_INFO("[hector_move_base]: called tolerance service successfully");
    }
    else {
        ROS_WARN("[hector_move_base]: calling tolerance service FAILED");
    }

    //make sure goal could be transformed to costmap frame
    newGoal.target_pose = goalToGlobalFrame(goal->goal.target_pose);
    if (newGoal.target_pose.header.frame_id != costmap_->getGlobalFrameID()) {
        ROS_ERROR("[hector_move_base]: tf transformation into global frame failed. goal will be canceled");
        //new Goal has to be set in order to publish goal aborted result
        pushCurrentGoal(newGoal);
        abortedGoal();
        return;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = newGoal.target_pose.header.frame_id;
    marker.header.stamp = newGoal.goal_id.stamp;
    marker.ns = "hector_move_base";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = newGoal.target_pose.pose;

    marker.scale.x = 0.3;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6;

    marker.lifetime = ros::Duration();
    goalmarker_pub_.publish(marker);

    pushCurrentGoal(newGoal);
    setNextState(planningState_);
    return;
}

void HectorMoveBase::simple_goalCB(const geometry_msgs::PoseStampedConstPtr &simpleGoal) {
    ROS_DEBUG("[hector_move_base]: In simple goal callback");
    abortedGoal();
    handlerActionGoal newGoal = handlerActionGoal();
    newGoal.goal_id.stamp = simpleGoal->header.stamp;
    newGoal.goal_id.id = "simple_goal";
    newGoal.target_pose = *simpleGoal;
    newGoal.distance = 0.0;
    newGoal.do_exploration = false;
    pushCurrentGoal(newGoal);
    setNextState(planningState_);
    return;
}

void HectorMoveBase::asCancelCB() {
  ROS_INFO("[hector_move_base]: In ActionServer Preempt callback");
  abortedGoal();
  setNextState(idleState_);
}

void HectorMoveBase::syscommandCB(const std_msgs::StringConstPtr &string) {
    ROS_DEBUG("[hector_move_base]: In syscommandCB callback: %s", string->data.c_str());

    if (string->data == "reset") {
      abortedGoal();
      setNextState(idleState_);

      // reset costmap
#ifdef LAYERED_COSTMAP_H_
      costmap_->resetLayers();
#endif // LAYERED_COSTMAP_H_
    }

    if (string->data == "stop") {
      abortedGoal();
      setNextState(idleState_);
    }

    if (string->data == "explore")
    {
      abortedGoal();
      handlerActionGoal newGoal = handlerActionGoal();
      newGoal.goal_id.stamp = ros::Time::now();
      newGoal.goal_id.id = "simple_explore";
      newGoal.do_exploration = false;
      pushCurrentGoal(newGoal);
      setNextState(planningState_);
    }

    return;
}

void HectorMoveBase::controllerResultCB(const hector_move_base_msgs::MoveBaseActionResultConstPtr &result) {
    ROS_DEBUG("[hector_move_base]: In controller result callback");

    hector_move_base_msgs::MoveBaseActionPath current_action_path = getCurrentActionPath();
    handlerActionGoal global_goal = getGlobalGoal();

    if (!((isGoalIDEqual(getCurrentGoal().goal_id, result->status.goal_id)) ||
          (isGoalIDEqual(global_goal.goal_id, result->status.goal_id)) ||
          isGoalIDEqual(current_action_path.goal_id, result->status.goal_id))) {
        ROS_INFO("[hector_move_base]: goal is outdated, ignoring controller feedback");
        return;
    }

    switch (result->status.status) {
    case actionlib_msgs::GoalStatus::ACTIVE:
        ROS_DEBUG("[hector_move_base]: received result from controller == ACTIVE");
        return;

    case actionlib_msgs::GoalStatus::PREEMPTED:
        ROS_INFO("[hector_move_base]: received result from controller == PREEMPTED");
        if ((currentState_ == stuckExplorationRecoveryState_) ||
                (nextState_ == stuckExplorationRecoveryState_) ||
                (currentState_ == stuckPlanningRecoveryState_) ||
                (nextState_ == stuckPlanningRecoveryState_)) {
            return;
        }
        preemptedGoal();
        return;

    case actionlib_msgs::GoalStatus::REJECTED:
        ROS_INFO("[hector_move_base]: received result from controller == REJECTED");
        setNextState(exploringState_);
        return;

    case actionlib_msgs::GoalStatus::ABORTED:
        ROS_INFO("[hector_move_base]: received result from controller == ABORTED");
        if (currentState_ == exploringState_ || currentState_ == waitForReexploringState_) {
            setNextState(stuckExplorationRecoveryState_);
            return;
        }
        if (currentState_ == planningState_ || currentState_ == refinePlanState_
                || currentState_ == waitForReplanningState_) {
            setNextState(stuckPlanningRecoveryState_);
            return;
        }
        ROS_ERROR_STREAM("[hector_move_base] current state is " << typeid(*currentState_.get()).name());
        ROS_WARN("[hector_move_base]: controller sent ABORTED. currentState is neither planning nor exploring"
                 " nor waiting for results of these actions.");
        return;

    case actionlib_msgs::GoalStatus::SUCCEEDED:
        ROS_INFO("[hector_move_base]: received result from controller == SUCCEEDED");
        // if status goal id is equal to global goal we are done.
        if (isGoalIDEqual(global_goal.goal_id, result->status.goal_id)) {
            ROS_DEBUG("[hector_move_base]: reached global goal");
            successGoal();
            return;
        }
        // else result id must match current goal id or path id
        // if we are in exploration mode discard all temporary goals and trigger exploration
        if (global_goal.do_exploration) {
            while (goals_.size() > 1) {
                popCurrentGoal();
            }
            currentState_->abort();
            setNextState(exploringState_);
            ROS_INFO("[hector_move_base]: restarting exploration");
            return;
        }
        // if result id equals current goal but not global goal
        if (isGoalIDEqual(current_action_path.goal_id, result->status.goal_id))
        {
            ROS_DEBUG("[hector_move_base]: number of goals: %lu", goals_.size());
            if(current_action_path.goal.target_path.poses.size() > 0)
            {
                double diff_x = fabs(current_action_path.goal.target_path.poses.back().pose.position.x - global_goal.target_pose.pose.position.x);
                double diff_y = fabs(current_action_path.goal.target_path.poses.back().pose.position.y - global_goal.target_pose.pose.position.y);
                if ((pow(diff_x, 2) + pow(diff_y, 2)) < pow(goalReachedRadius_, 2)) {
                    ROS_INFO("[hector_move_base]: path was followed to the end. path goal is close enough to global_goal");
                    successGoal();
                    return;
                }
            }
            setNextState(planningState_);
            ROS_INFO("[hector_move_base]: start planning, last path was followed to the end");
            return;
        }
        // if result id equals current goal but not global goal
        if ((goals_.size() > 1) && isGoalIDEqual(getCurrentGoal().goal_id, result->status.goal_id)) {
            popCurrentGoal();
            setNextState(planningState_);
            ROS_INFO("[hector_move_base]: start planning for next goal");
            return;
        }
        ROS_WARN("[hector_move_base]: result goal_id does not match global goal or current goal, nor action path id.");
        return;

    default:
        ROS_WARN("[hector_move_base]: controller_feedback result is %i. This is not handled.", result->status.status);
        return;
    }
}

void HectorMoveBase::moveBaseStep() {
    RESULT result = currentState_->handle();
    if (currentState_ != nextState_) {
        currentState_ = nextState_;
        ROS_DEBUG("[hector_move_base]: nextState_ was set, ignoring statemachine mapping");
        return;
    }
    switch (result) {
    case WAIT:
        ROS_DEBUG("[hector_move_base]: result is WAIT, currentState_ will be kept");
        return;

    default:
        currentState_ = statemachine_->getNextActionForHandler(currentState_, result);
        nextState_ = currentState_;
        return;
    }

}

void HectorMoveBase::abortedGoal() {
    currentState_->abort();
    hector_move_base_msgs::MoveBaseResult result;
    result.result = hector_move_base_msgs::MoveBaseResult::FAIL;
    action_server_.setAborted(result, "ABORTED");
    clearGoal();
}

void HectorMoveBase::preemptedGoal() {
    currentState_->abort();
    hector_move_base_msgs::MoveBaseResult result;
    result.result = hector_move_base_msgs::MoveBaseResult::FAIL;
    action_server_.setPreempted(result, "PREEMPTED");
    goals_.clear();
    setNextState(idleState_);
}

void HectorMoveBase::rejectedGoal() {
    currentState_->abort();
    publishRejectedState_->handle();
    clearGoal();
}

void HectorMoveBase::recoveryGoal() {
    currentState_->abort();
    setNextState(exploringState_);
}

void HectorMoveBase::successGoal() {
    currentState_->abort();
    hector_move_base_msgs::MoveBaseResult result;
    result.result = hector_move_base_msgs::MoveBaseResult::SUCCESS;
    action_server_.setSucceeded(result, "SUCCESS");
    clearGoal();
}

void HectorMoveBase::clearGoal() {
    goals_.clear();
    path_ = hector_move_base_msgs::MoveBaseActionPath();
    hector_move_base_msgs::MoveBaseActionPath empty_path = hector_move_base_msgs::MoveBaseActionPath();
    empty_path.header.frame_id = costmap_->getGlobalFrameID();
    empty_path.goal_id.id = "empty_path";
    sendActionPath(empty_path);
    setNextState(idleState_);
}

geometry_msgs::PoseStamped HectorMoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg) {
    std::string global_frame = costmap_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    //@TODO check if transform is possible for timestamp
    // at least throw ROS_ERROR
    goal_pose.stamp_ = ros::Time();

    try{
        tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
        ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                 goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
        return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;

}

bool HectorMoveBase::isGoalIDEqual(const actionlib_msgs::GoalID& firstGoalID, const actionlib_msgs::GoalID& secondGoalID) {
    return firstGoalID.stamp == secondGoalID.stamp && firstGoalID.id == secondGoalID.id;
}
}

int main(int argc, char** argv) {
    // in order to replace the original move_base seamlessly we use its name
    ros::init(argc, argv, "move_base");

    tf::TransformListener tf(ros::Duration(10));

    // a thread is started in the constructor
    hector_move_base::HectorMoveBase hector_move_base("move_base", tf);

    ros::Rate rate = ros::Rate(10.0);

    while (ros::ok()) {
        hector_move_base.moveBaseStep();
        ros::spinOnce();
        rate.sleep();
    }

    return(0);

}
