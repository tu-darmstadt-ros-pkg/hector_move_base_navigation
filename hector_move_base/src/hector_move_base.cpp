#include <hector_move_base/hector_move_base.h>

namespace hector_move_base {

HectorMoveBase::HectorMoveBase(std::string name, tf::TransformListener& tf) :
    costmap_(NULL),
    private_nh_("~"),
    statemachine_(new HectorMoveBaseStateMachine),
    tf_(tf),
    main_loop_thread_(NULL){

    ros::NodeHandle nh;

    double controllerFrequencyParam;
    private_nh_.param("controller_frequency", controllerFrequencyParam, 1.0);
    ros::Rate controllerFrequency(controllerFrequencyParam);

    private_nh_.param("circumscribed_radius", circumscribedRadius_, 0.3);
    private_nh_.param("time_to_trigger_replanning", timeToTriggerReplannning_, 2.0);
    private_nh_.param("time_to_trigger_exploration", timeToTriggerExploration_, 4.0);
    private_nh_.param("goal_reached_angular_variance", goalReachchedAngularVariance_, M_PI_4);
    double goalReachedLinearVariance;
    private_nh_.param("goal_reached_linear_variance", goalReachedLinearVariance, M_PI_4);
    goalReachedSquaredLinearVariance_ = goalReachedLinearVariance * goalReachedLinearVariance;
    private_nh_.param("controller_namespace", controller_namespace_, std::string("/controller"));

    costmap_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    ROS_DEBUG("[hector_move_base]: costmap loaded");

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
    stuckRecoveryState_.reset(new hector_move_base_handler::HectorStuckRecoveryHandler(this));
    idleState_.reset(new hector_move_base_handler::HectorIdleHandler(this));

    ROS_DEBUG("[hector_move_base]: all states created");

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForExploration;
    mappingForExploration.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, planningState_));
    mappingForExploration.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(ALTERNATIVE, refinePlanState_));
    mappingForExploration.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(FAIL, stuckRecoveryState_));
    statemachine_->addHandlerMapping(exploringState_, mappingForExploration);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPlanning;
    mappingForPlanning.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, refinePlanState_));
    mappingForPlanning.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(ALTERNATIVE, exploringState_));
    mappingForPlanning.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(FAIL, stuckRecoveryState_));
    statemachine_->addHandlerMapping(planningState_, mappingForPlanning);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForRefinePlan;
    mappingForRefinePlan.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, publishPathState_));
    mappingForRefinePlan.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(ALTERNATIVE, planningState_));
    mappingForRefinePlan.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(FAIL, stuckRecoveryState_));
    statemachine_->addHandlerMapping(refinePlanState_, mappingForRefinePlan);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishPath;
    mappingForPublishPath.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, publishFeedbackState_));
    statemachine_->addHandlerMapping(publishPathState_, mappingForPublishPath);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishFeedback;
    mappingForPublishFeedback.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, waitForReplanningState_));
    statemachine_->addHandlerMapping(publishFeedbackState_, mappingForPublishFeedback);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForWaitForReplanning;
    mappingForWaitForReplanning.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, waitForReexploringState_));
    mappingForWaitForReplanning.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(ALTERNATIVE, planningState_));
    statemachine_->addHandlerMapping(waitForReplanningState_, mappingForWaitForReplanning);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForWaitForReexploring;
    mappingForWaitForReexploring.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, publishFeedbackState_));
    mappingForWaitForReexploring.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(ALTERNATIVE, exploringState_));
    statemachine_->addHandlerMapping(waitForReexploringState_, mappingForWaitForReexploring);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishSuccess;
    mappingForPublishSuccess.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, idleState_));
    statemachine_->addHandlerMapping(publishSuccessState_, mappingForPublishSuccess);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishAbort;
    mappingForPublishAbort.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, idleState_));
    statemachine_->addHandlerMapping(publishAbortState_, mappingForPublishAbort);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishPreempted;
    mappingForPublishPreempted.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, idleState_));
    statemachine_->addHandlerMapping(publishPreemptedState_, mappingForPublishPreempted);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForPublishRejected;
    mappingForPublishRejected.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, idleState_));
    statemachine_->addHandlerMapping(publishRejectedState_, mappingForPublishRejected);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForStuckRecovery;
    mappingForStuckRecovery.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, exploringState_));
    mappingForStuckRecovery.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(FAIL, publishAbortState_));
    statemachine_->addHandlerMapping(stuckRecoveryState_, mappingForStuckRecovery);

    std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForIdleState;
    mappingForIdleState.insert(std::pair<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >(NEXT, idleState_));
    statemachine_->addHandlerMapping(idleState_, mappingForIdleState);

    ROS_DEBUG("[hector_move_base]: states connected to a statemachine");

    startState_ = exploringState_;
    currentState_ = idleState_;
    nextState_ = currentState_;
    path_ = hector_move_base_msgs::MoveBaseActionPath();

    ros::NodeHandle controller_nh(controller_namespace_);

    drivepath_pub_ = controller_nh.advertise<hector_move_base_msgs::MoveBaseActionPath>("path", 0 );
    goalmarker_pub_ = private_nh_.advertise<visualization_msgs::Marker>("goal_marker", 0);

    explore_sub_ = private_nh_.subscribe<hector_move_base_msgs::MoveBaseActionExplore>("explore", 1, boost::bind(&HectorMoveBase::exploreCB, this, _1));
    goal_sub_ = private_nh_.subscribe<hector_move_base_msgs::MoveBaseActionGoal>("goal", 1, boost::bind(&HectorMoveBase::goalCB, this, _1));
    observation_sub_ = private_nh_.subscribe<hector_move_base_msgs::MoveBaseActionGoal>("observe", 1, boost::bind(&HectorMoveBase::observationCB, this, _1));
    simple_goal_sub_ = private_nh_.subscribe<geometry_msgs::PoseStamped>("simple_goal", 1, boost::bind(&HectorMoveBase::simple_goalCB, this, _1));
    cancel_sub_ = private_nh_.subscribe<std_msgs::Empty>("cancel", 1, boost::bind(&HectorMoveBase::cancelCB, this, _1));
    syscommand_sub_ = nh.subscribe<std_msgs::String>("syscommand", 1, boost::bind(&HectorMoveBase::syscommandCB, this, _1));

    controller_result_sub_ = controller_nh.subscribe<hector_move_base_msgs::MoveBaseActionResult>("result", 1, boost::bind(&HectorMoveBase::controllerResultCB, this, _1));

    //    ROS_DEBUG("[hector_move_base]: going to create new boost thread for main loop");
    //    main_loop_thread_ = new boost::thread(boost::bind(&HectorMoveBase::moveBaseLoop, this, nh, controllerFrequency));
}

HectorMoveBase::~HectorMoveBase(){

    if(costmap_ != NULL)
        delete costmap_;

    if(main_loop_thread_)
        delete main_loop_thread_;
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

void HectorMoveBase::popCurrentGoal() {
    if (goals_.empty()) {
        return;
    }
    goals_.pop_back();
}

void HectorMoveBase::pushCurrentGoal(const handlerActionGoal &goalToAdd) {
    goals_.push_back(goalToAdd);
    visualization_msgs::Marker marker;
    marker.header.frame_id = goalToAdd.target_pose.header.frame_id;
    marker.header.stamp = goalToAdd.goal_id.stamp;
    marker.ns = "hector_move_base";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = goalToAdd.target_pose.pose;

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

void HectorMoveBase::sendActionGoal(const handlerActionGoal& goalToSend) {
    hector_move_base_msgs::MoveBaseActionPath pathToSend;

    pathToSend.goal_id = goalToSend.goal_id;
    pathToSend.goal.speed = goalToSend.speed;
    pathToSend.goal.target_path.header = goalToSend.target_pose.header;

    geometry_msgs::PoseStamped targetPose;
    targetPose.header = goalToSend.target_pose.header;
    targetPose.pose = goalToSend.target_pose.pose;
    pathToSend.goal.target_path.poses.push_back(targetPose);

    drivepath_pub_.publish(pathToSend);
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


void HectorMoveBase::exploreCB(const hector_move_base_msgs::MoveBaseActionExplore::ConstPtr& goal){
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

void HectorMoveBase::goalCB(const hector_move_base_msgs::MoveBaseActionGoal::ConstPtr& goal){
    ROS_DEBUG("[hector_move_base]: In goal callback");
    abortedGoal();

    handlerActionGoal newGoal = handlerActionGoal();
    newGoal.goal_id = goal->goal_id;
    newGoal.speed = goal->goal.speed;
    newGoal.do_exploration = false;

    //make sure goal could be transformed to costmap frame
    newGoal.target_pose = goalToGlobalFrame(goal->goal.target_pose);
    if (newGoal.target_pose.header.frame_id != costmap_->getGlobalFrameID()) {
        ROS_ERROR("[hector_move_base]: tf transformation into global frame failed. goal will be canceled");
        //new Goal has to be set in order to publish goal aborted result
        pushCurrentGoal(newGoal);
        abortedGoal();
        return;
    }
    pushCurrentGoal(newGoal);
    setNextState(planningState_);
    return;
}

void HectorMoveBase::observationCB(const hector_move_base_msgs::MoveBaseActionGoal::ConstPtr& goal){
    ROS_DEBUG("[hector_move_base]: In observation callback");
    abortedGoal();

    handlerActionGoal newGoal = handlerActionGoal();
    newGoal.goal_id = goal->goal_id;
    newGoal.speed = goal->goal.speed;
    newGoal.do_exploration = false;
    newGoal.distance = goal->goal.distance;

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

void HectorMoveBase::simple_goalCB(const geometry_msgs::PoseStamped::ConstPtr& simpleGoal){
    ROS_DEBUG("[hector_move_base]: In simple goal callback");
    abortedGoal();
    handlerActionGoal newGoal = handlerActionGoal();
    newGoal.goal_id.stamp = simpleGoal->header.stamp;
    newGoal.goal_id.id = "simple_goal";
    newGoal.target_pose = *simpleGoal;
    newGoal.do_exploration = false;
    pushCurrentGoal(newGoal);
    setNextState(planningState_);
    return;
}

void HectorMoveBase::cancelCB(const std_msgs::Empty::ConstPtr& empty){
    ROS_DEBUG("[hector_move_base]: In cancel callback");
    abortedGoal();
    setNextState(idleState_);
}

void HectorMoveBase::syscommandCB(const std_msgs::String::ConstPtr& string){
    ROS_DEBUG("[hector_move_base]: In syscommandCB callback: %s", string->data.c_str());

    if (string->data == "reset") {
      abortedGoal();
      setNextState(idleState_);

      // reset costmap
#ifdef LAYERED_COSTMAP_H_
      costmap_->getCostmap()->resetMap(0, 0, costmap_->getCostmap()->getSizeInCellsX(), costmap_->getCostmap()->getSizeInCellsX());
#endif // LAYERED_COSTMAP_H_
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

void HectorMoveBase::controllerResultCB(const hector_move_base_msgs::MoveBaseActionResult::ConstPtr& result){
    ROS_DEBUG("[hector_move_base]: In controller result callback");

    if (!((isGoalIDEqual(getCurrentGoal().goal_id, result->status.goal_id)) ||
          (isGoalIDEqual(getGlobalGoal().goal_id, result->status.goal_id)) ||
          isGoalIDEqual(getCurrentActionPath().goal_id, result->status.goal_id))) {
        ROS_DEBUG("[hector_move_base]: goal is outdated, ignoring controller feedback");
        return;
    }

    switch (result->status.status) {
    case actionlib_msgs::GoalStatus::ACTIVE:
        ROS_DEBUG("[hector_move_base]: received result from controller == ACTIVE");
        return;

    case actionlib_msgs::GoalStatus::PREEMPTED:
        ROS_DEBUG("[hector_move_base]: received result from controller == PREEMPTED");
        if ((currentState_ == stuckRecoveryState_) ||
                nextState_ == stuckRecoveryState_) {
            return;
        }
        preemptedGoal();
        return;

    case actionlib_msgs::GoalStatus::REJECTED:
        ROS_DEBUG("[hector_move_base]: received result from controller == REJECTED");
        setNextState(exploringState_);
        return;

    case actionlib_msgs::GoalStatus::ABORTED:
        ROS_DEBUG("[hector_move_base]: received result from controller == ABORTED");
        setNextState(stuckRecoveryState_);
        return;

    case actionlib_msgs::GoalStatus::SUCCEEDED:
        ROS_DEBUG("[hector_move_base]: received result from controller == SUCCEEDED");
        if (isGoalIDEqual(getGlobalGoal().goal_id, result->status.goal_id)) {
            ROS_DEBUG("[hector_move_base]: reached global goal");
            successGoal();
            return;
        }
        if (getGlobalGoal().do_exploration) {
            while (goals_.size() > 1) {
                popCurrentGoal();
            }
            currentState_->abort();
            setNextState(exploringState_);
            ROS_DEBUG("[hector_move_base]: restarting exploration");
            return;
        }
        if ((goals_.size() > 1) && isGoalIDEqual(getCurrentGoal().goal_id, result->status.goal_id)) {
            popCurrentGoal();
            setNextState(exploringState_);
            ROS_DEBUG("[hector_move_base]: start planning for next goal");
            return;
        }
        ROS_WARN("[hector_move_base]: result goal_id does not match global goal or current goal.");
        return;

    default:
        ROS_WARN("[hector_move_base]: controller_feedback result is %i. This is not handled.", result->status.status);
        return;
    }
}


void HectorMoveBase::moveBaseLoop(ros::NodeHandle& nh, ros::Rate rate) {
    ROS_DEBUG("[hector_move_base]: moveBaseLoop started");
    std::cout << "test" << std::endl;
    while (nh.ok()) {
        moveBaseStep();
        rate.sleep();
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
    publishAbortState_->handle();
    clearGoal();
}

void HectorMoveBase::preemptedGoal() {
    currentState_->abort();
    publishPreemptedState_->handle();
    // do not send an empty path when preempted. Preempted is triggered when controller received an ActionGoal/Path
//    clearGoal();
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
    publishSuccessState_->handle();
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

geometry_msgs::PoseStamped HectorMoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
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
    return ((firstGoalID.stamp == secondGoalID.stamp) && (firstGoalID.id == secondGoalID.id));
}
}

int main(int argc, char** argv){
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
