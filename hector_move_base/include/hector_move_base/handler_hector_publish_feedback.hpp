#ifndef HANDLER_HECTOR_PUBLISH_FEEDBACK_H_
#define HANDLER_HECTOR_PUBLISH_FEEDBACK_H_

#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_move_base_msgs/MoveBaseActionFeedback.h>

namespace hector_move_base_handler {

class HectorPublishFeedbackHandler : public HectorMoveBaseHandler {
private:
    costmap_2d::Costmap2DROS* costmap_;
    ros::Publisher feedback_pub_;
public:
    HectorPublishFeedbackHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        costmap_ = hectorMoveBaseInterface->getCostmap();
        ros::NodeHandle private_nh("~");
        feedback_pub_ = private_nh.advertise<hector_move_base_msgs::MoveBaseActionFeedback>("feedback", 0 );
    }

    hector_move_base::RESULT handle(){
        tf::Stamped<tf::Pose> robot_pose;
        costmap_->getRobotPose(robot_pose);
        geometry_msgs::PoseStamped current_position;
        tf::poseStampedTFToMsg(robot_pose, current_position);

        hector_move_base_msgs::MoveBaseActionFeedback action_feedback = hector_move_base_msgs::MoveBaseActionFeedback();
//        action_feedback.feedback.base_position = current_position;
        action_feedback.header.stamp = ros::Time::now();
        action_feedback.status.goal_id = hectorMoveBaseInterface->getGlobalGoal().goal_id;
        action_feedback.status.status = actionlib_msgs::GoalStatus::ACTIVE;
        feedback_pub_.publish(action_feedback);
        return hector_move_base::NEXT;
    }

    void abort(){
    }
};
}
#endif
