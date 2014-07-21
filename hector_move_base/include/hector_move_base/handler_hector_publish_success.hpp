#ifndef HANDLER_HECTOR_PUBLISH_SUCCESS_H_
#define HANDLER_HECTOR_PUBLISH_SUCCESS_H_

#include <hector_nav_core/hector_move_base_handler.h>

namespace hector_move_base_handler {

class HectorPublishSuccessHandler : public HectorMoveBaseHandler {
private:
    ros::Publisher result_pub_;
public:
    HectorPublishSuccessHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        ros::NodeHandle private_nh("~");
        result_pub_ = private_nh.advertise<hector_move_base_msgs::MoveBaseActionResult>("result", 0 );
    }

    hector_move_base::RESULT handle(){
        ROS_DEBUG("[publish_success_handler]: starting publish success");
        hector_move_base_msgs::MoveBaseActionResult action_result;
        action_result.header.stamp = ros::Time::now();
        action_result.status.goal_id = hectorMoveBaseInterface->getGlobalGoal().goal_id;
        action_result.status.status = actionlib_msgs::GoalStatus::SUCCEEDED;
        result_pub_.publish(action_result);
        return hector_move_base::NEXT;
    }

    void abort(){
    }
};
}
#endif
