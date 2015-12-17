#ifndef HANDLER_HECTOR_PUBLISH_ABORT_H_
#define HANDLER_HECTOR_PUBLISH_ABORT_H_

#include <hector_nav_core/hector_move_base_handler.h>

namespace hector_move_base_handler {

class HectorPublishAbortHandler : public HectorMoveBaseHandler {
private:
    ros::Publisher result_pub_;
public:
    HectorPublishAbortHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        ros::NodeHandle private_nh("~");
        result_pub_ = private_nh.advertise<hector_move_base_msgs::MoveBaseActionResult>("result", 0 );
    }

    hector_move_base::RESULT handle()
    {
        ROS_DEBUG("[publish_abort_handler]: starting publish abort");
        hector_move_base_msgs::MoveBaseActionResult action_result;
        action_result.header.stamp = ros::Time::now();
        action_result.status.goal_id = hectorMoveBaseInterface->getCurrentGoal().goal_id;
        action_result.status.status = actionlib_msgs::GoalStatus::ABORTED;
        result_pub_.publish(action_result);
        return hector_move_base::NEXT;
    }

    void abort(){
    }
};
}
#endif
