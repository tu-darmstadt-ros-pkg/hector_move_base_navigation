#ifndef HANDLER_HECTOR_WAIT_FOR_REPLANNING_H_
#define HANDLER_HECTOR_WAIT_FOR_REPLANNING_H_

#include <hector_nav_core/hector_move_base_handler.h>

namespace hector_move_base_handler {

class HectorWaitForReplanningHandler : public HectorMoveBaseHandler {
private:
    double max_time_to_replan_;
public:
    HectorWaitForReplanningHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        ros::NodeHandle private_nh("~");
        private_nh.param("max_time_to_replan", max_time_to_replan_, 3.0);
    }

    hector_move_base::RESULT handle(){
        if ((hectorMoveBaseInterface->getCurrentActionPath().goal.target_path.header.stamp + ros::Duration(max_time_to_replan_)) < ros::Time::now()) {
            ROS_DEBUG("[wait_for_replanning_handler]: ALTERNATIVE start replanning");
            return hector_move_base::ALTERNATIVE;
        }
        return hector_move_base::NEXT;
    }

    void abort(){
    }
};
}
#endif
