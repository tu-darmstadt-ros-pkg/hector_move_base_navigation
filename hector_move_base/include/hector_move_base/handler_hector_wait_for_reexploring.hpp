#ifndef HANDLER_HECTOR_WAIT_FOR_REEXPLORING_H_
#define HANDLER_HECTOR_WAIT_FOR_REEXPLORING_H_

#include <hector_nav_core/hector_move_base_handler.h>

namespace hector_move_base_handler {

class HectorWaitForReexploringHandler : public HectorMoveBaseHandler {
private:
    double time_to_trigger_exploration_;
public:
    HectorWaitForReexploringHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        ros::NodeHandle private_nh("~");
        private_nh.param("time_to_trigger_exploration", time_to_trigger_exploration_, 9.0);
    }

    hector_move_base::RESULT handle(){

        if (hectorMoveBaseInterface->getGlobalGoal().do_exploration) {
            ROS_DEBUG("[wait_for_reexploring_handler]: check reexploring time (%f < %f)", ros::Time::now().toSec(), (hectorMoveBaseInterface->getCurrentGoal().goal_id.stamp + ros::Duration(time_to_trigger_exploration_)).toSec());
            if ((hectorMoveBaseInterface->getCurrentGoal().goal_id.stamp + ros::Duration(time_to_trigger_exploration_)) < ros::Time::now()) {
                hectorMoveBaseInterface->getCurrentGoal().goal_id.stamp = ros::Time::now();
                ROS_INFO("[wait_for_reexploring_handler]: start reexploring");
                return hector_move_base::ALTERNATIVE;
            }
        }
        return hector_move_base::NEXT;
    }

    void abort(){
    }
};
}
#endif
