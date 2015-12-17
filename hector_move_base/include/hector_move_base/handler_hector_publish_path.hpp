#ifndef HANDLER_HECTOR_PUBLISH_PATH_H_
#define HANDLER_HECTOR_PUBLISH_PATH_H_

#include <hector_nav_core/hector_move_base_handler.h>

namespace hector_move_base_handler {

class HectorPublishPathHandler : public HectorMoveBaseHandler {
private:
    ros::Publisher drivepath_pub_;

public:
    HectorPublishPathHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        ros::NodeHandle private_nh("~");
        std::string controller_namespace;
        private_nh.param("controller_namespace", controller_namespace, std::string("/controller"));
        ros::NodeHandle controller_nh(controller_namespace);
        drivepath_pub_ = controller_nh.advertise<hector_move_base_msgs::MoveBaseActionPath>("path", 0 );
    }

    hector_move_base::RESULT handle()
    {
        ROS_DEBUG("[publish_path_handler]: starting publish path");
        drivepath_pub_.publish(hectorMoveBaseInterface->getCurrentActionPath());
        return hector_move_base::NEXT;
    }

    void abort()
    {

    }
};
}
#endif
