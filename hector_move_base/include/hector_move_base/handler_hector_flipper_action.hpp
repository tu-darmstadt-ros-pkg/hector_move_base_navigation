#ifndef HANDLER_FLIPPER_ACTION_H_
#define HANDLER_FLIPPER_ACTION_H_

#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_core/exploration_planner.h>
#include <pluginlib/class_loader.h>
#include <hector_sbpl_stairs_planner/Path_with_Flipper.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

namespace hector_move_base_handler {

class HectorFlipperActionHandler: public HectorMoveBaseHandler {
private:
    ros::Publisher flipper_pub_;
    ros::Subscriber joint_sub_;
    float current_front_=-0.5;
    float current_rear_=-0.5;
    float flipper_tolerance_;
    int current_path_segment_;
    bool published_=false;

public:
    HectorFlipperActionHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        ros::NodeHandle nh("");
        flipper_tolerance_=0.2;
        flipper_pub_= nh.advertise<trajectory_msgs::JointTrajectory>("/obelix_robot/flipper_traj_controller/command", 1 );
//        joint_sub_= nh.subscribe("/obelix_robot/flipper_traj_controller/state", 1, &HectorFlipperActionHandler::flipperCB, this);
        joint_sub_= nh.subscribe("/joint_states", 1, &HectorFlipperActionHandler::flipperCB, this);


        current_path_segment_=0;
    }

    hector_move_base::RESULT handle()
    {
        ROS_DEBUG("[move_base] [flipper_action_state] flipper_action started.");
        hector_sbpl_stairs_planner::Path_with_Flipper extended_path= hectorMoveBaseInterface->getCurrentExtendedPath();
        if(fabs(current_front_- extended_path.path.at(current_path_segment_).flipperFront)>flipper_tolerance_ || fabs(current_rear_+extended_path.path.at(current_path_segment_).flipperRear)>flipper_tolerance_){
            if(!published_){
                //publish flipper pos
                trajectory_msgs::JointTrajectory flipper_traj;
                trajectory_msgs::JointTrajectoryPoint traj_point;
                flipper_traj.header.frame_id=extended_path.path.at(current_path_segment_).segment.header.frame_id;
                flipper_traj.joint_names.push_back("front_flipper_joint");
                flipper_traj.joint_names.push_back("rear_flipper_joint");
                //caution in hector setup angles are inverted, in argos setup not
                traj_point.positions.push_back(extended_path.path.at(current_path_segment_).flipperFront);
                traj_point.positions.push_back(-extended_path.path.at(current_path_segment_).flipperRear);
                traj_point.time_from_start=ros::Duration(5.0);
                flipper_traj.points.push_back(traj_point);
                flipper_traj.header.stamp=ros::Time::now();
                flipper_pub_.publish(flipper_traj);
                published_=true;
            }
            return hector_move_base::NEXT; //this state again
        }else{
            current_path_segment_= current_path_segment_+1;
            if(current_path_segment_>=extended_path.path.size()){
                current_path_segment_=0;
                //back to normal state machine
                published_=false;
                return hector_move_base::FAIL;
            }
            published_=false;
            return hector_move_base::ALTERNATIVE; //back to stairs_driving_state
        }

    }

    void abort()
    {
        ROS_WARN("[move_base] [flipper_action_state] Abort was called in planning, this seams to lead to unresponsive behavior");
    }

    void flipperCB(const sensor_msgs::JointState &state){
        for(int i=0; i<state.name.size(); i++){
            if(state.name.at(i).compare(std::string("front_flipper_joint"))==0){
                current_front_=state.position.at(i);
                continue;
            }
            if(state.name.at(i).compare(std::string("rear_flipper_joint"))==0){
                current_rear_=state.position.at(i);
                continue;
            }
        }
    }

};
}
#endif
