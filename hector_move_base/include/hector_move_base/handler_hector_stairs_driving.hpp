#ifndef HANDLER_STAIRS_DRIVING_H_
#define HANDLER_STAIRS_DRIVING_H_

#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_core/exploration_planner.h>
#include <pluginlib/class_loader.h>
#include <hector_stairs_planner_msgs/Path_segment.h>
#include <hector_stairs_planner_msgs/Path_with_Flipper.h>

namespace hector_move_base_handler {

class HectorStairsDrivingHandler : public HectorMoveBaseHandler {
private:
    ros::Publisher drivepath_pub_;
    ros::Subscriber result_sub_;
    ros::Publisher drivepath_pub_debug;
    bool goal_reached_;
    bool goal_reached_reseted_;
    int current_path_segment_;
    actionlib_msgs::GoalStatus goalStatus_;

public:
    HectorStairsDrivingHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        ros::NodeHandle nh("");
        drivepath_pub_ = nh.advertise<hector_move_base_msgs::MoveBaseActionPath>("/controller/path", 1);
        result_sub_ = nh.subscribe<hector_move_base_msgs::MoveBaseActionResult>("/controller/result", 1, &HectorStairsDrivingHandler::resultCB, this);
        drivepath_pub_debug= nh.advertise<nav_msgs::Path>("hector_move_base/stairs_driving_path_segment", 1);
        current_path_segment_=0;
        goal_reached_=false;
        goal_reached_reseted_=false;
    }

    hector_move_base::RESULT handle()
    {
        ROS_DEBUG("[move_base] [stairs_driving_handler] stairs_driving started.");
        hector_stairs_planner_msgs::Path_with_Flipper extended_path= hectorMoveBaseInterface->getCurrentExtendedPath();
        drivepath_pub_debug.publish(extended_path.path.at(current_path_segment_));
        if(!goal_reached_){
            hector_move_base_msgs::MoveBaseActionPath path;
            path.header.frame_id=extended_path.path.at(current_path_segment_).segment.header.frame_id;
            path.goal.target_path=extended_path.path.at(current_path_segment_).segment;
            path.goal.speed=0.1;
            path.goal_id.id="stairs_driving_path";
            path.goal.fixed=false;
            //publish path
            if(goalStatus_.status != actionlib_msgs::GoalStatus::ACTIVE){
                drivepath_pub_.publish(path);
            }
            
            goal_reached_reseted_=false;
            return hector_move_base::NEXT; //call this state again
        }else{
            current_path_segment_=current_path_segment_+1;
            goal_reached_reseted_=true;
            goal_reached_=false;
            if(current_path_segment_ >= extended_path.path.size()){
                current_path_segment_=0;
            }
            return hector_move_base::ALTERNATIVE; //switch to flipper action state
        }
    }

    void abort()
    {
        ROS_WARN("[move_base] [stairs_driving_handler] Abort was called in planning, this seams to lead to unresponsive behavior");
    }

    void resultCB(const hector_move_base_msgs::MoveBaseActionResultConstPtr &result){
        goalStatus_.status= result->status.status;
        if(result->status.goal_id.id.compare(std::string("stairs_driving_path"))==0 && goalStatus_.status == actionlib_msgs::GoalStatus::SUCCEEDED){
            if(goal_reached_reseted_==false){
                goal_reached_=true;
            }
        }else{
            goal_reached_=false;
        }
    }

};
}
#endif
