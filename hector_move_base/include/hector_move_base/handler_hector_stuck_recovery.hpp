#ifndef HANDLER_HECTOR_STUCK_RECOVERY_H_
#define HANDLER_HECTOR_STUCK_RECOVERY_H_

#include <base_local_planner/costmap_model.h>
#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_msgs/GetRecoveryInfo.h>
#include <hector_move_base_msgs/move_base_action.h>
#include <hector_move_base_msgs/MoveBaseActionGeneric.h>
#include <hector_move_base_msgs/MoveBaseActionResult.h>
#include <nav_core/recovery_behavior.h>
#include <pluginlib/class_loader.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

namespace hector_move_base_handler {

class HectorStuckRecoveryHandler : public HectorMoveBaseHandler {
private:
    costmap_2d::Costmap2DROS* costmap_;
    costmap_2d::Costmap2D local_costmap_;
    tf::TransformListener& tf_;

    ros::ServiceClient inverse_trajectory_service_client_;
    ros::Publisher path_pub_;
    ros::Publisher inv_traj_back_pose_pub_;
    ros::Subscriber path_drive_feedback_sub_;

    hector_move_base_msgs::MoveBaseActionGeneric target_path_goal_;
    hector_move_base_msgs::MoveBaseActionResult target_path_result_;

    boost::condition condition_path_ready_;
    boost::mutex condition_mutex_;

    base_local_planner::CostmapModel* world_model_;
    unsigned int counter;

public:
    HectorStuckRecoveryHandler(hector_move_base::IHectorMoveBase* interface) :
        HectorMoveBaseHandler(interface),
        costmap_(interface->getCostmap()),
        tf_(interface->getTransformListener()) {

        ros::NodeHandle nh;

        inverse_trajectory_service_client_ = nh.serviceClient<hector_nav_msgs::GetRecoveryInfo>("trajectory_recovery_info");
        inv_traj_back_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("inv_traj_back_pose",1);

        ros::NodeHandle controller_nh("controller");
        path_pub_ = controller_nh.advertise<hector_move_base_msgs::MoveBaseActionGeneric>("generic",1);
        path_drive_feedback_sub_ = controller_nh.subscribe("result", 5, &HectorStuckRecoveryHandler::resultCallback, this);

        costmap_->getCostmapCopy(local_costmap_);
        world_model_ = new base_local_planner::CostmapModel(local_costmap_);

        counter = 0;
    }

    hector_move_base::RESULT handle(){

        ROS_INFO("Running inverse trajectory recovery");

        if (counter > 4) {
            reset();
            return hector_move_base::FAIL;
        }
        else {
            counter++;
        }

        if(costmap_ == NULL){
            ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
            return hector_move_base::FAIL;
        }

        hector_nav_msgs::GetRecoveryInfo trajectory_info;

        ros::Time curr_time = ros::Time::now();

        trajectory_info.request.request_time = curr_time;
        trajectory_info.request.request_radius = 0.5;

        bool service_success = inverse_trajectory_service_client_.call(trajectory_info);

        if (!service_success){
            ROS_INFO("Call to inverse trajectory service failed, aborting recovery behavior");
            return hector_move_base::FAIL;
        }else{
            ROS_INFO("Received inverse path with %lu poses", trajectory_info.response.trajectory_radius_entry_pose_to_req_pose.poses.size() );
        }

        boost::mutex::scoped_lock lock(condition_mutex_);

        target_path_goal_.header.stamp = curr_time;
        target_path_goal_.goal_id.id = "inverse trajectory trajectory";
        target_path_goal_.goal_id.stamp = curr_time;

        if (false){
            hector_move_base_msgs::MoveBasePath target_path;
            target_path.target_path = trajectory_info.response.trajectory_radius_entry_pose_to_req_pose;
            hector_move_base_msgs::setAction(target_path_goal_, target_path);
        }else{
            hector_move_base_msgs::MoveBaseGoal target_goal;
            target_goal.target_pose = trajectory_info.response.radius_entry_pose;
            target_goal.target_pose.header.stamp = trajectory_info.response.trajectory_radius_entry_pose_to_req_pose.header.stamp;
            inv_traj_back_pose_pub_.publish(target_goal.target_pose);
            hector_move_base_msgs::setAction(target_path_goal_, target_goal);
        }

        path_pub_.publish(target_path_goal_);

        //@TODO HACK
        sleep(5);
        return hector_move_base::NEXT;

        while (1){
            while (!condition_path_ready_.timed_wait(lock, ros::Duration(1.0).toBoost())){
                if ((ros::Time::now() - curr_time) > ros::Duration(30.0)){
                    ROS_WARN("Failed inverse trajectory recovery after X seconds");
                    return hector_move_base::FAIL;
                }
            }

            if (target_path_result_.status.status == actionlib_msgs::GoalStatus::SUCCEEDED){
                ROS_INFO("Recovery successful");
                return hector_move_base::NEXT;
            }else if (target_path_result_.status.status != actionlib_msgs::GoalStatus::ACTIVE){
                ROS_INFO("Recovery aborted, cause: %s",target_path_result_.status.text.c_str() );
                return hector_move_base::NEXT;
            }
        }

        ROS_INFO("This should never be reached");

        return hector_move_base::FAIL;
    }

    void reset() {
        counter = 0;
    }

    void resultCallback(const hector_move_base_msgs::MoveBaseActionResult& move_base_result) {
        if ((move_base_result.status.goal_id.id == target_path_goal_.goal_id.id) && (move_base_result.status.goal_id.stamp == target_path_goal_.goal_id.stamp)) {
            target_path_result_ = move_base_result;
            condition_path_ready_.notify_one();
        }
    }
};
}
#endif
