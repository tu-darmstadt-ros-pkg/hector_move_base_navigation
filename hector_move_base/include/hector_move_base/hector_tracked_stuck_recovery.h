#ifndef HECTOR_TRACKED_STUCK_RECOVERY_H
#define HECTOR_TRACKED_STUCK_RECOVERY_H

#include <base_local_planner/costmap_model.h>
#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_msgs/GetRecoveryInfo.h>
#include <hector_move_base_msgs/move_base_action.h>
#include <hector_move_base_msgs/MoveBaseActionGeneric.h>
#include <hector_move_base_msgs/MoveBaseActionResult.h>
#include <nav_core/recovery_behavior.h>
//#include <pluginlib/class_loader.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

namespace hector_move_base_handler
{

class HectorTrackedStuckRecoveryHandler : public HectorMoveBaseHandler
{
private:
    costmap_2d::Costmap2DROS* costmap_;
    tf::TransformListener& tf_;

    ros::ServiceClient inverse_trajectory_service_client_;
    ros::Publisher path_pub_;
    ros::Publisher inv_traj_back_pose_pub_;
    ros::Publisher cmd_flipper_toggle_pub_;

    ros::Subscriber path_drive_feedback_sub_;

    hector_move_base_msgs::MoveBaseActionGeneric target_path_goal_;
    hector_move_base_msgs::MoveBaseActionResult target_path_result_;

    boost::condition condition_path_ready_;
    boost::mutex condition_mutex_;

    base_local_planner::CostmapModel* world_model_;
    unsigned int counter;

public:
    HectorTrackedStuckRecoveryHandler(hector_move_base::IHectorMoveBase* interface) :
        HectorMoveBaseHandler(interface),
        costmap_(interface->getCostmap()),
        tf_(interface->getTransformListener())
    {

        ros::NodeHandle nh;

        inverse_trajectory_service_client_ = nh.serviceClient<hector_nav_msgs::GetRecoveryInfo>("trajectory_recovery_info");
        inv_traj_back_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("inv_traj_back_pose",1);
        cmd_flipper_toggle_pub_ = nh.advertise<std_msgs::Empty>("cmd_flipper_toggle", 1);

        ros::NodeHandle controller_nh("controller");
        path_pub_ = controller_nh.advertise<hector_move_base_msgs::MoveBaseActionGeneric>("generic",1);
        path_drive_feedback_sub_ = controller_nh.subscribe("result", 5, &HectorTrackedStuckRecoveryHandler::resultCallback, this);

        world_model_ = new base_local_planner::CostmapModel(*costmap_->getCostmap());

        counter = 0;
    }

    hector_move_base::RESULT handle()
    {
        ROS_INFO("[hector_move_base] Stuck recovery handler");
        ROS_INFO("                   Toggling flipper state");
        cmd_flipper_toggle_pub_.publish(std_msgs::Empty());

        ROS_INFO("                   Running inverse trajectory recovery for tracked vehicles.");

        if (counter++ > 4)
        {
            reset();
            return hector_move_base::FAIL;
        }

        counter++;

        if(costmap_ == NULL)
        {
            ROS_ERROR("                   Costmap passed to the ClearCostmapRecovery is NULL, aborting recovery behavior.");
            return hector_move_base::FAIL;
        }

        hector_nav_msgs::GetRecoveryInfo trajectory_info;

        ros::Time curr_time = ros::Time::now();

//        trajectory_info.request.request_time = curr_time;
//        trajectory_info.request.request_radius = 0.5;
//        bool service_success = inverse_trajectory_service_client_.call(trajectory_info);
//        if (!service_success)
//        {
//            ROS_INFO("                   Inverse trajectory service call failed, aborting recovery behavior");
//            return hector_move_base::FAIL;
//        }
//        else
//        {
//            ROS_INFO("                   Received inverse path with %lu poses",
//                     trajectory_info.response.trajectory_radius_entry_pose_to_req_pose.poses.size() );
//        }

        boost::mutex::scoped_lock lock(condition_mutex_);

        target_path_goal_.header.stamp = curr_time;
        target_path_goal_.goal_id.id = "inverse trajectory trajectory";
        target_path_goal_.goal_id.stamp = curr_time;

        hector_move_base_msgs::MoveBaseGoal target_goal;

        tf::Stamped<tf::Pose> rb_;
        geometry_msgs::PoseStamped rb;
        geometry_msgs::PoseStamped rbb;
        costmap_->getRobotPose(rb_);
        tf::poseStampedTFToMsg(rb_, rb);
        tf_.transformPose("base_link", rb, rbb);
        rbb.pose.position.x -= 0.2;
        tf_.transformPose(rb.header.frame_id, rbb, rb);
        rb.header.stamp = curr_time;

        target_goal.target_pose = rb;
        target_goal.target_pose.header.stamp = ros::Time(0);
        hector_move_base_msgs::setAction(target_path_goal_, target_goal);

        ROS_INFO("Publishing to %s,  frame id = %s", path_pub_.getTopic().c_str(), rb.header.frame_id.c_str());
        path_pub_.publish(target_path_goal_);

        //@TODO HACK
        sleep(5);
        return hector_move_base::NEXT;
    }

    void reset()
    {
        counter = 0;
    }

    void resultCallback(const hector_move_base_msgs::MoveBaseActionResult& move_base_result)
    {
        if (move_base_result.status.goal_id.id == target_path_goal_.goal_id.id
        &&  move_base_result.status.goal_id.stamp == target_path_goal_.goal_id.stamp)
        {
            target_path_result_ = move_base_result;
            condition_path_ready_.notify_one();
        }
    }
};
}

#endif // HECTOR_TRACKED_STUCK_RECOVERY_H
