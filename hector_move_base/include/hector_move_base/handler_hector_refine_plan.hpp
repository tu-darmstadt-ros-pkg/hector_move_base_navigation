#ifndef HANDLER_HECTOR_REFINE_PLAN_H_
#define HANDLER_HECTOR_REFINE_PLAN_H_

#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_msgs_tools/hector_nav_msgs_tools.h>

#include <angles/angles.h>

namespace hector_move_base_handler {

enum path_situation {PATH_OK,
                     WRONG_DIRECTION,
                     TOO_FAR_AWAY,
                     NO_PATH,
                     NOT_DETERMINED};

class HectorRefinePlanHandler : public HectorMoveBaseHandler {
private:
    hector_move_base_msgs::MoveBaseActionPath current_path;
    costmap_2d::Costmap2DROS* costmap_;
    double angular_limit_for_plan_, sq_xy_limit_for_plan_;
    int horizon_for_trajectory_;
    ros::Publisher pointer_on_path_pub_, robot_pointer_pub_;
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
    boost::shared_ptr<nav_core::BaseGlobalPlanner> trajectory_planner_;

public:
    HectorRefinePlanHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface)
      , bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
    {
        costmap_ = interface->getCostmap();
        ros::NodeHandle private_nh("~");

        private_nh.param("angular_limit_for_hgp", angular_limit_for_plan_, M_PI_4);
        double xy_limit_for_plan;
        private_nh.param("xy_limit_for_hgp", xy_limit_for_plan, 0.2);
        sq_xy_limit_for_plan_ = xy_limit_for_plan* xy_limit_for_plan;
        private_nh.param("horizon_for_trajectory", horizon_for_trajectory_, 20);

        pointer_on_path_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("plan_orientation", 0);
        robot_pointer_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("robot_orientation", 0);

        std::string path_planner = "SBPLLatticePlanner";
        private_nh.param("path_planner", path_planner, path_planner);

        try {
            //check if a non fully qualified name has potentially been passed in
            if(!bgp_loader_.isClassAvailable(path_planner)){
                std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
                for(unsigned int i = 0; i < classes.size(); ++i){
                    if(path_planner == bgp_loader_.getName(classes[i])){
                        //if we've found a match... we'll get the fully qualified name and break out of the loop
                        ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                 path_planner.c_str(), classes[i].c_str());
                        path_planner = classes[i];
                        break;
                    }
                }
            }
            trajectory_planner_ = bgp_loader_.createInstance(path_planner);
            trajectory_planner_->initialize(bgp_loader_.getName(path_planner), costmap_);
        }
        catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", path_planner.c_str(), ex.what());
            exit(1);
        }
        ROS_DEBUG("[refine_plan_handler]: constructed successfully");
    }

    hector_move_base::RESULT handle()
    {
        ROS_DEBUG("[refine_plan_handler]: starting refine plan");
        current_path = hectorMoveBaseInterface->getCurrentActionPath();
        if (current_path.goal.target_path.poses.empty()) {
            ROS_ERROR("[refine_plan_handler]: actionPath.goal.target_path.poses can not be followed; It is empty");
            return hector_move_base::FAIL;
        }

        ROS_DEBUG("[refine_plan_handler]: Check if path can be followed");
        std::vector<geometry_msgs::PoseStamped> trajectory;
        switch (canPathBeFollowed(current_path.goal.target_path.poses, sq_xy_limit_for_plan_, angular_limit_for_plan_, true))
        {
        case PATH_OK:
            ROS_DEBUG("[refine_plan_handler]: plan can be followed");
            return hector_move_base::NEXT;

        case WRONG_DIRECTION:
            ROS_DEBUG("[refine_plan_handler]: Wrong direction, generating trajectory");
            if (generatePath(current_path.goal.target_path.poses, trajectory)){
                ROS_DEBUG("[refine_plan_handler]: trajectory generated successfully");
                hector_move_base_msgs::MoveBaseActionPath new_path = hector_move_base_msgs::MoveBaseActionPath();
                new_path.goal_id = current_path.goal_id;
                std::stringstream goal_id_stream;
                goal_id_stream << current_path.goal_id.id << "_sbpl";
                new_path.goal_id.id = goal_id_stream.str();
                new_path.header.frame_id = current_path.header.frame_id;
                new_path.header.stamp = current_path.goal.target_path.header.stamp;
                new_path.goal.fixed = true;
                new_path.goal.speed = current_path.goal.speed;
                new_path.goal.target_path.header.frame_id = current_path.goal.target_path.header.frame_id;
                new_path.goal.target_path.header.stamp = ros::Time::now();
                new_path.goal.target_path.poses = trajectory;
                hectorMoveBaseInterface->setActionPath(new_path);
                ROS_DEBUG("[refine_plan_handler]: NEXT generated path, because of plan orientation ");
                return hector_move_base::NEXT;
            }else {
                // trjectory could not be generated, trying to follow plan
                ROS_WARN("[refine_plan_handler]: trajectory could not be generated");
                return hector_move_base::NEXT;
            }

        case TOO_FAR_AWAY:
            ROS_DEBUG("[refine_plan_handler]: ALTERNATIVE plan is too far away start replanning ");
            return hector_move_base::ALTERNATIVE;

        default:
            ROS_DEBUG("[refine_plan_handler]: undefined case.");
            break;
        }
        ROS_ERROR("[refine_plan_handler]: this should never be reached in handle()");
        return hector_move_base::FAIL;
    }

    void abort(){
    }

    path_situation canPathBeFollowed(std::vector<geometry_msgs::PoseStamped>& path, double& sq_distance_limit, double& angular_limit, bool shortenPath ){
        ROS_DEBUG("[refine_plan_handler]: canPathBeFollowed");

        tf::Stamped<tf::Pose> robotPose;
        if(!costmap_->getRobotPose(robotPose)){
            ROS_ERROR("[refine_plan_handler]: Failed to get RobotPose");
            return NOT_DETERMINED;
        }
        geometry_msgs::PoseStamped robotPoseMsg;
        tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

        double dist_to_robot;
        size_t idx;

        if (hector_nav_msgs_tools::getClosestPoseOnPath(path, Eigen::Vector2d(robotPoseMsg.pose.position.x, robotPoseMsg.pose.position.y), dist_to_robot, idx)){
            if ( (dist_to_robot*dist_to_robot) > sq_distance_limit){
                ROS_DEBUG("[refine_plan_handler]: robot is too far from path");
                return TOO_FAR_AWAY;
            }
        }else{
            //Only false if path is empty
            ROS_WARN("[refine_plan_handler]: path empty");
            return NO_PATH;
        }

        double yaw_path = 0.0;
        if (hector_nav_msgs_tools::getPathYawAngle(path, idx, yaw_path)){

            //Debug output pose on path
            if (pointer_on_path_pub_.getNumSubscribers() > 0){
                geometry_msgs::PoseStamped pointer = path[idx];

                pointer.pose.orientation.x = 0.0;
                pointer.pose.orientation.y = 0.0;
                pointer.pose.orientation.z = sin(yaw_path*0.5f);
                pointer.pose.orientation.w = cos(yaw_path*0.5f);

                pointer_on_path_pub_.publish(pointer);
            }

            //Debug output robot pose
            if (robot_pointer_pub_.getNumSubscribers() > 0){
                geometry_msgs::PoseStamped pointer = path[idx];

                pointer.pose.position = robotPoseMsg.pose.position;
                pointer.pose.orientation = robotPoseMsg.pose.orientation;

                robot_pointer_pub_.publish(pointer);
            }

            double yaw_robot = tf::getYaw(robotPoseMsg.pose.orientation);
            if ( abs(angles::shortest_angular_distance(yaw_robot, yaw_path)) < angular_limit){

                if (shortenPath && (idx > 0)) {
                    path.erase(path.begin(), path.begin()+(idx-1));
                }

                return PATH_OK;
            }else{
                ROS_DEBUG("[refine_plan_handler]: robot is not pointing in the direction of path");
                return WRONG_DIRECTION;
            }

        }else{
            //Just send this remaining pose to controller (hopefully prevent hangs with path having only one pose)
            ROS_DEBUG ("Only one pose in path, can't determine path yaw angle");
            return PATH_OK;
        }

        ROS_ERROR ("[refine_plan_hnadler]: End of canPathBeFollowed. This should never be reached");
        return NO_PATH;

    }

    bool generatePath(std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& trajectory){
        ROS_DEBUG("[refine_plan_handler]: generatePath");

        if(plan.empty()){
            ROS_ERROR("[refine_plan_handler]: given plan is empty");
            return false;
        }
        if(costmap_ == NULL){
            ROS_ERROR("[refine_plan_handler]: makePlan failed due to costmap being NULL pointer");
            return false;
        }

        // lock costmap
#ifdef LAYERED_COSTMAP_H_
        boost::unique_lock< boost::shared_mutex > lock(*(costmap_->getCostmap()->getLock()));
#endif // LAYERED_COSTMAP_H_

        //get the starting pose of the robot
        tf::Stamped<tf::Pose> global_pose;
        if(!costmap_->getRobotPose(global_pose)){
            ROS_WARN("[refine_plan_handler]: makePlan failed due to pose not being retrievable from costmap");
            return false;
        }
        geometry_msgs::PoseStamped start;
        tf::poseStampedTFToMsg(global_pose, start);
        geometry_msgs::PoseStamped goalForTrajectory;

        //shorten plan, so that trajectory is generated for short distances only
        if(plan.size() > (std::size_t) horizon_for_trajectory_) {
            ROS_DEBUG("[refine_plan_handler]: erasing part of global_plan, because it is too long (%lu > %i)", plan.size(), horizon_for_trajectory_);
            goalForTrajectory = plan[horizon_for_trajectory_];
        }
        else {
            goalForTrajectory = plan.back();
        }

        if(!(trajectory_planner_->makePlan(start, goalForTrajectory, trajectory))){
            ROS_INFO("[planning_handler]: execution of hector planner failed for goal (%.2f, %.2f)", goalForTrajectory.pose.position.x, goalForTrajectory.pose.position.y);
            return false;
        }
        ROS_DEBUG("[refine_plan_handler]: Generated a path");
        return true;
    }
};
}
#endif
