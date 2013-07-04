#ifndef HANDLER_HECTOR_PLANNING_H_
#define HANDLER_HECTOR_PLANNING_H_

#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_core/exploration_planner.h>
#include <pluginlib/class_loader.h>

namespace hector_move_base_handler {

class HectorPlanningHandler : public HectorMoveBaseHandler {
private:
    costmap_2d::Costmap2DROS* costmap_;
    hector_nav_core::ExplorationPlanner* exploration_planner_;

public:
    HectorPlanningHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
        costmap_ = interface->getCostmap();
        ros::NodeHandle private_nh("~");

        pluginlib::ClassLoader<hector_nav_core::ExplorationPlanner> expl_loader_("hector_nav_core", "hector_nav_core::ExplorationPlanner");

        std::string exploration_planner_name = "hector_nav_core_exploration_plugin/HectorNavCoreExplorationPlugin";
        private_nh.param("exploration_planner", exploration_planner_name, exploration_planner_name);


        try {
            //check if a non fully qualified name has potentially been passed in
            if(!expl_loader_.isClassAvailable(exploration_planner_name)){
                std::vector<std::string> classes = expl_loader_.getDeclaredClasses();
                for(unsigned int i = 0; i < classes.size(); ++i){
                    if(exploration_planner_name == expl_loader_.getName(classes[i])){
                        //if we've found a match... we'll get the fully qualified name and break out of the loop
                        ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                 exploration_planner_name.c_str(), classes[i].c_str());
                        exploration_planner_name = classes[i];
                        break;
                    }
                }
            }

            exploration_planner_ = expl_loader_.createClassInstance(exploration_planner_name);
            exploration_planner_->initialize(expl_loader_.getName(exploration_planner_name), costmap_);
        } catch (const pluginlib::PluginlibException& ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", exploration_planner_name.c_str(), ex.what());
            exit(0);
        }
    }

    hector_move_base::RESULT handle() {

        hector_move_base::handlerActionGoal current_goal = hectorMoveBaseInterface->getCurrentGoal();

        ROS_DEBUG("[planning_handler]: starting planning");
        std::vector<geometry_msgs::PoseStamped> plan;

        if(costmap_ == NULL){
            ROS_ERROR("[planning_handler]: makePlan failed due to costmap being NULL pointer");
            return hector_move_base::FAIL;
        }

        //get the starting pose of the robot
        tf::Stamped<tf::Pose> global_pose;
        if(!costmap_->getRobotPose(global_pose)){
            ROS_ERROR("[planning_handler]: makePlan failed due to pose not being retrievable from costmap");
            return hector_move_base::FAIL;
        }

        geometry_msgs::PoseStamped start;
        tf::poseStampedTFToMsg(global_pose, start);

        //if the planner fails or returns a zero length plan, planning failed
        if((!exploration_planner_->makePlan(start, current_goal.target_pose, plan, current_goal.distance)) || plan.empty()){
            ROS_INFO("[planning_handler]: execution of hector planner failed for goal (%.2f, %.2f)", current_goal.target_pose.pose.position.x, current_goal.target_pose.pose.position.y);
            if (hectorMoveBaseInterface->getGlobalGoal().do_exploration) {
                ROS_INFO("[planning_handler]: In Exploration. Looking for new frontier.");
                return hector_move_base::ALTERNATIVE;
        }
            return hector_move_base::FAIL;
        }

        hector_move_base_msgs::MoveBaseActionPath new_path = hector_move_base_msgs::MoveBaseActionPath();
        new_path.goal_id = current_goal.goal_id;
        new_path.header.frame_id = current_goal.target_pose.header.frame_id;
        new_path.header.stamp = current_goal.target_pose.header.stamp;
        new_path.goal.speed = current_goal.speed;
        new_path.goal.target_path.header.frame_id = new_path.header.frame_id ;
        new_path.goal.target_path.header.stamp = ros::Time::now() ;
        new_path.goal.target_path.poses = plan;
        hectorMoveBaseInterface->setActionPath(new_path);
        ROS_DEBUG("[planning_handler]: NEXT plan generated, continue");
        return hector_move_base::NEXT;
    }

    void abort() {
        ROS_WARN("[planning_handler]: abort was called in planning this seams to lead to unresponsive behavior");
    }
};
}
#endif
