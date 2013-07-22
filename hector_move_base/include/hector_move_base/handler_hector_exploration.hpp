#ifndef HANDLER_HECTOR_EXPLORATION_H_
#define HANDLER_HECTOR_EXPLORATION_H_

#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_core/exploration_planner.h>
#include <pluginlib/class_loader.h>

namespace hector_move_base_handler {

class HectorExplorationHandler : public HectorMoveBaseHandler {
private:
    costmap_2d::Costmap2DROS* costmap_;
    hector_nav_core::ExplorationPlanner* exploration_planner_;

public:
    HectorExplorationHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
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
        ROS_DEBUG("[exploration_handler]: start exploration");

        hector_move_base::handlerActionGoal current_goal = hectorMoveBaseInterface->getGlobalGoal();
        if (current_goal.do_exploration){
            std::vector<geometry_msgs::PoseStamped> plan;
            if (!getExplorationGoal(current_goal.target_pose, plan)) {
                ROS_ERROR("[exploration_planner]: FAIL, exploration_planner could not find a frontier.");
                return hector_move_base::FAIL;
            }
            hector_move_base::handlerActionGoal new_goal = hector_move_base::handlerActionGoal();
            new_goal.do_exploration = false;
            new_goal.speed = current_goal.speed;
            new_goal.goal_id.id = current_goal.goal_id.id;
            new_goal.goal_id.stamp = ros::Time::now();
            new_goal.target_pose.header.frame_id = current_goal.target_pose.header.frame_id;
            new_goal.target_pose.header.stamp = new_goal.goal_id.stamp;
            new_goal.target_pose.pose = plan.back().pose;
            hectorMoveBaseInterface->pushCurrentGoal(new_goal);

            hector_move_base_msgs::MoveBaseActionPath new_path = hector_move_base_msgs::MoveBaseActionPath();
            new_path.goal_id = new_goal.goal_id;
            new_path.header.frame_id = new_goal.target_pose.header.frame_id;
            new_path.header.stamp = new_goal.target_pose.header.stamp;
            new_path.goal.speed = new_goal.speed;
            new_path.goal.target_path.header.frame_id = new_path.header.frame_id;
            new_path.goal.target_path.header.stamp = ros::Time::now();
            new_path.goal.target_path.poses = plan;
            hectorMoveBaseInterface->setActionPath(new_path);

            ROS_DEBUG("[exploration_handler]: ALTERNATIVE, plan generated skipping planning state");
            return hector_move_base::ALTERNATIVE;
        }
        //do_exploration is false so we skip this handler
        ROS_DEBUG("[exploration_handler]: NEXT skipping, due to no exploration goal");
        return hector_move_base::NEXT;
    }

    void abort() {
    }

    bool getExplorationGoal(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
      ROS_DEBUG("[exploration_handler]: getExplorationGoal");

      //@TODO replace with exploration call, as soon as exploration interface is available for hgp
      geometry_msgs::PoseStamped explorationGoal = geometry_msgs::PoseStamped();
      if (!getGlobalPlan(explorationGoal, plan)) {
        return false;
      }
      goal = plan.back();
      return true;
    }

    bool getGlobalPlan(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

      ROS_DEBUG("[exploration_handler]: getGlobalPlan");

      if(costmap_ == NULL){
          ROS_ERROR("[exploration_handler]: makePlan failed due to costmap being NULL pointer");
          return false;
      }

      //get the starting pose of the robot
      tf::Stamped<tf::Pose> global_pose;
      if(!costmap_->getRobotPose(global_pose)){
          ROS_ERROR("[exploration_handler]: makePlan failed due to pose not being retrievable from costmap");
          return false;
      }

      geometry_msgs::PoseStamped start;
      tf::poseStampedTFToMsg(global_pose, start);

      //start hector_global_planner once for a new goal
      if((!exploration_planner_->doExploration(start, plan)) || plan.empty()) {
        ROS_WARN("[exploration_handler]: execution of hector_global_planner failed");
        return false;
      }

      ROS_DEBUG("[exploration_handler]: Generated a plan from the base_global_planner");
      return true;
    }

};
}
#endif