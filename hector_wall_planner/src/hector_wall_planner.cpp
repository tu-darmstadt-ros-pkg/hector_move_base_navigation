#include "hector_wall_planner.h"

PLUGINLIB_DECLARE_CLASS(hector_wall_planner, HectorWallPlanner, hector_wall_planner::HectorWallPlanner, nav_core::BaseGlobalPlanner);

using namespace hector_wall_planner;

HectorWallPlanner::HectorWallPlanner() : costmap_ros(NULL), initialized(false) {}

HectorWallPlanner::~HectorWallPlanner(){
    if(explorationTrans){
        delete[] explorationTrans;
    }
    if(obstacleTrans){
        delete[] obstacleTrans;
    }
    if(isGoal){
        delete[] isGoal;
    }
    if(frontierMap){
        delete[] frontierMap;
    }
}

HectorWallPlanner::HectorWallPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) :
        costmap_ros(NULL), initialized(false) {
    HectorWallPlanner::initialize(name, costmap_ros);
}

void HectorWallPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros){

    // unknown: 255, obstacle 254, inflated: 253, free: 0

    if(initialized){
        ROS_INFO("[wall_planner] HectorWallPlanner is already initialized!");
        return;
    }

    ROS_INFO("[wall_planner] Initializing HectorWallPlanner");

    // initialize costmaps
    this->costmap_ros = costmap_ros;
    costmap_ros->getCostmapCopy(costmap);
    occupancyGrid = costmap.getCharMap();
    mapWidth = costmap.getSizeInCellsX();
    mapHeight = costmap.getSizeInCellsY();
    mapPoints = mapWidth * mapHeight;

    // initialize explorationTrans, obstacleTrans, goalMap and frontierMap
    explorationTrans = new unsigned int[mapPoints];
    obstacleTrans = new unsigned int[mapPoints];
    isGoal = new bool[mapPoints];
    frontierMap = new int[mapPoints];
    clearFrontiers();
    resetMaps();
    startExploreWall=-1;

    // initialize parameters
    ros::NodeHandle private_nh_("~/" + name);
    ros::NodeHandle nh;
    visualization_pub_ = private_nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    private_nh_.param("security_constant", alpha, 0.5);
    private_nh_.param("min_obstacle_dist", minObstacleDist, 5);
    private_nh_.param("plan_in_unknown", planInUnknown, false);
    private_nh_.param("use_inflated_obstacle", useInflatedObs, true);
    private_nh_.param("goal_angle_penalty", goal_angle_penalty, 50);
    private_nh_.param("min_frontier_size", minFrontierSize, 5);
    private_nh_.param("dist_for_goal_reached", dist_for_goal_reached, 0.25);
    private_nh_.param("same_frontier_distance", same_frontier_dist, 0.25);

    path_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");

    ROS_DEBUG("[wall_planner] Parameter set. security_const: %f, min_obstacle_dist: %d, plan_in_unknown: %d, use_inflated_obstacle: %d, goal_angle_penalty:%d , min_frontier_size: %d, dist_for_goal_reached: %f, same_frontier: %f", alpha, minObstacleDist, planInUnknown, useInflatedObs, goal_angle_penalty, minFrontierSize,dist_for_goal_reached,same_frontier_dist);
    
    minObstacleDist = minObstacleDist * STRAIGHT_COST;

    this->name = name;
    this->initialized = true;
    this->previous_goal = -1;

}

bool HectorWallPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan){

    // do exploration? (not used anymore? -> call doExploration())
    bool explore = false;
    if(goal.pose.orientation.w == 0.0){
        if(goal.pose.orientation.x == 0.0){
            if(goal.pose.orientation.y == 0.0){
                if(goal.pose.orientation.z == 0.0){
                    explore = true;
                }
            }
        }
    }
    if(explore){
        return doExploration(start,plan);
    }

    // planning
    ROS_INFO("[wall_planner] planning: starting to make a plan to given goal point", explore);

    // setup maps and goals
    resetMaps();
    plan.clear();
    costmap_ros->getCostmapCopy(costmap);
    occupancyGrid = costmap.getCharMap();
    std::vector<geometry_msgs::PoseStamped> goals;

    // create obstacle tranform
    buildObstacleTrans();

    // plan to given goal
    goals.push_back(goal);

    // make plan
    if(!buildExplorationTrans(start,goals,true)){
        return false;
    }

    if(!getTrajectory(start,goals,plan)){
        return false;
    }

    //save and add last point
    plan.push_back(goal);
    unsigned int mx,my;
    costmap.worldToMap(goal.pose.position.x,goal.pose.position.y,mx,my);
    previous_goal = costmap.getIndex(mx,my);

    ROS_INFO("[wall_planner] planning: plan has been found! plansize: %d ", plan.size());
    return true;
}

bool HectorWallPlanner::doExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan){

    ROS_INFO("[wall_planner] exploration: starting exploration");

    // setup maps and goals
    resetMaps();
    clearFrontiers();
    plan.clear();
    costmap_ros->getCostmapCopy(costmap);
    occupancyGrid = costmap.getCharMap();
    std::vector<geometry_msgs::PoseStamped> goals;


    // create obstacle tranform
    buildObstacleTrans();

    //Flo //explore walls
    if(true){
        if(!exploreWalls(start, plan)){
            return false;
        }
    }
    else {
        // search for frontiers
        if(findFrontiers(goals)){
            ROS_INFO("[wall_planner] exploration: found %d frontiers!", goals.size());
        }
        else {
            ROS_INFO("[wall_planner] exploration: no frontiers have been found! starting inner-exploration");
            return doInnerExploration(start,plan);
        }
        // make plan
        if(!buildExplorationTrans(start,goals,true)){
            return false;
        }


        if(!getTrajectory(start,goals,plan)){
            ROS_INFO("[wall_planner] exploration: could not plan to frontier, starting inner-exploration");
            return doInnerExploration(start,plan);
        }

    }


    ROS_INFO("[wall_planner] exploration: wall-plan has been found! plansize: %d", plan.size());
    return true;
}

bool HectorWallPlanner::doInnerExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan){
    ROS_INFO("[wall_planner] inner-exploration: starting exploration");

    // setup maps and goals
    resetMaps();
    clearFrontiers();
    plan.clear();
    costmap_ros->getCostmapCopy(costmap);
    occupancyGrid = costmap.getCharMap();
    std::vector<geometry_msgs::PoseStamped> goals;


    // create obstacle tranform
    buildObstacleTrans();

    // search for frontiers
    if(findInnerFrontier(goals)){
        ROS_INFO("[wall_planner] inner-exploration: found %d inner-frontiers!", goals.size());
    } else {
        ROS_WARN("[wall_planner] inner-exploration: no inner-frontiers have been found! exploration failed!");
        return false;
    }

    // make plan
    if(!buildExplorationTrans(start,goals,true)){
        return false;
    }
    if(!getTrajectory(start,goals,plan)){
        ROS_WARN("[wall_planner] inner-exploration: could not plan to inner-frontier. exploration failed!");
        return false;
    }

    // cutoff last points of plan due to sbpl error when planning close to walls

    int plansize = plan.size() - 5;
      if(plansize > 0 ){
          plan.resize(plansize);
    }

    // update previous goal
    if(!plan.empty()){
        geometry_msgs::PoseStamped thisgoal = plan.back();
        unsigned int mx,my;
        costmap.worldToMap(thisgoal.pose.position.x,thisgoal.pose.position.y,mx,my);
        previous_goal = costmap.getIndex(mx,my);
    }

    ROS_INFO("[wall_planner] inner-exploration: plan to an inner-frontier has been found! plansize: %d", plan.size());
    return true;
}

bool HectorWallPlanner::doAlternativeExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &oldplan){
    ROS_INFO("[wall_planner] alternative exploration: starting alternative exploration");

    // setup maps and goals
    resetMaps();
    plan.clear();
    costmap_ros->getCostmapCopy(costmap);
    occupancyGrid = costmap.getCharMap();
    std::vector<geometry_msgs::PoseStamped> goals;

    std::vector<geometry_msgs::PoseStamped> old_frontier;
    old_frontier.push_back(oldplan.back());

    // create obstacle tranform
    buildObstacleTrans();

    // search for frontiers
    if(findFrontiers(goals,old_frontier)){
        ROS_INFO("[wall_planner] alternative exploration: found %d frontiers!", goals.size());
    } else {
        ROS_WARN("[wall_planner] alternative exploration: no frontiers have been found!");
        return false;
    }

    // make plan
    if(!buildExplorationTrans(start,goals,true)){
        return false;
    }
    if(!getTrajectory(start,goals,plan)){
        return false;
    }

    geometry_msgs::PoseStamped thisgoal = plan.back();
    unsigned int mx,my;
    costmap.worldToMap(thisgoal.pose.position.x,thisgoal.pose.position.y,mx,my);
    previous_goal = costmap.getIndex(mx,my);

    ROS_INFO("[wall_planner] alternative exploration: plan to a frontier has been found! plansize: %d ", plan.size());
    return true;
}


bool HectorWallPlanner::buildExplorationTrans(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, bool useAnglePenalty){

    ROS_DEBUG("[wall_planner] buildExplorationTrans");

    std::queue<int> myqueue;

    // initialize goals
    for(unsigned int i = 0; i < goals.size(); ++i){
        // setup goal positions
        unsigned int mx,my;

        if(!costmap.worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my)){
            ROS_WARN("[wall_planner] The goal coordinates are outside the costmap!");
            return false;
        }

        int goal_point = costmap.getIndex(mx,my);

        if(!isFree(goal_point)){
            ROS_WARN("[wall_planner] The goal coordinates are occupied! (obstacle, inflated obstacle or unknown)");
            return false;
        }

        unsigned int init_cost = 0;
        if(useAnglePenalty){
            init_cost = angleDanger(angleDifference(start,goals[i])) * getDistanceWeight(start,goals[i]);
        }

        explorationTrans[goal_point] = init_cost;


        // do not punish previous frontiers (oszillation)
        if(isValid(previous_goal)){
            if(isSameFrontier(goal_point, previous_goal)){
                ROS_DEBUG("[wall_planner] same frontier: init with 0");
                explorationTrans[goal_point] = 0;
            }
        }
        ROS_DEBUG("[wall_planner] Goal init cost: %d, point: %d", explorationTrans[goal_point], goal_point);
        isGoal[goal_point] = true;
        myqueue.push(goal_point);


    }

    // exploration transform algorithm
    while(myqueue.size()){
        int point = myqueue.front();
        myqueue.pop();

        unsigned int point_newvalue = explorationTrans[point];
        unsigned int minimum = explorationTrans[point];

        int straightPoints[4];
        getStraightPoints(point,straightPoints);
        int diagonalPoints[4];
        getDiagonalPoints(point,diagonalPoints);


        // calculate the minimum exploration value of all adjacent cells
        if(!isGoal[point]){

            for(int i = 0; i < 4; ++i){
                if(isFree(straightPoints[i]) && (explorationTrans[straightPoints[i]] + STRAIGHT_COST + cellDanger(point)) < minimum){
                    minimum = explorationTrans[straightPoints[i]] + STRAIGHT_COST + cellDanger(point);
                }
                if(isFree(diagonalPoints[i]) && (explorationTrans[diagonalPoints[i]] + DIAGONAL_COST + cellDanger(point)) < minimum){
                    minimum = explorationTrans[diagonalPoints[i]] + DIAGONAL_COST + cellDanger(point);
                }
            }
            point_newvalue = minimum;
        }

        // if explorationTrans of the point changes, add all adjacent cells (theirs might change too)
        if((point_newvalue < explorationTrans[point]) || (isGoal[point])){
            explorationTrans[point] = point_newvalue;

            for(int i = 0; i < 4; ++i){
                if(isFree(straightPoints[i]) && !isGoal[straightPoints[i]]){
                    myqueue.push(straightPoints[i]);
                }
                if(isFree(diagonalPoints[i]) && !isGoal[diagonalPoints[i]]){
                    myqueue.push(diagonalPoints[i]);
                }
            }
        }
    }

    ROS_DEBUG("[wall_planner] END: buildExplorationTrans");
    return true;

}

bool HectorWallPlanner::buildObstacleTrans(){
    ROS_DEBUG("[wall_planner] buildObstacleTrans");
    std::queue<int> myqueue;

    // init obstacles
    for(unsigned int i=0; i < mapPoints; ++i){
        if(occupancyGrid[i] == costmap_2d::LETHAL_OBSTACLE){
            myqueue.push(i);
            obstacleTrans[i] = 0;
        } else if(useInflatedObs){
            if(occupancyGrid[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                 myqueue.push(i);
                 obstacleTrans[i] = 0;
            }
        }
    }


    // obstacle transform algorithm
    while(myqueue.size()){
        int point = myqueue.front();
        myqueue.pop();

        unsigned int minimum = obstacleTrans[point];

        int straightPoints[4];
        getStraightPoints(point,straightPoints);
        int diagonalPoints[4];
        getDiagonalPoints(point,diagonalPoints);

        if(obstacleTrans[point] != 0){

            // check all 8 directions
            for(int i = 0; i < 4; ++i){
                if(isValid(straightPoints[i]) && ((obstacleTrans[straightPoints[i]] + STRAIGHT_COST) < minimum)){
                    minimum = obstacleTrans[straightPoints[i]] + STRAIGHT_COST;
                }
                if(isValid(diagonalPoints[i]) && ((obstacleTrans[diagonalPoints[i]] + DIAGONAL_COST) < minimum)){
                    minimum = obstacleTrans[diagonalPoints[i]] + DIAGONAL_COST;
                }
            }
        }

        // if obstacleTrans of the point changes, add all adjacent cells (theirs might change too)
        if((minimum < obstacleTrans[point]) || (obstacleTrans[point] == 0)){
            obstacleTrans[point] = minimum;

            for(int i = 0; i < 4; ++i){
                if(isFree(straightPoints[i])){
                    myqueue.push(straightPoints[i]);
                }
                if(isFree(diagonalPoints[i])){
                    myqueue.push(diagonalPoints[i]);
                }
            }
        }
    }
    ROS_DEBUG("[wall_planner] END: buildObstacleTrans");
    return true;
}



bool HectorWallPlanner::getTrajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, std::vector<geometry_msgs::PoseStamped> &plan){

    ROS_DEBUG("[wall_planner] getTrajectory");

    // setup start positions
    unsigned int mx,my;

    if(!costmap.worldToMap(start.pose.position.x,start.pose.position.y,mx,my)){
        ROS_WARN("[wall_planner] The start coordinates are outside the costmap!");
        return false;
    }

    int currentPoint = costmap.getIndex(mx,my);
    int nextPoint = currentPoint;
    int maxDelta = 0;

    while(!isGoal[currentPoint]){
        int thisDelta;
        int adjacentPoints[8];
        getAdjacentPoints(currentPoint,adjacentPoints);

        for(int i = 0; i < 8; ++i){
            if(isFree(adjacentPoints[i])){
                thisDelta = explorationTrans[currentPoint] - explorationTrans[adjacentPoints[i]];
                if(thisDelta > maxDelta){
                    maxDelta = thisDelta;
                    nextPoint = adjacentPoints[i];
                }
            }
        }


        if(maxDelta == 0){
            ROS_DEBUG("[wall_planner] No path to the goal could be found by following gradient!");
            return false;
        }

        // make trajectory point
        geometry_msgs::PoseStamped trajPoint;
        unsigned int sx,sy,gx,gy;
        costmap.indexToCells((unsigned int)currentPoint,sx,sy);
        costmap.indexToCells((unsigned int)nextPoint,gx,gy);
        double wx,wy;
        costmap.mapToWorld(sx,sy,wx,wy);
        std::string global_frame = costmap_ros->getGlobalFrameID();
        trajPoint.header.frame_id = global_frame;
        trajPoint.pose.position.x = wx;
        trajPoint.pose.position.y = wy;
        trajPoint.pose.position.z = 0.0;

        // assign orientation
        int dx = gx-sx;
        int dy = gy-sy;
        double yaw_path = std::atan2(dy,dx);
        trajPoint.pose.orientation.x = 0.0;
        trajPoint.pose.orientation.y = 0.0;
        trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
        trajPoint.pose.orientation.w = cos(yaw_path*0.5f);

        plan.push_back(trajPoint);

        currentPoint = nextPoint;
        maxDelta = 0;
    }

    ROS_DEBUG("[wall_planner] END: getTrajectory. Plansize %d", plan.size());
    return !plan.empty();
}

float HectorWallPlanner::angleDifferenceWall(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal){
    // setup start positions
    unsigned int mxs,mys;
    costmap.worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);

    unsigned int gx,gy;
    costmap.worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);

    int goal_proj_x = gx-mxs;
    int goal_proj_y = gy-mys;

    float start_angle = tf::getYaw(start.pose.orientation);
    float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

    float both_angle = 0;
    if(start_angle > goal_angle){
        both_angle = start_angle - goal_angle;
    } else {
        both_angle = goal_angle - start_angle;
    }

    return both_angle;
}



bool HectorWallPlanner::exploreWalls(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan){
	ROS_DEBUG("[wall_planner] exploreWalls");
	unsigned int mx,my;
	if(!costmap.worldToMap(start.pose.position.x, start.pose.position.y, mx, my)){
		ROS_WARN("[wall_planner] The start coordinates are outside the costmap!");
		return false;
	}
        int currentPoint=costmap.getIndex(mx, my);
        int nextPoint;
        int oldDirection = -1;
        int k=0;

        while(k<50){
            int adjacentPoints [8];
            getAdjacentPoints(currentPoint, adjacentPoints);
            int dirPoints [5];            

            int minDelta=INT_MAX;
            int maxDelta=0;
            int thisDelta;
            float minAngle=3.1415; //Rad -> 180°

            geometry_msgs::PoseStamped trajPoint;
            unsigned int gx,gy;

            if(oldDirection==-1){
                // finde Startausrichtung.
                for ( int i=0; i<8; i++){
                    costmap.indexToCells((unsigned int)adjacentPoints[i],gx,gy);
                    double wx,wy;
                    costmap.mapToWorld(gx,gy,wx,wy);
                    std::string global_frame = costmap_ros->getGlobalFrameID();
                    trajPoint.header.frame_id = global_frame;
                    trajPoint.pose.position.x = wx;
                    trajPoint.pose.position.y = wy;
                    trajPoint.pose.position.z = 0.0;
                    float yaw = angleDifferenceWall(start, trajPoint);
                    if(yaw < minAngle){
                        minAngle=yaw;
                        oldDirection=i;
                    }
                }
            }

            //Finde weg entlang der Wand

           if (oldDirection == 0){
                dirPoints[0]=oldDirection+4; //Diagonal Rechts-Vorne
                dirPoints[1]=oldDirection;   //Vorne
                dirPoints[2]=oldDirection+7; //Diagonal Links-Vorne
            }
            else if (oldDirection < 3){
                dirPoints[0]=oldDirection+4;
                dirPoints[1]=oldDirection;
                dirPoints[2]=oldDirection+3;
            }
            else if (oldDirection == 3){
                dirPoints[0]=oldDirection+4;
                dirPoints[1]=oldDirection;
                dirPoints[2]=oldDirection+3;
            }
            else if (oldDirection == 4){
                dirPoints[0]=oldDirection-3;
                dirPoints[1]=oldDirection;
                dirPoints[2]=oldDirection-4;
            }
            else if (oldDirection < 7){
                dirPoints[0]=oldDirection-3;
                dirPoints[1]=oldDirection;
                dirPoints[2]=oldDirection-4;
            }
            else if (oldDirection == 7){
                dirPoints[0]=oldDirection-7;
                dirPoints[1]=oldDirection;
                dirPoints[2]=oldDirection-4;
            }

           if(startExploreWall == -1){ //startExploreWall: -1 Regel finden, 0 RHR, 1 LHR
               if(dirPoints[0] <= dirPoints[2]){
                   //RHR -> hrechts -> vorne -> hlinks -> rechts -> links
                   startExploreWall = 0;
                   ROS_INFO("RHR");
               }
               else {
                   //LHR -> hlinks -> vorne -> hrechts -> links -> rechts
                   startExploreWall = 1;
                   ROS_INFO("LHR");
               }
           }

           //vertausche links und rechts falls LHR
           if(startExploreWall == 1){
               int temp=dirPoints[0];
               dirPoints[0]=dirPoints[2];
               dirPoints[2]=temp;
           }

           int t=0;
           for(int i=0; i<3; i++){
               thisDelta= obstacleTrans[adjacentPoints[dirPoints[i]]];
               //Block 1 + 3
               if(thisDelta >= minObstacleDist){
                   if(obstacleTrans[currentPoint] >= minObstacleDist){
                       if(abs(thisDelta - minObstacleDist) < minDelta){
                           minDelta = abs(thisDelta - minObstacleDist);
                           nextPoint = adjacentPoints[dirPoints[i]];
                           oldDirection = dirPoints[i];
                       }
                   }
                   if(obstacleTrans[currentPoint] < minObstacleDist){
                       if(thisDelta > maxDelta){
                           maxDelta = thisDelta;
                           nextPoint = adjacentPoints[dirPoints[i]];
                           oldDirection = dirPoints[i];
                       }
                   }
               }
               else {
                   if(thisDelta < obstacleTrans[currentPoint]){
                       t++;
                   }
                   if(thisDelta > maxDelta){
                       maxDelta = thisDelta;
                       nextPoint = adjacentPoints[dirPoints[i]];
                       oldDirection = dirPoints[i];

                   }
               }
           }

           if(t==3 && abs(obstacleTrans[adjacentPoints[dirPoints[0]]] - obstacleTrans[adjacentPoints[dirPoints[1]]]) < STRAIGHT_COST
                   && abs(obstacleTrans[adjacentPoints[dirPoints[0]]] - obstacleTrans[adjacentPoints[dirPoints[2]]]) < STRAIGHT_COST
                   && abs(obstacleTrans[adjacentPoints[dirPoints[1]]] - obstacleTrans[adjacentPoints[dirPoints[2]]]) < STRAIGHT_COST){
               nextPoint=adjacentPoints[dirPoints[2]];
               oldDirection=dirPoints[2];
           }

            //füge Punkt hinzu


            unsigned int sx,sy;
            costmap.indexToCells((unsigned int)currentPoint,sx,sy);
            costmap.indexToCells((unsigned int)nextPoint,gx,gy);
            double wx,wy;
            costmap.mapToWorld(sx,sy,wx,wy);
            std::string global_frame = costmap_ros->getGlobalFrameID();
            trajPoint.header.frame_id = global_frame;
            trajPoint.pose.position.x = wx;
            trajPoint.pose.position.y = wy;
            trajPoint.pose.position.z = 0.0;
            // assign orientation
            int dx = gx-sx;
            int dy = gy-sy;
            double yaw_path = std::atan2(dy,dx);
            trajPoint.pose.orientation.x = 0.0;
            trajPoint.pose.orientation.y = 0.0;
            trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
            trajPoint.pose.orientation.w = cos(yaw_path*0.5f);

            plan.push_back(trajPoint);

            currentPoint=nextPoint;
            k++;
        }
	ROS_DEBUG("[wall_planner] END: exploreWalls. Plansize %d", plan.size());
        return !plan.empty();
}



bool HectorWallPlanner::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers){
    std::vector<geometry_msgs::PoseStamped> empty_vec;
    return findFrontiers(frontiers,empty_vec);
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 */
bool HectorWallPlanner::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers, std::vector<geometry_msgs::PoseStamped> &noFrontiers){

    // get latest costmap
    costmap_ros->getCostmapCopy(costmap);
    occupancyGrid = costmap.getCharMap();
    clearFrontiers();

    // list of all frontiers in the occupancy grid
    std::vector<int> allFrontiers;

    // check for all cells in the occupancy grid whether or not they are frontier cells
    for(int i = 0; i < mapPoints; ++i){
        if(isFrontier(i)){
            allFrontiers.push_back(i);
        }
    }

    // value of the next blob
    int nextBlobValue = 1;
    std::list<int> usedBlobs;

    for(unsigned int i = 0; i < allFrontiers.size(); ++i){

        // get all adjacent blobs to the current frontier point
        int currentPoint = allFrontiers[i];
        int adjacentPoints[8];
        getAdjacentPoints(currentPoint,adjacentPoints);

        std::list<int> blobs;

        for(int j = 0; j < 8; j++){
            if(isValid(adjacentPoints[j]) && (frontierMap[adjacentPoints[j]] > 0)){
                blobs.push_back(frontierMap[adjacentPoints[j]]);
            }
        }
        blobs.unique();

        if(blobs.empty()){
            // create new blob
            frontierMap[currentPoint] = nextBlobValue;
            usedBlobs.push_back(nextBlobValue);
            nextBlobValue++;
        } else {
            // merge all found blobs
            int blobMergeVal = 0;

            for(std::list<int>::iterator adjBlob = blobs.begin(); adjBlob != blobs.end(); ++adjBlob){
                if(adjBlob == blobs.begin()){
                    blobMergeVal = *adjBlob;
                    frontierMap[currentPoint] = blobMergeVal;

                } else {

                    for(unsigned int k = 0; k < allFrontiers.size(); k++){
                        if(frontierMap[allFrontiers[k]] == *adjBlob){
                            usedBlobs.remove(*adjBlob);
                            frontierMap[allFrontiers[k]] = blobMergeVal;
                        }
                    }
                }
            }
        }
    }

    int id = 1;

    // summarize every blob into a single point (maximum obstacleTrans value)
    for(std::list<int>::iterator currentBlob = usedBlobs.begin(); currentBlob != usedBlobs.end(); ++currentBlob){
        int current_frontier_size = 0;
        int max_obs_idx = 0;

        for(unsigned int i = 0; i < allFrontiers.size(); ++i){
            int point = allFrontiers[i];

            if(frontierMap[point] == *currentBlob){
                current_frontier_size++;
                if(obstacleTrans[point] > obstacleTrans[allFrontiers[max_obs_idx]]){
                    max_obs_idx = i;
                }
            }

        }

        if(current_frontier_size < minFrontierSize){
            continue;
        }

        int frontier_point = allFrontiers[max_obs_idx];
        unsigned int x,y;
        costmap.indexToCells(frontier_point,x,y);

        // check if frontier is valid (not to close to robot and not in noFrontiers vector
        bool frontier_is_valid = true;

        if(isFrontierReached(frontier_point)){
            frontier_is_valid = false;
        }
        for(int i = 0; i < noFrontiers.size(); ++i){
            geometry_msgs::PoseStamped noFrontier = noFrontiers[i];
            unsigned int mx,my;
            costmap.worldToMap(noFrontier.pose.position.x,noFrontier.pose.position.y,mx,my);
            int no_frontier_point = costmap.getIndex(x,y);
            if(isSameFrontier(frontier_point,no_frontier_point)){
                frontier_is_valid = false;
            }
        }

        if(frontier_is_valid){
            geometry_msgs::PoseStamped finalFrontier;
            double wx,wy;
            costmap.mapToWorld(x,y,wx,wy);
            std::string global_frame = costmap_ros->getGlobalFrameID();
            finalFrontier.header.frame_id = global_frame;
            finalFrontier.pose.position.x = wx;
            finalFrontier.pose.position.y = wy;
            finalFrontier.pose.position.z = 0.0;

            double yaw = getYawToUnknown(costmap.getIndex(x,y));
            finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            frontiers.push_back(finalFrontier);

            // visualization (export to method?)
            if(visualization_pub_.getNumSubscribers() > 0){
              visualization_msgs::Marker marker;
              marker.header.frame_id = "map";
              marker.header.stamp = ros::Time();
              marker.ns = "hector_wall_planner";
              marker.id = id++;
              marker.type = visualization_msgs::Marker::ARROW;
              marker.action = visualization_msgs::Marker::ADD;
              marker.pose.position.x = wx;
              marker.pose.position.y = wy;
              marker.pose.position.z = 0.0;
              marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
              marker.scale.x = 0.5;
              marker.scale.y = 0.5;
              marker.scale.z = 0.5;
              marker.color.a = 1.0;
              marker.color.r = 1.0;
              marker.color.g = 0.0;
              marker.color.b = 0.0;
              marker.lifetime = ros::Duration(5,0);
              visualization_pub_.publish(marker);
            }
        }
    }
    return !frontiers.empty();
}

bool HectorWallPlanner::findInnerFrontier(std::vector<geometry_msgs::PoseStamped> &innerFrontier){
    clearFrontiers();

    // get the trajectory as seeds for the exploration transform
    hector_nav_msgs::GetRobotTrajectory srv_path;
    if (path_service_client_.call(srv_path)){

      std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);
      std::vector<geometry_msgs::PoseStamped> goals;
      size_t size = traj_vector.size();
      ROS_DEBUG("[wall_planner] size of trajectory vector %d", size);

      if(size > 0){
          geometry_msgs::PoseStamped lastPose = traj_vector[0];
          goals.push_back(lastPose);
          for(int i = 1; i < size; ++i){
              geometry_msgs::PoseStamped pose = traj_vector[i];
              unsigned int x,y;
              costmap.worldToMap(pose.pose.position.x,pose.pose.position.y,x,y);
              unsigned m_point = costmap.getIndex(x,y);

              double dx = lastPose.pose.position.x - pose.pose.position.x;
              double dy = lastPose.pose.position.y - pose.pose.position.y;

              // extract points with 0.5m distance (if free)
              if(isFree(m_point)){
                  if((dx*dx) + (dy*dy) > (0.5*0.5)){
                      goals.push_back(pose);
                      lastPose = pose;
                  }
              }
          }


          ROS_DEBUG("[wall_planner] pushed %d goals (trajectory) for inner frontier-search", goals.size());

          // make exploration transform
          tf::Stamped<tf::Pose> robotPose;
          if(!costmap_ros->getRobotPose(robotPose)){
            ROS_WARN("[hector_wall_planner]: Failed to get RobotPose");
          }
          geometry_msgs::PoseStamped robotPoseMsg;
          tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

          buildExplorationTrans(robotPoseMsg, goals, false);

          unsigned int x,y;
          costmap.worldToMap(robotPoseMsg.pose.position.x,robotPoseMsg.pose.position.y,x,y);


          // get point with maximal distance to trajectory
          int max_expl_point = costmap.getIndex(x,y);
          for(unsigned int i = 0; i < mapPoints; ++i){
              if(isFree(i)){
                  if(explorationTrans[i] < INT_MAX){
                        if(explorationTrans[i] > explorationTrans[max_expl_point]){
                          if(!isFrontierReached(i)){
                              max_expl_point = i;
                          }
                        }
                  }
              }
          }

          // reset exploration transform
          for(unsigned int i = 0; i < mapPoints; ++i){
              explorationTrans[i] = INT_MAX;
              isGoal[i] = false;
          }

          geometry_msgs::PoseStamped finalFrontier;
          unsigned int fx,fy;
          double wfx,wfy;
          costmap.indexToCells(max_expl_point,fx,fy);
          costmap.mapToWorld(fx,fy,wfx,wfy);
          std::string global_frame = costmap_ros->getGlobalFrameID();
          finalFrontier.header.frame_id = global_frame;
          finalFrontier.pose.position.x = wfx;
          finalFrontier.pose.position.y = wfy;
          finalFrontier.pose.position.z = 0.0;

          // assign orientation
          int dx = fx-x;
          int dy = fy-y;
          double yaw_path = std::atan2(dy,dx);
          finalFrontier.pose.orientation.x = 0.0;
          finalFrontier.pose.orientation.y = 0.0;
          finalFrontier.pose.orientation.z = sin(yaw_path*0.5f);
          finalFrontier.pose.orientation.w = cos(yaw_path*0.5f);

          innerFrontier.push_back(finalFrontier);

          if(visualization_pub_.getNumSubscribers() > 0){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "hector_wall_planner";
            marker.id = 100;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = wfx;
            marker.pose.position.y = wfy;
            marker.pose.position.z = 0.0;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_path);
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.lifetime = ros::Duration(5,0);
            visualization_pub_.publish(marker);
          }
          return !innerFrontier.empty();
      }
    }
    return false;
}

/*
 * checks if a given point is a frontier cell. a frontier cell is a cell in the occupancy grid,
 * that seperates known from unknown space. therefor the cell has to be free but at least three
 * of its neighbours need to be unknown
 */
bool HectorWallPlanner::isFrontier(int point){
    if(isFree(point)){

        int adjacentPoints[8];
        getAdjacentPoints(point,adjacentPoints);

        for(int i = 0; i < 8; ++i){
            if(isValid(adjacentPoints[i])){
                if(occupancyGrid[adjacentPoints[i]] == costmap_2d::NO_INFORMATION){

                    int no_inf_count = 0;
                    int noInfPoints[8];
                    getAdjacentPoints(adjacentPoints[i],noInfPoints);
                    for(int j = 0; j < 8; j++){
                        if(occupancyGrid[noInfPoints[j]] == costmap_2d::NO_INFORMATION){
                            no_inf_count++;
                        }
                    }
                    if(no_inf_count > 2){
                        return true;
                    }

                }
            }
        }
    }

    return false;
}


void HectorWallPlanner::resetMaps(){

    for(unsigned int i = 0; i < mapPoints; ++i){
        explorationTrans[i] = INT_MAX;
        obstacleTrans[i] = INT_MAX;
        isGoal[i] = false;
    }
}

void HectorWallPlanner::clearFrontiers(){

    for(unsigned int i = 0; i < mapPoints; ++i){
        frontierMap[i] = 0;
    }
}

bool HectorWallPlanner::isValid(int point){

    if(point>=0){
        return true;
    }

    return false;
}

bool HectorWallPlanner::isFree(int point){

    if(isValid(point)){
        // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free
        if(useInflatedObs){
            if(occupancyGrid[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                return true;
            }
        } else {
            if(occupancyGrid[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                return true;
            }
        }

        if(planInUnknown){
            if(occupancyGrid[point] == costmap_2d::NO_INFORMATION){
                return true;
            }
        }
    }
    return false;
}

bool HectorWallPlanner::isFrontierReached(int point){

    tf::Stamped<tf::Pose> robotPose;
    if(!costmap_ros->getRobotPose(robotPose)){
      ROS_WARN("[hector_wall_planner]: Failed to get RobotPose");
    }
    geometry_msgs::PoseStamped robotPoseMsg;
    tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

    unsigned int fx,fy;
    double wfx,wfy;
    costmap.indexToCells(point,fx,fy);
    costmap.mapToWorld(fx,fy,wfx,wfy);


    double dx = robotPoseMsg.pose.position.x - wfx;
    double dy = robotPoseMsg.pose.position.y - wfy;

    if ( (dx*dx) + (dy*dy) < (dist_for_goal_reached*dist_for_goal_reached)) {
      ROS_DEBUG("[hector_wall_planner]: frontier is within the squared range of: %f", dist_for_goal_reached);
      return true;
    }
    return false;

}

bool HectorWallPlanner::isSameFrontier(int frontier_point1, int frontier_point2){
    unsigned int fx1,fy1;
    unsigned int fx2,fy2;
    double wfx1,wfy1;
    double wfx2,wfy2;
    costmap.indexToCells(frontier_point1,fx1,fy1);
    costmap.indexToCells(frontier_point2,fx2,fy2);
    costmap.mapToWorld(fx1,fy1,wfx1,wfy1);
    costmap.mapToWorld(fx2,fy2,wfx2,wfy2);

    double dx = wfx1 - wfx2;
    double dy = wfy1 - wfy2;

    if((dx*dx) + (dy*dy) < (same_frontier_dist*same_frontier_dist)){
        return true;
    }
    return false;

}

unsigned int HectorWallPlanner::cellDanger(int point){
    unsigned int danger = 0;
    if((int)obstacleTrans[point] <= minObstacleDist){
        danger = alpha * std::pow(minObstacleDist - obstacleTrans[point],2);
    }
    return danger;
}


float HectorWallPlanner::angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal){
    // setup start positions
    unsigned int mxs,mys;
    costmap.worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);

    unsigned int gx,gy;
    costmap.worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);

    int goal_proj_x = gx-mxs;
    int goal_proj_y = gy-mys;

    float start_angle = tf::getYaw(start.pose.orientation);
    float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

    float both_angle = 0;
    if(start_angle > goal_angle){
        both_angle = start_angle - goal_angle;
    } else {
        both_angle = goal_angle - start_angle;
    }

    if(both_angle > M_PI){
        both_angle = (M_PI - std::abs(start_angle)) + (M_PI - std::abs(goal_angle));
    }

    return both_angle;
}

double HectorWallPlanner::getYawToUnknown(int point){
    int adjacentPoints[8];
    getAdjacentPoints(point,adjacentPoints);

    int max_obs_idx = 0;

    for(int i = 0; i < 8; ++i){
        if(isValid(adjacentPoints[i])){
            if(occupancyGrid[adjacentPoints[i]] == costmap_2d::NO_INFORMATION){
                if(obstacleTrans[adjacentPoints[i]] > obstacleTrans[adjacentPoints[max_obs_idx]]){
                    max_obs_idx = i;
                }
            }
        }
    }

    int orientationPoint = adjacentPoints[max_obs_idx];
    unsigned int sx,sy,gx,gy;
    costmap.indexToCells((unsigned int)point,sx,sy);
    costmap.indexToCells((unsigned int)orientationPoint,gx,gy);
    int x = gx-sx;
    int y = gy-sy;
    double yaw = std::atan2(y,x);

    return yaw;

}

unsigned int HectorWallPlanner::angleDanger(float angle){
    float angle_fraction = std::pow(angle,3);///M_PI;
    unsigned int result = static_cast<unsigned int>(goal_angle_penalty * angle_fraction);
    return result;
}

float HectorWallPlanner::getDistanceWeight(const geometry_msgs::PoseStamped &point1, const geometry_msgs::PoseStamped &point2){
    float distance = std::sqrt(std::pow(point1.pose.position.x - point2.pose.position.x,2) + std::pow(point1.pose.position.y - point2.pose.position.y,2));
    if(distance < 0.5){
        return 5.0;
    } else {
        return 1;
    }
}

/*
 These functions calculate the index of an adjacent point (left,upleft,up,upright,right,downright,down,downleft) to the
 given point. If there is no such point (meaning the point would cause the index to be out of bounds), -1 is returned.
 */
inline void HectorWallPlanner::getStraightPoints(int point, int points[]){

    points[0] = left(point);
    points[1] = up(point);
    points[2] = right(point);
    points[3] = down(point);

}

inline void HectorWallPlanner::getDiagonalPoints(int point, int points[]){

    points[0] = upleft(point);
    points[1] = upright(point);
    points[2] = downright(point);
    points[3] = downleft(point);

}

inline void HectorWallPlanner::getAdjacentPoints(int point, int points[]){

    points[0] = left(point);
    points[1] = up(point);
    points[2] = right(point);
    points[3] = down(point);
    points[4] = upleft(point);
    points[5] = upright(point);
    points[6] = downright(point);
    points[7] = downleft(point);

}

inline int HectorWallPlanner::left(int point){
    // only go left if no index error and if current point is not already on the left boundary
    if((point % mapWidth != 0)){
        return point-1;
    }
    return -1;
}
inline int HectorWallPlanner::upleft(int point){
    if((point % mapWidth != 0) && (point >= (int)mapWidth)){
        return point-mapWidth-1;
    }
    return -1;

}
inline int HectorWallPlanner::up(int point){
    if(point >= (int)mapWidth){
        return point-mapWidth;
    }
    return -1;
}
inline int HectorWallPlanner::upright(int point){
    if((point >= (int)mapWidth) && ((point + 1) % (int)mapWidth != 0)){
        return point-mapWidth+1;
    }
    return -1;
}
inline int HectorWallPlanner::right(int point){
    if((point + 1) % mapWidth != 0){
        return point+1;
    }
    return -1;

}
inline int HectorWallPlanner::downright(int point){
    if(((point + 1) % mapWidth != 0) && ((point/mapWidth) < (mapWidth-1))){
        return point+mapWidth+1;
    }
    return -1;

}
inline int HectorWallPlanner::down(int point){
    if((point/mapWidth) < (mapWidth-1)){
        return point+mapWidth;
    }
    return -1;

}
inline int HectorWallPlanner::downleft(int point){
    if(((point/mapWidth) < (mapWidth-1)) && (point % mapWidth != 0)){
        return point+mapWidth-1;
    }
    return -1;

}

//        // visualization (export to another method?)
//        visualization_msgs::Marker marker;
//        marker.header.frame_id = "map";
//        marker.header.stamp = ros::Time();
//        marker.ns = "hector_wall_planner";
//        marker.id = i + 500;
//        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//        marker.action = visualization_msgs::Marker::ADD;
//        marker.pose.position = goals[i].pose.position;
//        marker.scale.x = 0.2;
//        marker.scale.y = 0.2;
//        marker.scale.z = 0.2;
//        marker.color.a = 1.0;
//        marker.color.r = 0.0;
//        marker.color.g = 0.0;
//        marker.color.b = 1.0;
//        marker.lifetime = ros::Duration(5,0);
//        marker.text = boost::lexical_cast<std::string>((int)init_cost) + " - " + boost::lexical_cast<std::string>(getDistanceWeight(start,goals[i]));
//        visualization_pub_.publish(marker);

//void HectorWallPlanner::saveMaps(std::string path){

//    char costmapPath[1000];
//    sprintf(costmapPath,"%s.map",path.data());
//    char explorationPath[1000];
//    sprintf(explorationPath,"%s.expl",path.data());
//    char obstaclePath[1000];
//    sprintf(obstaclePath,"%s.obs",path.data());
//    char frontierPath[1000];
//    sprintf(frontierPath,"%s.front",path.data());


//    costmap.saveMap(costmapPath);
//    FILE *fp_expl = fopen(explorationPath,"w");
//    FILE *fp_obs = fopen(obstaclePath,"w");
//    FILE *fp_front = fopen(frontierPath,"w");

//    if (!fp_expl || !fp_obs || !fp_front)
//    {
//        ROS_WARN("[wall_planner] Cannot save maps");
//        return;
//    }

//    for(unsigned int y = 0; y < mapHeight; ++y){
//        for(unsigned int x = 0;x < mapWidth; ++x){
//            unsigned int expl = explorationTrans[costmap.getIndex(x,y)];
//            unsigned int obs = obstacleTrans[costmap.getIndex(x,y)];
//            int blobVal = frontierMap[costmap.getIndex(x,y)];
//            fprintf(fp_front,"%d\t", blobVal);
//            fprintf(fp_expl,"%d\t", expl);
//            fprintf(fp_obs, "%d\t", obs);
//        }
//        fprintf(fp_expl,"\n");
//        fprintf(fp_obs,"\n");
//        fprintf(fp_front,"\n");
//    }

//    fclose(fp_expl);
//    fclose(fp_obs);
//    fclose(fp_front);
//    ROS_INFO("[wall_planner] Maps have been saved!");
//    return;

//}

//    // add last point to path (goal point)
//    for(unsigned int i = 0; i < goals.size(); ++i){
//        unsigned int mx,my;
//        costmap.worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my);

//        if(currentPoint == (int)costmap.getIndex(mx,my)){
//            plan.push_back(goals[i]);
//            previous_goal = currentPoint;
//        }

//    }

