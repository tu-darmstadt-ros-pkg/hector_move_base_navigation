# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(hector_exploration_planner_test)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp hector_exploration_planner)

catkin_package(
    DEPENDS roscpp hector_exploration_planner
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(exploration_planner_test_node src/exploration_planner_test_node.cpp)
add_executable(pose_with_cov_to_tf_node src/pose_with_cov_to_tf_node.cpp)
     
target_link_libraries(exploration_planner_test_node ${catkin_LIBRARIES})
target_link_libraries(pose_with_cov_to_tf_node ${catkin_LIBRARIES})


