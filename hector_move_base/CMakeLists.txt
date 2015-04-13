# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(hector_move_base)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS actionlib_msgs monstertruck_msgs move_base_msgs sensor_msgs nav_core hector_move_base_msgs hector_nav_core hector_nav_msgs_tools std_msgs hector_exploration_planner inverse_trajectory_recovery hector_nav_core_exploration_plugin sbpl_lattice_planner)

find_package(Boost REQUIRED COMPONENTS system)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

catkin_package(
    DEPENDS actionlib_msgs monstertruck_msgs move_base_msgs sensor_msgs nav_core hector_move_base_msgs hector_nav_core hector_nav_msgs_tools std_msgs hector_exploration_planner inverse_trajectory_recovery hector_nav_core_exploration_plugin sbpl_lattice_planner
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

# hector_move_base
add_executable(hector_move_base src/hector_move_base.cpp src/hector_move_base_state_machine.cpp)

target_link_libraries(hector_move_base ${Boost_LIBRARIES} ${catkin_LIBRARIES})
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
