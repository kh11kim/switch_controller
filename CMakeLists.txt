cmake_minimum_required(VERSION 3.0.2)
project(switch_controller)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -lpthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  tf2
  tf2_ros
)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(catkin REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SwitchController.cfg
)

catkin_package()

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)
link_libraries(Franka::Franka Threads::Threads)

set(
SRC_FILES 
src/base/base.cpp
src/controllers/joint_space_imp_ctrl.cpp
src/controllers/task_space_imp_ctrl.cpp
src/robots/franka_panda.cpp
src/switch_controller.cpp
src/switch_controller_node.cpp
)

add_executable(switch_controller_node ${SRC_FILES})
target_link_libraries(switch_controller_node ${catkin_LIBRARIES})
add_dependencies(switch_controller_node ${PROJECT_NAME}_gencfg) ##dynamic reconfig
