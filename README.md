switch_controller
======================

`switch_controller` is a control framework that can change multiple controllers in one ROS node.

## Dependency
boost, eigen, libfranka

## Usage
1. Download and compile this module in src folder in the catkin workspace (+ source workspace).
2. rosrun switch_controller switch_controller_node
3. You can use GUI for changing configurations: rosrun rqt_reconfigure rqt_reconfigure
4. Also, you can use [dynamic_reconfigure client](http://wiki.ros.org/dynamic_reconfigure/Tutorials/UsingTheDynamicReconfigurePythonClient) to change settings in other nodes.
  

## Controllers/Topics
switch_controller is written based on the torque control of the current 7-axis robot. The implemented controller and the corresponding topics are as follows.
* Common:
  * `/joint_state` : joint information (sensor_msgs::JointState :  name, position, velocity, torque)
  * `/ee_pose` : End-effector pose (geometry_msgs::PoseStamped)
  * `ctrl_mode` (dynamic_reconfigure) : control mode (0:Stop, 1:Idle, 2:JointSpaceImpedance, 3:TaskSpaceImpedance)
* [Joint-space impedance control](https://github.com/kh11kim/switch_controller/blob/master/src/controllers/joint_space_imp_ctrl.cpp)
  * `/q_d` : desired joint angle (std_msgs::Float64MultiArray).
  * `joint_k` (dynamic_reconfigure) : joint stiffness
  * `joint_d` (dynamic_reconfigure) : joint damping
* [Task-space impedance control](https://github.com/kh11kim/switch_controller/blob/master/src/controllers/task_space_imp_ctrl.cpp)
  * `/ee_pose_d` : desired End-effector pose (geometry_msgs::PoseStamped)
  * `task_trans_k` (dynamic_reconfigure) : Task-space translational stiffness
  * `task_rot_k` (dynamic_reconfigure) : Task-space rotational stiffness
  
## Robot
switch_controller is currently based on the Franka Emika Panda. Kinova Gen3 will be added.


## Add your own controller
TBD

## Add your own robot
TBD
