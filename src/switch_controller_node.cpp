#include "switch_controller.h"
#include "robots.h"

//ROS messages
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_broadcaster.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "switch_controller/SwitchControllerConfig.h" //The file is in "devel/include"

namespace switch_controller{

class SwitchControllerNode : public SwitchController{
private:
  ros::NodeHandle n;
  ros::Publisher joint_state_pub;
  ros::Publisher ee_pose_pub;
  //ros::Publisher wrench_pub;

  ros::Subscriber q_d_sub;
  ros::Subscriber ee_pose_d_sub;

  dynamic_reconfigure::Server<switch_controller::SwitchControllerConfig> server;
  dynamic_reconfigure::Server<switch_controller::SwitchControllerConfig>::CallbackType f;
  tf2_ros::TransformBroadcaster br;

public:
  SwitchControllerNode(Robot7 *robot_ptr);
  // Communication
  void PublishJointState();
  void PublishEEPose();
  void Callback_q_d(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void Callback_ee_pose_d(const geometry_msgs::Pose::ConstPtr& msg);
  // API
  void SetControllerROSWrapper(int ctrl_mode);
  void DynConfigCallback(SwitchControllerConfig &config, uint32_t level);  
};

SwitchControllerNode::SwitchControllerNode(Robot7 *robot_ptr)
  :SwitchController(robot_ptr){
  //advertise topics
  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_state", 1);
  ee_pose_pub = n.advertise<geometry_msgs::PoseStamped>("ee_pose", 1);
  q_d_sub = n.subscribe("q_d", 1, &SwitchControllerNode::Callback_q_d, this);
  ee_pose_d_sub = n.subscribe("ee_pose_d", 1, &SwitchControllerNode::Callback_ee_pose_d, this);
  server.setCallback(boost::bind( &SwitchControllerNode::DynConfigCallback, this, _1, _2));

  SetControllerROSWrapper(CTRLMODE_IDLE); //Init
}

void SwitchControllerNode::DynConfigCallback(SwitchControllerConfig &config, uint32_t level){
  //ROS_INFO("Joint Impedance: k-%f, d-%f", config.joint_k_array, config.joint_d);
  //ROS_INFO("Task Impedance: trans_k-%f, rot_k-%f", config.task_trans_k, config.task_rot_k);
  if (config.ctrl_mode != GetCtrlMode()){
    SetControllerROSWrapper(config.ctrl_mode);
  }
  if (config.joint_k != ctrl_js_imp.GetJointK()){
    ROS_INFO("Joint Stiffness: %f -> %f", ctrl_js_imp.GetJointK(), config.joint_k);
    ctrl_js_imp.SetJointK(config.joint_k);
  }
  if (config.joint_d != ctrl_js_imp.GetJointD()){
    ROS_INFO("Joint Damping: %f -> %f", ctrl_js_imp.GetJointD(), config.joint_d);
    ctrl_js_imp.SetJointD(config.joint_d);
  }
  if (config.task_trans_k != ctrl_ts_imp.GetTaskTransK()){
    ROS_INFO("Task Stiffness (trans): %f -> %f", ctrl_ts_imp.GetTaskTransK(), config.task_trans_k);
    ctrl_ts_imp.SetTaskImpMatrixTrans(config.task_trans_k);
  }
  if (config.task_rot_k != ctrl_ts_imp.GetTaskRotK()){
    ROS_INFO("Task Stiffness (rot): %f -> %f", ctrl_ts_imp.GetTaskRotK(), config.task_rot_k);
    ctrl_ts_imp.SetTaskImpMatrixRot(config.task_rot_k);
  }
}

void SwitchControllerNode::Callback_q_d(const std_msgs::Float64MultiArray::ConstPtr& msg){
  std::copy(msg->data.begin(), msg->data.end(), ctrl_js_imp.q_d.data());
}

void SwitchControllerNode::Callback_ee_pose_d(const geometry_msgs::Pose::ConstPtr& msg){
  ctrl_ts_imp.EE_pos_d << msg->position.x, msg->position.y, msg->position.z;
  ctrl_ts_imp.EE_orn_d.x() = msg->orientation.x;
  ctrl_ts_imp.EE_orn_d.y() = msg->orientation.y;
  ctrl_ts_imp.EE_orn_d.z() = msg->orientation.z;
  ctrl_ts_imp.EE_orn_d.w() = msg->orientation.w;
}

void SwitchControllerNode::PublishJointState(){
  sensor_msgs::JointState msg;
  RobotState7 state = robot->GetState();

  msg.header.stamp = ros::Time::now();
  for (int i=0;i<7;i++){
    msg.name.push_back(robot->joint_names[i]);
    msg.position.push_back(state.q[i]);
    msg.velocity.push_back(state.dq[i]);
    msg.effort.push_back(state.tau[i]);
  }
  joint_state_pub.publish(msg);
}

void SwitchControllerNode::PublishEEPose(){
  //TODO: Publish TF?
  RobotState7 state = robot->GetState();

  geometry_msgs::PoseStamped msg;
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.EE_pose.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "panda_EE";
  msg.pose.position.x = position.x();
  msg.pose.position.y = position.y();
  msg.pose.position.z = position.z();
  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();
  msg.pose.orientation.w = orientation.w();
  ee_pose_pub.publish(msg);
}

void SwitchControllerNode::SetControllerROSWrapper(int ctrl_mode){
  switch(ctrl_mode){
    case switch_controller::CTRLMODE_STOP:
      ROS_INFO("Controller Stopped");
    break;
    case switch_controller::CTRLMODE_IDLE:
      ROS_INFO("Idle Mode");
    break;
    case switch_controller::CTRLMODE_JOINT_IMP:
      ROS_INFO("Joint-space Control Mode");
    break;
    case switch_controller::CTRLMODE_TASK_IMP:
      ROS_INFO("Task-space Control Mode");
    break;
  }
  SetController(ctrl_mode);
}

}

using namespace switch_controller;

int main(int argc, char **argv){
  ros::init(argc, argv, "switch_controller_node");
  
  ROS_INFO("Start switch_controller_node");

  const std::string ip = "172.16.0.2";
  Panda robot(ip);
  SwitchControllerNode n(&robot);
  ros::Rate rate(100);

  while(ros::ok()){
    if (robot.IsDataReceived()){
      n.PublishEEPose();
      n.PublishJointState();
    }

    if (n.IsStoppedByError()){
      ROS_INFO("Error: set to idle mode");
      n.SetControllerROSWrapper(CTRLMODE_IDLE);
      n.ClearError();
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
