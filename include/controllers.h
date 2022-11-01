#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "base.h"

namespace switch_controller{

class JointSpaceImpCtrl : protected Controller{
private:
  double joint_k;
  double joint_d;

public:
  std::mutex m;
  std::array<double, 7> q_d;
  std::array<double, 7> joint_k_array;
  std::array<double, 7> joint_d_array;

  JointSpaceImpCtrl(Robot7 *robot) 
    : Controller(robot)
    , joint_k(200.0)
    , joint_d(30.0)
  {
    SetJointK(joint_k);
    SetJointD(joint_d);
  };
  double GetJointK() {return joint_k;};
  double GetJointD() {return joint_d;};
  void SetJointK(double joint_k_cmd) {
    joint_k = joint_k_cmd;
    joint_k_array.fill(joint_k);
  };
  void SetJointD(double joint_d_cmd) {
    joint_d = joint_d_cmd;
    joint_d_array.fill(joint_d);
  };
  void init();
  RobotTorque7 loop(const RobotState7 &robot_state);
  void stop();
};

class TaskSpaceImpCtrl : protected Controller{
private:
  double k_trans;
  double k_rot;

public:
  std::mutex m;
  Eigen::Vector3d EE_pos_d;
  Eigen::Quaterniond EE_orn_d;
  Eigen::MatrixXd task_k_matrix;
  Eigen::MatrixXd task_d_matrix;

  TaskSpaceImpCtrl(Robot7 *robot) 
    : Controller(robot)
    , task_k_matrix(6,6)
    , task_d_matrix(6,6)
    , k_trans(250.0)
    , k_rot(10.0)
  {
    SetTaskImpMatrixTrans(k_trans);
    SetTaskImpMatrixRot(k_rot);
  };
  double GetTaskTransK() {return k_trans;};
  double GetTaskRotK() {return k_rot;};
  void SetTaskImpMatrixTrans(double k_trans);
  void SetTaskImpMatrixRot(double k_rot);
  //void SetTaskImpMatrix(double k_trans, double k_rot);
  void init();
  RobotTorque7 loop(const RobotState7 &robot_state);
  void stop();
};

}

#endif //CONTROLLERS_H