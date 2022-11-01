#ifndef ROBOTS_H
#define ROBOTS_H

//frankalib
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>

#include "base.h"

namespace switch_controller{

/**********************************
 * Franka Emika Panda
 * *******************************/

typedef std::function<franka::Torques(const franka::RobotState&, franka::Duration)> TorqueCtrlLoopPanda;

class Panda : public Robot7{
protected:
  std::string ip;
  franka::Robot panda;
  franka::Model model;
  franka::RobotState panda_state_temp;

public:
  std::string joint_names[7] = {
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", 
    "panda_joint5", "panda_joint6", "panda_joint7"
  };
  void IdleControl(); //Idle loop
  void TorqueControl(InitFn, TorqueCtrlLoopFn, StopFn);
  void UpdateState(franka::RobotState franka_state);

  //API-model
  RobotTorque7 GetCoriolisVector();
  RobotTorque7 GetGravityVector();
  Jacobian7 GetTaskJacobian();

  Panda(const std::string &ip); //const std::string& ip
  ~Panda();
};

}


#endif //ROBOTS_H