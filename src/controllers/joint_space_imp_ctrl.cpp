#include "controllers.h"

namespace switch_controller{

/*********************************
 * joint space impedance control
 * *******************************/

//init function
void JointSpaceImpCtrl::init(){
    RobotState7 state = robot->GetState(); //assume that robot state is read.
    q_d = state.q;
};

//torque control loop function
RobotTorque7 JointSpaceImpCtrl::loop(const RobotState7 &robot_state){
  RobotTorque7 tau_d;
  RobotState7 state = robot->GetState();
  RobotTorque7 coriolis = robot->GetCoriolisVector();
  for (size_t i = 0; i < 7; i++) {
    tau_d[i] =
        joint_k_array[i] * (q_d[i] - state.q[i]) - joint_d_array[i] * state.dq[i] + coriolis[i];
  }
  return tau_d;
};

//stop
void JointSpaceImpCtrl::stop(){
  RobotState7 state = robot->GetState();
  q_d = state.q;
}

}