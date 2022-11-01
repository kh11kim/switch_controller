#include "robots.h"

namespace switch_controller{

/**
 * @brief Construct a new Panda:: Panda object
 * 
 * @param ip 
 */
Panda::Panda(const std::string &ip) //"172.16.0.2"
  : panda(ip)
  , model(panda.loadModel())
{
  ;
}

/**
 * @brief Destroy the Panda:: Panda object
 * 
 */
Panda::~Panda(){
  ;
}


/**
 * @brief Belows functions should be implemented.
 * 
 */
//update panda_state_temp, robot_state
void Panda::UpdateState(franka::RobotState panda_state){
  has_data = true;
  panda_state_temp = panda_state;
  robot_state.q = panda_state.q;
  robot_state.dq = panda_state.dq;
  robot_state.theta = panda_state.theta;
  robot_state.dtheta = panda_state.dtheta;
  robot_state.tau = panda_state.tau_J;
  robot_state.tau_ext = panda_state.tau_ext_hat_filtered;
  robot_state.EE_pose = panda_state.O_T_EE;
  //std::cout << "q:" << robot_state.q << std::endl; //DEBUG
}

RobotTorque7 Panda::GetCoriolisVector(){
  return model.coriolis(panda_state_temp);
}

RobotTorque7 Panda::GetGravityVector(){
  return model.gravity(panda_state_temp);
}

Jacobian7 Panda::GetTaskJacobian(){
  Jacobian7 jacobian = model.zeroJacobian(franka::Frame::kEndEffector, panda_state_temp);
  return jacobian;
}
/**
 * @brief This function has an Idle loop of the switch controller.
 * 
 */
void Panda::IdleControl(){
  std::function<bool(const franka::RobotState&)> loop_fn_panda; 
  stop_ctrl = false;

  // define lambda function for state read loop
  loop_fn_panda = [&](const franka::RobotState& panda_state){
    if (m.try_lock()){
      UpdateState(panda_state);
      m.unlock();
    }
    return !stop_ctrl; //if stop_ctrl is true, break
  };
  // run loop
  panda.read(loop_fn_panda);
  return;
}

/**
 * @brief This function has an Torque control loop of the switch controller.
 * 
 */
void Panda::TorqueControl(InitFn init_fn, TorqueCtrlLoopFn loop_fn, StopFn stop_fn){
  TorqueCtrlLoopPanda loop_fn_panda; 
  RobotTorque7 torque;
  stop_ctrl = false;

  //ctrl-loop function for libfranka : make wrapper lambda function
  loop_fn_panda = [&](const franka::RobotState& panda_state, franka::Duration) -> franka::Torques{
    if (m.try_lock()){
      UpdateState(panda_state);
      torque = loop_fn(robot_state);
      // stop : if stop_ctrl flag turned 'true', this loop should break
      m.unlock();
    }
    if (stop_ctrl){
      return franka::MotionFinished(franka::Torques(torque));
    }
    return torque;
  };

  //procedure
  UpdateState(panda.readOnce()); //read state once for initialize
  init_fn();
  panda.control(loop_fn_panda); //run loop
  stop_fn();
}


}