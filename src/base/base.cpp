#include "base.h"

namespace switch_controller{

/**
 * @brief Construct a new Robot 7:: Robot 7 object
 * 
 */
Robot7::Robot7(){
  has_data = false;
  stop_ctrl = false;
}

/**
 * @brief Destroy the Robot 7:: Robot 7 object
 * 
 */
Robot7::~Robot7(){
  stop_ctrl = true;
}

RobotState7 Robot7::GetState(){
  if(has_data){
    has_data = false;
  }
  return robot_state;
}

void Robot7::StopControl(){
  stop_ctrl = true;
}

bool Robot7::IsDataReceived(){
  return has_data;
}
/**********************************************
 * Belows functions should be implemented.
***********************************************/
/*
1. state update functions
2. robot model acquisition functions
3. control loop functions
*/

/**
 * @brief This function inputs the state of the real robot
 * , and updates robot_state(RobotState7) and/or saves a current real robot state temporarily.
 */
void Robot7::UpdateState(){
  if (m.try_lock()){
    has_data = true;
    // update robot_states
  }
}

/**
 * @brief Get the current coriolis vector of the robot
 * 
 * @return RobotTorque7 
 */
RobotTorque7 Robot7::GetCoriolisVector(){
  RobotTorque7 coriolis;
  // get current coriolis vector of the robot.
  return coriolis;
}

/**
 * @brief Get the current gravity vector of the robot
 * 
 * @return RobotTorque7 
 */
RobotTorque7 Robot7::GetGravityVector(){
  RobotTorque7 gravity;
  // get current gravity vector of the robot.
  return gravity;
}

Jacobian7 Robot7::GetTaskJacobian(){
  Jacobian7 jacobian;
  return jacobian;
}



/**
 * @brief This function has an Idle loop of the switch controller.
 * 
 */
void Robot7::IdleControl(){
  std::cout << "nothing" << std::endl;
  return;
}

/**
 * @brief This function has an Torque control loop of the switch controller.
 * 
 */
void Robot7::TorqueControl( \
  InitFn init_fn, TorqueCtrlLoopFn loop_fn, StopFn stop_fn \
){
  // // Example
  // RobotTorque7 torque;
  // bool loop_trigger;
  // RobotState7 state;

  // while(true){
  //   // ex. loop_trigger = robotAPI.loop_trigger()
  //   if(loop_trigger){
  //     UpdateState(state);
  //     torque = ctrl_loop_fn(state);
  //   }

  //   // The loop code should end if 'stop_ctrl' is true.
  //   if(stop_ctrl){
  //     break;
  //   }
  // }
}



/**
 * @brief Initialize a thread pool
 * 
 */
ThreadPool::ThreadPool()
  : num_threads(2)
  , stop_all(false)
{
  worker_threads_.reserve(num_threads);
  for (size_t i=0; i<num_threads; i++){
    worker_threads_.emplace_back([this](){this->WorkerThread();});
  }
}

/**
 * @brief Destroy the thread pool
 * 
 */
ThreadPool::~ThreadPool(){
  stop_all = true;
  cv_job_q.notify_all();
  for (auto& t : worker_threads_){
    t.join();
  }
}

/**
 * @brief Thread pool loop for changable controller
 * 
 */
void ThreadPool::WorkerThread(){
  while (true){
    std::unique_lock<std::mutex> lock(m_job_q);
    cv_job_q.wait(lock, [this](){ return !this->jobs.empty() || stop_all; });
    if (stop_all && this->jobs.empty()){
      return;
    }
    // pop first job
    std::function<void()> job = std::move(jobs.front());
    jobs.pop();
    lock.unlock();

    // do job
    job();
  }
}

/**
 * @brief Enqueue job(control thread)
 * 
 * @param job Function to execute
 */
void ThreadPool::EnqueueJob(std::function<void()> job) {
  if (stop_all) {
    throw std::runtime_error("Stop All Thread in this pool!");
  }
  {
    std::lock_guard<std::mutex> lock(m_job_q);
    jobs.push(std::move(job));
  }
  cv_job_q.notify_one();
}
}