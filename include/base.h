#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <string>
#include <array>
#include <functional>
#include <mutex>
#include <thread>
#include <queue>
#include <chrono>
#include <iterator>
#include <Eigen/Dense>
#include <condition_variable>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace switch_controller{

struct RobotState7{
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> theta;
  std::array<double, 7> dtheta;
  std::array<double, 7> tau; //measured joint torque
  std::array<double, 7> tau_ext; //external joint torque, if available
  std::array<double, 16> EE_pose;
};

typedef std::array<double, 7> RobotTorque7;
typedef std::function<void(void)> InitFn;
typedef std::function<RobotTorque7(const RobotState7&)> TorqueCtrlLoopFn;
typedef std::function<void(void)> StopFn;
typedef std::array<double, 42> Jacobian7;

class Robot7{
protected:
  RobotState7 robot_state;
  bool stop_ctrl;
  bool has_data;
  std::mutex m;

public:
  std::string joint_names[7]; //this should be matched with the robot description file
  bool IsDataReceived();
  //user-defined function for each robot
  virtual void IdleControl(); //Idle loop
  virtual void TorqueControl(InitFn init_fn, TorqueCtrlLoopFn loop_fn, StopFn stop_fn);
  virtual void UpdateState();
  virtual RobotTorque7 GetCoriolisVector();
  virtual RobotTorque7 GetGravityVector();
  virtual Jacobian7 GetTaskJacobian();
  
  //predefined
  RobotState7 GetState();
  void StopControl();

  Robot7();
  ~Robot7();
};



class Controller
{
public:
  Controller(Robot7 *robot_ptr)
    : robot(robot_ptr){};

protected:
  Robot7 *robot;
};


class ThreadPool
{
protected:
  size_t num_threads;
  bool stop_all;
  std::vector<std::thread> worker_threads_;
  std::mutex m_job_q;
  std::queue<std::function<void()>> jobs;
  std::condition_variable cv_job_q;

  ThreadPool();
  ~ThreadPool();
  void EnqueueJob(std::function<void()> job);
  void WorkerThread();
public:

};

}
namespace {
// make print function for array
template <class T, size_t N> std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) 
    {
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
}
}// anonymous namespace
#endif //BASE_H