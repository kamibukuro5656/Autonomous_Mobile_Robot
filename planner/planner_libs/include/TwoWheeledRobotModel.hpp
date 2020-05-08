#ifndef TWOWHEELEDROBOTMODEL
#define TWOWHEELEDROBOTMODEL

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

class TwoWheeledRobotModel{
  typedef std::shared_ptr<TwoWheeledRobotModel> Ptr;
  typedef std::shared_ptr<const TwoWheeledRobotModel> ConstPtr;

  double max_acceleration;
  double min_acceleration;
  double max_ang_acceleration;
  double min_ang_acceleration;
  double max_velocity;
  double min_velocity;
  double max_omega;
  double min_omega;

  geometry_msgs::Pose2D state_pose;
  geometry_msgs::Twist state_twist;

public:
  TwoWheeledRobotModel(const double _max_acc = 1.0, const double _min_acc = -1.0, 
      const double _max_ang_acc = 5.0, const double _min_ang_acc = -5.0, 
      const double _max_vel = 0.5, const double _min_vel = 0.0,
      const double _max_omega = 2.5, const double _min_omega = -2.5){
    SetAccelerationLimit(_max_acc, _min_acc);
    SetAngAccelerationLimit(_max_ang_acc, _min_ang_acc);
    SetVelocityLimit(_max_vel, _min_vel);
    SetOmegaLimit(_max_omega, _min_omega);
    geometry_msgs::Pose2D init_pose;
    init_pose.x = 0.0; init_pose.y = 0.0; init_pose.theta = 0.0;
    SetStatePose(init_pose);
    geometry_msgs::Twist init_twist;
    init_twist.linear.x = 0.0; init_twist.linear.y = 0.0; init_twist.linear.z = 0.0;
    init_twist.angular.x = 0.0; init_twist.angular.y = 0.0; init_twist.angular.z = 0.0;
    SetStateTwist(init_twist);
  };
  ~TwoWheeledRobotModel(){};

  inline void SetAccelerationLimit(const double _max, const double _min){ max_acceleration = _max; min_acceleration = _min; };
  inline void GetAccelerationLimit(double &_max, double &_min){ _max = max_acceleration; _min = min_acceleration; };
  inline void SetAngAccelerationLimit(const double _max, const double _min){ max_ang_acceleration = _max; min_ang_acceleration = _min; };
  inline void GetAngAccelerationLimit(double &_max, double &_min){ _max = max_ang_acceleration; _min = min_ang_acceleration; };
  inline void SetVelocityLimit(const double _max, const double _min){ max_velocity = _max; min_velocity = _min; };
  inline void GetVelocityLimit(double &_max, double &_min){ _max = max_velocity; _min = min_velocity; };
  inline void SetOmegaLimit(const double _max, const double _min){ max_omega = _max; min_omega = _min; };
  inline void GetOmegaLimit(double &_max, double &_min){ _max = max_omega; _min = min_omega; };
  inline void SetStatePose(const geometry_msgs::Pose2D &_pose){ state_pose = _pose; };
  inline void SetStateTwist(const geometry_msgs::Twist &_twist){ state_twist = _twist; };
  inline geometry_msgs::Pose2D GetStatePose(){ return state_pose; };
  inline geometry_msgs::Twist GetStateTwist(){ return state_twist; };

  inline geometry_msgs::Pose2D PredictNextState(const geometry_msgs::Twist &_twist, const double _dt, 
      geometry_msgs::Twist &_next_twist) const {
    double tmp_acc = (state_twist.linear.x - _twist.linear.x) / _dt;
    double tmp_ang_acc = (state_twist.angular.z - _twist.angular.z) / _dt;
    if(tmp_acc > max_acceleration)
      tmp_acc = max_acceleration;
    else if(tmp_acc < min_acceleration)
      tmp_acc = max_acceleration;
    if(tmp_ang_acc > max_ang_acceleration)
      tmp_ang_acc = max_ang_acceleration;
    else if(tmp_ang_acc < min_ang_acceleration)
      tmp_ang_acc = min_ang_acceleration;

    _next_twist.linear.x = state_twist.linear.x + tmp_acc * _dt;
    _next_twist.angular.z = state_twist.angular.z + tmp_ang_acc * _dt;

    geometry_msgs::Pose2D next_pose;
    next_pose.x = _next_twist.linear.x * std::cos(state_pose.theta) * _dt + state_pose.x;
    next_pose.y = _next_twist.linear.x * std::sin(state_pose.theta) * _dt + state_pose.y;
    next_pose.theta = _next_twist.angular.z * _dt + state_pose.theta;

    return next_pose;
  };

  inline geometry_msgs::Pose2D UpdateState(const geometry_msgs::Twist &_twist, const double _dt){
    geometry_msgs::Twist next_twist;
    state_pose = PredictNextState(_twist, _dt, state_twist);

    return state_pose;
  };

  inline std::vector<geometry_msgs::Pose2D> GeneratePredictionPath(const double _time, const double _dt) const {
    std::vector<geometry_msgs::Pose2D> path;
    path.reserve(static_cast<int>(_time / _dt));
    double t = 0.0;
    geometry_msgs::Pose2D next_pose = state_pose;
    while( t < _time ){
      geometry_msgs::Pose2D tmp_pose;
      tmp_pose.x = state_twist.linear.x * std::cos(next_pose.theta) * _dt + next_pose.x;
      tmp_pose.y = state_twist.linear.x * std::sin(next_pose.theta) * _dt + next_pose.y;
      tmp_pose.theta = state_twist.angular.z * _dt + next_pose.theta;
      while(next_pose.theta <= -M_PI) next_pose.theta += 2.0 * M_PI;
      while(next_pose.theta > M_PI) next_pose.theta -= 2.0 * M_PI;

      path.emplace_back(tmp_pose);
      next_pose = tmp_pose;
      t += _dt;
    }
    return path;
  };

  inline void CalcDynamicWindow(const double _dt, double &_min_v, double &_max_v, double &_min_omega, double &_max_omega){
    _min_v = state_twist.linear.x + min_acceleration * _dt;
    _max_v = state_twist.linear.x + max_acceleration * _dt;
    _min_omega = state_twist.angular.z + min_ang_acceleration * _dt;
    _max_omega = state_twist.angular.z + max_ang_acceleration * _dt;
    if(_min_v < min_velocity)
      _min_v = min_velocity;
    if(_max_v > max_velocity)
      _max_v = max_velocity;
    if(_min_omega < min_omega)
      _min_omega = min_omega;
    if(_max_omega > max_omega)
      _max_omega = max_omega;
  };
};

#endif
