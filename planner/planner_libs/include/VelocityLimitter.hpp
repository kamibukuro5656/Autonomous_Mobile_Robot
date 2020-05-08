//三点からカーブRを求めて横Gの制限値を超えないような速度を返却する
//全然厳密じゃないがカーブ前で適当に減速してくれれば良い

#ifndef VELOCITYLIMITTER
#define VELOCITYLIMITTER

#include <geometry_msgs/Pose2D.h>
#include <Utils.hpp>

class VelocityLimitter{
private:
  double max_vel, min_vel;
  double lat_acc;
  
  inline double Calculate(const geometry_msgs::Pose2D &_p1, const geometry_msgs::Pose2D &_p2){
    double dist = Distance(_p1, _p2);
    double diff_theta = std::fabs(_p2.theta - _p1.theta);
    if(diff_theta < 1e-10)
      return max_vel;
    double r = dist / diff_theta;

    double vel = std::sqrt(r * lat_acc);
    if(vel > max_vel)
      return max_vel;
    else if(vel < min_vel)
      return min_vel;
    else
      return vel;
  };

public:
  VelocityLimitter(const double _max_vel = 1.0, const double _min_vel = 0.05, const double _lat_acc = 0.35){
    SetVelocityRange(_max_vel, _min_vel);
    SetLatAcc(_lat_acc);
  };
  virtual ~VelocityLimitter(){};

  inline void SetVelocityRange(const double _max_vel, const double _min_vel){
    max_vel = _max_vel;
    min_vel = _min_vel;
  };
  inline void GetVelocityRange(double &_max_vel, double &_min_vel){
    _max_vel = max_vel;
    _min_vel = min_vel;
  };
  inline void SetLatAcc(const double _lat_acc){ lat_acc = _lat_acc; };

  void PathVelocityCalculate(const std::vector<geometry_msgs::Pose2D> &_path, 
      const double _init_v, const double _max_acc, std::vector<double> &_velocity_list){
    _velocity_list.resize(_path.size());
    if(_path.size() < 3){
      for(int i = 0; i < _path.size(); i++)
        _velocity_list[i] = 0.0;
      return;
    }

    double init_v = _init_v;
    if(init_v < min_vel)
      init_v = min_vel;

    _velocity_list[0] = init_v;
    for(int i = 1; i < _path.size() - 1; i++)
      _velocity_list[i] = Calculate(_path[i], _path[i + 1]);
    _velocity_list.back() = 0.0;
    
    for(int i = 1; i < _velocity_list.size() - 1; i++)//適当に平滑化
      _velocity_list[i] = (_velocity_list[i - 1] + _velocity_list[i] + _velocity_list[i + 1]) / 3.0;
    
    for(int i = 1; i < _velocity_list.size(); i++){
      double dist = Distance(_path[i - 1], _path[i]);
      double v_max = std::sqrt(2.0 * dist * _max_acc + _velocity_list[i - 1] * _velocity_list[i - 1]);
      if(_velocity_list[i] > v_max)
        _velocity_list[i] = v_max;
    }

    for(int i = _velocity_list.size() - 2; i >= 0; i--){
      double dist = Distance(_path[i + 1], _path[i]);
      double v_max = std::sqrt(2.0 * dist * _max_acc + _velocity_list[i + 1] * _velocity_list[i + 1]);
      if(_velocity_list[i] > v_max)
        _velocity_list[i] = v_max;
    }
  };
};

#endif
