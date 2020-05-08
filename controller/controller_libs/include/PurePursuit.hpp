#ifndef PUREPURSUITE
#define PUREPURSUITE

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <Utils.hpp>
#include <ObstacleGridMap.hpp>
#include <TwoWheeledRobotModel.hpp>

class PurePursuit{ //とりあえず後退は考えない
protected:
  std::vector<geometry_msgs::Pose2D> path;
  double max_ref_radius;
  double min_ref_radius;
  double gain;

  //自己位置を中心とする参照円とパスが交差しなかった場合は最近傍点から参照円分伸ばした座標を返す
  geometry_msgs::Pose2D CalcReferencePointDummy(const geometry_msgs::Pose2D &_pose, double _ref_radius) const {
    double min_dist;
    int min_idx = SearchMinDistIdx(_pose, path, min_dist);

    int ref_idx;
    for(ref_idx = min_idx; ref_idx < path.size(); ref_idx++){
      double dist = Distance(path[min_idx], path[ref_idx]);
      if(_ref_radius < dist)
        break;
    }
    if(ref_idx == path.size()){
      return path.back();
    }
    else{
      return path[ref_idx];
    }
  };

  geometry_msgs::Pose2D CalcInterpolateWaypoint(const geometry_msgs::Pose2D &_st, const geometry_msgs::Pose2D &_en, 
      const geometry_msgs::Pose2D _pose, const double _ref_radius) const {
    double theta = std::atan2(_en.y - _st.y, _en.x - _st.x);
    geometry_msgs::Pose2D rotated_center;
    rotated_center.x = _st.x + (_pose.x - _st.x) * std::cos(-theta) - (_pose.y - _st.y) * std::sin(-theta);
    rotated_center.y = _st.y + (_pose.x - _st.x) * std::sin(-theta) + (_pose.y - _st.y) * std::cos(-theta);

    double dist = std::fabs(rotated_center.y - _st.y);
    
    geometry_msgs::Pose2D rotated_cross1, rotated_cross2;
    rotated_cross1.x = rotated_center.x - std::sqrt(_ref_radius * _ref_radius - dist * dist);
    rotated_cross1.y = _st.y;
    rotated_cross2.x = rotated_center.x + std::sqrt(_ref_radius * _ref_radius - dist * dist);
    rotated_cross2.y = _st.y; 

    geometry_msgs::Pose2D cross1, cross2;
    cross1.x = _st.x + (rotated_cross1.x - _st.x) * std::cos(theta) - (rotated_cross1.y - _st.y) * std::sin(theta);
    cross1.y = _st.y + (rotated_cross1.x - _st.x) * std::sin(theta) + (rotated_cross1.y - _st.y) * std::cos(theta);
    cross2.x = _st.x + (rotated_cross2.x - _st.x) * std::cos(theta) - (rotated_cross2.y - _st.y) * std::sin(theta);
    cross2.y = _st.y + (rotated_cross2.x - _st.x) * std::sin(theta) + (rotated_cross2.y - _st.y) * std::cos(theta);

    if(Distance(cross1, _en) < Distance(cross2, _en))
      return cross1;
    else
      return cross2;
  }

  geometry_msgs::Pose2D CalcReferencePoint(const geometry_msgs::Pose2D &_pose, double _ref_radius) const {
    int ref_idx = 0;
    for(int i = 0; i < path.size(); i++){
      double dist = Distance(_pose, path[i]);
      if(dist < _ref_radius)
        ref_idx = i;
    }

    if(ref_idx == path.size() - 1)
      return path.back();
    else if(ref_idx == 0){
      return CalcReferencePointDummy(_pose, _ref_radius);
    }
    else{
      return CalcInterpolateWaypoint(path[ref_idx], path[ref_idx + 1], _pose, _ref_radius);
    }
  };

  inline double CalcReferenceCircle(const double _vel){
    double radius = std::fabs(_vel * gain) + min_ref_radius;
    if(radius > max_ref_radius)
      radius = max_ref_radius;
    return radius;
  }
  
  inline double CalcTurningRadius(const geometry_msgs::Pose2D &_pose, const geometry_msgs::Pose2D &_ref_pose, const double _ref_circle){
    double angle = std::atan2(_ref_pose.y - _pose.y, _ref_pose.x - _pose.x) - _pose.theta;

    double angle_sin = std::sin(angle);
    if(-0.0000001 < angle_sin && angle_sin < 0.0000001)
      angle_sin = 0.0000001;

    return _ref_circle / (2.0 * angle_sin);
  }

public:
  PurePursuit(const double _max_ref_radius = 0.5, const double _min_ref_radius = 0.1, const double _gain = 0.5){
    SetRadius(_max_ref_radius, _min_ref_radius);
    SetGain(_gain);
  };
  virtual ~PurePursuit(){};

  inline void SetPath(const std::vector<geometry_msgs::Pose2D> &_path){ path = _path; };
  inline void SetRadius(const double _max_ref_radius, const double _min_ref_radius){
    max_ref_radius = _max_ref_radius;
    min_ref_radius = _min_ref_radius;
  };
  inline void SetGain(const double _gain){ gain = _gain; }

  double Calculate(const geometry_msgs::Pose2D &_pose, const double _vel, geometry_msgs::Pose2D &_ref_pose){
    if(path.size() < 2)
      return 0.0;
    
    double ref_circle = CalcReferenceCircle(_vel);
    geometry_msgs::Pose2D ref_pose = CalcReferencePoint(_pose, ref_circle);
    double turning_radius = CalcTurningRadius(_pose, ref_pose, ref_circle);
    
    _ref_pose = ref_pose;
    return _vel / turning_radius;;
  };
};

//衝突回避付き
class PurePursuitWithVelocity : public PurePursuit{
protected:
  ObstacleGridMap obstacle_map;
  double delta_time;
  double sample_num;
  double pred_time;

  TwoWheeledRobotModel model;
  
  bool CheckCollision(const geometry_msgs::Pose2D &_pose, const geometry_msgs::Twist &_twist){
    model.SetStateTwist(_twist);
    model.SetStatePose(_pose);
    std::vector<geometry_msgs::Pose2D> path = model.GeneratePredictionPath(pred_time, 0.025);
    for(int i = 0; i < path.size(); i++){
      if(obstacle_map.GetValue(path[i].x, path[i].y) == ObstacleGridMap::EXIST)
        return true;
    }
    return false;
  }

public:
  PurePursuitWithVelocity(const double _max_ref_radius = 0.5, const double _min_ref_radius = 0.1, const double _gain = 0.5,
      const double _loop_rate = 50.0, const int _sample_num = 30, const double _pred_time = 0.5) : 
    PurePursuit(_max_ref_radius, _min_ref_radius, _gain){
    delta_time = 1.0 / _loop_rate;
    sample_num = _sample_num;
    pred_time = _pred_time;
  };

  virtual ~PurePursuitWithVelocity(){};

  inline void SetGridMap(const ObstacleGridMap &_grid_map, const double _robot_radius){
    obstacle_map = _grid_map;
    obstacle_map.Inflate(_robot_radius);
  };
  inline void SetGridMap(const nav_msgs::OccupancyGrid &_ocp_msg, const double _robot_radius){
    obstacle_map.InputOccupancyGrid(_ocp_msg);
    obstacle_map.Inflate(_robot_radius);
  };
  void SetRoopLate(const double _loop_rate){ delta_time = 1.0 / _loop_rate; };
  void SetRobotModel(const TwoWheeledRobotModel &_model){ model = _model; };
  void SetSampleNum(const int _sample_num){ sample_num = _sample_num; };
  void SetPredTime(const double _pred_time){ pred_time = _pred_time; };
  double GetPredTime(){ return pred_time; };
  
  geometry_msgs::Twist Calculate(const geometry_msgs::Pose2D &_pose, const geometry_msgs::Twist &_bef_vel,
      const double _target_vel, geometry_msgs::Pose2D &_ref_pose){
    if(path.size() < 2){
      geometry_msgs::Twist dummy;
      dummy.linear.x = 0.0;
      dummy.angular.z = 0.0;
      return dummy;
    }


    double ref_circle = CalcReferenceCircle(_bef_vel.linear.x);
    geometry_msgs::Pose2D ref_pose = CalcReferencePoint(_pose, ref_circle);
    double turning_radius = CalcTurningRadius(_pose, ref_pose, ref_circle);
    _ref_pose = ref_pose;
    
    model.SetStateTwist(_bef_vel); 

    double min_v, max_v, min_omega, max_omega;
    model.CalcDynamicWindow(delta_time, min_v, max_v, min_omega, max_omega);
    
    double target_vel = _target_vel;
    if(target_vel <= min_v)
      target_vel = min_v;
    else if(target_vel >= max_v)
      target_vel = max_v;

    double velocity_step = (max_v - min_v) / (double)(sample_num);
    geometry_msgs::Twist tmp_twist;
    double tmp_v;
    for(tmp_v = target_vel; tmp_v >= min_v; tmp_v -= velocity_step){
      tmp_twist.linear.x = tmp_v;
      tmp_twist.angular.z = tmp_twist.linear.x / turning_radius;
      if(tmp_twist.angular.z <= min_omega)
        tmp_twist.angular.z = min_omega;
      else if(tmp_twist.angular.z >= max_omega)
        tmp_twist.angular.z = max_omega;

      if(!CheckCollision(_pose, tmp_twist))
        break;
    }

    if(tmp_v < min_v){
      tmp_twist.linear.x = min_v;
      tmp_twist.angular.z = tmp_twist.linear.x / turning_radius;
      if(tmp_twist.angular.z <= min_omega)
        tmp_twist.angular.z = min_omega;
      else if(tmp_twist.angular.z >= max_omega)
        tmp_twist.angular.z = max_omega;
    }

    return tmp_twist;
  };
};

#endif
