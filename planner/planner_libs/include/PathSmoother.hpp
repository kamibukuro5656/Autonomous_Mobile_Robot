/*
 * ref1: A G2 Continuous Path-smoothing Algorithm Using Modified Quadratic Polynomial Interpolation
 * ref2: A Collision-Free G2 Continuous Path-Smoothing Algorithm Using Quadratic Polynomial Interpolation
*/

#ifndef PATHSMOOTHER
#define PATHSMOOTHER

#include <geometry_msgs/Pose2D.h>
#include <ObstacleGridMap.hpp>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/LU>

class PathSmoother{
  static const double COLLISION_CHECK_STEP;

  std::vector<geometry_msgs::Pose2D> waypoint_list;
  std::vector<Eigen::Vector3d> params_list_x;
  std::vector<Eigen::Vector3d> params_list_y;
  std::vector<double> cum_dist;

  inline double CalcDistance(const geometry_msgs::Pose2D &_p0, const geometry_msgs::Pose2D &_p1) const {
    double x = _p0.x - _p1.x;
    double y = _p0.y - _p1.y;
    return std::sqrt(x * x + y * y);
  };

  void GenerateCumulativeDistance(){
    cum_dist.clear();
    cum_dist.reserve(waypoint_list.size());
    
    double dist = 0.0;
    for(int i = 0; i < (int)waypoint_list.size() - 1; i++){
      cum_dist.emplace_back(dist);
      dist += CalcDistance(waypoint_list[i], waypoint_list[i + 1]);
    }
    cum_dist.emplace_back(dist);
  };

  inline void CalcParam(const int _idx, Eigen::Vector3d &_param_x, Eigen::Vector3d &_param_y){
    Eigen::Matrix3d mat;
    mat << cum_dist[_idx - 1] * cum_dist[_idx - 1], cum_dist[_idx - 1], 1,
           cum_dist[_idx] * cum_dist[_idx], cum_dist[_idx], 1,
           cum_dist[_idx + 1] * cum_dist[_idx + 1], cum_dist[_idx + 1], 1;
    Eigen::Matrix3d inv = mat.inverse();
    
    Eigen::Vector3d x, y;
    x << waypoint_list[_idx - 1].x, waypoint_list[_idx].x, waypoint_list[_idx + 1].x;
    y << waypoint_list[_idx - 1].y, waypoint_list[_idx].y, waypoint_list[_idx + 1].y;

    _param_x = inv * x;
    _param_y = inv * y;
  }

  void GenerateParamList(){
    params_list_x.clear();
    params_list_x.reserve(waypoint_list.size());
    params_list_y.clear();
    params_list_y.reserve(waypoint_list.size());

    Eigen::Vector3d dummy = Eigen::Vector3d::Zero();
    params_list_x.emplace_back(dummy);
    params_list_y.emplace_back(dummy);

    int i = 1;
    while(i < (int)waypoint_list.size() - 1){
      Eigen::Vector3d param_x, param_y;
      CalcParam(i, param_x, param_y);
      params_list_x.emplace_back(param_x);
      params_list_y.emplace_back(param_y);
      i++;
    }

    params_list_x.emplace_back(dummy);
    params_list_y.emplace_back(dummy);
  };

  inline geometry_msgs::Pose2D CalcQuadraticPolynomial(const double _d, 
      const Eigen::Vector3d &_param_x, const Eigen::Vector3d &_param_y) const {
    geometry_msgs::Pose2D res;
    res.x = _d * _d * _param_x(0) + _d * _param_x(1) +  _param_x(2);
    res.y = _d * _d * _param_y(0) + _d * _param_y(1) +  _param_y(2);
    return res;
  };

  inline geometry_msgs::Pose2D CalcWaitedQuadratic(const double _d, const int _idx) const{
    geometry_msgs::Pose2D res, prev, next;
    
    if(_idx == 0)
      return CalcQuadraticPolynomial(_d, params_list_x[_idx + 1], params_list_y[_idx + 1]);

    if(_idx >= waypoint_list.size() - 2)
      return CalcQuadraticPolynomial(_d, params_list_x[_idx], params_list_y[_idx]);

    prev = CalcQuadraticPolynomial(_d, params_list_x[_idx], params_list_y[_idx]);
    next = CalcQuadraticPolynomial(_d, params_list_x[_idx + 1], params_list_y[_idx + 1]);

    double w_prev, w_next;
    w_prev = (cum_dist[_idx + 1] - _d) / (cum_dist[_idx + 1] - cum_dist[_idx]);
    w_next = (_d - cum_dist[_idx]) / (cum_dist[_idx + 1] - cum_dist[_idx]);
    res.x = w_prev * prev.x + w_next * next.x;
    res.y = w_prev * prev.y + w_next * next.y;

    return res;
  }

  double CheckCollision(const ObstacleGridMap &_grid_map, geometry_msgs::Pose2D &_collision_pose) const {
    double d = 0.0;
    while(d <= cum_dist.back()){
      geometry_msgs::Pose2D tmp_pose = CalcInterpolateWaypoint(d);
      if(_grid_map.GetValue(tmp_pose.x, tmp_pose.y) == ObstacleGridMap::EXIST){
        int idx = SearchIdx(d);
        if(d > cum_dist[idx] + 0.1 && d < cum_dist[idx + 1] - 0.1){ //waypointの周囲はチェックしない
          _collision_pose = tmp_pose;
          return d;
        }
      }
      d += COLLISION_CHECK_STEP;
    }
    return -1.0;
  }

  geometry_msgs::Pose2D CalcPerpPose(const int _idx, const geometry_msgs::Pose2D &_pose) const {
    geometry_msgs::Pose2D res_pose;
    if(waypoint_list[_idx].x == waypoint_list[_idx + 1].x){
      res_pose.x = waypoint_list[_idx].x;
      res_pose.y = _pose.y;
    }
    else if(waypoint_list[_idx].y == waypoint_list[_idx + 1].y){
      res_pose.x = _pose.x;
      res_pose.y = waypoint_list[_idx].y;
    }
    else{
      double a1, a2, b1, b2;
      a1 = (waypoint_list[_idx + 1].y - waypoint_list[_idx].y) / (waypoint_list[_idx + 1].x - waypoint_list[_idx].x);
      b1 = waypoint_list[_idx].y - a1 * waypoint_list[_idx].x;
      a2 = -1.0 / a1;
      b2 = _pose.y - a2 * _pose.x;

      res_pose.x = (b2 - b1) / (a1 - a2);
      res_pose.y = (b2 * a1 - b1 * a2) / (a1 - a2);
    }

    return res_pose;
  }

  geometry_msgs::Pose2D CalcLinearInterpolate(const double _d, const int _idx) const {
    double relate_dist = _d / cum_dist[_idx + 1];
    geometry_msgs::Pose2D res_pose = waypoint_list[_idx];
    double a = waypoint_list[_idx + 1].x - waypoint_list[_idx].x;
    double b = waypoint_list[_idx + 1].y - waypoint_list[_idx].y;
    res_pose.x += a * relate_dist;
    res_pose.y += b * relate_dist;
    return res_pose;
  }

public:
  PathSmoother(){};
  ~PathSmoother(){};

  bool SetWaypoints(const std::vector<geometry_msgs::Pose2D> &_waypoints){
    if(_waypoints.size() < 1)
      return false;

    waypoint_list = _waypoints;
    GenerateCumulativeDistance();
    GenerateParamList();

    return true;
  };
  
  std::vector<geometry_msgs::Pose2D> GetWaypoints() const { return waypoint_list; };
  inline double GetTotalLength() const { return cum_dist.back(); };
  
  inline bool InsertWaypoint(const int _idx, const geometry_msgs::Pose2D &_waypoint){
    if(_idx < 0 || _idx > waypoint_list.size() - 1)
      return false;

    waypoint_list.insert(waypoint_list.begin() + _idx + 1, _waypoint);

    if(waypoint_list.size() >= 2){
      GenerateCumulativeDistance();
      GenerateParamList();
    }
    return true;
  };

  int SearchIdx(const double _d) const { //リストのいちばんうしろは出力しない仕様のはず
    if(_d < 0.0 || cum_dist.back() < _d)
      return -1;

    int min_idx = 0;
    int max_idx = cum_dist.size() - 1;
    while(min_idx < max_idx){
      int mid_idx = min_idx + ((max_idx - min_idx) >> 2);
      if(cum_dist[mid_idx] <= _d)
        min_idx = mid_idx + 1;
      else
        max_idx = mid_idx;
    }
    
    if(cum_dist[min_idx] < _d)
      return min_idx;
    else
      return min_idx - 1;
  };

  inline geometry_msgs::Pose2D CalcInterpolateWaypoint(const double _d) const {
    geometry_msgs::Pose2D res_pose;
    int idx = SearchIdx(_d);
    if(idx < 0){
      res_pose.x = 0; res_pose.y = 0; res_pose.theta = 0;
      return res_pose;
    }
    
    if(waypoint_list.size() > 2){
      res_pose = CalcWaitedQuadratic(_d, idx);
      geometry_msgs::Pose2D tmp_pose;
      tmp_pose = CalcWaitedQuadratic(_d + 1e-10, idx);
      res_pose.theta = std::atan2(tmp_pose.y - res_pose.y, tmp_pose.x - res_pose.x); //雑に数値微分して算出
    }
    else{
      res_pose = CalcLinearInterpolate(_d, idx);
    }
    return res_pose;
  };

  inline bool GenerateCollisionFreePath(const ObstacleGridMap &_gridmap, const int _iteration_num, PathSmoother &_col_free_path) const {
    _col_free_path = *this;
    return _col_free_path.GenerateCollisionFreePath(_gridmap, _iteration_num);
  };

  bool GenerateCollisionFreePath(const ObstacleGridMap &_gridmap, const int _iteration_num){
    for(int i = 0; i < _iteration_num; i++){
      geometry_msgs::Pose2D collision_pose;
      double collision_d = CheckCollision(_gridmap, collision_pose);
      if(collision_d < 0.0)
        return true;

      int idx = SearchIdx(collision_d);
      geometry_msgs::Pose2D add_pose = CalcPerpPose(idx, collision_pose);
      InsertWaypoint(idx, add_pose);
    }
    return false;
  };

  inline void GenerateDiscreteWaypoints(const double _step, std::vector<geometry_msgs::Pose2D> &_waypoints){
    if(waypoint_list.size() < 1)
      return;

    _waypoints.clear();
    _waypoints.reserve((int)(cum_dist.back() / _step));

    double d = 0.0;
    while(d <= cum_dist.back()){
      _waypoints.emplace_back(CalcInterpolateWaypoint(d));
      d += _step;
    }
  };
};

const double PathSmoother::COLLISION_CHECK_STEP = 0.001;

#endif
