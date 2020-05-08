//Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
//↑の速度プランニング抜き
//初期条件を与えるのにロボットの姿勢と速度を使わず、前回生成したローカルパスの近傍座標の値を使うので初期条件が適当

#ifndef FRENETPLANNER
#define FRENETPLANNER

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <ObstacleGridMap.hpp>
#include <PotentialGridMap.hpp>
#include <Utils.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <Eigen/LU>

class FrenetPlanner{
  struct QuinticPolynominal{
    double a0, a1, a2, a3, a4, a5;

    QuinticPolynominal(const double _st_dist, const double _st_vel, const double _st_acc,
        const double  _en_dist, const double _en_vel, const double _en_acc, const double _t){
      a0 = _st_dist;
      a1 = _st_vel;
      a2 = _st_acc / 2.0;
      
      Eigen::Matrix3d A;
      A << _t * _t * _t, _t * _t * _t * _t,  _t * _t * _t * _t * _t,
             3 * _t * _t, 4 * _t * _t * _t, 5 * _t * _t * _t * _t,
             6 * _t, 12 * _t * _t, 20 * _t * _t * _t;
      Eigen::Matrix3d inv_A = A.inverse();
    
      Eigen::Vector3d b, x;
      b << _en_dist - a0 - a1 * _t - a2 * _t * _t,
        _en_vel - a1 - 2 * a2 * _t,
        _en_acc - 2 * a2;
      
      x = inv_A * b;
      a3 = x(0);
      a4 = x(1);
      a5 = x(2);
    }

    inline double Calc(const double _t){
      return a0 + 
        a1 * _t + 
        a2 * _t * _t + 
        a3 * _t * _t * _t + 
        a4 * _t * _t * _t * _t +
        a5 * _t * _t * _t * _t * _t;
    };

    inline double FirstDeviation(const double _t){
      return a1 + 
        2 * a2 * _t + 
        3 * a3 * _t * _t + 
        4 * a4 * _t * _t * _t +
        5 * a5 * _t * _t * _t * _t;
    }

    inline double SecondDeviation(const double _t){
      return 2 * a2 + 
        6 * a3 * _t + 
        12 * a4 * _t * _t +
        20 * a5 * _t * _t * _t;
    }

    inline double ThirdDeviation(const double _t){
      return 6 * a3 + 
        24 * a4 * _t +
        60 * a5 * _t * _t;
    }
  };

  struct FrenetPath{
    typedef std::shared_ptr<FrenetPath> Ptr;
    typedef std::shared_ptr<const FrenetPath> ConstPtr;
    FrenetPath(const int init_size = 1000){
      d.reserve(init_size);
      d_d.reserve(init_size);
      d_dd.reserve(init_size);
      d_ddd.reserve(init_size);
      cost = 0.0;
      collision_idx = -1;
    }
    void Erase(int _st, int _end){
      this->path.erase(this->path.begin() + _st, this->path.begin() + _end);
      this->d.erase(this->d.begin() + _st, this->d.begin() + _end);
      this->d_d.erase(this->d_d.begin() + _st, this->d_d.begin() + _end);
      this->d_dd.erase(this->d_dd.begin() + _st, this->d_dd.begin() + _end);
      this->d_ddd.erase(this->d_ddd.begin() + _st, this->d_ddd.begin() + _end);
    }
    std::vector<geometry_msgs::Pose2D> path;
    std::vector<double> d;
    std::vector<double> d_d;
    std::vector<double> d_dd;
    std::vector<double> d_ddd;
    double cost;
    int collision_idx;
  };

  std::vector<geometry_msgs::Pose2D> g_path;
  std::vector<double> cum_dist;
  std::vector<FrenetPath::Ptr> frenet_paths;
  FrenetPath::Ptr generated_frenet_path;
  int generated_path_base_idx;
  int traveled_distance;
  ObstacleGridMap obstacle_map;
  PotentialGridMap danger_map;

  double max_road_width;
  double road_width_step;
  double max_pred_dist;
  double min_pred_dist;
  double pred_dist_step;
  int regenerate_timing;
  bool extend_collision_check;
  double lat_vel_limit;
  double kj, kt, kd, kcd, kdanger;

  static const int COLLISION_OFF_SET;
  static const int REGENERATE_LIMIT_LENGTH;

  void CalcCartesianPath(int _st_g_path_idx, FrenetPath &_frenet_path) const {
    _frenet_path.path.clear();

    for(int i = 0; i < _frenet_path.d.size(); i++){ //xy座標に変換
      int g_path_idx = i + _st_g_path_idx;
      if(g_path_idx >= g_path.size())
        break;

      double d = _frenet_path.d[i];
      geometry_msgs::Pose2D pose;
      pose.x = g_path[g_path_idx].x + d * std::cos(g_path[g_path_idx].theta + M_PI / 2.0);
      pose.y = g_path[g_path_idx].y + d * std::sin(g_path[g_path_idx].theta + M_PI / 2.0);
      _frenet_path.path.emplace_back(pose);
    }

    for(int i = 0; i < _frenet_path.path.size() - 1; i++)//theta
      _frenet_path.path[i].theta = std::atan2(_frenet_path.path[i + 1].y - _frenet_path.path[i].y,
          _frenet_path.path[i + 1].x - _frenet_path.path[i].x);

    if(_frenet_path.path.size() <= 1)
      _frenet_path.path.back().theta = 0.0;
    else
      _frenet_path.path.back().theta = _frenet_path.path[_frenet_path.path.size() - 2].theta;
  }

  bool CheckInversionNode(const int _st_g_path_idx, const std::vector<geometry_msgs::Pose2D> &_path){ //折り返すような挙動が無いかチェック
    for(int i = 0; i < _path.size() && (i + _st_g_path_idx) < g_path.size(); i++){
      double theta = NormalizeAngle(g_path[i + _st_g_path_idx].theta - _path[i].theta);
      if(theta > M_PI / 2.0 || theta < -M_PI / 2.0)
        return true;
    }
    return false;
  }

  FrenetPath::Ptr GenerateFrenetPath(const int _st_g_path_idx, const double _st_dist, const double _st_vel, const double _st_acc, 
      const double _en_dist, const double _pred_dist) const {
    FrenetPath::Ptr frenet_path = std::make_shared<FrenetPath>();
    QuinticPolynominal q_poly(_st_dist, _st_vel, _st_acc, _en_dist, 0.0, 0.0, _pred_dist);

    int idx = _st_g_path_idx;
    double dist = 0.0;
    while(dist <= max_pred_dist){
      if(dist <= _pred_dist){
        frenet_path->d.emplace_back(q_poly.Calc(dist));
        frenet_path->d_d.emplace_back(q_poly.FirstDeviation(dist));
        frenet_path->d_dd.emplace_back(q_poly.SecondDeviation(dist));
        frenet_path->d_ddd.emplace_back(q_poly.ThirdDeviation(dist));
      }
      else{
        frenet_path->d.emplace_back(frenet_path->d.back());
        frenet_path->d_d.emplace_back(0.0);
        frenet_path->d_dd.emplace_back(0.0);
        frenet_path->d_ddd.emplace_back(0.0);
      }
      if(idx < cum_dist.size())
        dist = cum_dist[idx] - cum_dist[_st_g_path_idx];
      else
        break;
      idx++;
    }

    double sum_sq_jark = 0.0;
    for(int i = 0; i < frenet_path->d_ddd.size(); i++)
      sum_sq_jark += frenet_path->d_ddd[i] * frenet_path->d_ddd[i];

    frenet_path->cost = kj * sum_sq_jark + kt * _pred_dist * _pred_dist + kd * _en_dist * _en_dist;

    CalcCartesianPath(_st_g_path_idx, *frenet_path);

    return frenet_path;
  };

  bool GenerateFrenetPaths(const int _st_g_path_idx, const double _st_dist, const double _st_vel, const double _st_acc){
    double to_goal_dist = cum_dist.back() - cum_dist[_st_g_path_idx];
    if(_st_g_path_idx >= g_path.size()) //グローバルパスの終端までの距離がmin_pred_dist以下なら再生成をやめる
      return false;
    
    int path_num = (int)((max_pred_dist - min_pred_dist) / pred_dist_step) * (int)(max_road_width * 2.0 / road_width_step);
    frenet_paths.clear();

    //終端座標を超える距離が必要なパスは生成しない
    for(double pred_dist = min_pred_dist; 
        (pred_dist <= max_pred_dist) && (pred_dist <= to_goal_dist); pred_dist += pred_dist_step){
      FrenetPath::Ptr tmp_ptr;
      for(double road_width = 0.0; road_width <= max_road_width; road_width += road_width_step){
          tmp_ptr = GenerateFrenetPath(_st_g_path_idx, _st_dist, _st_vel, _st_acc, road_width, pred_dist);
          if(!CheckInversionNode(_st_g_path_idx, tmp_ptr->path) && CalcMaxD(*tmp_ptr) < lat_vel_limit)
            frenet_paths.emplace_back(tmp_ptr);

          tmp_ptr = GenerateFrenetPath(_st_g_path_idx, _st_dist, _st_vel, _st_acc, -road_width, pred_dist);
          if(!CheckInversionNode(_st_g_path_idx, tmp_ptr->path) && CalcMaxD(*tmp_ptr) < lat_vel_limit)
            frenet_paths.emplace_back(tmp_ptr);
      }
    }

    for(int i = 0; i < frenet_paths.size(); i++){
      FrenetPath::Ptr ptr = frenet_paths[i];
      ptr->collision_idx = CheckCollision(ptr->path);
      ptr->cost += CalcDangerCost(ptr->path) * kdanger;

      int idx = ptr->collision_idx;
      if(idx == -1)
        idx = ptr->path.size();
      idx += _st_g_path_idx;
      if(idx >= g_path.size())
        idx = g_path.size() - 1;

      double dist = cum_dist[idx] - cum_dist[_st_g_path_idx];
      ptr->cost -= dist * dist * kcd; //長いほうがいいので減算しておく

      RemoveCollisionSection(*ptr); //重ければけしてもおｋ
    }

    if(frenet_paths.empty())
      return false;
    else
      return true;
  };

  double CalcMaxD(const FrenetPath &_frenet_path) const {
    double max_d = 0.0;
    for(int i = 0; i < _frenet_path.d_d.size(); i++){
      double d = _frenet_path.d_d[i];
      if(d < 0.0)
        d *= -1.0;
      if(max_d < d)
        max_d = d;
    }
    return max_d;
  }

  int CheckCollision(const std::vector<geometry_msgs::Pose2D> &_path) const {
    for(int i = 0; i < _path.size(); i++){
      const geometry_msgs::Pose2D& pose = _path[i];
      if(obstacle_map.GetValue(pose.x, pose.y) == ObstacleGridMap::EXIST)
        return i;
    }
    return -1;
  }
  
  double CalcDangerCost(const std::vector<geometry_msgs::Pose2D> &_path) const {
    double cost = 0.0;
    for(int i = 0; i < _path.size() - 1; i++){
      const geometry_msgs::Pose2D& pose = _path[i];
      cost += danger_map.GetValue(pose.x, pose.y);
    }
    return cost;
  }

  inline bool CheckLeft(const geometry_msgs::Pose2D &_pose, 
      const geometry_msgs::Pose2D &_st, const geometry_msgs::Pose2D &_en) const { //線分の左右チェック
      double dx1 = _en.x - _st.x;
      double dy1 = _en.y - _st.y;
      double dx2 = _pose.x - _st.x;
      double dy2 = _pose.y - _st.y;
      double ans = dx1 * dy2 - dy1 * dx2;

      if(ans < 0.0)
        return false;
      else
        return true;
  }

  //直行座標の最小値でオフセットしてるので正確じゃないが妥協
  inline double CalcFrenetPose(const geometry_msgs::Pose2D &_pose, int &_min_idx) const {
    double min_dist;
    int min_idx = SearchMinDistIdx(_pose, g_path, min_dist);
    if(min_idx != g_path.size() - 1){ //横のオフセット量計算
      if(!CheckLeft(_pose, g_path[min_idx], g_path[min_idx + 1]))
        min_dist *= -1.0;
    }
    else{
      geometry_msgs::Pose2D dummy;
      dummy.x = std::cos(g_path[min_idx].theta) + g_path[min_idx].x;
      dummy.x = std::sin(g_path[min_idx].theta) + g_path[min_idx].y;
      if(!CheckLeft(_pose, g_path[min_idx], dummy))
        min_dist *= -1.0;
    }
    _min_idx = min_idx;
    return min_dist;
  }

  inline void UpdateFrenetPath(const int _frenet_min_idx, FrenetPath &_frenet_path) const { //通り過ぎた分を削除
    if(_frenet_min_idx > 0 && _frenet_min_idx <= _frenet_path.path.size() - 1){ //空にはしない
      _frenet_path.Erase(0, _frenet_min_idx);

      if(extend_collision_check){
        for(int i = 0; i < _frenet_min_idx; i++){
          _frenet_path.d.emplace_back(_frenet_path.d.back());
          _frenet_path.d_d.emplace_back(_frenet_path.d_d.back());
          _frenet_path.d_dd.emplace_back(_frenet_path.d_dd.back());
          _frenet_path.d_ddd.emplace_back(_frenet_path.d_ddd.back());
        }
      }
    }
  }

  inline void RemoveCollisionSection(FrenetPath &_frenet_path) const {//空にはしない
    if(_frenet_path.path.size() <= COLLISION_OFF_SET || _frenet_path.collision_idx == -1)
      return;

    int collision_idx = _frenet_path.collision_idx - COLLISION_OFF_SET;
    if(collision_idx <= COLLISION_OFF_SET) //先頭要素は消さない
      collision_idx = COLLISION_OFF_SET;

    _frenet_path.Erase(collision_idx, _frenet_path.path.size());
  }

public:
  enum GenerateResult{ GENERATED, NOT_YET_GENERATED, ALL_COLLISION, CAN_NOT_GENERATE, FAULT };

  FrenetPlanner(const double _max_road_width = 1.0, const double _road_width_step = 0.05,
      const double _max_pred_dist = 2.5, const double _min_pred_dist = 1.0, const double _pred_dist_step = 0.5,
      const double _k_jerk = 0.1, const double _k_pred = 0.1, const double _k_dist = 10.0, 
      const double _k_col_dist = 10.0, const double _k_danger = 3.0, const double _lat_vel_limit = 1.25,
      const int _regenerate_timing = 3, const bool _extend_collision_check = false){
    SetRoadWidth(_max_road_width, _road_width_step);
    SetPredDist(_max_pred_dist, _min_pred_dist, _pred_dist_step);
    SetGain(_k_jerk, _k_pred, _k_dist, _k_col_dist, _k_danger);
    SetRegenerateTiming(_regenerate_timing);
    SetExtendCollisionCheck(_extend_collision_check);
    SetLateralVelocityLimit(_lat_vel_limit);
    generated_path_base_idx = 0;
    traveled_distance = 0;
  }

  virtual ~FrenetPlanner(){}

  inline void SetRoadWidth(const double _max_road_width, const double _road_width_step){
    max_road_width = _max_road_width;
    road_width_step = _road_width_step;
  }
  inline void SetPredDist(const double _max_pred_dist, const double _min_pred_dist, const double _pred_dist_step){
    max_pred_dist = _max_pred_dist;
    min_pred_dist = _min_pred_dist;
    pred_dist_step = _pred_dist_step;
  }
  inline void SetGain(const double _k_jerk, const double _k_pred, const double _k_dist, const double _k_col_dist, const double _k_danger){
    kj = _k_jerk;
    kt = _k_pred;
    kd = _k_dist;
    kcd = _k_col_dist;
    kdanger = _k_danger;
  }
  inline void SetRegenerateTiming(const int _regenerate_timing){ regenerate_timing = _regenerate_timing; }
  inline void SetLateralVelocityLimit(const double _lat_vel_limit){ lat_vel_limit = _lat_vel_limit; }

  inline void SetGlobalPath(const std::vector<geometry_msgs::Pose2D> &_g_path){
    g_path = _g_path; 
    cum_dist.clear();

    cum_dist.emplace_back(0.0);
    double dist = 0.0;
    for(int i = 1; i < g_path.size(); i++){
      dist += Distance(g_path[i], g_path[i - 1]);
      cum_dist.emplace_back(dist);
    };

    generated_frenet_path = std::make_shared<FrenetPath>();
    for(int i = 0; i < g_path.size(); i++){ //とりあえずグローバルパス入れとく
      generated_frenet_path->path.emplace_back(g_path[i]);
      generated_frenet_path->d.emplace_back(0.0);
      generated_frenet_path->d_d.emplace_back(0.0);
      generated_frenet_path->d_dd.emplace_back(0.0);
      generated_frenet_path->d_ddd.emplace_back(0.0);
      if(cum_dist[i] > max_pred_dist)
        break;
    }
    generated_path_base_idx = 0;
    traveled_distance = 0;
  };

  inline void SetGridMap(const ObstacleGridMap &_grid_map, const double _robot_radius){
    obstacle_map = _grid_map;
    obstacle_map.Inflate(_robot_radius);
    danger_map.SetGainObstacle(1.0);
    danger_map.SetGainRadius(2.0);
    danger_map.GeneratePotential(obstacle_map);
  };
  
  inline void SetGridMap(const nav_msgs::OccupancyGrid &_ocp_msg, const double _robot_radius){
    obstacle_map.InputOccupancyGrid(_ocp_msg);
    obstacle_map.Inflate(_robot_radius);
    danger_map.SetGainObstacle(1.0);
    danger_map.SetGainRadius(2.0);
    danger_map.GeneratePotential(obstacle_map);
  };
  
  const ObstacleGridMap* GetGridMap(){ return &obstacle_map; };

  inline void SetExtendCollisionCheck(const bool _extend_collision_check){ extend_collision_check = _extend_collision_check; }

  GenerateResult Generate(const geometry_msgs::Pose2D _pose, std::vector<geometry_msgs::Pose2D> &_path){
    if(g_path.empty())
      return GenerateResult::FAULT;

    if(cum_dist.back() < min_pred_dist){ //グローバルパスの長さがmin_pred_distより小さい場合
      _path = g_path;
      return GenerateResult::CAN_NOT_GENERATE;
    }

    int min_g_path_idx;
    double min_dist = CalcFrenetPose(_pose, min_g_path_idx);

    double frenet_min_dist;
    int traveled_idx = SearchMinDistIdx(_pose, generated_frenet_path->path, frenet_min_dist);
    traveled_distance += traveled_idx;

    generated_path_base_idx += traveled_idx;
    UpdateFrenetPath(traveled_idx, *generated_frenet_path);

    CalcCartesianPath(generated_path_base_idx, *generated_frenet_path);
    generated_frenet_path->collision_idx = CheckCollision(generated_frenet_path->path);
    GenerateResult generate_result;

    if(traveled_distance < (generated_frenet_path->path.size() / regenerate_timing) && 
        generated_frenet_path->collision_idx == -1 || generated_frenet_path->path.size() < REGENERATE_LIMIT_LENGTH){
      _path = generated_frenet_path->path;
      generate_result = GenerateResult::NOT_YET_GENERATED;
    }
    else{
      bool success = GenerateFrenetPaths(min_g_path_idx, min_dist, 
          generated_frenet_path->d_d[0], generated_frenet_path->d_dd[0]);
      generate_result = GenerateResult::CAN_NOT_GENERATE;
      if(success){
        int min_idx = 0;
        double min_cost = DBL_MAX;
        for(int i = 0; i < frenet_paths.size(); i++){ //最小コストのパスを抽出
          if(min_cost > frenet_paths[i]->cost){
            min_idx = i;
            min_cost = frenet_paths[i]->cost;
          }
        }
        generated_frenet_path = frenet_paths[min_idx];
        traveled_distance = 0;
        generated_path_base_idx = min_g_path_idx;
        generate_result =  GenerateResult::GENERATED;
      }
    }
    
    RemoveCollisionSection(*generated_frenet_path);
    _path = generated_frenet_path->path;
    return generate_result;
  };

  void GenerateVisualizationMsg(const std_msgs::Header &_header, visualization_msgs::Marker &_vis_path){
    if(frenet_paths.empty())
      return;

    _vis_path.header = _header;
    _vis_path.ns = "line";
    _vis_path.action = visualization_msgs::Marker::ADD;
    _vis_path.pose.orientation.w = 1.0;
    _vis_path.id = 0;
    _vis_path.type = visualization_msgs::Marker::LINE_LIST;
    _vis_path.scale.x = 0.01;
    _vis_path.color.r = 0.0;
    _vis_path.color.g = 1.0;
    _vis_path.color.b = 0.5;
    _vis_path.color.a = 0.3;

    _vis_path.points.clear();
    _vis_path.points.reserve((frenet_paths.back()->path.size() + 100) * frenet_paths.size() * 2);
    for(int i = 0; i < frenet_paths.size(); i++){
      FrenetPath::Ptr frenet_path = frenet_paths[i];
      for(int j = 0; j < frenet_path->path.size() - 1; j++){
        geometry_msgs::Point p1, p2;
        p1.x = frenet_path->path[j].x; p1.y = frenet_path->path[j].y; p1.z = 0.0;
        p2.x = frenet_path->path[j + 1].x; p2.y = frenet_path->path[j + 1].y; p2.z = 0.0;
        _vis_path.points.emplace_back(p1);
        _vis_path.points.emplace_back(p2);
      }
    }
  }
};

const int FrenetPlanner::COLLISION_OFF_SET = 5;
const int FrenetPlanner::REGENERATE_LIMIT_LENGTH = 50;

#endif
