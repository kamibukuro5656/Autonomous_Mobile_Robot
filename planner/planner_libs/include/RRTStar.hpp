#ifndef RRTSTAR
#define RRTSTAR

#include <chrono>
#include <nanoflann.hpp>
#include <ProbabilityGridMap.hpp>
#include <ObstacleGridMap.hpp>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <Utils.hpp>
#include <random>
#include <list>
#include <vector>

class RRTStar{
protected:
  struct Node{
    typedef std::shared_ptr<Node> Ptr;
    typedef std::shared_ptr<const Node> ConstPtr;

    Node() : prev(nullptr), cost(DBL_MAX){};
    Node(const double _cost, const geometry_msgs::Pose2D &_pose, const Ptr &_prev = nullptr) :
      prev(_prev), cost(_cost), pose(_pose){};

    double cost;
    geometry_msgs::Pose2D pose;

    Ptr prev;
  };

  struct NodeList{
    std::vector<Node::Ptr> list;
    inline size_t kdtree_get_point_count() const { return list.size(); }
	  inline double kdtree_get_pt(const size_t idx, const size_t dim) const
	  {
      if(dim == 0)
        return list[idx]->pose.x;
      
      return list[idx]->pose.y;
	  }
    template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
  };

  typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, NodeList> , NodeList, 2> NanoFlannIndexType;

  static const double MAP_SIZE_OFFSET;

  bool use_informed_rrt;
  int iteration;
  double step;
  double r_param;
  double collision_radius;
  double goal_select_prob;
  double time_out;
  double improvement_amount;
  ObstacleGridMap grid_map;
    
  std::vector<std::pair<size_t, double>> ret_matches_cache; //再確保したくないので雑にメンバに

  NodeList node_list;
  std::vector<Node::Ptr> goal_list;
  Node::Ptr min_goal;
  std::vector<geometry_msgs::Pose2D> min_path;

  std::default_random_engine* engine;
  std::uniform_real_distribution<> rand_gen_0_1;
  std::uniform_real_distribution<> rand_gen_x;
  std::uniform_real_distribution<> rand_gen_y;

  geometry_msgs::Pose2D RandomSampling(const geometry_msgs::Pose2D &_goal){
    geometry_msgs::Pose2D res_pose;

    double prob = rand_gen_0_1(*engine);
    if(prob > goal_select_prob){
      res_pose.x = rand_gen_x(*engine);
      res_pose.y = rand_gen_y(*engine);
    }
    else{
      res_pose = _goal;
    }

    return res_pose;
  };

  geometry_msgs::Pose2D EllipsoidalSampling(const geometry_msgs::Pose2D &_start,
      const geometry_msgs::Pose2D &_goal, const double _best_cost){
    geometry_msgs::Pose2D res_pose;
    double prob = rand_gen_0_1(*engine);
    if(prob > goal_select_prob){
      double min_cost = Distance(_start, _goal);
      geometry_msgs::Pose2D rand_pose;
      double r = sqrt(rand_gen_0_1(*engine));
      double theta = rand_gen_0_1(*engine) * 2.0 * M_PI;
      rand_pose.x = r * std::cos(theta);
      rand_pose.y = r * std::sin(theta);
      
      geometry_msgs::Pose2D center;
      center.x = (_goal.x + _start.x) * 0.5;
      center.y = (_goal.y + _start.y) * 0.5;

      res_pose.x = rand_pose.x * _best_cost * 0.5 + center.x;
      res_pose.y = rand_pose.y * sqrt(_best_cost * _best_cost - min_cost * min_cost) * 0.5 + center.y;
    }
    else{
      res_pose = _goal;
    }
    return res_pose;
  }

  virtual Node::Ptr CreateNewNode(const geometry_msgs::Pose2D &_sample_pose, const NanoFlannIndexType &_index){

    double query[2] = {_sample_pose.x, _sample_pose.y};
    size_t ret_index;
    double ret_dist;
    nanoflann::KNNResultSet<double> result_set(1);
    result_set.init(&ret_index, &ret_dist);
    _index.findNeighbors(result_set, query, nanoflann::SearchParams(4));
    Node::Ptr nearest_ptr = node_list.list[ret_index];

    double theta = std::atan2(_sample_pose.y - nearest_ptr->pose.y, _sample_pose.x - nearest_ptr->pose.x);
    geometry_msgs::Pose2D tmp_pose;
    tmp_pose.x = nearest_ptr->pose.x + step * std::cos(theta);
    tmp_pose.y = nearest_ptr->pose.y + step * std::sin(theta);
    tmp_pose.theta = theta;
    
    Node::Ptr res_ptr = nullptr;
    if(grid_map.GetValue(tmp_pose.x, tmp_pose.y) == ObstacleGridMap::FREE)
      res_ptr = std::make_shared<Node>(DBL_MAX, tmp_pose);
    
    return res_ptr;
  };
  
  inline double CalcSearchRadius() const {
    return r_param * std::sqrt(std::log((double)node_list.list.size()) / (double)node_list.list.size() );
  };

  virtual inline double CalcEdgeCost(const geometry_msgs::Pose2D &_st, const geometry_msgs::Pose2D &_end) const {
    return Distance(_st, _end);  
  }

  int CreateNearNodeList(const Node &_node, const NanoFlannIndexType &_index, std::vector<Node::Ptr> &_near_list,
      std::vector<double> &_edge_cost_list, std::vector<double> &_cost_list){
    _near_list.clear();
    _cost_list.clear();
    _edge_cost_list.clear();
    ret_matches_cache.clear();
    double search_radius = CalcSearchRadius();

    double query[2] = {_node.pose.x, _node.pose.y};
    
  nanoflann::RadiusResultSet<double, size_t> result_set(search_radius, ret_matches_cache);
    _index.findNeighbors(result_set, query, nanoflann::SearchParams());
    
    int min_index = -1;
    double min_cost = DBL_MAX;
    for(int i = 0; i < ret_matches_cache.size(); i++){
      int idx = ret_matches_cache[i].first;
      if(CheckCollisionForEdge(_node.pose, node_list.list[idx]->pose))
        continue;
      _near_list.emplace_back(node_list.list[idx]);
      double cost = CalcCost(node_list.list[idx]);
      _cost_list.emplace_back(cost);
      _edge_cost_list.emplace_back(CalcEdgeCost(_node.pose, node_list.list[idx]->pose));
      double tmp_sum_cost = cost + _edge_cost_list.back();
      if(tmp_sum_cost < min_cost){
        min_cost = tmp_sum_cost;
        min_index = _near_list.size() - 1;
      }
    }
    return min_index;
  };

  double CalcCost(const Node::Ptr &_target){
    double cost = 0.0;
    Node::Ptr tmp_ptr = _target;
    while(tmp_ptr != nullptr){
      if(tmp_ptr->cost == DBL_MAX)
        return DBL_MAX;
      cost += tmp_ptr->cost;
      tmp_ptr = tmp_ptr->prev;
    }
    return cost;
  };

  void Rewire(const Node::Ptr &_new_node, std::vector<Node::Ptr> &_near_list, 
      const std::vector<double> &_edge_cost_list, const std::vector<double> &_cost_list){
    double path_cost = CalcCost(_new_node);
    if(path_cost == DBL_MAX || _near_list.empty())
      return;

    for(int i = 0; i < _near_list.size(); i++){
      if(_cost_list[i] > (path_cost + _edge_cost_list[i])){
        _near_list[i]->prev = _new_node;
        _near_list[i]->cost = _edge_cost_list[i];
      }
    }
  };

  void GeneratePath(const Node::Ptr &_node, std::vector<geometry_msgs::Pose2D> &_result){
    if(_node->prev == nullptr){
      _result.emplace_back(_node->pose);
      return;
    }

    GeneratePath(_node->prev, _result);
    _result.back().theta = std::atan2(_node->pose.y - _result.back().y, _node->pose.x - _result.back().x);
    if(_node->pose.x != _result.back().x && _node->pose.y != _result.back().y) //同じ座標のノードが入ることがあったので防止
      _result.emplace_back(_node->pose);
  };

  void PathCleanup(const std::vector<geometry_msgs::Pose2D> &_path, std::vector<geometry_msgs::Pose2D> &_result){ //至近距離のノードを省略する
    _result.clear();
    _result.reserve(_path.size());
    if(_path.size() < 3){
      _result = _path;
      return;
    }

    double threshold = step * 1.0;

    _result.emplace_back(_path[0]);
    for(int i = 1; i < _path.size() - 1; i++){
      if((Distance(_result.back(), _path[i]) > threshold) || CheckCollisionForEdge(_result.back(), _path[i + 1]))
          _result.emplace_back(_path[i]);
    }
    
    if(!((Distance(_result.back(), _path.back()) > threshold) || CheckCollisionForEdge(*(_result.end() - 2), _path.back())))
      _result.erase(_result.end() - 1);

    _result.emplace_back(_path.back());
    return;
  }

  bool CheckCollisionForEdge(const geometry_msgs::Pose2D &_st_pose, const geometry_msgs::Pose2D &_end_pose){
    int idx_st_x, idx_st_y, idx_end_x, idx_end_y;
    grid_map.CalcIdx(_st_pose.x, _st_pose.y, idx_st_x, idx_st_y);
    grid_map.CalcIdx(_end_pose.x, _end_pose.y, idx_end_x, idx_end_y);
  
    int idx_w, idx_h;
    grid_map.GetMapSizePixel(idx_w, idx_h);

    if(idx_st_x < 0 || idx_st_x >= idx_w || idx_st_y < 0 || idx_st_y >= idx_h)
      return false;
    if(idx_end_x < 0 || idx_end_x >= idx_w || idx_end_y < 0 || idx_end_y >= idx_h)
      return false;

    const std::vector<int8_t> *map_ptr = grid_map.GetMap();

    int dx = idx_end_x - idx_st_x, dy = idx_end_y - idx_st_y;
    int x = idx_st_x, y = idx_st_y;
    
    int x_add, y_add;
    if(idx_st_x < idx_end_x) x_add = 1;
    else x_add = -1;
    if(idx_st_y < idx_end_y) y_add = 1;
    else y_add = -1;
    
    int dx2 = 2 * dx * x_add, dy2 = 2 * dy * y_add;
    if(std::abs(dx) >= std::abs(dy)){
      int d = -dx * x_add;
      while(x != idx_end_x){
 	      if(d > 0){
          y += y_add; 
          d -= dx2;
 	      }
        d += dy2;
        if((*map_ptr)[x + y * idx_w] == ObstacleGridMap::EXIST)
          return true;
        x += x_add;
      }
    }
    else{
      int d = -dy * y_add;
      while(y != idx_end_y){
        if(d > 0){
          x += x_add;
  	      d -= dy2;
  		  }
        d += dx2;
        if((*map_ptr)[x + y * idx_w] == ObstacleGridMap::EXIST)
          return true;
        y += y_add;
      }
    }
    return false;
  }
  
  inline void InitRandGenerator(){
    double x_min, x_max, y_min, y_max;
    grid_map.GetMapSize(x_min, x_max, y_min, y_max);
    std::uniform_real_distribution<>::param_type param_x(x_min - MAP_SIZE_OFFSET, x_max + MAP_SIZE_OFFSET);
    rand_gen_x.param(param_x);
    std::uniform_real_distribution<>::param_type param_y(y_min - MAP_SIZE_OFFSET, y_max + MAP_SIZE_OFFSET);
    rand_gen_y.param(param_y);
  }

public:
  RRTStar(const int _iteration = 5000, const double _step = 0.1, const double _r_param = 50.0, const double _goal_select_prob = 0.2,
      const bool _use_informed_rrt = false, const double _time_out = 3.0, const double _improvement_amout = 0.01) : rand_gen_0_1(0.0, 1.0){
    collision_radius = 0.15;
    std::random_device seed_gen;
    SetIteration(_iteration);
    SetTimeOut(_time_out);
    SetImprovementAmount(_improvement_amout);
    SetStep(_step);
    SetRParam(_r_param);
    SetGoalSelectProbability(_goal_select_prob);
    SetUseInformedRRT(_use_informed_rrt);
    engine = new std::default_random_engine(seed_gen());
    ret_matches_cache.reserve(iteration >> 1);
  };
  virtual ~RRTStar(){
    delete engine;
  };

  virtual inline void SetGridMap(const ObstacleGridMap &_grid_map, const double _collision_radius){
    collision_radius = _collision_radius;
    grid_map = _grid_map;
    grid_map.Inflate(_collision_radius);
    InitRandGenerator();
  };
  virtual inline void SetGridMap(const nav_msgs::OccupancyGrid &_ocp_msg, const double _collision_radius){
    collision_radius = _collision_radius;
    grid_map.InputOccupancyGrid(_ocp_msg);
    grid_map.Inflate(_collision_radius);
    InitRandGenerator();
  };

  inline const ObstacleGridMap* GetGridMap() const { return &grid_map; };
  inline double GetCollisionRadius() const { return collision_radius; };

  inline void SetIteration(const int _iteration){ iteration = _iteration; };
  inline void SetImprovementAmount(const double _improvement_amout){ improvement_amount = _improvement_amout; };
  inline void SetTimeOut(const double _time_out){ time_out = _time_out; };
  inline void SetStep(const double _step){ step = _step; };
  inline void SetRParam(const int _r_param){ r_param = _r_param; };
  inline void SetGoalSelectProbability(const int _goal_select_prob){ goal_select_prob = _goal_select_prob; };
  inline void SetUseInformedRRT(const bool _use_informed_rrt){ use_informed_rrt = _use_informed_rrt; };

  bool Generate(const geometry_msgs::Pose2D &_start, const geometry_msgs::Pose2D &_goal, std::vector<geometry_msgs::Pose2D> &_result){
    _result.clear();
    node_list.list.clear();
    goal_list.clear();
    min_path.clear();
    min_goal = nullptr;

    if(!CheckCollisionForEdge(_start, _goal)){//例外 直線でコリジョンないならそれを返却
      _result.emplace_back(_start);
      _result.emplace_back(_goal);
      double theta = std::atan2(_goal.y - _start.y, _goal.x - _start.x);
      _result[0].theta = theta;
      _result[1].theta = theta;
      return true;
    }

    _result.reserve(iteration / 10);
    node_list.list.reserve(iteration);
    goal_list.reserve(iteration / 100);
    min_path.reserve(iteration / 100);
   
    NanoFlannIndexType nano_flann_index(2, node_list, nanoflann::KDTreeSingleIndexAdaptorParams(8));

    Node::Ptr start_ptr = std::make_shared<Node>(0.0, _start);
    node_list.list.emplace_back(start_ptr);
    nano_flann_index.addPoints(node_list.list.size() - 1, node_list.list.size() - 1);
    
    std::vector<Node::Ptr> near_node_list;
    std::vector<double> edge_cost_list;
    std::vector<double> cost_list;
    near_node_list.reserve(100);
    cost_list.reserve(100);
    edge_cost_list.reserve(100);

    double time = 0.0;
    double bef_min_cost = DBL_MAX;
    ros::WallTime st = ros::WallTime::now();
    for(int i = 0; i < iteration; i++){
      time = (ros::WallTime::now() - st).toSec();
      if(time > time_out)
        break;

      geometry_msgs::Pose2D rand_pose;
      if(min_goal == nullptr || !use_informed_rrt)
        rand_pose = RandomSampling(_goal);
      else
        rand_pose = EllipsoidalSampling(_start, _goal, CalcCost(min_goal));
      Node::Ptr new_node = CreateNewNode(rand_pose, nano_flann_index);
      if(new_node == nullptr){
        i--;
        continue;
      }
      
      int min_idx = CreateNearNodeList(*new_node, nano_flann_index, near_node_list, edge_cost_list, cost_list);
      if(min_idx != -1){
        new_node->prev = near_node_list[min_idx];
        new_node->cost = edge_cost_list[min_idx];
        Rewire(new_node, near_node_list, edge_cost_list, cost_list);
      }

      node_list.list.emplace_back(new_node); //追加
      nano_flann_index.addPoints(node_list.list.size() - 1, node_list.list.size() - 1);

      if(Distance(new_node->pose, _goal) < step)
        goal_list.emplace_back(new_node);
      
      double min_cost = DBL_MAX;
      for(int i = 0; i < goal_list.size(); i++){
        double tmp_cost = CalcCost(goal_list[i]);
        if(tmp_cost < min_cost)
          min_cost = tmp_cost;
      }
      if(min_cost < bef_min_cost){
        if(bef_min_cost - min_cost < improvement_amount)
          break;
        bef_min_cost = min_cost;
      }
    }
    
    double min_cost = DBL_MAX;
    for(int i = 0; i < goal_list.size(); i++){
      double tmp_cost = CalcCost(goal_list[i]);
      if(tmp_cost < min_cost){
        min_goal = goal_list[i];
        min_cost = tmp_cost;
      }
    }
    if(min_goal != nullptr){
      GeneratePath(min_goal, min_path);
      min_path.back().theta = std::atan2(_goal.y - min_path.back().y, _goal.x - min_path.back().x);
      min_path.emplace_back(_goal);
      PathCleanup(min_path, _result);
      return true;
    }
    else{
      return false;
    }
  };

  void GenerateVisualizationMsg(const std_msgs::Header &_header, visualization_msgs::Marker &_nodes, 
      visualization_msgs::Marker &_edges, visualization_msgs::Marker &_result){

    _nodes.header = _header;
    _edges.header = _header;
    _result.header = _header;

    _nodes.ns = "rrt_nodes";
    _edges.ns = "rrt_edges";
    _result.ns = "rrt_result";
    _nodes.action = _edges.action = _result.action = visualization_msgs::Marker::ADD;
    _nodes.pose.orientation.w = _edges.pose.orientation.w = _result.pose.orientation.w = 1.0;
    _nodes.id = 0;
    _edges.id = 1;
    _result.id = 2;

    _nodes.type = visualization_msgs::Marker::POINTS;
    _edges.type = visualization_msgs::Marker::LINE_LIST;
    _result.type = visualization_msgs::Marker::LINE_LIST;

    _nodes.scale.x = 0.05;
    _nodes.scale.y = 0.05;
    _edges.scale.x = 0.01;
    _result.scale.x = 0.05;

    _nodes.color.g = 1.0;
    _nodes.color.a = 0.3;
    _edges.color.b = 1.0;
    _edges.color.a = 0.1;
    _result.color.r = 1.0;
    _result.color.a = 0.5;

    _nodes.points.clear(); _edges.points.clear(); _result.points.clear();
    _nodes.points.reserve(node_list.list.size());
    _edges.points.reserve(node_list.list.size() * 2);
    _result.points.reserve(node_list.list.size() / 10);
    for(int i = 0; i < node_list.list.size(); i++){
      geometry_msgs::Point p;
      p.x = node_list.list[i]->pose.x; p.y = node_list.list[i]->pose.y; p.z = 0.0;
      _nodes.points.emplace_back(p);
      if(node_list.list[i]->prev != nullptr){
        geometry_msgs::Point p2;
        p2.x = node_list.list[i]->prev->pose.x; p2.y = node_list.list[i]->prev->pose.y; p2.z = 0.0;
        _edges.points.emplace_back(p);
        _edges.points.emplace_back(p2);
      }
    }
    
    if(min_path.empty())
      return;

    for(int i = 0; i < min_path.size() - 1; i++){
      geometry_msgs::Point p;
      p.x = min_path[i].x; p.y = min_path[i].y; p.z = 0.0;
      geometry_msgs::Point p2;
      double dist = Distance(min_path[i], min_path[i + 1]);
      p2.x = min_path[i].x + dist * std::cos(min_path[i].theta);
      p2.y = min_path[i].y + dist * std::sin(min_path[i].theta);
      p2.z = 0.0;
      _result.points.emplace_back(p);
      _result.points.emplace_back(p2);
    }
  };
};

#endif
