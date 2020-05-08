#ifndef POTENTIALGRIDMAP
#define POTENTIALGRIDMAP

#include <BaseGridMap.hpp>
#include <ObstacleGridMap.hpp>
#include <Utils.hpp>

class PotentialGridMap : public BaseGridMap<double>{
protected:
  const static double ABORT_THRESHOLD;

  double gain_kg;
  double gain_ko;
  double gain_radius;

  inline double RepulsivePotential(const double &_sq_dist){
    return gain_ko * std::exp(-_sq_dist * gain_radius);
  }

  void CalcRepulsivePotential(const ObstacleGridMap &_obs_map){
    int idx_radius;
    for(idx_radius = 0; idx_radius < (idx_w / 2); idx_radius++){ //一定値以下になる半径を算出
      double dist = (double)idx_radius * cell_size;
      if(RepulsivePotential(dist * dist) < ABORT_THRESHOLD)
        break;
    }
    double sq_radius = (double)idx_radius * cell_size;
    sq_radius *= sq_radius;
    
    std::vector<std::pair<double, std::pair<int, int>>> circle_idxs;
    for(int y = -idx_radius; y <= idx_radius; y++){
      for(int x = -idx_radius; x <= idx_radius; x++){
        double sq_dist = (double)(x * x + y * y) * cell_size;
        if(sq_radius >= sq_dist){
          std::pair<double, std::pair<int, int>> tmp_pair;
          tmp_pair.first = RepulsivePotential(sq_dist);
          tmp_pair.second.first = x;
          tmp_pair.second.second = y;
          circle_idxs.emplace_back(tmp_pair);
        }
      }
    }
   
    const std::vector<int8_t> *obs_map_ptr = _obs_map.GetMap();
    std::vector<double> tmp_map;
    int tmp_w = idx_w + idx_radius * 2, tmp_h = idx_h + idx_radius * 2; //半径分拡大したマップを用意
    tmp_map.resize(tmp_w * tmp_h);
    tmp_map.assign(tmp_map.size(), 0.0);
    for(int y = 0; y < idx_h; y++){
      for(int x = 0; x < idx_w; x++){
        if((*obs_map_ptr)[x + y * idx_w] == ObstacleGridMap::EXIST){
          for(int i = 0; i < circle_idxs.size(); i++){
            int tmp_x = x + circle_idxs[i].second.first + idx_radius;
            int tmp_y = y + circle_idxs[i].second.second + idx_radius;
            double val = circle_idxs[i].first;
            tmp_map[tmp_x + tmp_y * tmp_w] += val;
          }
        }
      }
    }

    const std::vector<int> *obs_count_map_ptr = _obs_map.GetCountMap();
    for(int y = 0; y < idx_h; y++){ //対象部分だけコピー
      for(int x = 0; x < idx_w; x++){
        int idx = x + y * idx_w;
        int tmp_idx_x = x + idx_radius;
        int tmp_idx_y = y + idx_radius;
        int tmp_idx = tmp_idx_x + tmp_idx_y * tmp_w;
        map[idx] = tmp_map[tmp_idx];
        if((*obs_count_map_ptr)[idx] != ObstacleGridMap::COUNT_INIT_VAL){
          count_map[idx] = counter;
        }
      }
    }
  }
  void CalcGoalPotential(const geometry_msgs::Pose2D &_goal){
    double goal_x, goal_y;
    CalcIdx(_goal.x, _goal.y, goal_x, goal_y);
    for(int y = 0; y < idx_h; y++){
      for(int x = 0; x < idx_w; x++){
        double diff_x = goal_x - (double)x;
        double diff_y = goal_y - (double)y;
        double val = gain_kg * (diff_x * diff_x + diff_y * diff_y);
        map[x + y * idx_w] += val;
      }
    }
  }
public:
  static const double INIT_VAL;

  PotentialGridMap(const double _cell_size = 0.05, const double _x_min = -20.0, const double _x_max = 20.0, 
      const double _y_min = -20.0, const double _y_max = 20.0,
      const double _gain_kg = 1.0, const double _gain_ko = 1.0, const double _gain_radius = 0.1) : 
      BaseGridMap(INIT_VAL, _cell_size, _x_min, _x_max, _y_min, _y_max){
      SetGainGoal(_gain_kg);
      SetGainObstacle(_gain_ko);
      SetGainRadius(_gain_radius);
    };
  virtual ~PotentialGridMap(){};
  
  void SetGainObstacle(const double _gain){ gain_ko = _gain; };
  void SetGainRadius(const double _gain){ gain_radius = _gain; };
  void SetGainGoal(const double _gain){ gain_kg = _gain; };
  
  void AddGoalPotential(const geometry_msgs::Pose2D &_goal){ 
    CalcGoalPotential(_goal);
  };

  bool GeneratePotential(const ObstacleGridMap &_obs_map){
    double tmp_cell_size, tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y;
    tmp_cell_size = _obs_map.GetCellSize();
    _obs_map.GetMapSize(tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y);
    InitGridMap(tmp_cell_size, tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y, INIT_VAL);

    CalcRepulsivePotential(_obs_map);
    counter++;
  };
  void Convert2OccupancyGrid(const std_msgs::Header &_header, nav_msgs::OccupancyGrid &_ocp_grid) const {
    _ocp_grid.header = _header;
    _ocp_grid.data.clear();
    _ocp_grid.data.resize(map.size());
    _ocp_grid.info.map_load_time = _header.stamp;
    _ocp_grid.info.resolution = cell_size;
    _ocp_grid.info.width = idx_w;
    _ocp_grid.info.height = idx_h;
    _ocp_grid.info.origin.position.x = x_min;
    _ocp_grid.info.origin.position.y = y_min;
    _ocp_grid.info.origin.position.z = 0.0;
    _ocp_grid.info.origin.orientation.x = 0.0;
    _ocp_grid.info.origin.orientation.y = 0.0;
    _ocp_grid.info.origin.orientation.z = 0.0;
    _ocp_grid.info.origin.orientation.w = 1.0;

    double max = *std::max_element(map.begin(), map.end());
    double min = *std::min_element(map.begin(), map.end());
    for(int i = 0; i < _ocp_grid.data.size(); i++){
      double per = ((map[i] - min) / max) * 100.0;
      if(per > 100.0) per = 100.0;
      else if(per < 0.0) per = 0.0;
      
      if(count_map[i] != COUNT_INIT_VAL)
        _ocp_grid.data[i] = (int8_t)(per + 0.5);
      else
        _ocp_grid.data[i] = -1;
    }
  }
};

#endif
