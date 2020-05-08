#ifndef OBSTACLEGRIDMAP
#define OBSTACLEGRIDMAP

#include <BaseGridMap.hpp>
#include <ProbabilityGridMap.hpp>

class ObstacleGridMap : public BaseGridMap<int8_t>{
protected:

public:
  static const int8_t INIT_VAL;
  static const int8_t FREE;
  static const int8_t EXIST;
  static const int8_t OCP_CONVERT_THRESHOLD;
  
  typedef std::shared_ptr<ObstacleGridMap> Ptr;
  typedef std::shared_ptr<const ObstacleGridMap> ConstPtr;

  ObstacleGridMap(const double _cell_size = 0.05, const double _x_min = -20.0, const double _x_max = 20.0, 
      const double _y_min = -20.0, const double _y_max = 20.0) :
    BaseGridMap(INIT_VAL, _cell_size, _x_min, _x_max, _y_min, _y_max){
  };
  virtual ~ObstacleGridMap(){};

  void InputProbabilityMap(const ProbabilityGridMap &_prob_map, const double _prob_threshold){
    double tmp_cell_size, tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y;
    tmp_cell_size = _prob_map.GetCellSize();
    _prob_map.GetMapSize(tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y);
    InitGridMap(tmp_cell_size, tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y, INIT_VAL);

    const std::vector<double> *prob_map_ptr = _prob_map.GetMap();
    const std::vector<int> *count_map_ptr = _prob_map.GetCountMap();
    for(int i = 0; i < prob_map_ptr->size(); i++){
      if((*count_map_ptr)[i] == COUNT_INIT_VAL){
        count_map[i] = COUNT_INIT_VAL;
        map[i] = INIT_VAL;
      }
      else{
        count_map[i] = (*count_map_ptr)[i];
        if((*prob_map_ptr)[i] > _prob_threshold)
          map[i] = EXIST;
        else
          map[i] = FREE;
      }
    }
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

    for(int i = 0; i < _ocp_grid.data.size(); i++){
      if(count_map[i] != COUNT_INIT_VAL){
        _ocp_grid.data[i] = map[i];
      }
      else{
        _ocp_grid.data[i] = -1;
      }
    }
  };

  void InputOccupancyGrid(const nav_msgs::OccupancyGrid &_ocp_grid){
    double tmp_w = (double)(_ocp_grid.info.width) * _ocp_grid.info.resolution;
    double tmp_h = (double)(_ocp_grid.info.height) * _ocp_grid.info.resolution;
    double origin_x = _ocp_grid.info.origin.position.x;
    double origin_y = _ocp_grid.info.origin.position.y;
  
    cell_size = _ocp_grid.info.resolution;
    x_min = origin_x; x_max = origin_x + tmp_w; w = tmp_w;
    y_min = origin_y; y_max = origin_y + tmp_h; h = tmp_h;
    idx_w = _ocp_grid.info.width; idx_h = _ocp_grid.info.height;

    map.resize(idx_w * idx_h);
    count_map.resize(idx_w * idx_h);

    for(int i = 0; i < _ocp_grid.data.size(); i++){
      if(_ocp_grid.data[i] <= -1){
        count_map[i] = COUNT_INIT_VAL;
        map[i] = INIT_VAL;
      }
      else{
        count_map[i] = counter;
        if(_ocp_grid.data[i] > OCP_CONVERT_THRESHOLD)
          map[i] = EXIST;
        else
          map[i] = FREE;
      }
    }
    counter++;
  };

  void Inflate(const double _radius){
    double radius = _radius + cell_size;
    std::vector<std::pair<int, int>> circle_idxs;
    int idx_radius = (int)(radius / cell_size);
 
    bool collision = false;
    for(int y = -idx_radius; y <= idx_radius; y++){
      for(int x = -idx_radius; x <= idx_radius; x++){
        double dist = std::sqrt((double)(x * x + y * y)) * cell_size;
        if(radius >= dist)
          circle_idxs.emplace_back(std::pair<int, int>(x, y));
      }
    }
 
    std::vector<int8_t> tmp_map;
    int tmp_w = idx_w + idx_radius * 2, tmp_h = idx_h + idx_radius * 2;
    tmp_map.resize(tmp_w * tmp_h);
    for(int y = 0; y < idx_h; y++){
      for(int x = 0; x < idx_w; x++){
        if(map[x + y * idx_w] == EXIST){
          for(int i = 0; i < circle_idxs.size(); i++){
             int tmp_idx_x = x + circle_idxs[i].first + idx_radius;
             int tmp_idx_y = y + circle_idxs[i].second + idx_radius;
             tmp_map[tmp_idx_x + tmp_idx_y * tmp_w] = EXIST;
          }
        }
      }
    }

    for(int y = 0; y < idx_h; y++){
      for(int x = 0; x < idx_w; x++){
        int idx = x + y * idx_w;
        int tmp_idx_x = x + idx_radius;
        int tmp_idx_y = y + idx_radius;
        int tmp_idx = tmp_idx_x + tmp_idx_y * tmp_w;
        map[idx] = tmp_map[tmp_idx];
        if(tmp_map[tmp_idx] == EXIST){
          map[idx] = EXIST;
          count_map[idx] = counter;
        }
      }
    }
  };
  void AddMap(const ObstacleGridMap &_grid_map){
    for(int y = 0; y < _grid_map.idx_h; y++){
      for(int x = 0; x < _grid_map.idx_w; x++){
        double x_dbl = ((double)x) * _grid_map.cell_size + _grid_map.x_min;
        double y_dbl = ((double)y) * _grid_map.cell_size + _grid_map.y_min;
        int this_x, this_y;
        this->CalcIdx(x_dbl, y_dbl, this_x, this_y);
        if(_grid_map.count_map[x + y * _grid_map.idx_w] != COUNT_INIT_VAL){
          int8_t val =  _grid_map.map[x + y * _grid_map.idx_w];
          if((this->GetCount(this_x, this_y) == COUNT_INIT_VAL && val != INIT_VAL) || val == EXIST)
            this->SetValue(this_x, this_y, val);
        }
      }
    }
    counter++;
  };
};

#endif
