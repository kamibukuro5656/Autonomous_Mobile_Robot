#ifndef BASEGRIDMAP
#define BASEGRIDMAP

#include <LPoint2DArrayClass.hpp>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <fstream>
#include <cstring>

template <typename T>
class BaseGridMap{
protected:
  T init_val;
  int map_num; //Subマップ使いたくなったとき用
  double cell_size;
  double x_min, x_max, w;
  double y_min, y_max, h;
  int idx_w, idx_h;
  int counter;

  std::vector<T> map;
  std::vector<int> count_map;

public:
  static const int COUNT_INIT_VAL;
  
  typedef std::shared_ptr<BaseGridMap> Ptr;
  typedef std::shared_ptr<const BaseGridMap> ConstPtr;

  BaseGridMap(const T _init_val, const double _cell_size = 0.05, const double _x_min = -20.0, const double _x_max = 20.0, 
      const double _y_min = -20.0, const double _y_max = 20.0){
    counter = 0;
    map_num = 0;
    init_val = _init_val;
    InitGridMap(_cell_size, _x_min, _x_max, _y_min, _y_max, _init_val);
  };

  virtual ~BaseGridMap(){};

  inline void SetMapNum(const int _map_num){ map_num = _map_num; };
  inline int GetMapNum() const { return map_num; };
  inline int GetNowCount() const { return counter; };
  inline const std::vector<T>* GetMap() const { return &map; };
  inline const std::vector<int>* GetCountMap() const { return &count_map; };
  inline bool SetMap( std::vector<T> &_map){
    if(_map.size() == map.size()){
      map = _map;
      return true;
    }
    else
      return false;
  };
  inline bool SetCountMap( std::vector<int> &_map){
    if(_map.size() == count_map.size()){
      count_map = _map;
      return true;
    }
    else
      return false;
  };
  inline double GetCellSize() const { return cell_size; };
  inline void GetMapSize(double &_x_min, double &_x_max, double &_y_min, double &_y_max) const{
    _x_min = x_min; _x_max = x_max;
    _y_min = y_min; _y_max = y_max;
  };
  inline void GetMapSizePixel(int &_idx_w, int &_idx_h) const{
    _idx_w = idx_w; _idx_h = idx_h;
  };
  inline void SetValue(const int _x, const int _y, const T _val){
    if(_x >= 0 && _x < idx_w && _y >= 0 && _y < idx_h){
      map[_x + _y * idx_w] = _val;
      count_map[_x + _y * idx_w] = counter;
    }
  }
  inline T GetValue(const int _x, const int _y) const{
    if(_x < 0 || _x >= idx_w || _y < 0 || _y >= idx_h)
      return init_val;
    return map[_x + _y * idx_w];
  };
  inline T GetValue(const double _x, const double _y) const{
    double idx_x, idx_y;
    CalcIdx(_x, _y, idx_x, idx_y);
    int int_idx_x = (int)idx_x, int_idx_y = (int)idx_y;
    return GetValue(int_idx_x, int_idx_y);
  };
  inline int GetCount(const int _x, const int _y) const{
    if(_x < 0 || _x >= idx_w || _y < 0 || _y >= idx_h)
      return COUNT_INIT_VAL;
    return count_map[_x + _y * idx_w];
  }
  void InitGridMap(const double _cell_size, const double _x_min, const double _x_max, const double _y_min, const double _y_max, const T _init_val){
    cell_size = _cell_size;
    x_min = _x_min; x_max = _x_max; w = x_max - x_min;
    y_min = _y_min; y_max = _y_max; h = y_max - y_min;
    idx_w = (int)(w / cell_size); idx_h = (int)(h / cell_size);

    map.resize(idx_w * idx_h);
    count_map.resize(idx_w * idx_h);

    for(int i = 0; i < map.size(); i++){
      map[i] = _init_val;
      count_map[i] = COUNT_INIT_VAL;
    }
  };
   
  inline void CalcIdx(const double _x, const double _y, int &_idx_x, int &_idx_y) const{
    double tmp_x, tmp_y;
    CalcIdx(_x, _y, tmp_x, tmp_y);
    tmp_x += 0.5;
    tmp_y += 0.5;

    _idx_x = (int)(tmp_x);
    _idx_y = (int)(tmp_y);
  };

  inline void CalcIdx(const double _x, const double _y, double &_idx_x, double &_idx_y) const{
    _idx_x = ((_x - x_min) / cell_size);
    _idx_y = ((_y - y_min) / cell_size);
  };
};

template <class T>
const int BaseGridMap<T>::COUNT_INIT_VAL = -1;

#endif
