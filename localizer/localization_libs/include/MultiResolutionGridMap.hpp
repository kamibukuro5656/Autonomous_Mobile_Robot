#ifndef MULTIRESOLUTIONGRIDMAP
#define MULTIRESOLUTIONGRIDMAP

#include <ProbabilityGridMap.hpp>

class MultiResolutionGridMap{
  std::vector<ProbabilityGridMap> maps;
  int depth;

public:
  MultiResolutionGridMap(const int _depth = 4, const double _cell_size = 0.05, const double _x_min = -20.0, const double _x_max = 20.0, 
      const double _y_min = -20.0, const double _y_max = 20.0,
      const double _prob_hit = 0.51, const double _prob_miss = 0.49){
    InitGridMaps(_depth, _cell_size, _x_min, _x_max, _y_min, _y_max, _prob_hit, _prob_miss);
  };

  ~MultiResolutionGridMap(){};

  void InitGridMaps(const int _depth, const double _cell_size, const double _x_min, const double _x_max, 
      const double _y_min, const double _y_max,
      const double _prob_hit, const double _prob_miss){
    if(depth <= 0)
      depth = 1;
    depth = _depth;
    maps.clear();
    maps.reserve(depth);
    int rate = 1;
    for(int i = 0; i < depth; i++){
      maps.emplace_back(ProbabilityGridMap(_cell_size * ((double)rate), _x_min, _x_max, _y_min, _y_max, _prob_hit, _prob_miss));
      rate *= 2;
    }
  };

  inline const ProbabilityGridMap* GetGridMap(int _depth) const{
    if(_depth >= depth || _depth < 0)
      return NULL;
    else
      return &maps[_depth];
  };

  int GetDepth() const{
    return depth;
  }

  bool Save(std::string _directory_path, std::string _name_header){
    bool success = true;
    int rate = 1;
    for(int i = 0; i < depth; i++){
      std::string file_full_path = _directory_path + _name_header + "_1_" + std::to_string(rate) + ".map";
      success = success & maps[i].Save(file_full_path);
      rate *= 2;
    }
    return success;
  };

  int Load(int _depth, std::string _directory_path, std::string _name_header){
    depth = _depth;
    maps.clear();
    maps.resize(depth);

    bool success = true;
    int rate = 1;
    for(int i = 0; i < depth; i++){
      std::string file_full_path = _directory_path + _name_header + "_1_" + std::to_string(rate) + ".map";
      success &= maps[i].Load(file_full_path);
      rate *= 2;
    }
    return success;
  };

  void ManualDrawLine(const geometry_msgs::Point &_begin, const geometry_msgs::Point &_end, const double _prob){
    for(int i = 0; i < depth; i++){
      maps[i].ManualDrawLine(_begin, _end, _prob);
    }
  }

  void Convert2OccupancyGrid(const std_msgs::Header &_header, nav_msgs::OccupancyGrid &_ocp_msg) const {
    maps[0].Convert2OccupancyGrid(_header, _ocp_msg);
  };

  void AddPointCloud(ClassLPoint2DArray &_input, geometry_msgs::Pose2D _center){
    for(int i = 0; i < depth; i++){
      maps[i].AddPointCloud(_input, _center);
    }
  };
};

#endif
