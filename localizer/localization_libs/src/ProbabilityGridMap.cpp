#include <ProbabilityGridMap.hpp>

const double ProbabilityGridMap::PROB_INIT_VAL = 0.5;
const double ProbabilityGridMap::PROB_LIMIT_MIN = 0.001;
const double ProbabilityGridMap::PROB_LIMIT_MAX = 0.999;

inline double CalcBicubicWeight(double _d, double _a){
  if(_d <= 1.0)
    return ((_a + 2.0) * (_d * _d * _d)) - ((_a + 3.0) * (_d * _d)) + 1.0;
  else if(_d <= 2.0)
    return (_a * (_d * _d * _d)) - (5.0 * _a * (_d * _d)) + (8.0 * _a * _d) - (4.0 * _a);
  else
    return 0.0;
}

inline double CalcBicubic(double _x_pix, double _y_pix, double _a, const Eigen::Matrix4d &_pixels){
  double sub_x = _x_pix - std::floor(_x_pix);
  double sub_y = _y_pix - std::floor(_y_pix);
  
  Eigen::Matrix<double, 4, 1> weight_x;
  Eigen::Matrix<double, 1, 4> weight_y;

  weight_x(0, 0) = CalcBicubicWeight(1.0 + sub_x, _a);
  weight_x(1, 0) = CalcBicubicWeight(sub_x, _a);
  weight_x(2, 0) = CalcBicubicWeight(-sub_x + 1.0, _a);
  weight_x(3, 0) = CalcBicubicWeight(-sub_x + 2.0, _a);

  weight_y(0, 0) = CalcBicubicWeight(1.0 + sub_y, _a);
  weight_y(0, 1) = CalcBicubicWeight(sub_y, _a);
  weight_y(0, 2) = CalcBicubicWeight(-sub_y + 1.0, _a);
  weight_y(0, 3) = CalcBicubicWeight(-sub_y + 2.0, _a);

  return weight_y * _pixels * weight_x;
}

void ProbabilityGridMap::AddPointCloud(const ClassLPoint2DArray &_input, const geometry_msgs::Pose2D &_center){
  localization_msgs::LPoint2D center;
  center.x = _center.x;
  center.y = _center.y;

  for(size_t i = 0; i < _input.pcd.points.size(); i++){
    UpdateBresenhamLine(center, _input.pcd.points[i]);
  }
  counter++;
}

double ProbabilityGridMap::GetProbability(const double _x, const double _y, const double _param, bool &_is_not_observed) const{
  Eigen::Matrix4d pixels;
  double idx_x, idx_y;
  CalcIdx(_x, _y, idx_x, idx_y);

  int int_idx_x = (int)idx_x, int_idx_y = (int)idx_y;

  int sum = 0;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      pixels(j, i) = GetValue(int_idx_x + i - 1, int_idx_y + j - 1);
      sum += GetCount(int_idx_x + i - 1, int_idx_y + j - 1);
    }
  }

  if(sum == COUNT_INIT_VAL * 16){
    _is_not_observed = true;
    return 0.5;
  }
  else{
    _is_not_observed = false;
    double tmp = CalcBicubic(idx_x, idx_y, _param, pixels);
    if(tmp < 0.0) tmp = 0.0;
    if(tmp > 1.0) tmp = 1.0;
    return tmp;
  }
}

void ProbabilityGridMap::Convert2OccupancyGrid(const std_msgs::Header &_header, nav_msgs::OccupancyGrid &_ocp_grid) const {
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
      double per = map[i] * 100.0;
      if(per > 100.0) per = 100.0;
      _ocp_grid.data[i] = (int8_t)(per + 0.5);
    }
    else{
      _ocp_grid.data[i] = -1;
    }
  }
}

void ProbabilityGridMap::InputOccupancyGrid(const nav_msgs::OccupancyGrid &_ocp_grid)
{
  double tmp_w = (double)(_ocp_grid.info.width) * _ocp_grid.info.resolution;
  double tmp_h = (double)(_ocp_grid.info.height) * _ocp_grid.info.resolution;
  double origin_x = _ocp_grid.info.origin.position.x;
  double origin_y = _ocp_grid.info.origin.position.y;
  
  cell_size = _ocp_grid.info.resolution;
  x_min = origin_x; x_max = origin_x + tmp_w; w = tmp_w;
  y_min = origin_y; y_max = origin_y + tmp_h; h = tmp_h;
  idx_w = _ocp_grid.info.width; idx_h = _ocp_grid.info.height;

  map.clear();
  count_map.clear();
  map.resize(idx_w * idx_h);
  count_map.resize(idx_w * idx_h);

  for(int i = 0; i < _ocp_grid.data.size(); i++){
    if(_ocp_grid.data[i] <= -1 || _ocp_grid.data[i] > 100){
      count_map[i] = COUNT_INIT_VAL;
      map[i] = PROB_INIT_VAL;
    }
    else{
      count_map[i] = counter;
      map[i] = (double)_ocp_grid.data[i] * 0.01;
      if(map[i] > PROB_LIMIT_MAX)
        map[i] = PROB_LIMIT_MAX;
      else if(map[i] < PROB_LIMIT_MIN)
        map[i] = PROB_LIMIT_MIN;
    }
  }

  counter++;
}

inline void ProbabilityGridMap::UpdateBresenhamLine(const localization_msgs::LPoint2D &_begin, const localization_msgs::LPoint2D &_end){
  int idx_st_x, idx_st_y, idx_end_x, idx_end_y;
  CalcIdx(_begin.x, _begin.y, idx_st_x, idx_st_y);
  CalcIdx(_end.x, _end.y, idx_end_x, idx_end_y);

  UpdateGrid(idx_end_x, idx_end_y, true);

  int dx = idx_end_x - idx_st_x;
  int dy = idx_end_y - idx_st_y;

  int x = idx_st_x;
  int y = idx_st_y;
    
  int x_add, y_add;
  if(idx_st_x < idx_end_x) x_add = 1;
  else x_add = -1;
  if(idx_st_y < idx_end_y) y_add = 1;
  else y_add = -1;
    
  int dx2 = 2 * dx * x_add;
  int dy2 = 2 * dy * y_add;
  if(std::abs(dx) >= std::abs(dy)){
    int d = -dx * x_add;
    while(x != idx_end_x){
 	  if(d > 0){
        y += y_add;
  		  d -= dx2;
 	  }
      d += dy2;
      UpdateGrid(x, y, false);
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
  	  UpdateGrid(x, y, false);
      y += y_add;
    }
  }
}

bool ProbabilityGridMap::Save(const std::string &_path)
{
  std::fstream file(_path, std::ios::binary | std::ios::out | std::ios::trunc);
  if(!file){
    return false;
  }

  char header[8] = "GridMap";
  file.write((char *)header, sizeof(char) * 8);
  file.write((char *)&map_num, sizeof(int));
  file.write((char *)&cell_size, sizeof(double));
  file.write((char *)&x_min, sizeof(double));
  file.write((char *)&x_max, sizeof(double));
  file.write((char *)&w, sizeof(double));
  file.write((char *)&y_min, sizeof(double));
  file.write((char *)&y_max, sizeof(double));
  file.write((char *)&h, sizeof(double));
  file.write((char *)&idx_w, sizeof(int));
  file.write((char *)&idx_h, sizeof(int));
  file.write((char *)&counter, sizeof(int));

  for(int i = 0; i < map.size(); i++){
    file.write((char *)&map[i], sizeof(double));
  }
  for(int i = 0; i < count_map.size(); i++){
    file.write((char *)&count_map[i], sizeof(int));
  }

  return true;
}

bool ProbabilityGridMap::Load(const std::string &_path)
{
  std::fstream file(_path, std::ios::binary | std::ios::in);
  if(!file){
    return false;
  }

  char header[8];
  file.read((char *)header, sizeof(char) * 8);
  if(std::strcmp(header, "GridMap") != 0){
    return false;
  }

  file.read((char *)&map_num, sizeof(int));
  file.read((char *)&cell_size, sizeof(double));
  file.read((char *)&x_min, sizeof(double));
  file.read((char *)&x_max, sizeof(double));
  file.read((char *)&w, sizeof(double));
  file.read((char *)&y_min, sizeof(double));
  file.read((char *)&y_max, sizeof(double));
  file.read((char *)&h, sizeof(double));
  file.read((char *)&idx_w, sizeof(int));
  file.read((char *)&idx_h, sizeof(int));
  file.read((char *)&counter, sizeof(int));
  
  InitGridMap(cell_size, x_min, x_max, y_min, y_max, PROB_INIT_VAL);

  for(int i = 0; i < map.size(); i++){
    file.read((char *)&map[i], sizeof(double));
  }
  for(int i = 0; i < count_map.size(); i++){
    file.read((char *)&count_map[i], sizeof(int));
  }

  return true;
}

void ProbabilityGridMap::ManualDrawLine(const geometry_msgs::Point &_begin, const geometry_msgs::Point &_end, const double _prob){
  int idx_st_x, idx_st_y, idx_end_x, idx_end_y;
  CalcIdx(_begin.x, _begin.y, idx_st_x, idx_st_y);
  CalcIdx(_end.x, _end.y, idx_end_x, idx_end_y);

  int dx = idx_end_x - idx_st_x;
  int dy = idx_end_y - idx_st_y;

  int x = idx_st_x;
  int y = idx_st_y;
    
  int x_add, y_add;
  if(idx_st_x < idx_end_x) x_add = 1;
  else x_add = -1;
  if(idx_st_y < idx_end_y) y_add = 1;
  else y_add = -1;
    
  int dx2 = 2 * dx * x_add;
  int dy2 = 2 * dy * y_add;
  if(std::abs(dx) >= std::abs(dy)){
    int d = -dx * x_add;
    while(x != idx_end_x){
 	  if(d > 0){
        y += y_add;
  		  d -= dx2;
 	  }
      d += dy2;
      map[x + y * idx_w] = _prob;
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
      map[x + y * idx_w] = _prob;
      y += y_add;
    }
  }
}
