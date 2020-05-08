#ifndef RRTSTARUSINGPOTENTIAL
#define RRTSTARUSINGPOTENTIAL

#include <RRTStar.hpp>
#include <PotentialGridMap.hpp>

class RRTStarUsingPotential : public RRTStar{
protected:
  static const double POTENTIAL_GAIN_OBSTACLE;
  static const double POTENTIAL_GAIN_RADIUS;

  PotentialGridMap potential_grid_map;
  double potential_gain;
  
  double CalcPotentialCost(const geometry_msgs::Pose2D &_st_pose, const geometry_msgs::Pose2D &_end_pose) const {
    int idx_st_x, idx_st_y, idx_end_x, idx_end_y;
    potential_grid_map.CalcIdx(_st_pose.x, _st_pose.y, idx_st_x, idx_st_y);
    potential_grid_map.CalcIdx(_end_pose.x, _end_pose.y, idx_end_x, idx_end_y);
  
    int idx_w, idx_h;
    potential_grid_map.GetMapSizePixel(idx_w, idx_h);

    if(idx_st_x < 0 || idx_st_x >= idx_w || idx_st_y < 0 || idx_st_y >= idx_h)
      return DBL_MAX;
    if(idx_end_x < 0 || idx_end_x >= idx_w || idx_end_y < 0 || idx_end_y >= idx_h)
      return DBL_MAX;

    const std::vector<double> *map_ptr = potential_grid_map.GetMap();
    
    int dx = idx_end_x - idx_st_x, dy = idx_end_y - idx_st_y;
    int x = idx_st_x, y = idx_st_y;
    
    int x_add, y_add;
    if(idx_st_x < idx_end_x) x_add = 1;
    else x_add = -1;
    if(idx_st_y < idx_end_y) y_add = 1;
    else y_add = -1;
    
    double cost = 0.0;
    int dx2 = 2 * dx * x_add, dy2 = 2 * dy * y_add;
    if(std::abs(dx) >= std::abs(dy)){
      int d = -dx * x_add;
      while(x != idx_end_x){
 	      if(d > 0){
          y += y_add;
  		    d -= dx2;
 	      }
        d += dy2;
        cost += (*map_ptr)[x + y * idx_w];
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
        cost += (*map_ptr)[x + y * idx_w];
        y += y_add;
      }
    }
    return cost;
  }

  virtual inline double CalcEdgeCost(const geometry_msgs::Pose2D &_st, const geometry_msgs::Pose2D &_end) const override {
    double cost = CalcPotentialCost(_st, _end) * potential_gain;
    cost += Distance(_st, _end);

    return cost;
  };

public:
  RRTStarUsingPotential(const int _iteration = 5000, const double _step = 0.1, 
      const double _r_param = 50.0, const double _goal_select_prob = 0.2,
      const bool _use_informed_rrt = false, const double _time_out = 3.0, 
      const double _improvement_amout = 0.01, const double _potential_gain = 0.005) :
    RRTStar(_iteration, _step, _r_param, _goal_select_prob, _use_informed_rrt, _time_out, _improvement_amout){
    SetPotentialGain(_potential_gain);
  };
  virtual ~RRTStarUsingPotential(){
  };

  inline void SetPotentialGain(const double _potential_gain){ potential_gain = _potential_gain; };

  virtual inline void SetGridMap(const ObstacleGridMap &_grid_map, const double _collision_radius) override {
    collision_radius = _collision_radius;
    grid_map = _grid_map;
    grid_map.Inflate(_collision_radius);
    potential_grid_map.SetGainObstacle(POTENTIAL_GAIN_OBSTACLE);
    potential_grid_map.SetGainRadius(POTENTIAL_GAIN_RADIUS);
    potential_grid_map.SetGainGoal(0.0);
    potential_grid_map.GeneratePotential(grid_map);
    RRTStar::InitRandGenerator();
  };
  virtual inline void SetGridMap(const nav_msgs::OccupancyGrid &_ocp_msg, const double _collision_radius) override {
    collision_radius = _collision_radius;
    grid_map.InputOccupancyGrid(_ocp_msg);
    grid_map.Inflate(_collision_radius);
    potential_grid_map.SetGainObstacle(POTENTIAL_GAIN_OBSTACLE);
    potential_grid_map.SetGainRadius(POTENTIAL_GAIN_RADIUS);
    potential_grid_map.SetGainGoal(0.0);
    potential_grid_map.GeneratePotential(grid_map);
    RRTStar::InitRandGenerator();
  };
};

#endif
