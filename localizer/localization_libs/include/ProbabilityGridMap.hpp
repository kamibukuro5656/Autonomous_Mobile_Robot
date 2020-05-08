#ifndef PROBABILITYGRIDMAP
#define PROBABILITYGRIDMAP

#include <BaseGridMap.hpp>
#include <LPoint2DArrayClass.hpp>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <fstream>
#include <cstring>

class ProbabilityGridMap : public BaseGridMap<double>{
protected:
  double prob_hit, prob_miss;
  double prob_fix_threshold;
  double odds_hit, odds_miss;

public:
  static const double PROB_INIT_VAL;
  static const double PROB_LIMIT_MIN;
  static const double PROB_LIMIT_MAX;
  
  typedef std::shared_ptr<ProbabilityGridMap> Ptr;
  typedef std::shared_ptr<const ProbabilityGridMap> ConstPtr;

  ProbabilityGridMap(const double _cell_size = 0.05, const double _x_min = -20.0, const double _x_max = 20.0, 
      const double _y_min = -20.0, const double _y_max = 20.0,
      const double _prob_hit = 0.51, const double _prob_miss = 0.49, const double _prob_fix_threshold = 0.95) : 
    BaseGridMap(PROB_INIT_VAL, _cell_size, _x_min, _x_max, _y_min, _y_max){
    SetProbHitMiss(_prob_hit, _prob_miss);
    SetProbFixThreshold(_prob_fix_threshold);
  };

  virtual ~ProbabilityGridMap(){};

  inline void SetMapNum(const int _map_num){ map_num = _map_num; };
  inline int GetMapNum() const { return map_num; };
  inline void SetProbHitMiss(const double _prob_hit, const double _prob_miss){
    prob_hit = _prob_hit;
    if(prob_hit > PROB_LIMIT_MAX)
      prob_hit = PROB_LIMIT_MAX;
    prob_miss = _prob_miss;
    if(prob_miss < PROB_LIMIT_MIN)
      prob_miss = PROB_LIMIT_MIN;
    odds_hit = prob_hit / (1 - prob_hit);
    odds_miss = prob_miss / (1 - prob_miss);
  };
  inline void SetProbFixThreshold(const double _prob_fix_threshold){
    if(_prob_fix_threshold < 0.0)
      prob_fix_threshold = PROB_LIMIT_MAX + 0.1; //固定しない
    else
      prob_fix_threshold = _prob_fix_threshold;
  };

  void AddPointCloud(const ClassLPoint2DArray &_input, const geometry_msgs::Pose2D &_center);
  inline void AddPointCloud(const ClassLPoint2DArray &_input, const tf::StampedTransform &_center){
    geometry_msgs::Pose2D center;
    center.x = _center.getOrigin().x();
    center.y = _center.getOrigin().y();
    AddPointCloud(_input, center);
  };
  double GetProbability(const double _x, const double _y, const double _param, bool &_is_not_observed) const;
  ProbabilityGridMap::Ptr CutOutMap(const double _cell_size, const double _x_min, const double _x_max,
      const double _y_min, const double _y_max, const double _threshold = -1.0) const {
    ProbabilityGridMap::Ptr res = std::make_shared<ProbabilityGridMap>(_cell_size, _x_min, _x_max, _y_min, _y_max);
    for(int y = 0; y < res->idx_h; y++){
      for(int x = 0; x < res->idx_w; x++){
        double x_dbl = ((double)x) * res->cell_size + res->x_min;
        double y_dbl = ((double)y) * res->cell_size + res->y_min;
        int orig_x_idx, orig_y_idx;
        this->CalcIdx(x_dbl, y_dbl, orig_x_idx, orig_y_idx);

        double prob = this->GetValue(orig_x_idx, orig_y_idx);
        if(prob > _threshold)
          res->map[x + res->idx_w * y] = prob;
        else
          res->map[x + res->idx_w * y] = PROB_INIT_VAL;

        int count = this->GetCount(orig_x_idx, orig_y_idx);
        if(count == COUNT_INIT_VAL)
          res->count_map[x + res->idx_w * y] = COUNT_INIT_VAL;
        else
          res->count_map[x + res->idx_w * y] = res->counter;
      }
    }
    res->counter++;
    return res;
  };
  void Convert2OccupancyGrid(const std_msgs::Header &_header, nav_msgs::OccupancyGrid &_ocp_grid) const;
  void InputOccupancyGrid(const nav_msgs::OccupancyGrid &_ocp_grid);
  bool Save(const std::string &_path);
  bool Load(const std::string &_path);
    
  void ManualDrawLine(const geometry_msgs::Point &_begin, const geometry_msgs::Point &_end, const double _prob); //手動修正用

private:
  void UpdateBresenhamLine(const localization_msgs::LPoint2D &_begin, const localization_msgs::LPoint2D &_end);

  inline double CalcProb(const double _old_prob, const bool _is_hit) const{
    double new_odds;
    double old_odds = _old_prob / (1 - _old_prob);
    if(_is_hit)
      new_odds = old_odds * odds_hit;
    else
      new_odds = old_odds * odds_miss;

    double new_prob = new_odds / (1 + new_odds);
    if(PROB_LIMIT_MIN > new_prob)
      new_prob = PROB_LIMIT_MIN;
    else if(PROB_LIMIT_MAX < new_prob)
      new_prob = PROB_LIMIT_MAX;

    return new_prob;
  };

  inline void UpdateGrid(const int _x, const int _y, const bool _is_hit){
    if(_x < 0 || _x >= idx_w || _y < 0 || _y >= idx_h)
      return;
    int idx = _x + _y * idx_w;
    if(count_map[idx] != counter || _is_hit){
      if(map[idx] < prob_fix_threshold || _is_hit){
        map[idx] = CalcProb(map[idx], _is_hit);
        count_map[idx] = counter;
      }
    }
  };
};

#endif
