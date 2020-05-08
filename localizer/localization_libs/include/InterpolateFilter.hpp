#ifndef INTERPOLATEFILTER
#define INTERPOLATEFILTER

#include "LPoint2DFilter.hpp"
#include "LPoint2DArrayClass.hpp"
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <Eigen/Core>
#include <localization_msgs/LPoint2D.h>
#include <localization_msgs/LPoint2DArray.h>

class InterpolateFilter:public Filter{
  double dthre_s;
  double dthre_l;
  double dis;

  bool FindInterpolatePoint(const localization_msgs::LPoint2D &_cp, 
      const localization_msgs::LPoint2D &_pp, localization_msgs::LPoint2D &_np,
      bool &_inserted);

  public:
    InterpolateFilter() : dthre_s(0.025), dthre_l(0.1), dis(0.0){};
    ~InterpolateFilter(){};
    void Execute(ClassLPoint2DArray &_output);
    void SetThreshold(double _s, double _l){
      dthre_l = _l;
      dthre_s = _s;
    };
};

#endif
