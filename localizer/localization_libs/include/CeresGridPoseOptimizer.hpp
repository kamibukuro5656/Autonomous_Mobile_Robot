#ifndef CERESPOSEOPTIMIZER
#define CERESPOSEOPTIMIZER

#include <ProbabilityGridMap.hpp>
#include <MultiResolutionGridMap.hpp>
#include <LPoint2DArrayClass.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <vector>
#include <memory>

class CeresGridPoseOptimizer{
  struct RotationCostFunctor{
    RotationCostFunctor(const double _angle, const double _factor) : angle(_angle), factor(_factor){};

    template <typename T>
    bool operator()(const T* const _pose, T* _residual) const {
      _residual[0] = factor * (_pose[2] - angle);
      return true;
    };

    private:
      const double factor, angle;
  };

  struct TranslationCostFunctor{
    TranslationCostFunctor(const double _x, const double _y, const double _factor) : 
     x(_x), y(_y), factor(_factor){};

    template <typename T>
    bool operator()(const T* const _pose, T* _residual) const {
      _residual[0] = factor * (_pose[0] - x);
      _residual[1] = factor * (_pose[1] - y);
      return true;
    };

    private:
      const double factor, x, y;
  };

  struct GridCostFunctor{
    GridCostFunctor(const ProbabilityGridMap &_grid, const ClassLPoint2DArray &_scan, const double _factor = 1.0) 
      : grid_map(_grid), points(_scan), factor(_factor){};

    bool operator()(const double *const _pose, double *_residual) const {
      double tx = _pose[0]; //x
      double ty = _pose[1]; //y
      double ttheta = _pose[2]; //theta
      int counter = 0;

      for(int i = 0; i < points.pcd.points.size(); i++){
        double point_x = points.pcd.points[i].x;
        double point_y = points.pcd.points[i].y;
        double transformed_x = point_x * std::cos(ttheta) - point_y * std::sin(ttheta) + tx;
        double transformed_y = point_x * std::sin(ttheta) + point_y * std::cos(ttheta) + ty;

        bool is_not_observed;
        _residual[i] = 1.0 - grid_map.GetProbability(transformed_x, transformed_y, -1.0, is_not_observed);
        if(is_not_observed){
          counter++;
        }

        _residual[i] = factor * _residual[i];
      }
      
      return true;
    };

    private:
      const ProbabilityGridMap &grid_map;
      const ClassLPoint2DArray &points;
      const double factor;
  };

public:
  CeresGridPoseOptimizer(){};
  ~CeresGridPoseOptimizer(){};

  double Optimize(const ProbabilityGridMap &_grid_map, const ClassLPoint2DArray &_scan, 
      const geometry_msgs::Pose2D &_init_pose, geometry_msgs::Pose2D &_result,
      double _scan_factor = 1.0, double _translation_factor = 200.0, double _theta_factor = 80.0)
  {
    std::vector<double> pose;
    pose.resize(3);
    pose[0] = _init_pose.x; pose[1] = _init_pose.y; pose[2] = _init_pose.theta;

    ceres::Problem problem;
    int scan_num = _scan.pcd.points.size();
    ceres::CostFunction *cost_function = 
      new ceres::NumericDiffCostFunction<GridCostFunctor, ceres::CENTRAL, ceres::DYNAMIC, 3>(
          new GridCostFunctor(_grid_map, _scan, _scan_factor),
          ceres::TAKE_OWNERSHIP, _scan.pcd.points.size());
    problem.AddResidualBlock(cost_function, NULL, pose.data());

    ceres::CostFunction *rotation_function = 
      new ceres::AutoDiffCostFunction<RotationCostFunctor, 1, 3>(
          new RotationCostFunctor(_init_pose.theta, _theta_factor));
    problem.AddResidualBlock(rotation_function, NULL, pose.data());

    ceres::CostFunction *translation_function = 
      new ceres::AutoDiffCostFunction<TranslationCostFunctor, 2, 3>(
          new TranslationCostFunctor(_init_pose.x, _init_pose.y, _translation_factor));
    problem.AddResidualBlock(translation_function, NULL, pose.data());
    
    ceres::Solver::Options options;
    options.function_tolerance = 1e-35;
    options.gradient_tolerance = 1e-35;
    options.parameter_tolerance = 1e-35;
    options.min_trust_region_radius = 1e-45;
    options.max_num_iterations = 75;
    //options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    _result.x = pose[0]; _result.y = pose[1]; _result.theta = pose[2];
    while(_result.theta >= M_PI) _result.theta -= M_PI * 2.0;
    while(_result.theta < -M_PI) _result.theta += M_PI * 2.0;
    
    //std::cout << summary.BriefReport() << std::endl;
    return summary.final_cost;
 }
 double Optimize(const MultiResolutionGridMap &_grid_map, const ClassLPoint2DArray &_scan, 
     const geometry_msgs::Pose2D &_init_pose, geometry_msgs::Pose2D &_result,
     double _scan_factor = 1.0, double _translation_factor = 5.0, double _theta_factor = 5.0){
    int depth = _grid_map.GetDepth();
    _result = _init_pose;
    std::vector<geometry_msgs::Pose2D> poses;
    poses.resize(depth + 1);

    double rate = 1.0;
    double cost;
    poses[depth] = _init_pose;
    for(int i = depth - 1; i >= 0; i--){
      const ProbabilityGridMap *map_pt = _grid_map.GetGridMap(i);
      if(map_pt == NULL)
        return -1.0;
      
      cost = Optimize(*map_pt, _scan, poses[i + 1], poses[i], _scan_factor, _translation_factor * rate, _theta_factor * rate);
      rate *= 2.0;
    }

    _result = poses[0];
    return cost;
 }
};

#endif
