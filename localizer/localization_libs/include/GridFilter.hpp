#ifndef GRIDFILTER
#define GRIDFILTER

#include "LPoint2DFilter.hpp"
#include "LPoint2DArrayClass.hpp"
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <Eigen/Core>
#include <localization_msgs/LPoint2D.h>
#include <localization_msgs/LPoint2DArray.h>


class GridFilter:public Filter{
  double cell_size;
  double min_x, max_x, min_y, max_y;
  int pix_width;
  int pix_height;
  int n_theshold;
  std::vector<localization_msgs::LPoint2D> grid_sum;
  std::vector<int> grid_count;

  public:
    GridFilter(int _n_th = 3, double _cell_size = 0.025, 
        double _min_x = 30.0, double _max_x = 30.0, double _min_y = -30.0, double _max_y = -30.0){
      SetThreshold(_n_th);
      SetGridParam(_cell_size, _min_x, _max_x, _min_y, _max_y);
    };

    ~GridFilter(){};

    void Execute(ClassLPoint2DArray &_output){
      _output.pcd.points.clear();
      _output.pcd.points.reserve(pix_width * pix_height);
      
      for(size_t i = 0; i < grid_count.size(); i++){
        if(grid_count[i] > n_theshold){
          localization_msgs::LPoint2D tmp;
          tmp.ptype = localization_msgs::LPoint2D::PTYPE_LINE;
          tmp.x = grid_sum[i].x / (double)grid_count[i];
          tmp.y = grid_sum[i].y / (double)grid_count[i];
          tmp.nx = grid_sum[i].nx / (double)grid_count[i];
          tmp.ny = grid_sum[i].ny / (double)grid_count[i];
          _output.pcd.points.emplace_back(tmp);
        }
      }
    };

    void SetGridParam(double _cell_size, double _min_x, double _max_x, double _min_y, double _max_y){
      cell_size = _cell_size;
      min_x = _min_x;
      max_x = _max_x;
      min_y = _min_y;
      max_y = _max_y;
      pix_width = (int)((max_x - min_x) / cell_size);
      pix_height = (int)((max_x - min_x) / cell_size);
      grid_sum.clear();
      grid_count.clear();
      grid_sum.resize(pix_width * pix_height);
      grid_count.resize(pix_width * pix_height);
    };

    void SetThreshold(int _n_theshold){
      n_theshold = _n_theshold;
    };

    void SetPointCloud(const ClassLPoint2DArray &_input){
      localization_msgs::LPoint2D zero; zero.x = 0.0; zero.y = 0.0; zero.nx = 0.0; zero.ny = 0.0;
      zero.ptype = localization_msgs::LPoint2D::PTYPE_LINE;
      for(int i = 0; i < grid_sum.size(); i++){
        grid_sum[i] = zero;
        grid_count[i] = 0.0;
      }
      AddPointCloud(_input);
    };

    void AddPointCloud(const ClassLPoint2DArray &_input){
      for(size_t i = 0; i < _input.pcd.points.size(); i++){
        int x_pos = (int)(((_input.pcd.points[i].x - min_x) / cell_size) + 0.5);
        int y_pos = (int)(((_input.pcd.points[i].y - min_y) / cell_size) + 0.5);
        if(x_pos >= 0 && x_pos < pix_width && y_pos >= 0 && y_pos < pix_height){
          int idx = x_pos + y_pos * pix_width;
          grid_sum[idx].x += _input.pcd.points[i].x;
          grid_sum[idx].y += _input.pcd.points[i].y;
          grid_sum[idx].nx += _input.pcd.points[i].nx;
          grid_sum[idx].ny += _input.pcd.points[i].ny;
          grid_count[idx]++;
        }
      }
    };
};

#endif
