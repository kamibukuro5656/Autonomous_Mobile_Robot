#ifndef ROBOTUTILS
#define ROBOTUTILS

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

inline geometry_msgs::Pose2D Convert2Pose2D(const tf::StampedTransform &_transform){
  geometry_msgs::Pose2D pose;
  pose.x = _transform.getOrigin().x();
  pose.y = _transform.getOrigin().y();
  tf::Quaternion q = _transform.getRotation();
  double roll, pitch;
  tf::Matrix3x3(q).getRPY(roll, pitch, pose.theta);
  return pose;
}

inline geometry_msgs::Pose2D Convert2Pose2D(const tf::Transform &_transform){
  geometry_msgs::Pose2D pose;
  pose.x = _transform.getOrigin().x();
  pose.y = _transform.getOrigin().y();
  tf::Quaternion q = _transform.getRotation();
  double roll, pitch;
  tf::Matrix3x3(q).getRPY(roll, pitch, pose.theta);
  return pose;
}

inline void NormalizeAngle(double &_angle){
  while(_angle <= -M_PI) _angle += M_PI * 2.0;
  while(_angle > M_PI) _angle -= M_PI * 2.0;
}

inline double NormalizeAngle(const double &_angle){
  double angle = _angle;
  while(angle <= -M_PI) angle += M_PI * 2.0;
  while(angle > M_PI) angle -= M_PI * 2.0;
  return angle;
}

inline double Distance(const geometry_msgs::Pose2D &_a, const geometry_msgs::Pose2D &_b){
  double dist_x = _a.x - _b.x;
  double dist_y = _a.y - _b.y;
  return std::sqrt(dist_x * dist_x + dist_y * dist_y);
}

inline int SearchMinDistIdx(const geometry_msgs::Pose2D &_pose, const std::vector<geometry_msgs::Pose2D> &_path, double &_min_dist){
  int min_idx = 0;
  double min_dist = DBL_MAX;
  for(int i = 0; i < _path.size(); i++){
    double dist_x = _pose.x - _path[i].x;
    double dist_y = _pose.y - _path[i].y;
    double dist = dist_x * dist_x + dist_y * dist_y;
    if(min_dist > dist){
      min_idx = i;
      min_dist = dist;
    }
  }
  
  _min_dist = std::sqrt(min_dist);
  return min_idx;
}

inline void ConvertPose2DToLineMarker(const std::vector<geometry_msgs::Pose2D> _path,
      const std_msgs::Header &_header, const std_msgs::ColorRGBA &_color, const double _size, visualization_msgs::Marker &_vis_path){
    _vis_path.header = _header;
    _vis_path.ns = "line";
    _vis_path.action = visualization_msgs::Marker::ADD;
    _vis_path.pose.orientation.w = 1.0;
    _vis_path.id = 0;
    _vis_path.type = visualization_msgs::Marker::LINE_LIST;
    _vis_path.scale.x = _size;
    _vis_path.color = _color;

    _vis_path.points.clear();
    _vis_path.points.reserve(_path.size() * 2);
    for(int i = 0; i < ((int)_path.size()) - 1; i++){
      geometry_msgs::Point p1, p2;
      p1.x = _path[i].x; p1.y = _path[i].y; p1.z = 0.0;
      p2.x = _path[i + 1].x; p2.y = _path[i + 1].y; p2.z = 0.0;
      _vis_path.points.emplace_back(p1);
      _vis_path.points.emplace_back(p2);
    }
};

inline void ConvertPose2DToPointsMarker(const std::vector<geometry_msgs::Pose2D> _path,
      const std_msgs::Header &_header, const std_msgs::ColorRGBA &_color, const double _size, visualization_msgs::Marker &_vis_path){
    _vis_path.header = _header;
    _vis_path.ns = "points";
    _vis_path.action = visualization_msgs::Marker::ADD;
    _vis_path.pose.orientation.w = 1.0;
    _vis_path.id = 0;
    _vis_path.type = visualization_msgs::Marker::LINE_LIST;
    _vis_path.scale.x = _size;
    _vis_path.scale.y = _size;
    _vis_path.color = _color;

    _vis_path.points.clear();
    _vis_path.points.reserve(_path.size());
    for(int i = 0; i < _path.size(); i++){
      geometry_msgs::Point p1;
      p1.x = _path[i].x; p1.y = _path[i].y; p1.z = 0.0;
      _vis_path.points.emplace_back(p1);
    }
}

inline double CalcR(const geometry_msgs::Pose2D &_p1, const geometry_msgs::Pose2D &_p2, const geometry_msgs::Pose2D &_p3){
  double cx, cy, a, b, c, d;
  double r;

  a = _p1.x - _p2.x;
  b = _p1.y - _p2.y;
  c = _p2.x - _p3.x;
  d = _p2.y - _p3.y;

  double A = 2 * (a * d - b * c);

  if(std::fabs(A) < 0.0000000001)
    return DBL_MAX;

  A = 1 / A;
  double X1 = _p1.x * _p1.x + _p1.y * _p1.y;
  double X2 = _p2.x * _p2.x + _p2.y * _p2.y;
  double X3 = _p3.x * _p3.x + _p3.y * _p3.y;
  cx = A * (d * (X1 - X2) - b * (X2 - X3));
  cy = A * (-c * (X1 - X2) + a * (X2 - X3));

  double tmp_x = _p2.x - cx;
  double tmp_y = _p2.y - cy;
  return std::sqrt(tmp_x * tmp_x + tmp_y * tmp_y);
}

#endif
