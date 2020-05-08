#ifndef LPOINT2D_ARRAY_CLASS
#define LPOINT2D_ARRAY_CLASS

#include <cmath>
#include <ros/ros.h>
#include <Eigen/Core>
#include <memory>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <localization_msgs/LPoint2D.h>
#include <localization_msgs/LPoint2DArray.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>

class ClassLPoint2DArray{
  static const double FPDMIN;
  static const double FPDMAX;
  static const double CRTHRE;
  static const double INVALID;

  bool CheckNormalVector(int _idx, int _dir, Eigen::Vector2d &_normal);
public:
  typedef std::shared_ptr<ClassLPoint2DArray> Ptr;
  typedef std::shared_ptr<const ClassLPoint2DArray> ConstPtr;

  localization_msgs::LPoint2DArray pcd;

  ClassLPoint2DArray(){};
  ~ClassLPoint2DArray(){};
  
  void InputLaserScan(const sensor_msgs::LaserScan &_laserscan, const double _max_range = -1.0, const double _angular_vel = 0.0);
  void Convert2PointCloud(sensor_msgs::PointCloud &_pcd_msg);
  void Transform(const Eigen::Matrix3d &_matrix);
  void Transform(const geometry_msgs::Pose2D &_pose);
  void Transform(const tf::StampedTransform &_tf);
  void Transform(const Eigen::Matrix3d &_matrix, ClassLPoint2DArray &_transformed) const{
    _transformed = *this;
    _transformed.Transform(_matrix);
  };
  void Transform(const geometry_msgs::Pose2D &_pose, ClassLPoint2DArray &_transformed) const{
    _transformed = *this;
    _transformed.Transform(_pose);
  };
  void Transform(const tf::StampedTransform &_tf, ClassLPoint2DArray &_transformed) const{
    _transformed = *this;
    _transformed.Transform(_tf);
  };
  void CalcNormalVector();
};

#endif
