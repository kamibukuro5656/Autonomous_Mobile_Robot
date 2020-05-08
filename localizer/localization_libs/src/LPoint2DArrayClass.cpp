#include "LPoint2DArrayClass.hpp"

const double ClassLPoint2DArray::FPDMIN = 0.05;
const double ClassLPoint2DArray::FPDMAX = 1.0;
const double ClassLPoint2DArray::CRTHRE = 45.0;
const double ClassLPoint2DArray::INVALID = -1.0;


void ClassLPoint2DArray::InputLaserScan(const sensor_msgs::LaserScan &_laserscan, const double _max_range, const double _angular_vel)
{
  double range = _laserscan.angle_max - _laserscan.angle_min;
  unsigned int array_size = (unsigned int)(range / _laserscan.angle_increment) + 50;

  pcd.points.clear();
  pcd.header = _laserscan.header;
  pcd.points.reserve(array_size);

  double max_range = _laserscan.range_max;
  if(_max_range >= 0.0)
    max_range = _max_range;

  double angle = _laserscan.angle_min;
  for(unsigned int i = 0; i < _laserscan.ranges.size(); i++){
    if(_laserscan.ranges[i] > _laserscan.range_min && _laserscan.ranges[i] < max_range){
      localization_msgs::LPoint2D tmp;
      tmp.x = _laserscan.ranges[i] * std::cos(angle);
      tmp.y = _laserscan.ranges[i] * std::sin(angle);
      pcd.points.emplace_back(tmp);
    }
    angle += _laserscan.angle_increment;
    angle += _angular_vel * _laserscan.time_increment;
  }

  CalcNormalVector();
}

void ClassLPoint2DArray::Convert2PointCloud(sensor_msgs::PointCloud &_pcd_msg)
{
  _pcd_msg.header = pcd.header;

  _pcd_msg.points.clear();
  _pcd_msg.channels.clear();

  _pcd_msg.points.reserve(pcd.points.size());

  for(unsigned int i = 0; i < pcd.points.size(); i++){
    geometry_msgs::Point32 tmp;
    tmp.x = (float)pcd.points[i].x;
    tmp.y = (float)pcd.points[i].y;
    tmp.z = 0.0;

    _pcd_msg.points.emplace_back(tmp);
  }
}

void ClassLPoint2DArray::Transform(const Eigen::Matrix3d &_matrix)
{
  for(unsigned int i = 0; i < pcd.points.size(); i++){
    localization_msgs::LPoint2D tmp;
    tmp.ptype = pcd.points[i].ptype;
    tmp.x = pcd.points[i].x * _matrix(0, 0) + pcd.points[i].y * _matrix(0, 1) + _matrix(0, 2);
    tmp.y = pcd.points[i].x * _matrix(1, 0) + pcd.points[i].y * _matrix(1, 1) + _matrix(1, 2);
    tmp.nx = pcd.points[i].nx * _matrix(0, 0) + pcd.points[i].ny * _matrix(0, 1) + _matrix(0, 2);
    tmp.ny = pcd.points[i].nx * _matrix(1, 0) + pcd.points[i].ny * _matrix(1, 1) + _matrix(1, 2);
    pcd.points[i] = tmp;
  }
}

void ClassLPoint2DArray::Transform(const geometry_msgs::Pose2D &_pose)
{
  for(unsigned int i = 0; i < pcd.points.size(); i++){
    localization_msgs::LPoint2D tmp;
    tmp.ptype = pcd.points[i].ptype;
    tmp.x = pcd.points[i].x * std::cos(_pose.theta) - pcd.points[i].y * std::sin(_pose.theta) + _pose.x;
    tmp.y = pcd.points[i].x * std::sin(_pose.theta) + pcd.points[i].y * std::cos(_pose.theta) + _pose.y;
    tmp.nx = pcd.points[i].nx * std::cos(_pose.theta) - pcd.points[i].ny * std::sin(_pose.theta) + _pose.x;
    tmp.ny = pcd.points[i].nx * std::sin(_pose.theta) + pcd.points[i].ny * std::cos(_pose.theta) + _pose.y;
    pcd.points[i] = tmp;
  }
}

void ClassLPoint2DArray::Transform(const tf::StampedTransform &_tf)
{
  double x = _tf.getOrigin().x();
  double y = _tf.getOrigin().y();
  tf::Quaternion q = _tf.getRotation();
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  for(unsigned int i = 0; i < pcd.points.size(); i++){
    localization_msgs::LPoint2D tmp;
    tmp.ptype = pcd.points[i].ptype;
    tmp.x = pcd.points[i].x * std::cos(yaw) - pcd.points[i].y * std::sin(yaw) + x;
    tmp.y = pcd.points[i].x * std::sin(yaw) + pcd.points[i].y * std::cos(yaw) + y;
    tmp.nx = pcd.points[i].nx * std::cos(yaw) - pcd.points[i].ny * std::sin(yaw) + x;
    tmp.ny = pcd.points[i].nx * std::sin(yaw) + pcd.points[i].ny * std::cos(yaw) + y;
    pcd.points[i] = tmp;
  }
}

bool ClassLPoint2DArray::CheckNormalVector(int _idx, int _dir, Eigen::Vector2d &_normal)
{
  const localization_msgs::LPoint2D &cp = pcd.points[_idx];
  for(int i = _idx + _dir; i >= 0 && i < pcd.points.size(); i += _dir){
    const localization_msgs::LPoint2D &lp = pcd.points[i];

    double dx = lp.x - cp.x;
    double dy = lp.y - cp.y;
    double d = std::sqrt(dx*dx + dy*dy);
    if (d>=FPDMIN && d<=FPDMAX) {
      _normal[0] = dy/d;
      _normal[1] = -dx/d;
      return(true);
    }

    if (d > FPDMAX)
      break;
  }

  return(false);
}

void ClassLPoint2DArray::CalcNormalVector()
{
  double costh = cos(CRTHRE * 3.1415 / 180.0);
  for(int i = 0; i < pcd.points.size(); i++){
    localization_msgs::LPoint2D &lp = pcd.points[i];
    uint8_t type;
    Eigen::Vector2d nl, nr, normal;

    bool flag_l = CheckNormalVector(i, -1, nl);
    bool flag_r = CheckNormalVector(i, 1, nr);

    nr[0] = -nr[0];
    nr[1] = -nr[1];

    if(flag_l){
      if(flag_r){
        if(std::fabs(nl[0] * nr[0] + nl[1] * nr[1]) >= costh)
          type = localization_msgs::LPoint2D::PTYPE_LINE;
        
        else
          type = localization_msgs::LPoint2D::PTYPE_CORNER;  
      
        normal = nl + nr;
        double l = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
        normal = normal / l;
      }
      else{
        type = localization_msgs::LPoint2D::PTYPE_LINE;
        normal = nl;
      }
    }
    else{
      if(flag_r){
        type = localization_msgs::LPoint2D::PTYPE_LINE;
        normal = nr;
      }
      else{
        type = localization_msgs::LPoint2D::PTYPE_ISOLATE;
        normal[0] = INVALID;
        normal[1] = INVALID;
      }
    }

    lp.nx = normal[0];
    lp.ny = normal[1];
    lp.ptype = type;
  }
}
