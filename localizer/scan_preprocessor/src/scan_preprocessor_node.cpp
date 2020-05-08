#include "scan_preprocessor_node.hpp"

std::string __scan_topic_name("/laser_link/scan");
std::string __visualize_topic_name("/visualize_pcd");
std::string __preprocessed_topic_name("/preprocessed_pcd");

ros::Publisher __pub_visualize_topic;
ros::Publisher __pub_preprocessed_topic;

GridFilter *__grid_filter;

static double __range = -1.0;

void ScanCB(const sensor_msgs::LaserScan::ConstPtr &_scan)
{
  ClassLPoint2DArray::Ptr lpcd = std::make_shared<ClassLPoint2DArray>();
  ClassLPoint2DArray::Ptr filtered_cloud = std::make_shared<ClassLPoint2DArray>();
  if(__range < 0.0)
    lpcd->InputLaserScan(*_scan, _scan->range_max);
  else
    lpcd->InputLaserScan(*_scan, __range);

  __grid_filter->SetPointCloud(*lpcd);
  __grid_filter->Execute(*filtered_cloud);
  filtered_cloud->pcd.header = _scan->header;

  sensor_msgs::PointCloud pcd_msg;
  filtered_cloud->Convert2PointCloud(pcd_msg);
  __pub_visualize_topic.publish(pcd_msg);
  __pub_preprocessed_topic.publish(filtered_cloud->pcd);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "scan_preprocessor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("scan_topic_name", __scan_topic_name);
  nh_private.getParam("visualize_topic_name", __visualize_topic_name);
  nh_private.getParam("preprocessed_topic_name", __preprocessed_topic_name);
  
  __pub_visualize_topic = nh.advertise<sensor_msgs::PointCloud>(__visualize_topic_name, 1);
  __pub_preprocessed_topic = nh.advertise<localization_msgs::LPoint2DArray>(__preprocessed_topic_name, 1);

  ros::Subscriber sub_scan = nh.subscribe(__scan_topic_name, 1, ScanCB);

  int n_threshold = 0;
  double cell_size = 0.1, min_x = -10.0, max_x = 10.0, min_y = -10.0, max_y = 10.0;
  nh_private.getParam("n_threshold", n_threshold);
  nh_private.getParam("cell_size", cell_size);
  nh_private.getParam("min_x", min_x);
  nh_private.getParam("max_x", max_x);
  nh_private.getParam("min_y", min_y);
  nh_private.getParam("max_y", max_y);
  nh_private.getParam("range", __range);
  __grid_filter = new GridFilter(n_threshold, cell_size, min_x, max_x, min_y, max_y);

  ros::spin();

  delete __grid_filter;

  return 0;
}
