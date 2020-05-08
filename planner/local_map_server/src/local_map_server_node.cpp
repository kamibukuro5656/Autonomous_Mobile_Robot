#include "local_map_server_node.hpp"

static ros::Publisher __pub_map;
static ros::Publisher __pub_map_request;

static tf::TransformListener *__tf_listener;
static std::string __scan_topic_name("/laser_link/scan");
static std::string __map_topic_name("local_map");
static std::string __base_map_topic_name("grid_map");
static std::string __map_tf_name("map");
static std::string __base_tf_name("base_link");
static std::string __map_request_topic_name("map_load_request");

static double __cell_size = 0.01;
static double __x_min = -2.5;
static double __x_max = 2.5;
static double __y_min = -2.5;
static double __y_max = 2.5;
static double __prob_hit = 0.9;
static double __prob_miss = 0.1;
static double __prob_fix =  0.51;
static double __prob_threshold = 0.5;

static double __scan_stack_time = 1.0;
static std::deque<ClassLPoint2DArray> scan_queue;
static std::deque<geometry_msgs::Pose2D> pose_queue;

static ProbabilityGridMap __base_map;
static bool __base_map_received = false;

void MapCB(const nav_msgs::OccupancyGrid::ConstPtr &_ocp_msg){
  __base_map.InputOccupancyGrid(*_ocp_msg);
  __base_map_received = true;
}

void ScanCB(const sensor_msgs::LaserScan::ConstPtr &_scan){
  tf::StampedTransform scan_transform, base_transform;
  try{
    __tf_listener->waitForTransform(__map_tf_name, _scan->header.frame_id, _scan->header.stamp, ros::Duration(0.1));
    __tf_listener->lookupTransform(__map_tf_name, _scan->header.frame_id, _scan->header.stamp, scan_transform);
    __tf_listener->lookupTransform(__map_tf_name, __base_tf_name, ros::Time(0), base_transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }

  ros::Time now = ros::Time::now();

  geometry_msgs::Pose2D scan_pose = Convert2Pose2D(scan_transform);
  pose_queue.emplace_back(scan_pose);
  scan_queue.emplace_back(ClassLPoint2DArray());
  scan_queue.back().InputLaserScan(*_scan);
  scan_queue.back().Transform(scan_pose);
  geometry_msgs::Pose2D base_pose = Convert2Pose2D(base_transform);

  ProbabilityGridMap::Ptr grid_map;
  if(__base_map_received){
    grid_map = __base_map.CutOutMap(__cell_size, __x_min + base_pose.x, __x_max + base_pose.x,
        __y_min + base_pose.y, __y_max + base_pose.y, 0.5);
    grid_map->SetProbHitMiss(__prob_hit, __prob_miss);
    grid_map->SetProbFixThreshold(__prob_fix);
  }
  else{
    grid_map = std::make_shared<ProbabilityGridMap>(__cell_size, __x_min + base_pose.x, __x_max + base_pose.x,
        __y_min + base_pose.y, __y_max + base_pose.y, __prob_hit, __prob_miss, __prob_fix);
    std_msgs::Bool req;
    req.data = true;
    __pub_map_request.publish(req);
  }
  for(int i = 0; i < scan_queue.size(); i++)
    grid_map->AddPointCloud(scan_queue[i], pose_queue[i]);

  while(!scan_queue.empty()){
    if((now - scan_queue.front().pcd.header.stamp).toSec() > __scan_stack_time){
      scan_queue.pop_front();
      pose_queue.pop_front();
    }
    else
      break;
  }

  ObstacleGridMap obst_map;
  obst_map.InputProbabilityMap(*grid_map, __prob_threshold);
  std_msgs::Header header;
  header.frame_id = __map_tf_name;
  header.stamp = ros::Time::now();
  nav_msgs::OccupancyGrid ocp_msg;
  obst_map.Convert2OccupancyGrid(header, ocp_msg);
  __pub_map.publish(ocp_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Local_Map_Server");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("map_topic_name", __map_topic_name);
  nh_private.getParam("scan_topic_name", __scan_topic_name);
  nh_private.getParam("base_map_topic_name", __base_map_topic_name);
  nh_private.getParam("map_tf_name", __map_tf_name);
  nh_private.getParam("base_tf_name", __base_tf_name);

  nh_private.getParam("cell_size", __cell_size);
  nh_private.getParam("x_min", __x_min);
  nh_private.getParam("x_max", __x_max);
  nh_private.getParam("y_min", __y_min);
  nh_private.getParam("y_max", __y_max);
  nh_private.getParam("prob_hit", __prob_hit);
  nh_private.getParam("prob_miss", __prob_miss);
  nh_private.getParam("prob_fix", __prob_fix);
  
  nh_private.getParam("scan_stack_time", __scan_stack_time);

  __tf_listener = new tf::TransformListener;

  __pub_map = nh.advertise<nav_msgs::OccupancyGrid>(__map_topic_name, 1);
  __pub_map_request = nh.advertise<std_msgs::Bool>(__map_request_topic_name, 1);

  ros::Subscriber sub_pcd, sub_map;
  sub_pcd = nh.subscribe(__scan_topic_name, 1, ScanCB);
  sub_map = nh.subscribe(__base_map_topic_name, 1, MapCB);

  ros::spin();

  delete __tf_listener;
}
