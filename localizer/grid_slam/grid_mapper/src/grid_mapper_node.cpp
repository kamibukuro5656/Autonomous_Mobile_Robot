#include "grid_mapper_node.hpp"

static ros::Publisher __pub_map;
static ros::Publisher __pub_est_pcd;

static tf::TransformListener *__tf_listener;
static tf::TransformBroadcaster *__tf_broadcaster;
static std::string __pcd_topic_name("preprocessed_pcd");
static std::string __correction_tf_name("odom");
static std::string __scan_topic_name("scan");
static std::string __map_topic_name("grid_map");
static std::string __parent_frame_name("map");
static std::string __save_file_path("/home/main/ROS_Log_Data/Map/");
static std::string __save_file_name("grid");
static std::string __save_request_topic_name("map_save_request");
static std::string __load_request_topic_name("map_load_request");

static int __counter = 0;
static double __correction_x = 0;
static double __correction_y = 0;
static double __correction_theta = 0;

static double __cell_size = 0.025;
static double __x_min = -20.0;
static double __x_max = 20.0;
static double __y_min = -20.0;
static double __y_max = 20.0;
static double __weight_translation = 10.0;
static double __weight_angular = 10.0;
static int __resolution_depth = 4;
static double __prob_hit = 0.51;
static double __prob_miss = 0.49;

static MultiResolutionGridMap* __grid_map;
static CeresGridPoseOptimizer __optimizer;

static int __start_est_count = 3;
static double __score_threshold = 100.0;

static tf::Transform __correction_tf;

static std::mutex __mtx;

void ProcGridMap(ClassLPoint2DArray &_input){
  bool cant_get_tf = false;

  tf::StampedTransform transform;
  try{
    __tf_listener->waitForTransform(__correction_tf_name, _input.pcd.header.frame_id, _input.pcd.header.stamp, ros::Duration(0.05));
    __tf_listener->lookupTransform(__correction_tf_name, _input.pcd.header.frame_id, _input.pcd.header.stamp, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    cant_get_tf = true;
  }

  if(!cant_get_tf){
    tf::Transform map_to_laser =  __correction_tf * transform;
    geometry_msgs::Pose2D pose = Convert2Pose2D(map_to_laser);

    if(__counter > __start_est_count){
      geometry_msgs::Pose2D est_pose = pose;
      double cost = __optimizer.Optimize(*__grid_map, _input, pose, est_pose, 1.0, __weight_translation, __weight_angular);
      
      double sub_theta = NormalizeAngle(est_pose.theta - pose.theta);
      double tmp_x = pose.x - __correction_x;
      double tmp_y = pose.y - __correction_y;
      double sub_x = est_pose.x - (tmp_x * cos(sub_theta) - tmp_y * sin(sub_theta) + __correction_x);
      double sub_y = est_pose.y - (tmp_x * sin(sub_theta) + tmp_y * cos(sub_theta) + __correction_y);

      std::cout << "est_cost:" << cost << std::endl;
      std::cout << "init_x:" << pose.x << " init_y:" << pose.y << " init_theta:" << pose.theta << std::endl;
      std::cout << "est_x:" << est_pose.x << " est_y:" << est_pose.y << " est_theta:" << est_pose.theta << std::endl;
      std::cout << "sub_x:" << sub_x << " sub_y:" << sub_y << " sub_theta:" << sub_theta << std::endl;
      if(cost >= 0.0 && cost < __score_threshold){
        std::cout << "Success" << std::endl;
        pose = est_pose;
        __correction_x += sub_x;
        __correction_y += sub_y;
        __correction_theta += sub_theta;
        NormalizeAngle(__correction_theta);
      }
      else{
        std::cout << "Failure" << std::endl;
      }
    }

    _input.Transform(pose);
    __grid_map->AddPointCloud(_input, pose);

    sensor_msgs::PointCloud pcd_msg;
    _input.Convert2PointCloud(pcd_msg);
    pcd_msg.header.frame_id = "map";
    __pub_est_pcd.publish(pcd_msg);
  }
  
  std_msgs::Header header;
  header.frame_id = __parent_frame_name;
  header.stamp = ros::Time::now();
  nav_msgs::OccupancyGrid ocp_msg;
  __grid_map->Convert2OccupancyGrid(header, ocp_msg);
  __pub_map.publish(ocp_msg);

  {
    std::lock_guard<std::mutex> lock(__mtx);
    __correction_tf.setOrigin(tf::Vector3(__correction_x, __correction_y, 0.0));
    tf::Quaternion correction_q;
    correction_q.setRPY(0, 0, __correction_theta);
    __correction_tf.setRotation(correction_q);
  }

  __counter++;
}

void PointCloudCB(const localization_msgs::LPoint2DArray::ConstPtr &_pcd){
  ClassLPoint2DArray input;
  input.pcd = *_pcd;
  ProcGridMap(input);
}

void ScanCB(const sensor_msgs::LaserScan::ConstPtr &_scan){
  ClassLPoint2DArray input;
  input.InputLaserScan(*_scan);
  ProcGridMap(input);
}

void SaveRequestCB(const std_msgs::Bool _req){
  if(_req.data == true){
    if(__grid_map->Save(__save_file_path, __save_file_name))
      std::cout << "Save Success" << std::endl;
    else
      std::cout << "Save Failure" << std::endl;
  }
}

void LoadRequestCB(const std_msgs::Bool _req){
  if(_req.data == true){
    if(__grid_map->Load(__resolution_depth, __save_file_path, __save_file_name)){
      std::cout << "Load Success" << std::endl;
      std_msgs::Header header;
      header.frame_id = __parent_frame_name;
      header.stamp = ros::Time::now();
      nav_msgs::OccupancyGrid ocp_msg;
      __grid_map->Convert2OccupancyGrid(header, ocp_msg);
    }
    else{
      std::cout << "Load Failure" << std::endl;
    }
  }
}

void PointCB(const geometry_msgs::PointStamped::ConstPtr &_point){ //手動修正用
  static unsigned int count = 0;
  static geometry_msgs::Point point1;

  if(count % 2){//二点来たら
    __grid_map->ManualDrawLine(point1, _point->point, 0.9);
    std_msgs::Header header;
    header.frame_id = __parent_frame_name;
    header.stamp = ros::Time::now();
    nav_msgs::OccupancyGrid ocp_msg;
    __grid_map->Convert2OccupancyGrid(header, ocp_msg);
    __pub_map.publish(ocp_msg);
  }

  point1 = _point->point;
  count++;
}

void TFBroadcastThread(){
  ros::Rate rate(100);
  while(ros::ok()){
    {
      std::lock_guard<std::mutex> lock(__mtx);
      __tf_broadcaster->sendTransform(tf::StampedTransform(__correction_tf, ros::Time::now(),
          __parent_frame_name, __correction_tf_name));
    }
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Grid_Mapper");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("pcd_topic_name", __pcd_topic_name);
  nh_private.getParam("map_topic_name", __map_topic_name);
  nh_private.getParam("scan_topic_name", __scan_topic_name);
  nh_private.getParam("correction_tf_name", __correction_tf_name);
  nh_private.getParam("score_threshold", __score_threshold);
  nh_private.getParam("map_save_file_path", __save_file_path);
  nh_private.getParam("map_save_file_name", __save_file_name);
  nh_private.getParam("map_save_request_topic_name", __save_request_topic_name);

  bool mode_scan = true;
  nh_private.getParam("mode_scan", mode_scan);
  nh_private.getParam("cell_size", __cell_size);
  nh_private.getParam("resolution_depth", __resolution_depth);
  nh_private.getParam("x_min", __x_min);
  nh_private.getParam("x_max", __x_max);
  nh_private.getParam("y_min", __y_min);
  nh_private.getParam("y_max", __y_max);
  nh_private.getParam("weight_translation", __weight_translation);
  nh_private.getParam("weight_angular", __weight_angular);
  nh_private.getParam("prob_hit", __prob_hit);
  nh_private.getParam("prob_miss", __prob_miss);

  __tf_listener = new tf::TransformListener;
  __tf_broadcaster = new tf::TransformBroadcaster;
  __grid_map = new MultiResolutionGridMap(__resolution_depth, __cell_size, __x_min, __x_max, __y_min, __y_max, __prob_hit, __prob_miss);

  __correction_tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion correction_q;
  correction_q.setRPY(0.0, 0.0, 0.0);
  __correction_tf.setRotation(correction_q);
  
  __pub_map = nh.advertise<nav_msgs::OccupancyGrid>(__map_topic_name, 1);
  __pub_est_pcd = nh.advertise<sensor_msgs::PointCloud>("est_points", 1);

  ros::Subscriber sub_pcd;
  if(mode_scan)
    sub_pcd = nh.subscribe(__scan_topic_name, 5, ScanCB);
  else
    sub_pcd = nh.subscribe(__pcd_topic_name, 5, PointCloudCB);

  ros::Subscriber sub_save_req, sub_load_req, sub_point;
  sub_save_req = nh.subscribe(__save_request_topic_name, 1, SaveRequestCB);
  sub_load_req = nh.subscribe(__load_request_topic_name, 1, LoadRequestCB);
  sub_point = nh.subscribe("clicked_point", 5, PointCB);

  std::thread thread(TFBroadcastThread);

  ros::spin();

  delete __tf_listener;
  delete __tf_broadcaster;
  delete __grid_map;
}
