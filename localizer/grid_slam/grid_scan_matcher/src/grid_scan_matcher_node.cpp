#include "grid_scan_matcher_node.hpp"

static ros::Publisher __pub_est_points;
static ros::Publisher __pub_map;
static ros::Publisher __pub_matching_result;

static tf::TransformListener *__tf_listener;
static tf::TransformBroadcaster *__tf_broadcaster;
static std::string __pcd_topic_name("preprocessed_pcd");
static std::string __correction_tf_name("odom");
static std::string __base_tf_name("base_link");
static std::string __scan_topic_name("scan");
static std::string __map_topic_name("grid_map");
static std::string __matching_result_topic_name("map_matching_result");
static std::string __init_pose_topic_name("initialpose");
static std::string __parent_frame_name("map");
static std::string __load_file_path("/home/main/ROS_Log_Data/Map/");
static std::string __load_file_name("grid");
static std::string __load_request_topic_name("map_load_request");

static double __init_x = 0.0;
static double __init_y = 0.0;
static double __init_theta = 0.0;

static MultiResolutionGridMap* __grid_map;
static CeresGridPoseOptimizer __optimizer;

static double __score_threshold = 100.0;

static bool __map_loaded = false;

static double __correction_x = 0;
static double __correction_y = 0;
static double __correction_theta = 0;

static double __weight_translation = 10.0;
static double __weight_angular = 10.0;
static int __resolution_depth = 3;

static double __step_threshold_x = 0.75;
static double __step_threshold_y = 0.75;
static double __step_threshold_theta = M_PI / 2;

static tf::Transform __correction_tf;

static std::mutex __mtx;

void ProcGridMap(ClassLPoint2DArray &_input){
  static int counter = 0;
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

  localization_msgs::MatchingResult result;
  result.success = false;
  result.score = DBL_MAX;
  result.header.stamp = ros::Time::now();
  if(!cant_get_tf && __map_loaded){
    tf::Transform map_to_laser =  __correction_tf * transform;
    geometry_msgs::Pose2D pose = Convert2Pose2D(map_to_laser);

    geometry_msgs::Pose2D est_pose = pose;
    double cost = __optimizer.Optimize(*__grid_map, _input, pose, est_pose, 1.0, __weight_translation, __weight_angular);
    result.score = cost;
    result.pose = est_pose;
      
    double sub_theta = NormalizeAngle(est_pose.theta - pose.theta);
    double tmp_x = pose.x - __correction_x;
    double tmp_y = pose.y - __correction_y;
    double sub_x = est_pose.x - (tmp_x * cos(sub_theta) - tmp_y * sin(sub_theta) + __correction_x);
    double sub_y = est_pose.y - (tmp_x * sin(sub_theta) + tmp_y * cos(sub_theta) + __correction_y);
      
    if(cost >= 0.0 && cost < __score_threshold &&
        std::fabs(sub_x) < __step_threshold_x && 
        std::fabs(sub_y) < __step_threshold_y &&
        std::fabs(sub_theta) < __step_threshold_theta){
      pose = est_pose;
      __correction_x += sub_x;
      __correction_y += sub_y;
      __correction_theta += sub_theta;
      NormalizeAngle(__correction_theta);

      result.success = true;
    }
    _input.Transform(pose);
    sensor_msgs::PointCloud pcd_msg;
    _input.Convert2PointCloud(pcd_msg);
    pcd_msg.header.frame_id = "map";
    __pub_est_points.publish(pcd_msg);
  }
  __pub_matching_result.publish(result);
  
  {
    std::lock_guard<std::mutex> lock(__mtx);
    __correction_tf.setOrigin(tf::Vector3(__correction_x, __correction_y, 0.0));
    tf::Quaternion correction_q;
    correction_q.setRPY(0, 0, __correction_theta);
    __correction_tf.setRotation(correction_q);
  }

  counter++;
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

void LoadRequestCB(const std_msgs::Bool _req){
  if(_req.data == true){
    if(__grid_map->Load(__resolution_depth, __load_file_path, __load_file_name)){
      std_msgs::Header header;
      header.frame_id = __parent_frame_name;
      header.stamp = ros::Time::now();
      __map_loaded = true;
      nav_msgs::OccupancyGrid ocp_msg;
      __grid_map->Convert2OccupancyGrid(header, ocp_msg);
      __pub_map.publish(ocp_msg);
    }
    else{
      __map_loaded = false;
      ROS_ERROR("Map_Load_Fail");
    }
  }
}

void InitPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &_pose)
{
  double x = _pose->pose.pose.position.x;
  double y = _pose->pose.pose.position.y;
  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(_pose->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  tf::StampedTransform transform;
  try{
    __tf_listener->lookupTransform(__correction_tf_name, __base_tf_name, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  tf::Transform map_to_base =  __correction_tf * transform;
  geometry_msgs::Pose2D base = Convert2Pose2D(map_to_base);

  double sub_theta = (yaw - base.theta);
  double tmp_x = base.x - __correction_x;
  double tmp_y = base.y - __correction_y;
  double sub_x = x - (tmp_x * cos(sub_theta) - tmp_y * sin(sub_theta) + __correction_x);
  double sub_y = y - (tmp_x * sin(sub_theta) + tmp_y * cos(sub_theta) + __correction_y);
  __correction_x += sub_x;
  __correction_y += sub_y;
  __correction_theta += sub_theta;
  NormalizeAngle(__correction_theta);
  
  {
    std::lock_guard<std::mutex> lock(__mtx);
    __correction_tf.setOrigin(tf::Vector3(__correction_x, __correction_y, 0.0));
    tf::Quaternion correction_q;
    correction_q.setRPY(0, 0, __correction_theta);
    __correction_tf.setRotation(correction_q);
  }
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
  ros::init(argc, argv, "Grid_Scan_Matcher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("pcd_topic_name", __pcd_topic_name);
  nh_private.getParam("map_topic_name", __map_topic_name);
  nh_private.getParam("matching_result_topic_name", __matching_result_topic_name);
  nh_private.getParam("scan_topic_name", __scan_topic_name);
  nh_private.getParam("correction_tf_name", __correction_tf_name);
  nh_private.getParam("base_tf_name", __base_tf_name);
  nh_private.getParam("score_threshold", __score_threshold);
  nh_private.getParam("step_threshold_x", __step_threshold_x);
  nh_private.getParam("step_threshold_y", __step_threshold_y);
  nh_private.getParam("step_threshold_theta", __step_threshold_theta);
  nh_private.getParam("map_load_file_path", __load_file_path);
  nh_private.getParam("map_load_file_name", __load_file_name);
  nh_private.getParam("map_load_request_topic_name", __load_request_topic_name);

  bool mode_scan = true;
  nh_private.getParam("mode_scan", mode_scan);
  nh_private.getParam("weight_translation", __weight_translation);
  nh_private.getParam("weight_angular", __weight_angular);
  nh_private.getParam("resolution_depth", __resolution_depth);

  nh_private.getParam("init_x", __init_x);
  nh_private.getParam("init_y", __init_y);
  nh_private.getParam("init_theta", __init_theta);
  __correction_x = __init_x; __correction_y = __init_y; __correction_theta = __init_theta;
  __correction_tf.setOrigin(tf::Vector3(__correction_x, __correction_y, 0.0));
  tf::Quaternion correction_q;
  correction_q.setRPY(0, 0, __correction_theta);
  __correction_tf.setRotation(correction_q);

  __tf_listener = new tf::TransformListener;
  __tf_broadcaster = new tf::TransformBroadcaster;
  __grid_map = new MultiResolutionGridMap();
  
  __pub_est_points = nh.advertise<sensor_msgs::PointCloud>("est_points", 1);
  __pub_map = nh.advertise<nav_msgs::OccupancyGrid>(__map_topic_name, 1);
  __pub_matching_result = nh.advertise<localization_msgs::MatchingResult>(__matching_result_topic_name, 1);

  ros::Subscriber sub_pcd;
  if(mode_scan)
    sub_pcd = nh.subscribe(__scan_topic_name, 5, ScanCB);
  else
    sub_pcd = nh.subscribe(__pcd_topic_name, 5, PointCloudCB);

  ros::Subscriber sub_load_req;
  sub_load_req = nh.subscribe(__load_request_topic_name, 1, LoadRequestCB);

  ros::Subscriber sub_init_pose;
  sub_init_pose = nh.subscribe(__init_pose_topic_name, 1, InitPoseCB);

  if(__grid_map->Load(__resolution_depth, __load_file_path, __load_file_name)){
      //std::cout << "Load Success" << std::endl;
      __map_loaded = true;
      std_msgs::Header header;
      header.frame_id = __parent_frame_name;
      header.stamp = ros::Time::now();
      nav_msgs::OccupancyGrid ocp_msg;
      __grid_map->Convert2OccupancyGrid(header, ocp_msg);
    }
  else{
      __map_loaded = false;
      ROS_ERROR("Map_Load_Fail");
  }

  std::thread tf_thread(TFBroadcastThread);
  ros::spin();

  delete __tf_listener;
  delete __tf_broadcaster;
  delete __grid_map;
}
