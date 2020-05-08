#include "pure_pursuit_node.hpp"

static tf::TransformListener *__tf_listener;

static ros::Publisher __pub_twist;
static ros::Publisher __pub_vis_path;
static ros::Publisher __pub_vis_ref_point;

static std::string __twist_cmd_topic_name("/neet_bot/diff_drive_controller/cmd_vel");
static std::string __vis_path_topic_name("pure_pursuit_path");
static std::string __vis_ref_point_topic_name("pure_pursuit_ref_point");
static std::string __motion_plan_topic_name("motion_plan");
static std::string __base_tf_name("base_link");
static std::string __map_tf_name("map");
static std::string __grid_map_topic_name("local_map");

static bool __initial_angle_corrected = false;
static bool __path_received = false;
static bool __path_end = false;
static std::vector<geometry_msgs::Pose2D> __path;
static std::vector<double> __velocity_list;
static PurePursuitWithVelocity* __pure_pursuit;
static TwoWheeledRobotModel* __model;
static geometry_msgs::Twist __bef_twist;

static double __end_radius = 0.1;
static double __robot_radius = 0.13;

void GridMapCB(const nav_msgs::OccupancyGrid::ConstPtr &_ocp_grid)
{
  //std::cout << "Subscribed GridMap" << std::endl;
  __pure_pursuit->SetGridMap(*_ocp_grid, __robot_radius);
}

void MotionPlanCB(const planner_msgs::MotionPlan::ConstPtr _motion_plan){
//  std::cout << "sub path" << std::endl;
  static unsigned int bef_path_id = UINT32_MAX;
  if(!_motion_plan->path.array.empty()){
    __path = _motion_plan->path.array;
    __pure_pursuit->SetPath(_motion_plan->path.array);
    __velocity_list = _motion_plan->velocity_list;
    __path_received = true;
    __path_end = false;

    if(bef_path_id != _motion_plan->path.id){
      __initial_angle_corrected = false;
      __bef_twist.linear.x = 0.0;
      __bef_twist.angular.z = 0.0;
      bef_path_id = _motion_plan->path.id;
    }
  }
}

geometry_msgs::Twist ProcessPurePursuit(){
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;
  
  tf::StampedTransform transform;
  try{
    __tf_listener->lookupTransform(__map_tf_name, __base_tf_name, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return twist;
  }
  geometry_msgs::Pose2D pose = Convert2Pose2D(transform);
  
  double min_dist;
  int min_idx = SearchMinDistIdx(pose, __path, min_dist);
  double target_vel = __velocity_list[min_idx];
  geometry_msgs::Pose2D ref_pose;
  twist = __pure_pursuit->Calculate(pose, __bef_twist, target_vel, ref_pose);
  
  double dist = Distance(__path.back(), pose);
  if(dist < __end_radius || __path_end){
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    __path_end = true;
  }

  __model->SetStatePose(pose);
  __model->SetStateTwist(twist);
  std::vector<geometry_msgs::Pose2D> pure_pursuit_path = 
    __model->GeneratePredictionPath(__pure_pursuit->GetPredTime() * 3.0, 0.25);
  
  visualization_msgs::Marker vis_path;
  std_msgs::Header header;
  header.frame_id = __map_tf_name;
  header.stamp = ros::Time::now();

  std_msgs::ColorRGBA color; color.r = 0.0; color.g = 0.1; color.b = 0.5; color.a = 0.8;
  ConvertPose2DToLineMarker(pure_pursuit_path, header, color, 0.05, vis_path);

  geometry_msgs::PointStamped ref_point;
  ref_point.header = header;
  ref_point.point.x = ref_pose.x;
  ref_point.point.y = ref_pose.y;
  ref_point.point.z = 0.0;

  __bef_twist = twist;
  __pub_vis_path.publish(vis_path);
  __pub_vis_ref_point.publish(ref_point);
  
  return twist;
  //std::cout << "twist_x:" << twist.linear.x << " twist_yaw:" << twist.angular.z << std::endl;
}

geometry_msgs::Twist ProcessInitialAngleCorrection(){
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;

  tf::StampedTransform transform;
  try{
    __tf_listener->lookupTransform(__map_tf_name, __base_tf_name, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return twist;
  }
  geometry_msgs::Pose2D pose = Convert2Pose2D(transform);
  
  double diff_angle = NormalizeAngle(__path[0].theta - pose.theta);
  double omega_max, omega_min;
  __model->GetOmegaLimit(omega_max, omega_min);
  double gain = 1.0;
  double omega = diff_angle * (omega_max / M_PI) * gain; //適当なのでもうちょいちゃんとしたい
  if(omega > 3.5)
    omega = 3.5;
  else if(omega < -3.5)
    omega = -3.5;
  
  twist.linear.x = 0.0;
  twist.angular.z = omega;
  if(std::fabs(diff_angle) < (M_PI * 0.01)){
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    __initial_angle_corrected = true;
  }
  
  __bef_twist = twist;
  return twist;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PurePursuit");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("twist_cmd_topic_name", __twist_cmd_topic_name);
  nh_private.getParam("vis_path_topic_name", __vis_path_topic_name);
  nh_private.getParam("vis_ref_point_topic_name", __vis_ref_point_topic_name);
  nh_private.getParam("motion_plan_topic_name", __motion_plan_topic_name);
  nh_private.getParam("base_tf_name", __base_tf_name);
  nh_private.getParam("map_tf_name", __map_tf_name);
  nh_private.getParam("grid_map_topic_name", __grid_map_topic_name);
  nh_private.getParam("robot_radius", __robot_radius);
  nh_private.getParam("end_radius", __end_radius);
  
  int publish_rate = 50;
  double max_ref_radius = 0.5, min_ref_radius = 0.1, gain = 0.5, pred_time = 0.5;
  int sample_num = 30;
  double max_vel_robot = 1.0, min_vel_robot = 0.0, max_acc_robot = 1.0, min_acc_robot = -1.0;
  double max_omega_robot = 5.0, min_omega_robot = -5.0, max_d_omega_robot = 5.0, min_d_omega_robot = -5.0;
  nh_private.getParam("publish_rate", publish_rate);
  nh_private.getParam("max_ref_radius", max_ref_radius);
  nh_private.getParam("min_ref_radius", min_ref_radius);
  nh_private.getParam("gain", gain);
  nh_private.getParam("sample_num", sample_num);
  nh_private.getParam("pred_time", pred_time);
  nh_private.getParam("max_vel_robot", max_vel_robot);
  nh_private.getParam("min_vel_robot", min_vel_robot);
  nh_private.getParam("max_acc_robot", max_acc_robot);
  nh_private.getParam("min_acc_robot", min_acc_robot);
  nh_private.getParam("max_omega_robot", max_omega_robot);
  nh_private.getParam("min_omega_robot", min_omega_robot);
  nh_private.getParam("max_d_omega_robot", max_d_omega_robot);
  nh_private.getParam("min_d_omega_robot", min_d_omega_robot);
  
  __pure_pursuit = new PurePursuitWithVelocity(max_ref_radius, min_ref_radius, gain, (double)publish_rate, sample_num, pred_time);
  __model = new TwoWheeledRobotModel(max_acc_robot, min_acc_robot, max_d_omega_robot, min_d_omega_robot,
      max_vel_robot, min_vel_robot, max_omega_robot, min_omega_robot);
  __pure_pursuit->SetRobotModel(*__model);

  ros::Subscriber sub_motion_plan, sub_grid_map;
  sub_motion_plan = nh.subscribe(__motion_plan_topic_name, 1, MotionPlanCB);
  sub_grid_map = nh.subscribe(__grid_map_topic_name, 1, GridMapCB);
  
  __pub_twist = nh.advertise<geometry_msgs::Twist>(__twist_cmd_topic_name, 1);
  __pub_vis_path = nh.advertise<visualization_msgs::Marker>(__vis_path_topic_name, 1);
  __pub_vis_ref_point = nh.advertise<geometry_msgs::PointStamped>(__vis_ref_point_topic_name, 1);
  
  __tf_listener = new tf::TransformListener;
  __bef_twist.linear.x = 0.0;
  __bef_twist.angular.z = 0.0;

  ros::Rate rate(publish_rate);
  while(ros::ok()){
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0; twist.angular.z = 0.0;
    if(__path_received){
      if(__initial_angle_corrected)
        twist = ProcessPurePursuit();
      else
        twist = ProcessInitialAngleCorrection();
    }
    __pub_twist.publish(twist);
    ros::spinOnce();
    rate.sleep();
  }

  delete __tf_listener;
  delete __pure_pursuit;
  delete __model;
}
