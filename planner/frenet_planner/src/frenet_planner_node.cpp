#include "frenet_planner_node.hpp"

static tf::TransformListener *__tf_listener;
static std::string __vis_paths_topic_name("frenet_paths");
static std::string __vis_path_topic_name("frenet_path");
static std::string __grid_map_topic_name("local_map");
static std::string __base_tf_name("base_link");
static std::string __map_tf_name("map");
static std::string __motion_plan_topic_name("motion_plan");
static std::string __global_path_topic_name("global_path");
static std::string __obs_map_topic_name("obs_map");
static std::string __twist_topic_name("/neet_bot/diff_drive_controller/cmd_vel");
static ros::Publisher __pub_vis_paths;
static ros::Publisher __pub_vis_path;
static ros::Publisher __pub_motion_plan;
static ros::Publisher __pub_obs_map;

static FrenetPlanner *__frenet_planner;
static bool __global_path_received = false;
static unsigned int __global_path_id;

static double __robot_radius = 0.145;

static VelocityLimitter *__velocity_limitter;
static double __max_acc = 0.5;
static double __max_lat_acc = 0.35;
static double __max_vel = 0.75;
static double __min_vel = 0.05;

static geometry_msgs::Twist __twist;

void TwistCB(const geometry_msgs::Twist::ConstPtr &_twist){
  __twist = *_twist;
}

void GridMapCB(const nav_msgs::OccupancyGrid::ConstPtr &_ocp_grid)
{
  //std::cout << "Subscribed GridMap" << std::endl;
  __frenet_planner->SetGridMap(*_ocp_grid, __robot_radius);

  const ObstacleGridMap* grid = __frenet_planner->GetGridMap();
  std_msgs::Header header;
  header.frame_id = __map_tf_name;
  header.stamp = ros::Time::now();
  nav_msgs::OccupancyGrid ocp;
  grid->Convert2OccupancyGrid(header, ocp);
  __pub_obs_map.publish(ocp);
}

void GlobalPathCB(const planner_msgs::Pose2DArray::ConstPtr &_g_path)
{
  //std::cout << "Subscribed GlobalPath" << std::endl;
  if(!_g_path->array.empty()){
    __frenet_planner->SetGlobalPath(_g_path->array);
    __global_path_id = _g_path->id;
    __global_path_received = true;
  }
}

void ProcessLocalPath()
{
  tf::StampedTransform transform;
  try{
    __tf_listener->lookupTransform(__map_tf_name, __base_tf_name, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }

  planner_msgs::MotionPlan motion_plan;
  geometry_msgs::Pose2D pose = Convert2Pose2D(transform);

  std_msgs::Header header;
  header.frame_id = __map_tf_name;
  header.stamp = ros::Time::now();
  
  FrenetPlanner::GenerateResult res = __frenet_planner->Generate(pose, motion_plan.path.array);
  motion_plan.path.header = header;
  motion_plan.path.id = __global_path_id;
  __velocity_limitter->PathVelocityCalculate(motion_plan.path.array, __twist.linear.x, __max_acc, motion_plan.velocity_list);
  __pub_motion_plan.publish(motion_plan);
    
  visualization_msgs::Marker vis_paths;
  __frenet_planner->GenerateVisualizationMsg(header, vis_paths);
  __pub_vis_paths.publish(vis_paths);

  visualization_msgs::Marker vis_path;
  std_msgs::ColorRGBA color; color.r = 0.0; color.g = 0.50; color.b = 0.0; color.a = 0.8;
  ConvertPose2DToLineMarker(motion_plan.path.array, header, color, 0.03, vis_path);
  __pub_vis_path.publish(vis_path);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Frenet_Planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("grid_map_topic_name", __grid_map_topic_name);
  nh_private.getParam("base_tf_name", __base_tf_name);
  nh_private.getParam("map_tf_name", __map_tf_name);
  nh_private.getParam("vis_path_topic_name", __vis_path_topic_name);
  nh_private.getParam("vis_paths_topic_name", __vis_paths_topic_name);
  nh_private.getParam("motion_plan_topic_name", __motion_plan_topic_name);
  nh_private.getParam("global_path_topic_name", __global_path_topic_name);
  nh_private.getParam("obs_map_topic_name", __obs_map_topic_name);
  nh_private.getParam("twist_topic_name", __twist_topic_name);

  double max_road_width = 1.0, road_width_step = 0.05, max_pred_dist = 2.5, min_pred_dist = 1.0, pred_dist_step = 0.5;
  double k_jerk = 0.1, k_pred = 0.1, k_dist = 10.0, k_col_dist = 10.0, k_danger = 3.0;
  double lat_vel_limit = 1.25;
  int regenerate_timing = 3;
  bool extend_collision_check = false;
  int publish_rate = 10;
  nh_private.getParam("robot_radius", __robot_radius);
  nh_private.getParam("max_road_width", max_road_width);
  nh_private.getParam("road_width_step", road_width_step);
  nh_private.getParam("max_pred_dist", max_pred_dist);
  nh_private.getParam("min_pred_dist", min_pred_dist);
  nh_private.getParam("pred_dist_step", pred_dist_step);
  nh_private.getParam("k_jerk", k_jerk);
  nh_private.getParam("k_pred", k_pred);
  nh_private.getParam("k_dist", k_dist);
  nh_private.getParam("k_col_dist", k_col_dist);
  nh_private.getParam("k_danger", k_danger);
  nh_private.getParam("lat_vel_limit", lat_vel_limit);
  nh_private.getParam("regenerate_timing", regenerate_timing);
  nh_private.getParam("extend_collision_check", extend_collision_check);
  nh_private.getParam("publish_rate", publish_rate);
  
  nh_private.getParam("max_vel", __max_vel);
  nh_private.getParam("min_vel", __min_vel);
  nh_private.getParam("max_acc", __max_acc);
  nh_private.getParam("max_lat_acc", __max_lat_acc);
  
  __frenet_planner = new FrenetPlanner(max_road_width, road_width_step, max_pred_dist, min_pred_dist, pred_dist_step,
      k_jerk, k_pred, k_dist, k_col_dist, k_danger, lat_vel_limit, regenerate_timing, extend_collision_check);
  __velocity_limitter = new VelocityLimitter(__max_vel, __min_vel, __max_lat_acc);

  __pub_vis_path = nh.advertise<visualization_msgs::Marker>(__vis_path_topic_name, 1);
  __pub_vis_paths = nh.advertise<visualization_msgs::Marker>(__vis_paths_topic_name, 1);
  __pub_motion_plan = nh.advertise<planner_msgs::MotionPlan>(__motion_plan_topic_name, 1);
  __pub_obs_map = nh.advertise<nav_msgs::OccupancyGrid>(__obs_map_topic_name, 1);

  ros::Subscriber sub_grid_map, sub_global_path, sub_twist;
  sub_grid_map = nh.subscribe(__grid_map_topic_name, 1, GridMapCB);
  sub_global_path = nh.subscribe(__global_path_topic_name, 1, GlobalPathCB);
  sub_twist = nh.subscribe(__twist_topic_name, 1, TwistCB);

  __tf_listener = new tf::TransformListener;

  ros::Rate loop_rate(publish_rate);
  while(ros::ok()){
    ros::spinOnce();
    if(__global_path_received){
      ProcessLocalPath();
    }
    loop_rate.sleep();
  }

  delete __tf_listener;
  delete __frenet_planner;
}
