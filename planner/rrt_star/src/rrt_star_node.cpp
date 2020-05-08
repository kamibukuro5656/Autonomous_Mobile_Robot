#include "rrt_star_node.hpp"

static tf::TransformListener *__tf_listener;
static std::string __vis_nodes_topic_name("vis_nodes");
static std::string __vis_edges_topic_name("vis_edges");
static std::string __vis_path_topic_name("vis_path");
static std::string __vis_smoothed_path_topic_name("vis_smoothed_path");
static std::string __grid_map_topic_name("grid_map");
static std::string __c_space_topic_name("cspace_map");
static std::string __local_map_topic_name("local_map");
static std::string __base_tf_name("base_link");
static std::string __map_tf_name("map");
static std::string __goal_topic_name("goal");
static std::string __map_request_topic_name("map_load_request");
static std::string __path_topic_name("global_path");
static std::string __service_name("rrt_star");
static ros::Publisher __pub_vis_nodes;
static ros::Publisher __pub_vis_edges;
static ros::Publisher __pub_vis_path;
static ros::Publisher __pub_vis_smoothed_path;
static ros::Publisher __pub_map_request;
static ros::Publisher __pub_path;
static ros::Publisher __pub_cspace;

static RRTStar *__rrt_star;
static ObstacleGridMap __grid_map;
static ObstacleGridMap __local_map;
static bool __map_received = false;
static bool __local_map_received = false;

static double __robot_radius = 0.15;
static double __prob_threshold = 0.40;
static double __smoothed_path_step = 0.01;
static double __smoothed_path_cspace_offset = 0.05;

static unsigned int __counter = 0;

void LocalMapCB(const nav_msgs::OccupancyGrid::ConstPtr &_ocp_grid)
{
  __local_map.InputOccupancyGrid(*_ocp_grid);
  __local_map_received = true;
}

void GridMapCB(const nav_msgs::OccupancyGrid::ConstPtr &_ocp_grid)
{
  ProbabilityGridMap tmp_prob_map;
  tmp_prob_map.InputOccupancyGrid(*_ocp_grid);
  __grid_map.InputProbabilityMap(tmp_prob_map, __prob_threshold);
  __map_received = true;
}

bool ProcRRTStar(const geometry_msgs::PoseStamped &_goal_pose, const std::vector<nav_msgs::OccupancyGrid> &_additional_map,
  std::vector<geometry_msgs::Pose2D> &_path, std::vector<geometry_msgs::Pose2D> &_smoothed_path){
  if(!__map_received){
    std_msgs::Bool req;
    req.data = true;
    __pub_map_request.publish(req);
    return false;
  }

  tf::StampedTransform transform;
  try{
    __tf_listener->lookupTransform(__map_tf_name, __base_tf_name, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }

  ObstacleGridMap tmp_grid_map = __grid_map;
  if(__local_map_received){
    tmp_grid_map.AddMap(__local_map);
  }
  for(int i = 0; i < _additional_map.size(); i++){
    ObstacleGridMap add_map;
    add_map.InputOccupancyGrid(_additional_map[i]);
    tmp_grid_map.AddMap(add_map);
  }
  __rrt_star->SetGridMap(tmp_grid_map, __robot_radius);
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = __map_tf_name;
  nav_msgs::OccupancyGrid ocp_msg_cspace;
  __rrt_star->GetGridMap()->Convert2OccupancyGrid(header, ocp_msg_cspace);
  __pub_cspace.publish(ocp_msg_cspace);
  
  geometry_msgs::Pose2D start = Convert2Pose2D(transform);
  geometry_msgs::Pose2D goal;

  goal.x = _goal_pose.pose.position.x;
  goal.y = _goal_pose.pose.position.y;
  tf::Quaternion q;
  tf::quaternionMsgToTF(_goal_pose.pose.orientation, q);
  double roll, pitch;
  tf::Matrix3x3(q).getRPY(roll, pitch, goal.theta);
  
  bool result = false;
  PathSmoother path_smoother;
  if(__rrt_star->Generate(start, goal, _path)){
    path_smoother.SetWaypoints(_path);
    tmp_grid_map.Inflate(__robot_radius + __smoothed_path_cspace_offset);
    if(path_smoother.GenerateCollisionFreePath(tmp_grid_map, 100));{
      path_smoother.GenerateDiscreteWaypoints(__smoothed_path_step, _smoothed_path);
      result = true;
    }
  }
  
  visualization_msgs::Marker vis_nodes, vis_edges, vis_path;
  __rrt_star->GenerateVisualizationMsg(header, vis_nodes, vis_edges, vis_path);
  __pub_vis_nodes.publish(vis_nodes);
  __pub_vis_edges.publish(vis_edges);
  __pub_vis_path.publish(vis_path);
  
  visualization_msgs::Marker vis_smoothed_path;
  std_msgs::ColorRGBA color; color.r = 1.0; color.g = 0.5; color.b = 0.0; color.a = 0.8;
  ConvertPose2DToLineMarker(_smoothed_path, header, color, 0.02, vis_smoothed_path);
  __pub_vis_smoothed_path.publish(vis_smoothed_path);
  
  return result;
}

void GoalPoseCB(const geometry_msgs::PoseStamped::ConstPtr &_pose)
{
  std_msgs::Header header;
  header.frame_id = __map_tf_name;
  header.stamp = ros::Time::now();
  
  std::vector<geometry_msgs::Pose2D> path;
  std::vector<geometry_msgs::Pose2D> smoothed_path;
  std::vector<nav_msgs::OccupancyGrid> dummy;
  if(ProcRRTStar(*_pose, dummy, path, smoothed_path)){
    planner_msgs::Pose2DArray path_msg;
    path_msg.header = header;
    path_msg.array = smoothed_path;
    path_msg.id = __counter;
    __pub_path.publish(path_msg);
  }

  __counter++;
}

bool ServiceCB(planner_msgs::PlanningRequest::Request &_req,
                      planner_msgs::PlanningRequest::Response &_path)
{
  std_msgs::Header header;
  header.frame_id = __map_tf_name;
  header.stamp = ros::Time::now();
  
  std::vector<geometry_msgs::Pose2D> path;
  std::vector<geometry_msgs::Pose2D> smoothed_path;

  bool result = ProcRRTStar(_req.goal, _req.additional_maps, path, smoothed_path);
  planner_msgs::Pose2DArray path_msg;
  _path.path.header = header;
  _path.path.array = smoothed_path;
  _path.path.id = __counter;

  __counter++;

  return result;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "RRTStar_Planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("grid_map_topic_name", __grid_map_topic_name);
  nh_private.getParam("local_map_topic_name", __local_map_topic_name);
  nh_private.getParam("goal_topic_name", __goal_topic_name);
  nh_private.getParam("base_tf_name", __base_tf_name);
  nh_private.getParam("map_tf_name", __map_tf_name);
  nh_private.getParam("vis_path_topic_name", __vis_path_topic_name);
  nh_private.getParam("vis_edges_topic_name", __vis_edges_topic_name);
  nh_private.getParam("vis_nodes_topic_name", __vis_nodes_topic_name);
  nh_private.getParam("vis_smoothed_path_topic_name", __vis_smoothed_path_topic_name);
  nh_private.getParam("map_request_topic_name", __map_request_topic_name);
  nh_private.getParam("path_topic_name", __path_topic_name);
  nh_private.getParam("service_name", __service_name);
  nh_private.getParam("c_space_topic_name", __c_space_topic_name);

  int iteration_num = 30000;
  double step_size = 0.15;
  double r_param = 75.0;
  double select_goal_prob = 0.025;
  double time_out = 3.0;
  double improvement_amount = 0.001;
  bool use_informed_rrt = true;
  double potential_gain = 0.0001;
  nh_private.getParam("iteration_num", iteration_num);
  nh_private.getParam("step_size", step_size);
  nh_private.getParam("r_param", r_param);
  nh_private.getParam("select_goal_prob", select_goal_prob);
  nh_private.getParam("use_informed_rrt", use_informed_rrt);
  nh_private.getParam("robot_radius", __robot_radius);
  nh_private.getParam("prob_threshold", __prob_threshold);
  nh_private.getParam("smoothed_path_step", __smoothed_path_step);
  nh_private.getParam("smoothed_path_cspace_offset", __smoothed_path_cspace_offset);
  nh_private.getParam("time_out", time_out);
  nh_private.getParam("improvement_amount", improvement_amount);
  nh_private.getParam("potential_gain", potential_gain);
  __rrt_star = new RRTStarUsingPotential(iteration_num, step_size, r_param, select_goal_prob, use_informed_rrt, 
      time_out, improvement_amount, potential_gain);

  __pub_vis_nodes = nh.advertise<visualization_msgs::Marker>(__vis_nodes_topic_name, 1);
  __pub_vis_edges = nh.advertise<visualization_msgs::Marker>(__vis_edges_topic_name, 1);
  __pub_vis_path = nh.advertise<visualization_msgs::Marker>(__vis_path_topic_name, 1);
  __pub_vis_smoothed_path = nh.advertise<visualization_msgs::Marker>(__vis_smoothed_path_topic_name, 1);
  __pub_map_request = nh.advertise<std_msgs::Bool>(__map_request_topic_name, 1);
  __pub_path = nh.advertise<planner_msgs::Pose2DArray>(__path_topic_name, 5);
  __pub_cspace = nh.advertise<nav_msgs::OccupancyGrid>(__c_space_topic_name, 1);

  ros::Subscriber sub_grid_map, sub_goal, sub_local_map;
  sub_grid_map = nh.subscribe(__grid_map_topic_name, 1, GridMapCB);
  sub_local_map = nh.subscribe(__local_map_topic_name, 1, LocalMapCB);
  sub_goal = nh.subscribe(__goal_topic_name, 5, GoalPoseCB);
  
  ros::ServiceServer rrt_star_service = nh.advertiseService(__service_name, ServiceCB);

  __tf_listener = new tf::TransformListener;

  ros::spin();

  delete __tf_listener;
  delete __rrt_star;
}
