#include "state_machine_node.hpp"

static tf::TransformListener *__tf_listener;
static std::string __local_map_topic_name("local_map");
static std::string __base_tf_name("base_link");
static std::string __map_tf_name("map");
static std::string __goal_topic_name("goal");
static std::string __path_topic_name("global_path");
static std::string __vis_goal_topic_name("vis_goal");
static std::string __vis_vel_topic_name("vis_plot_vel");
static std::string __vis_omega_topic_name("vis_plot_omega");
static std::string __vis_state_num_topic_name("vis_state_num");
static std::string __vis_state_text_topic_name("vis_state_text");
static std::string __matching_result_topic_name("map_matching_result");
static std::string __pub_twist_topic_name("/neet_bot/diff_drive_controller/cmd_vel");
static std::string __sub_twist_topic_name("/pure_pursuit_twist");
static std::string __rrt_star_service_name("/rrt_star_planner");
static ros::Publisher __pub_twist;
static ros::Publisher __pub_path;
static ros::Publisher __pub_vis_goal;
static ros::Publisher __pub_vis_vel;
static ros::Publisher __pub_vis_omega;
static ros::Publisher __pub_vis_state_num;
static ros::Publisher __pub_vis_state_text;
static ros::ServiceClient __service_rrt_star;

static ObstacleGridMap __obs_map;
static nav_msgs::OccupancyGrid::ConstPtr __now_map = NULL;
static std::deque<nav_msgs::OccupancyGrid::ConstPtr> __stuck_map_queue;
static double __robot_radius = 0.13;
static double __goal_radius = 0.2;

static int __planning_retry_num = 3;
static int __matching_fail_limit = 10;
static double __twist_time_out = 2.0;
static double __goal_judgment_time = 1.0;
static double __stuck_judgment_time = 2.0;
static double __stuck_map_use_limit = 30.0;

enum State{ FAULT, STANDBY, EXECUTE, STUCK };

static geometry_msgs::Twist __req_twist;
static ros::Time __req_twist_time;
static std::deque<geometry_msgs::PoseStamped> __goal_queue;
static planner_msgs::Pose2DArray __global_path;
static bool __matching_result = false;

inline void PubGoalPos(const std::deque<geometry_msgs::PoseStamped> &_goal_queue){
  std_msgs::Header header;
  header.frame_id = __map_tf_name;
  header.stamp = ros::Time::now();
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "goals";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.emplace_back(marker);
  for(int i = 0; i < _goal_queue.size(); i++){
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "goals";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = _goal_queue[i].pose;
    marker.scale.z = 0.5;
    marker.color.r = 0.0; marker.color.g = 0.4; marker.color.b = 1.0; marker.color.a = 1.0;
    marker.text = std::to_string(i + 1);
    marker_array.markers.emplace_back(marker);
  }

  __pub_vis_goal.publish(marker_array);
}

inline void PubOverRayMessage(const State &_state, const geometry_msgs::Twist &_twist){
  float state_num = (float)(State::FAULT);
  jsk_rviz_plugins::OverlayText text;
  text.action = jsk_rviz_plugins::OverlayText::ADD;
  text.font = "Ricty Diminished";
  text.bg_color.a = 0.0;
  switch(_state){
    case State::STANDBY:
      state_num = (float)(State::STANDBY);
      text.text = "\"STANDBY\"";
      text.fg_color.r = 1.0; text.fg_color.g = 1.0; text.fg_color.b = 0.0; text.fg_color.a = 0.8;
      break;
    case State::EXECUTE:
      state_num = (float)(State::EXECUTE);
      text.text = "\"EXECUTE\"";
      text.fg_color.r = 0.0; text.fg_color.g = 1.0; text.fg_color.b = 0.0; text.fg_color.a = 0.8;
      break;
    case State::STUCK:
      state_num = (float)(State::STUCK);
      text.text = "\" STUCK \"";
      text.fg_color.r = 0.0; text.fg_color.g = 1.0; text.fg_color.b = 1.0; text.fg_color.a = 0.8;
      break;
    default:
      state_num = (float)(State::FAULT);
      text.text = "\" FAULT \"";
      text.fg_color.r = 1.0; text.fg_color.g = 0.0; text.fg_color.b = 0.0; text.fg_color.a = 0.8;
      break;
  }

  std_msgs::Float32 tmp_msg;
  __pub_vis_state_text.publish(text);
  tmp_msg.data = state_num;
  __pub_vis_state_num.publish(tmp_msg);
  tmp_msg.data = (float)_twist.linear.x;
  __pub_vis_vel.publish(tmp_msg);
  tmp_msg.data = (float)_twist.angular.z;
  __pub_vis_omega.publish(tmp_msg);
}

void LocalMapCB(const nav_msgs::OccupancyGrid::ConstPtr &_ocp_grid){
  __now_map = _ocp_grid;
  __obs_map.InputOccupancyGrid(*_ocp_grid);
  __obs_map.Inflate(__robot_radius);
}
void GoalPoseCB(const geometry_msgs::PoseStamped::ConstPtr &_pose){
  __goal_queue.emplace_back(*_pose);
  PubGoalPos(__goal_queue); //表示
}
void TwistCB(const geometry_msgs::Twist::ConstPtr &_twist){
  __req_twist = *_twist;
  __req_twist_time = ros::Time::now();
}
void MatchingResultCB(const localization_msgs::MatchingResult::ConstPtr &_result){
  static int count = 0;
  if(_result->success){
    count = 0;
    __matching_result = true;
  }
  else{
    count++;
    if(count > __matching_fail_limit)
      __matching_result = false;
  }
}

inline State ProcStanby(const geometry_msgs::Pose2D &_pose){
  PubGoalPos(__goal_queue); //表示
  
  if(!__matching_result || __obs_map.GetValue(_pose.x, _pose.y) == ObstacleGridMap::EXIST){
    return State::FAULT;
  }
  else{
    if(__goal_queue.empty())
      return State::STANDBY;

    planner_msgs::PlanningRequest req;
    req.request.goal = __goal_queue.front();
    bool rrt_star_result = false;
    for(int i = 0; i < __planning_retry_num; i++){
      rrt_star_result = __service_rrt_star.call(req);
      if(rrt_star_result)
        break;
    }

    if(rrt_star_result){
      __global_path = req.response.path;
      __pub_path.publish(req.response.path);
      return State::EXECUTE;
    }
  }

  //__goal_queue.pop_front();
  return State::STANDBY; //スキップ
}

inline State ProcExecute(const geometry_msgs::Pose2D &_pose){
  static bool is_first = true;
  static geometry_msgs::Twist bef_twist;
  static double stop_elapsed_time = 0.0;
  static ros::Time stop_start_time;
  if(is_first){
    bef_twist.linear.x = 0.0; bef_twist.angular.z = 0.0;
    stop_start_time = ros::Time::now();
    is_first = false;
  }
  else if((bef_twist.linear.x != 0.0 || bef_twist.angular.z != 0.0) &&
      (__req_twist.linear.x == 0.0 && __req_twist.angular.z == 0.0)){
    stop_start_time = ros::Time::now();
  }

  if(__req_twist.linear.x == 0.0 && __req_twist.angular.z == 0.0)
    stop_elapsed_time = (ros::Time::now() - stop_start_time).toSec();
  else
    stop_elapsed_time = 0.0;

  bef_twist = __req_twist;

  geometry_msgs::Pose2D tmp_goal;
  tmp_goal.x = __goal_queue.front().pose.position.x; tmp_goal.y = __goal_queue.front().pose.position.y;
  double dist_goal = Distance(_pose, tmp_goal);
  if(!__matching_result ||  (ros::Time::now() - __req_twist_time).toSec() > __twist_time_out){
    is_first = true;
    return State::FAULT;
  }
  else if(dist_goal < __goal_radius && stop_elapsed_time > __goal_judgment_time){
    is_first = true;
    __goal_queue.pop_front();
    return State::STANDBY;
  }
  else if(dist_goal >= __goal_radius && stop_elapsed_time > __stuck_judgment_time){
    is_first = true;
    return State::STUCK;
  }
  return State::EXECUTE;
}

inline State ProcStuck(const geometry_msgs::Pose2D &_pose){
  if(!__matching_result || __obs_map.GetValue(_pose.x, _pose.y) == ObstacleGridMap::EXIST){
    return State::FAULT;
  }
  else{
    ros::Time now = ros::Time::now();
    while(!__stuck_map_queue.empty()){ //古いの削除
      if((now - __stuck_map_queue.front()->header.stamp).toSec() > __stuck_map_use_limit)
        __stuck_map_queue.pop_front();
      else
        break;
    }

    planner_msgs::PlanningRequest req;
    req.request.goal = __goal_queue.front();
    for(int i = 0; i < __stuck_map_queue.size(); i++){
      req.request.additional_maps.emplace_back(*__stuck_map_queue[i]);
    }
    bool rrt_star_result = false;
    for(int i = 0; i < __planning_retry_num; i++){
      rrt_star_result = __service_rrt_star.call(req);
      if(rrt_star_result)
        break;
    }

    if(rrt_star_result){
      if(__now_map != NULL)
        __stuck_map_queue.emplace_back(__now_map); //次回用に保存
      __global_path = req.response.path;
      __pub_path.publish(req.response.path);
      return State::EXECUTE;
    }
  }
  return State::FAULT;
}

inline State ProcFault(const geometry_msgs::Pose2D &_pose){
  if(__matching_result && !(__obs_map.GetValue(_pose.x, _pose.y) == ObstacleGridMap::EXIST)){
    return State::STANDBY;
  }
  return State::FAULT;
}

geometry_msgs::Twist ProcStateMachine()
{
  static State state = State::STANDBY;
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0; twist.angular.z = 0.0;

  tf::StampedTransform transform;
  try{
    __tf_listener->lookupTransform(__map_tf_name, __base_tf_name, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return twist;
  }
  geometry_msgs::Pose2D pose = Convert2Pose2D(transform);

  switch(state){
    case State::STANDBY:
      state = ProcStanby(pose);
      //std::cout << "STANDBY" << std::endl;
      break;
    case State::EXECUTE:
      state = ProcExecute(pose);
      twist = __req_twist;
      //std::cout << "EXECUTE" << std::endl;
      break;
    case State::STUCK:
      state = ProcStuck(pose);
      //std::cout << "STUCK" << std::endl;
      break;
    case State::FAULT:
      state = ProcFault(pose);
      //std::cout << "FAULT" << std::endl;
      break;
    default:
      state = State::FAULT;
      //std::cout << "DEFAULT" << std::endl;
      break;
  }

  PubOverRayMessage(state, twist);

  return twist;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "State_Machine");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("local_map_topic_name", __local_map_topic_name);
  nh_private.getParam("base_tf_name", __base_tf_name);
  nh_private.getParam("map_tf_name", __map_tf_name);
  nh_private.getParam("goal_topic_name", __goal_topic_name);
  nh_private.getParam("path_topic_name", __path_topic_name);
  nh_private.getParam("vis_goal_topic_name", __vis_goal_topic_name);
  nh_private.getParam("vis_vel_topic_name", __vis_vel_topic_name);
  nh_private.getParam("vis_omega_topic_name", __vis_omega_topic_name);
  nh_private.getParam("vis_state_num_topic_name", __vis_state_num_topic_name);
  nh_private.getParam("vis_state_text_topic_name", __vis_state_text_topic_name);
  nh_private.getParam("matching_result_topic_name", __matching_result_topic_name);
  nh_private.getParam("pub_twist_topic_name", __pub_twist_topic_name);
  nh_private.getParam("sub_twist_topic_name", __sub_twist_topic_name);
  nh_private.getParam("rrt_star_service_name", __rrt_star_service_name);

  nh_private.getParam("robot_radius", __robot_radius);
  nh_private.getParam("goal_radius", __goal_radius);
  nh_private.getParam("planning_retry_num", __planning_retry_num);
  nh_private.getParam("matching_fail_limit", __matching_fail_limit);
  nh_private.getParam("twist_time_out", __twist_time_out);
  nh_private.getParam("goal_judgment_time", __goal_judgment_time);
  nh_private.getParam("stuck_judgment_time", __stuck_judgment_time);
  nh_private.getParam("stuck_map_use_limit", __stuck_map_use_limit);

  __pub_twist = nh.advertise<geometry_msgs::Twist>(__pub_twist_topic_name, 1);
  __pub_path = nh.advertise<planner_msgs::Pose2DArray>(__path_topic_name, 1);
  __pub_vis_goal = nh.advertise<visualization_msgs::MarkerArray>(__vis_goal_topic_name, 5);
  __pub_vis_vel = nh.advertise<std_msgs::Float32>(__vis_vel_topic_name, 5);
  __pub_vis_omega = nh.advertise<std_msgs::Float32>(__vis_omega_topic_name, 5);
  __pub_vis_state_num = nh.advertise<std_msgs::Float32>(__vis_state_num_topic_name, 5);
  __pub_vis_state_text = nh.advertise<jsk_rviz_plugins::OverlayText>(__vis_state_text_topic_name, 5);

  ros::Subscriber sub_local_map, sub_goal, sub_twist, sub_matching_result;
  sub_local_map = nh.subscribe(__local_map_topic_name, 1, LocalMapCB);
  sub_goal = nh.subscribe(__goal_topic_name, 5, GoalPoseCB);
  sub_twist = nh.subscribe(__sub_twist_topic_name, 1, TwistCB);
  sub_matching_result = nh.subscribe(__matching_result_topic_name, 1, MatchingResultCB);

  __service_rrt_star = nh.serviceClient<planner_msgs::PlanningRequest>(__rrt_star_service_name);

  __req_twist.linear.x = 0.0; __req_twist.angular.z = 0.0;
  __req_twist_time = ros::Time::now();

  __tf_listener = new tf::TransformListener;

  ros::Rate rate(100);
  while(ros::ok()){
    ros::spinOnce();
    geometry_msgs::Twist twist = ProcStateMachine();
    __pub_twist.publish(twist);
    rate.sleep();
  }

  delete __tf_listener;
}
