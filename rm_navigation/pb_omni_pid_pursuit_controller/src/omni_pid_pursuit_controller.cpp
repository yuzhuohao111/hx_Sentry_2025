// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace pb_omni_pid_pursuit_controller
{

void OmniPidPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 1.0;
  double control_frequency = 20.0;
  max_robot_pose_search_dist_ = getCostmapMaxExtent();
  // 初始化脱困状态
  escape_count_ = 0;
  in_escape_phase_ = false;
  escape_start_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  // 初始化卡住状态
  stuck_start_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  is_stuck_ = false;

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bihuanv_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bihuanv_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bihuanv_kd", rclcpp::ParameterValue(0.3));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bihuanw_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bihuanw_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bihuanw_kd", rclcpp::ParameterValue(0.3));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".limit_i_v", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".limit_i_w", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".limit_i_v1", rclcpp::ParameterValue(0.9));

  // 添加脱困参数声明
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".escape_duration", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_actual_speed_threshold", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_command_speed_threshold", rclcpp::ParameterValue(1.0));

  // 获取脱困参数
  node->get_parameter(plugin_name_ + ".escape_duration", escape_duration_);
  node->get_parameter(plugin_name_ + ".min_actual_speed_threshold", min_actual_speed_threshold_);
  node->get_parameter(plugin_name_ + ".max_command_speed_threshold", max_command_speed_threshold_);

  node->get_parameter(plugin_name_ + ".limit_i_v", limit_i_v);
  node->get_parameter(plugin_name_ + ".limit_i_v1", limit_i_v1);
  node->get_parameter(plugin_name_ + ".limit_i_w", limit_i_w);
  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);

  node->get_parameter(plugin_name_ + ".bihuanv_kp", bihuanv_kp_);
  node->get_parameter(plugin_name_ + ".bihuanv_ki", bihuanv_ki_);
  node->get_parameter(plugin_name_ + ".bihuanv_kd", bihuanv_kd_);

  node->get_parameter(plugin_name_ + ".bihuanw_kp", bihuanw_kp_);
  node->get_parameter(plugin_name_ + ".bihuanw_ki", bihuanw_ki_);
  node->get_parameter(plugin_name_ + ".bihuanw_kd", bihuanw_kd_);

  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(
    plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_,
      "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(
    plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(
    plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);

  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_points_pub_ =
    node_.lock()
      ->create_publisher<visualization_msgs::msg::MarkerArray>(  // 初始化 MarkerArray Publisher
        "curvature_points_marker_array", rclcpp::QoS(10));
  flag_pub_ = node->create_publisher<rm_decision_interfaces::msg::NavFlag>("nav_flag", 1);
  mode_sub_ = node->create_subscription<rm_decision_interfaces::msg::Mode>(
        "/mode",  // 确保与发送方的主题一致
        rclcpp::SensorDataQoS(),
      std::bind(&OmniPidPursuitController::modeCallback, this, std::placeholders::_1));
  mode1_sub_ = node->create_subscription<rm_decision_interfaces::msg::Mode>(
        "/mode_nav",  // 确保与发送方的主题一致
        rclcpp::SensorDataQoS(),
      std::bind(&OmniPidPursuitController::mode1Callback, this, std::placeholders::_1));
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry",  // 里程计话题名称，可根据实际情况修改
    rclcpp::SensorDataQoS(),
    std::bind(&OmniPidPursuitController::odomCallback, this, std::placeholders::_1));

  move_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
    translation_ki_);
  bihuanv_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, bihuanv_kp_, bihuanv_kd_,
    bihuanv_ki_);
  bihuanw_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, bihuanw_kp_, bihuanw_kd_,
    bihuanw_ki_);
  heading_pid_ = std::make_shared<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);

  
}

void OmniPidPursuitController::modeCallback(const rm_decision_interfaces::msg::Mode::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    current_mode_ = *msg;
    // RCLCPP_INFO(get_logger(), "云台模式 updated: %d", current_mode_.mode);
}
void OmniPidPursuitController::mode1Callback(const rm_decision_interfaces::msg::Mode::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mode1_mutex_);
    current_mode1_ = *msg;
    // RCLCPP_INFO(get_logger(), "云台模式 updated: %d", current_mode_.mode);
}

// 修改odomCallback函数，添加线程保护
void OmniPidPursuitController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  current_linear_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  // RCLCPP_INFO(logger_,"1111current_linear_speed_%f",current_linear_speed_);
  current_angular_speed_ = msg->twist.twist.angular.z;
}

void OmniPidPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
  flag_pub_.reset();
}

void OmniPidPursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();
  flag_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

void OmniPidPursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  flag_pub_->on_deactivate();
  dyn_params_handler_.reset();
  
}

geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  // 使用互斥锁保护代码段，防止多线程竞争
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // 获取当前时间
  auto now = clock_->now();

  // 获取当前代价地图
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  // 获取代价地图的互斥锁，确保在操作代价地图时不会被其他线程修改
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));


  // 将全局路径转换到机器人基坐标系下
  auto transformed_plan = transformGlobalPlan(pose);

  // 计算前瞻距离，并找到路径上的前瞻点，然后发布该点
  double lookahead_dist = getLookAheadDistance(velocity);

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  // 计算机器人到前瞻点的直线距离和角度
  double lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y); //-pi到pi
  double theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);


  double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);
  // RCLCPP_INFO(logger_,"22222222222222theta_dist:%f",theta_dist);


  // 使用PID控制器计算线速度和角速度
  auto lin_vel = move_pid_->calculate(lin_dist, 0, limit_i_v);
  
  // auto lin_velbi = bihuanv_pid_->calculate(current_linear_speed_, lin_vel , limit_i_v1);
  // RCLCPP_INFO(logger_,"2222lin_vel:%f",lin_vel);

  double angular_vel = 0.0;
  
  if((current_mode1_.mode==2&& current_mode_.mode==12 )||(current_mode1_.mode==4&& current_mode_.mode==14 ))
  { 
    angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal ,0, limit_i_w) : 0.0;
    //  RCLCPP_INFO(logger_,"222222222222222222");
  }
  else if(current_mode_.mode==4)
  { 
    angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal ,0, limit_i_w) : 0.0;
    //  RCLCPP_INFO(logger_,"222222222222222222");
  }
  else if((current_mode1_.mode==1&& current_mode_.mode==11 )||(current_mode1_.mode==3&& current_mode_.mode==13 ))
  {
     angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal+0.367, 0, limit_i_w) : 0.0;
    // RCLCPP_INFO(logger_,"11111111111111111111");

  }
  else
  {
     angular_vel = enable_rotation_ ? heading_pid_->calculate(theta_dist, 0, limit_i_w) : 0.0;
    // RCLCPP_INFO(logger_,"00000000000000000");

  }
  // 应用曲率限制，确保路径的平滑性
  // applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);

  // 应用接近目标时的速度缩放
  applyApproachVelocityScaling(pose, lin_vel);//pose
  // RCLCPP_INFO(logger_,"2222lin_vel:%f",lin_velbi);
  // 创建速度命令消息
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = now;
  cmd_vel.header.frame_id = "base_link";

  // float buchang;
  // float angle;
  // if(fabs(theta_dist)<=1.57)
  // {
  //   buchang=-0.24;
  // }
  // else if(fabs(theta_dist)>1.57){
  //   buchang=-0.24;
  // }
  // angle = theta_dist+buchang;
  // if(angle >3.14){
  //   angle =-6.28+angle;
  // }
  // else if(angle <-3.14){
  //   angle = 6.28+angle;
  // }
  cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);
  cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);


  cmd_vel.twist.angular.z = angular_vel;
  
  // 计算命令的线速度大小
  double command_linear_speed = std::hypot(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y);
  
  // 获取实际线速度（带线程保护）
  double actual_linear_speed;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    actual_linear_speed = current_linear_speed_;
  }

  RCLCPP_DEBUG(
    logger_, 
    "Command speed: %.3f m/s, Actual speed: %.3f m/s, Stuck threshold: %.3f/%.3f",
    command_linear_speed, actual_linear_speed,
    max_command_speed_threshold_, min_actual_speed_threshold_
  );

  double x1=6.0;//狗洞1判断线
  double x2=10.0;
  double x3=13.0;
  double x4=11.0;

  // 获取目标点的x坐标
  double target_x = 0.0;
  if (!global_plan_.poses.empty()) {
    target_x = global_plan_.poses.back().pose.position.x;
    // RCLCPP_INFO(logger_, "Goal set at (%.2f)", target_x);
  }

  // 发布导航失败标志
  rm_decision_interfaces::msg::NavFlag nav_flag_msg;
  nav_flag_msg.nav_flag = 0;

  // 检测区域变化并重置脱困计数
  static int last_region = -1;
  int current_region = -1;
  
  if (current_mode_.mode == 11 && target_x > x1) current_region = 1;
  else if (current_mode_.mode == 11 && target_x <= x1) current_region = 2;
  else if (current_mode_.mode == 12 && target_x > x2) current_region = 3;
  else if (current_mode_.mode == 12 && target_x <= x2) current_region = 4;
  else if (current_mode_.mode == 13 && target_x > x3) current_region = 5;
  else if (current_mode_.mode == 13 && target_x <= x3) current_region = 6;
  else if (current_mode_.mode == 14 && target_x > x4) current_region = 7;
  else if (current_mode_.mode == 14 && target_x <= x4) current_region = 8;
  
  // 如果区域发生变化，重置脱困计数
  if (last_region != -1 && current_region != last_region) {
    escape_count_ = 0;
    RCLCPP_INFO(logger_, "1111Region changed from %d to %d, reset escape count to 0", last_region, current_region);
  }
  last_region = current_region;

  if ((current_mode_.mode == 11 && current_mode1_.mode == 1) || 
      (current_mode_.mode == 12 && current_mode1_.mode == 2) ||
      (current_mode_.mode == 13 && current_mode1_.mode == 3) ||
      (current_mode_.mode == 14 && current_mode1_.mode == 4)  ) 
  {

    
    // 检查是否卡住（命令速度大但实际速度小）
    if (command_linear_speed > max_command_speed_threshold_ && 
        actual_linear_speed < min_actual_speed_threshold_) {
      
      // 如果是第一次检测到卡住，记录开始时间
      if (!is_stuck_) {
        stuck_start_time_ = now;
        is_stuck_ = true;
        // RCLCPP_INFO(logger_, "Stuck condition detected. Starting timer...");
      }
      
      // 计算卡住持续时间
      double stuck_duration = (now - stuck_start_time_).seconds();
      // RCLCPP_INFO(logger_, "Stuck for %.1f seconds (threshold: %.1f)", 
      //             stuck_duration, stuck_duration_threshold_);

      // 检查是否达到卡住持续时间阈值
      if (stuck_duration >= stuck_duration_threshold_) {
        // RCLCPP_WARN(logger_, "Robot stuck for %.1f seconds. Starting escape behavior, count: %d", 
        //             stuck_duration, escape_count_ );
        in_escape_phase_ = true;
        escape_start_time_ = now;
        is_stuck_ = false;  // 重置卡住状态
      }
      
    } else {
      // 没有卡住，重置卡住状态
      if (is_stuck_) {
        // RCLCPP_INFO(logger_, "Stuck condition resolved. Resetting timer.");
        is_stuck_ = false;
      }
      
      // // 如果没有卡住，重置脱困计数
      // if (!in_escape_phase_) {
      //   escape_count_ = 0;
      // }
    }
  

    // 检查是否正在脱困
    if (in_escape_phase_) {
        // 检查脱困时间是否结束
        if ((now - escape_start_time_).seconds() >= escape_duration_) {
            in_escape_phase_ = false;
            
            // 先检查是否已经达到最大尝试次数
            if (escape_count_ >= 2) {

                if(current_mode1_.mode == 1||current_mode1_.mode == 4)
                {
                  escape_count_ = 0;
                  nav_flag_msg.nav_flag = 1;
                }
                else if(current_mode1_.mode == 2||current_mode1_.mode == 3)
                {
                  escape_count_ = 0;
                  nav_flag_msg.nav_flag = 2;
                }
                flag_pub_->publish(nav_flag_msg);                 
                // 返回0速度
                geometry_msgs::msg::TwistStamped cmd_vel;
                cmd_vel.header.stamp = now;
                cmd_vel.header.frame_id = "base_link";
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.linear.y = 0.0;
                cmd_vel.twist.angular.z = 0.0;
                return cmd_vel;
            }
            else {
                // 未达到最大尝试次数时递增计数器
                escape_count_++;
               
                RCLCPP_WARN(logger_, "Escape attempt %d completed", escape_count_);
            }

        }
        else {
            geometry_msgs::msg::TwistStamped cmd_vel;
            double vx = 1.5;
            cmd_vel.header.stamp = now;
            cmd_vel.header.frame_id = "base_link";
            // cmd_vel.twist.linear.x = -1.5;
            // cmd_vel.twist.linear.y = 0.0;
            // cmd_vel.twist.angular.z = 0.0;
            if(current_mode1_.mode == 1)
            {
              if(target_x>x1)//认为前进
              {
                cmd_vel.twist.linear.x = -vx;
              }
              else if(target_x<=x1)//认为后退
              {
                cmd_vel.twist.linear.x = vx;
              }
            }
            else if(current_mode1_.mode == 2)
            {
              if(target_x>x2)//认为前进
              {
                cmd_vel.twist.linear.x = -vx;
              }
              else if(target_x<=x2)//认为后退
              {
                cmd_vel.twist.linear.x = vx;
              }
            }
            else if(current_mode1_.mode == 3)
            {
              if(target_x>x3)//认为前进
              {
                cmd_vel.twist.linear.x = -vx;
              }
              else if(target_x<=x3)//认为后退
              {
                cmd_vel.twist.linear.x = vx;
              }
            }            
            else if(current_mode1_.mode == 4)
            {
              if(target_x>x4)//认为前进
              {
                cmd_vel.twist.linear.x = -vx;
              }
              else if(target_x<=x4)//认为后退
              {
                cmd_vel.twist.linear.x = vx;
              }
            }
            cmd_vel.twist.linear.y = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            return cmd_vel;
        }
    }
  }
  
  flag_pub_->publish(nav_flag_msg); 
  // RCLCPP_INFO(logger_, "11111111111111111111Maximum escape nav_flag_msg.nav_flag:%d",nav_flag_msg.nav_flag);

  return cmd_vel;
}
// 设置全局路径
void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path) { global_plan_ = path; }

// 设置速度限制（未实现）
void OmniPidPursuitController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
}

// 将全局路径转换到机器人基坐标系
nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // 如果全局路径为空，抛出异常
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // 获取机器人在全局路径坐标系下的位姿
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // 获取代价地图的最大范围
  double max_costmap_extent = getCostmapMaxExtent();

  // 找到路径上距离机器人最近的点的上限
  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // 找到路径上距离机器人最近的点
  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // 找到路径上距离机器人超过最大转换距离的点
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

  // 定义将全局位姿转换到局部坐标系的lambda函数
  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  // 将全局路径的局部部分转换到机器人坐标系
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // 删除已经通过的全局路径部分，避免重复处理
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  // 如果转换后的路径为空，抛出异常
  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

// 创建胡萝卜点消息
std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // 发布在地图上方，使其突出显示
  return carrot_msg;
}

// 获取路径上的前瞻点
geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  // 找到路径上第一个距离超过前瞻距离的点
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // 如果没有找到符合条件的点，取路径上最后一个点
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // 如果启用了插值，找到两个点之间的线段上距离恰好为前瞻距离的点
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

// 计算线段与圆的交点
geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  // 使用数学公式计算线段与圆的交点
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double d = x1 * y2 - x2 * y1;

  // 确保返回的点在线段上
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - d * d);
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

// 获取代价地图的最大范围
double OmniPidPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

// 将位姿从一个坐标系转换到另一个坐标系
bool OmniPidPursuitController::transformPose(
  const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  // 如果输入位姿的坐标系与目标坐标系相同，直接返回
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // 使用TF库进行坐标转换
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}


// 获取前瞻距离// 获取前瞻距离
double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // 如果启用了速度缩放的前瞻距离，计算并限制距离
  double lookahead_dist = lookahead_dist_;

  // 检查是否启用了速度缩放的前视距离功能
  if (use_velocity_scaled_lookahead_dist_) {
      // 计算当前速度的合速度（使用hypot函数计算二维向量的模）
      // speed.linear.x 是x方向的线速度
      // speed.linear.y 是y方向的线速度
      // lookahead_time_ 是预定义的前视时间参数
      // 前视距离 = 速度大小 × 前视时间
      lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
      
      // 使用std::clamp函数将前视距离限制在最小和最大值之间
      // min_lookahead_dist_ 是最小前视距离
      // max_lookahead_dist_ 是最大前视距离
      // 这样可以确保前视距离不会超出合理范围
      lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }
  // RCLCPP_INFO(logger_,"111111lookahead_dist：%f",lookahead_dist);
  return lookahead_dist;
}

double OmniPidPursuitController::calculateDistanceToGoal(const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  if (global_plan_.poses.empty()) {
    return std::numeric_limits<double>::max();
  }

  // 获取目标位姿
  const auto & goal_pose = global_plan_.poses.back();
  
  // 计算机器人到目标的直线距离
  return std::hypot(
    goal_pose.pose.position.x - robot_pose.pose.position.x,
    goal_pose.pose.position.y - robot_pose.pose.position.y);
}

double OmniPidPursuitController::approachVelocityScalingFactor(
  const geometry_msgs::msg::PoseStamped & robot_pose) const  // 改为接收机器人位姿
{
  // 计算机器人到目标的直线距离
  double remaining_distance = calculateDistanceToGoal(robot_pose);
  
  // RCLCPP_INFO(
  //   logger_, "Remaining distance to goal: %.2f meters", 
    // remaining_distance);
  
  if (remaining_distance < approach_velocity_scaling_dist_) {
    // return 0.0;
    return std::max(
      remaining_distance / approach_velocity_scaling_dist_, 
      min_approach_linear_velocity_ / v_linear_max_);
  }
  return 1.0;
}
void OmniPidPursuitController::applyApproachVelocityScaling(
  const geometry_msgs::msg::PoseStamped & robot_pose,  // 接收机器人位姿
  double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(robot_pose);
  // RCLCPP_INFO(logger_,"22222222222velocity_scaling%f",velocity_scaling);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (fabs(unbounded_vel) < fabs(min_approach_linear_velocity_)) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // 取接近速度和其他约束中的最小值
  if(fabs(linear_vel) <fabs(approach_vel)){
    linear_vel = linear_vel;
  }else {
    linear_vel = approach_vel;
  }

}


// // 计算接近目标时的速度缩放因子
// double OmniPidPursuitController::approachVelocityScalingFactor(
//   const nav_msgs::msg::Path & transformed_path) const
// {
//   // 计算路径上剩余的距离
//   double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
//   RCLCPP_INFO(logger_,"11111111111111%f",remaining_distance);
//   if (remaining_distance < approach_velocity_scaling_dist_) {
//     auto & last = transformed_path.poses.back();
//     // 计算机器人到路径上最后一个点的距离
//     double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
//     return distance_to_last_pose / approach_velocity_scaling_dist_;
//   } else {
//     return 1.0;
//   }
// }

// // 应用接近目标时的速度缩放
// void OmniPidPursuitController::applyApproachVelocityScaling(
//   const nav_msgs::msg::Path & path, double & linear_vel) const
// {
//   double approach_vel = linear_vel;
//   double velocity_scaling = approachVelocityScalingFactor(path);
//   double unbounded_vel = approach_vel * velocity_scaling;
//   if (unbounded_vel < min_approach_linear_velocity_) {
//     approach_vel = min_approach_linear_velocity_;
//   } else {
//     approach_vel *= velocity_scaling;
//   }

//   // 取接近速度和其他约束中的最小值
//   linear_vel = std::min(linear_vel, approach_vel);
// }

void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  // 计算路径的曲率
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

  // 初始化缩放后的线速度
  double scaled_linear_vel = linear_vel;
  // 如果曲率大于最小曲率限制
  if (curvature > curvature_min_) {
    // 初始化速度缩减比例
    double reduction_ratio = 0.8;
    // 如果曲率大于最大曲率限制
    if (curvature > curvature_max_) {
      // 使用高曲率下的缩减比例
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      // 根据曲率在最小和最大曲率之间的比例计算缩减比例
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    // 计算目标缩放后的速度
    double target_scaled_vel = linear_vel * reduction_ratio;
    // 根据上一次的速度缩放因子和最大速度缩放因子变化率，计算新的缩放后的速度
    scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
  }
  // 确保缩放后的速度不小于最小接近速度的两倍
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

  // 更新线速度为缩放后的速度和原始线速度中的较小值
  linear_vel = std::min(linear_vel, scaled_linear_vel);
  // 更新上一次的速度缩放因子
  last_velocity_scaling_factor_ = linear_vel;
}

double OmniPidPursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  // 定义前后两个点的位姿
  geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
  // 计算路径的累积距离
  std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

  // 计算前瞻点的累积距离
  double lookahead_pose_cumulative_distance = 0.0;
  geometry_msgs::msg::PoseStamped robot_base_frame_pose;
  robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
  lookahead_pose_cumulative_distance =
    nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

  // 找到距离前瞻点向后一定距离的位姿
  backward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

  // 找到距离前瞻点向前一定距离的位姿
  forward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

  // 计算曲率半径
  double curvature_radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
  // 计算曲率
  double curvature = 1.0 / curvature_radius;
  // 可视化曲率点
  visualizeCurvaturePoints(backward_pose, forward_pose);
  RCLCPP_INFO(logger_,"11111curvature%f",curvature);
  return curvature;
}

double OmniPidPursuitController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  // 提取三个点的坐标
  double x1 = near_point.x, y1 = near_point.y;
  double x2 = current_point.x, y2 = current_point.y;
  double x3 = far_point.x, y3 = far_point.y;

  // 计算圆心坐标
  double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                     (x3 * x3 + y3 * y3) * (y1 - y2)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                     (x3 * x3 + y3 * y3) * (x2 - x1)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  // 计算半径
  double radius = std::hypot(x2 - center_x, y2 - center_y);
  // 如果半径无效（NaN或无穷大），返回一个很大的值
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  // 创建可视化标记数组
  visualization_msgs::msg::MarkerArray marker_array;

  // 创建近点标记
  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  // 创建远点标记
  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  // 将标记添加到标记数组
  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  // 发布标记数组
  curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  // 初始化累积距离数组
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  // 遍历路径中的每个点，计算累积距离
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  // 如果路径或累积距离为空，返回空位姿
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  // 如果目标距离小于等于0，返回路径的第一个点
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  // 如果目标距离大于等于累积距离的最后一个值，返回路径的最后一个点
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }
  // 使用二分查找找到目标距离的位置
  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  size_t index = std::distance(cumulative_distances.begin(), it);

  // 如果索引为0，返回路径的第一个点
  if (index == 0) {
    return path.poses.front();
  }

  // 计算插值比例
  double ratio = (target_distance - cumulative_distances[index - 1]) /
                 (cumulative_distances[index] - cumulative_distances[index - 1]);
  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  // 插值计算目标位姿
  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}

rcl_interfaces::msg::SetParametersResult OmniPidPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".min_max_sum_error") {
        min_max_sum_error_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".use_rotate_to_heading_treshold") {
        use_rotate_to_heading_treshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

};  // namespace pb_omni_pid_pursuit_controller
// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)