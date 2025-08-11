// Copyright 2024 Polaris Xia
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

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

namespace pb_nav2_behaviors
{

// BackUpFreeSpace类的配置函数，用于初始化参数和服务
void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 声明并获取ROS参数
  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);

  // 创建costmap服务客户端
  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  // 初始化可视化标记发布器
  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

// 资源清理函数，重置服务客户端和发布器
void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

// 执行备份行为的主运行函数
nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  // 等待costmap服务可用
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }

  // 请求并获取当前costmap
  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
    return nav2_behaviors::Status::FAILED;
  }

  // 获取机器人初始位姿
  auto costmap = result.get()->map;
  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }
  
  // 计算最佳后退方向
  geometry_msgs::msg::Pose2D pose;
  pose.x = initial_pose_.pose.position.x;
  pose.y = initial_pose_.pose.position.y;
  pose.theta = tf2::getYaw(initial_pose_.pose.orientation);
  float best_angle = findBestDirection(costmap, pose, -M_PI, M_PI, max_radius_, M_PI / 32.0);

  // twist_x_ = std::cos(best_angle) * command->speed;
  // twist_y_ = std::sin(best_angle) * command->speed;

  best_angle_global_ = best_angle;
  command_speed_ = command->speed;

  command_x_ = command->target.x;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  RCLCPP_WARN(
    logger_, "backing up %f meters towards free space at angle %f", command_x_, best_angle);
  return nav2_behaviors::Status::SUCCEEDED;
}


// 周期更新函数，处理运动控制和状态检查
nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  // 检查超时
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "Exceeded time allowance");
    return nav2_behaviors::Status::FAILED;
  }

  // 获取当前位姿并计算移动距离
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  float distance = hypot(diff_x, diff_y);

  // 发布反馈信息
  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  // 检查是否到达目标距离
  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }


  // 获取当前朝向
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  
  // 将全局方向转换为本体坐标系
  double vx_global = -cos(best_angle_global_);
  double vy_global = -sin(best_angle_global_);
  
  // 应用旋转变换矩阵（全局->本体）
  double vx_local = vx_global * cos(current_yaw) + vy_global * sin(current_yaw);
  double vy_local = -vx_global * sin(current_yaw) + vy_global * cos(current_yaw);
  // 计算速度分量（考虑后退方向）
  double twist_x = -command_speed_ * vx_local; // 后退取反
  double twist_y = -command_speed_ * vy_local;

  twist_x_ = twist_x;
  twist_y_ = twist_y;

  // 创建速度指令
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = twist_x_;
  cmd_vel->linear.y = twist_y_;;


  // 碰撞检测
  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);
  if (!isCollisionFree(distance, cmd_vel.get(), pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));
  return nav2_behaviors::Status::RUNNING;
}

// 寻找最佳后退方向的核心算法
float BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment)
{
  // 初始化安全区域记录变量
  float best_angle = start_angle;
  float first_safe_angle = -1.0f;
  float last_unsafe_angle = -1.0f;
  float final_safe_angle = 0.0f;
  float final_unsafe_angle = 0.0f;

  // 获取costmap元数据
  float resolution = costmap.metadata.resolution;
  float origin_x = costmap.metadata.origin.position.x;
  float origin_y = costmap.metadata.origin.position.y;
  int size_x = costmap.metadata.size_x;
  int size_y = costmap.metadata.size_y;

  // 角度遍历检查安全区域
  for (float angle = start_angle; angle <= end_angle; angle += angle_increment) {
    bool is_safe = true;
    // 半径方向逐点检测
    for (float r = 0.3f; r <= radius; r += resolution) {
      // 计算检测点坐标
      float x = pose.x + r * std::cos(angle);
      float y = pose.y + r * std::sin(angle);
      // 坐标有效性检查
      if (x < origin_x || x >= origin_x + size_x * resolution || 
          y < origin_y || y >= origin_y + size_y * resolution) {
        is_safe = false;
        break;
      }
      // 计算网格索引并检查代价值
      int i = static_cast<int>((x - origin_x) / resolution);
      int j = static_cast<int>((y - origin_y) / resolution);
      if (i >= 0 && i < size_x && j >= 0 && j < size_y) {
        if (costmap.data[i + j * size_x] >= 253) { // 253-255为致命障碍
          is_safe = false;
          break;
        }
      }
    }
    // 更新最大安全扇形区域
    if (is_safe) {
      if (first_safe_angle == -1.0f) first_safe_angle = angle;
    } else {
      if (first_safe_angle != -1.0f) {
        last_unsafe_angle = angle;
        // 比较并记录最大安全区域
        if ((last_unsafe_angle - first_safe_angle) > (final_unsafe_angle - final_safe_angle)) {
          final_safe_angle = first_safe_angle;
          final_unsafe_angle = last_unsafe_angle;
        }
        first_safe_angle = -1.0f;
      }
      last_unsafe_angle = -1.0f;
    }
  }
  // 计算最佳角度（安全区域中点）
  best_angle = (final_safe_angle + final_unsafe_angle) / 2.0f;

  // 可视化处理
  if (visualize_) {
    visualize(pose, radius, final_safe_angle, final_unsafe_angle);
  }

  return best_angle;
}

std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_unsafe_angle, 2, 0.0f, 0.0f, 1.0f));

  const float best_angle = (first_safe_angle + last_unsafe_angle) / 2.0f;
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)