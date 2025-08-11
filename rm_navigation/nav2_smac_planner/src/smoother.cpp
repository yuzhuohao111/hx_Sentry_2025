// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <vector>
#include <memory>
#include "nav2_smac_planner/smoother.hpp"

namespace nav2_smac_planner
{
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT

Smoother::Smoother(const SmootherParams & params)
{
  tolerance_ = params.tolerance_;
  max_its_ = params.max_its_;
  data_w_ = params.w_data_;
  smooth_w_ = params.w_smooth_;
  is_holonomic_ = params.holonomic_;
  do_refinement_ = params.do_refinement_;
}

void Smoother::initialize(const double & min_turning_radius)
{
  min_turning_rad_ = min_turning_radius;
  state_space_ = std::make_unique<ompl::base::DubinsStateSpace>(min_turning_rad_);
}

bool Smoother::smooth(
  nav_msgs::msg::Path & path, // 输入参数：待平滑的路径
  const nav2_costmap_2d::Costmap2D * costmap, // 输入参数：代价地图，用于路径平滑时的障碍物检测
  const double & max_time) // 输入参数：最大允许的平滑时间
{
  // 如果最大迭代次数为0，则跳过路径平滑过程，直接返回false
  if (max_its_ == 0) {
    return false;
  }

  refinement_ctr_ = 0; // 重置平滑过程中的迭代计数器
  steady_clock::time_point start = steady_clock::now(); // 记录平滑过程开始的时间点
  double time_remaining = max_time; // 初始化剩余时间为最大允许时间
  bool success = true, reversing_segment; // 初始化成功标志和是否为反向路径段的标志
  nav_msgs::msg::Path curr_path_segment; // 当前正在处理的路径段
  curr_path_segment.header = path.header; // 复制路径段的头部信息
  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path); // 将路径分割成多个方向一致的路径段

  // 遍历所有路径段
  for (unsigned int i = 0; i != path_segments.size(); i++) {
    // 如果当前路径段的长度大于10，则进行平滑处理
    if (path_segments[i].end - path_segments[i].start > 10) {
      // 清空当前路径段的姿态信息
      curr_path_segment.poses.clear();
      // 将原始路径中对应的路径段复制到当前路径段中
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // 检查剩余时间是否足够进行平滑处理
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time - duration_cast<duration<double>>(now - start).count();

      // 对当前路径段进行平滑处理
      const geometry_msgs::msg::Pose start_pose = curr_path_segment.poses.front().pose; // 获取路径段的起始姿态
      const geometry_msgs::msg::Pose goal_pose = curr_path_segment.poses.back().pose; // 获取路径段的终点姿态
      bool local_success =
        smoothImpl(curr_path_segment, reversing_segment, costmap, time_remaining); // 调用平滑实现函数
      success = success && local_success; // 更新全局成功标志

      // 如果不是全向机器人，则强制执行边界条件
      if (!is_holonomic_ && local_success) {
        enforceStartBoundaryConditions(start_pose, curr_path_segment, costmap, reversing_segment); // 强制执行起始边界条件
        enforceEndBoundaryConditions(goal_pose, curr_path_segment, costmap, reversing_segment); // 强制执行终点边界条件
      }

      // 将平滑后的路径段更新到原始路径中
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success; // 返回平滑是否成功的标志
}
bool Smoother::smoothImpl(
  nav_msgs::msg::Path & path,  // 输入的路径，将被平滑处理
  bool & reversing_segment,  // 是否在反向段中
  const nav2_costmap_2d::Costmap2D * costmap,  // 代价地图，用于检查路径是否可行
  const double & max_time)  // 平滑处理的最大允许时间
{
  steady_clock::time_point a = steady_clock::now();  // 记录当前时间，用于计算处理时间
  rclcpp::Duration max_dur = rclcpp::Duration::from_seconds(max_time);  // 将最大允许时间转换为rclcpp的Duration类型

  int its = 0;  // 迭代次数计数器
  double change = tolerance_;  // 初始化变化量，用于判断平滑是否收敛
  const unsigned int & path_size = path.poses.size();  // 获取路径中点的数量
  double x_i, y_i, y_m1, y_ip1, y_i_org;  // 用于存储路径点的坐标和临时变量
  unsigned int mx, my;  // 用于存储代价地图中的坐标

  nav_msgs::msg::Path new_path = path;  // 创建一个新的路径，用于存储平滑后的路径
  nav_msgs::msg::Path last_path = path;  // 创建一个备份路径，用于在平滑失败时恢复

  while (change >= tolerance_) {  // 当变化量大于等于容忍度时，继续平滑处理
    its += 1;  // 迭代次数加1
    change = 0.0;  // 重置变化量

    // 确保平滑函数会收敛
    if (its >= max_its_) {  // 如果迭代次数超过最大限制
      RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "Number of iterations has exceeded limit of %i.", max_its_);  // 输出调试信息
      path = last_path;  // 恢复到上一次的路径
      updateApproximatePathOrientations(path, reversing_segment);  // 更新路径的方向
      return false;  // 返回平滑失败
    }

    // 确保仍有时间处理
    steady_clock::time_point b = steady_clock::now();  // 记录当前时间
    rclcpp::Duration timespan(duration_cast<duration<double>>(b - a));  // 计算已用时间
    if (timespan > max_dur) {  // 如果已用时间超过最大允许时间
      RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "Smoothing time exceeded allowed duration of %0.2f.", max_time);  // 输出调试信息
      path = last_path;  // 恢复到上一次的路径
      updateApproximatePathOrientations(path, reversing_segment);  // 更新路径的方向
      return false;  // 返回平滑失败
    }

    for (unsigned int i = 1; i != path_size - 1; i++) {  // 遍历路径中的每个点（不包括首尾点）
      for (unsigned int j = 0; j != 2; j++) {  // 遍历每个点的x和y坐标
        x_i = getFieldByDim(path.poses[i], j);  // 获取原始路径点的坐标
        y_i = getFieldByDim(new_path.poses[i], j);  // 获取新路径点的坐标
        y_m1 = getFieldByDim(new_path.poses[i - 1], j);  // 获取前一个点的坐标
        y_ip1 = getFieldByDim(new_path.poses[i + 1], j);  // 获取后一个点的坐标
        y_i_org = y_i;  // 备份当前点的坐标

        // 基于局部3点邻域和原始数据位置进行平滑
        y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
        setFieldByDim(new_path.poses[i], j, y_i);  // 更新新路径点的坐标
        change += abs(y_i - y_i_org);  // 计算变化量
      }

      // 验证更新是否可行，仅在提供有效代价地图指针时检查代价
      float cost = 0.0;
      if (costmap) {  // 如果提供了代价地图
        costmap->worldToMap(
          getFieldByDim(new_path.poses[i], 0),
          getFieldByDim(new_path.poses[i], 1),
          mx, my);  // 将世界坐标转换为代价地图坐标
        cost = static_cast<float>(costmap->getCost(mx, my));  // 获取代价
      }

      if (cost > MAX_NON_OBSTACLE && cost != UNKNOWN) {  // 如果代价超过非障碍物最大值且不为未知
        RCLCPP_DEBUG(
          rclcpp::get_logger("SmacPlannerSmoother"),
          "Smoothing process resulted in an infeasible collision. "
          "Returning the last path before the infeasibility was introduced.");  // 输出调试信息
        path = last_path;  // 恢复到上一次的路径
        updateApproximatePathOrientations(path, reversing_segment);  // 更新路径的方向
        return false;  // 返回平滑失败
      }
    }

    last_path = new_path;  // 将新路径备份为上一次的路径
  }

  // 进行额外的细化处理，通常不会花费太多时间，但可以显著提高路径质量
  if (do_refinement_ && refinement_ctr_ < 4) {  // 如果启用了细化且细化次数小于4
    refinement_ctr_++;  // 细化次数加1
    smoothImpl(new_path, reversing_segment, costmap, max_time);  // 递归调用平滑函数
  }

  updateApproximatePathOrientations(new_path, reversing_segment);  // 更新新路径的方向
  path = new_path;  // 将新路径赋值给输入路径
  return true;  // 返回平滑成功
}

// Smoother类的成员函数getFieldByDim，用于根据指定的维度获取PoseStamped消息中的位置坐标
double Smoother::getFieldByDim(
  const geometry_msgs::msg::PoseStamped & msg, // 输入参数：一个PoseStamped类型的消息，包含位置和姿态信息
  const unsigned int & dim) // 输入参数：一个无符号整数，表示要获取的维度（0表示x，1表示y，2表示z）
{
  // 如果dim为0，表示要获取x坐标
  if (dim == 0) {
    return msg.pose.position.x; // 返回PoseStamped消息中位置的x坐标
  } 
  // 如果dim为1，表示要获取y坐标
  else if (dim == 1) {
    return msg.pose.position.y; // 返回PoseStamped消息中位置的y坐标
  } 
  // 如果dim既不是0也不是1，则表示要获取z坐标
  else {
    return msg.pose.position.z; // 返回PoseStamped消息中位置的z坐标
  }
}

void Smoother::setFieldByDim(
  geometry_msgs::msg::PoseStamped & msg, // 传入一个PoseStamped类型的消息引用
  const unsigned int dim, // 传入一个无符号整数，表示维度（0代表x，1代表y，2代表z）
  const double & value) // 传入一个双精度浮点数，表示要设置的值
{
  if (dim == 0) { // 如果维度为0
    msg.pose.position.x = value; // 将消息中的x坐标设置为传入的值
  } else if (dim == 1) { // 如果维度为1
    msg.pose.position.y = value; // 将消息中的y坐标设置为传入的值
  } else { // 如果维度既不是0也不是1
    msg.pose.position.z = value; // 将消息中的z坐标设置为传入的值
  }
}

std::vector<PathSegment> Smoother::findDirectionalPathSegments(const nav_msgs::msg::Path & path)
{
  // 定义一个存储路径段的向量
  std::vector<PathSegment> segments;
  // 定义当前路径段
  PathSegment curr_segment;
  // 设置当前路径段的起始点为0
  curr_segment.start = 0;

  // 如果是全向机器人，路径中没有方向变化，可能会有从网格搜索中产生的突然的角变化
  if (is_holonomic_) {
    // 设置当前路径段的结束点为路径的最后一个点
    curr_segment.end = path.poses.size() - 1;
    // 将当前路径段添加到路径段向量中
    segments.push_back(curr_segment);
    // 返回路径段向量
    return segments;
  }

  // 遍历路径以确定尖点的位置
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // 我们有两个向量用于点积 OA 和 AB。确定这些向量。
    double oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    double oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    double ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    double ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // 使用点积检查路径中是否存在尖点
    double dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      // 如果点积小于0，说明存在尖点，设置当前路径段的结束点为当前索引
      curr_segment.end = idx;
      // 将当前路径段添加到路径段向量中
      segments.push_back(curr_segment);
      // 设置下一个路径段的起始点为当前索引
      curr_segment.start = idx;
    }

    // 检查是否存在原地旋转
    double cur_theta = tf2::getYaw(path.poses[idx].pose.orientation);
    double next_theta = tf2::getYaw(path.poses[idx + 1].pose.orientation);
    double dtheta = angles::shortest_angular_distance(cur_theta, next_theta);
    if (fabs(ab_x) < 1e-4 && fabs(ab_y) < 1e-4 && fabs(dtheta) > 1e-4) {
      // 如果存在原地旋转，设置当前路径段的结束点为当前索引
      curr_segment.end = idx;
      // 将当前路径段添加到路径段向量中
      segments.push_back(curr_segment);
      // 设置下一个路径段的起始点为当前索引
      curr_segment.start = idx;
    }
  }

  // 设置最后一个路径段的结束点为路径的最后一个点
  curr_segment.end = path.poses.size() - 1;
  // 将最后一个路径段添加到路径段向量中
  segments.push_back(curr_segment);
  // 返回路径段向量
  return segments;
}

void Smoother::updateApproximatePathOrientations(
  nav_msgs::msg::Path & path,  // 输入的路径对象，包含一系列的位姿点
  bool & reversing_segment)    // 输出参数，表示当前路径段是否为反向行驶
{
  double dx, dy, theta, pt_yaw;  // 定义一些局部变量，用于计算路径的方向
  reversing_segment = false;     // 初始化reversing_segment为false，表示当前路径段不是反向行驶

  // 检查当前路径段是否为反向行驶
  dx = path.poses[2].pose.position.x - path.poses[1].pose.position.x;  // 计算第2个点和第1个点在x轴上的差值
  dy = path.poses[2].pose.position.y - path.poses[1].pose.position.y;  // 计算第2个点和第1个点在y轴上的差值
  theta = atan2(dy, dx);  // 计算这两个点之间的角度
  pt_yaw = tf2::getYaw(path.poses[1].pose.orientation);  // 获取第1个点的方向角
  if (!is_holonomic_ && fabs(angles::shortest_angular_distance(pt_yaw, theta)) > M_PI_2) {  // 如果不是全向机器人且角度差大于90度
    reversing_segment = true;  // 设置reversing_segment为true，表示当前路径段为反向行驶
  }

  // 计算路径中每个点的方向角
  for (unsigned int i = 0; i != path.poses.size() - 1; i++) {  // 遍历路径中的每一个点
    dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;  // 计算当前点和下一个点在x轴上的差值
    dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;  // 计算当前点和下一个点在y轴上的差值
    theta = atan2(dy, dx);  // 计算这两个点之间的角度

    // 如果点重叠，则跳过
    if (fabs(dx) < 1e-4 && fabs(dy) < 1e-4) {
      continue;
    }

    // 如果当前路径段为反向行驶，则翻转角度
    if (reversing_segment) {
      theta += M_PI;  // 翻转角度，orientationAroundZAxis函数会进行归一化处理
    }

    path.poses[i].pose.orientation = orientationAroundZAxis(theta);  // 更新当前点的方向角
  }
}

unsigned int Smoother::findShortestBoundaryExpansionIdx(
  const BoundaryExpansions & boundary_expansions)
{
  // 初始化最小路径长度为一个非常大的值
  double min_length = 1e9;
  // 初始化最短边界扩展的索引为一个非常大的值
  int shortest_boundary_expansion_idx = 1e9;
  
  // 遍历所有的边界扩展
  for (unsigned int idx = 0; idx != boundary_expansions.size(); idx++) {
    // 检查当前边界扩展是否满足以下条件：
    // 1. 扩展路径长度小于当前最小路径长度
    // 2. 扩展路径没有碰撞
    // 3. 扩展路径的终点索引大于0
    // 4. 扩展路径长度大于0
    if (boundary_expansions[idx].expansion_path_length < min_length &&
      !boundary_expansions[idx].in_collision &&
      boundary_expansions[idx].path_end_idx > 0.0 &&
      boundary_expansions[idx].expansion_path_length > 0.0)
    {
      // 更新最小路径长度
      min_length = boundary_expansions[idx].expansion_path_length;
      // 更新最短边界扩展的索引
      shortest_boundary_expansion_idx = idx;
    }
  }

  // 返回最短边界扩展的索引
  return shortest_boundary_expansion_idx;
}
void Smoother::findBoundaryExpansion(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & end,
  BoundaryExpansion & expansion,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  static ompl::base::ScopedState<> from(state_space_), to(state_space_), s(state_space_);

  from[0] = start.position.x;
  from[1] = start.position.y;
  from[2] = tf2::getYaw(start.orientation);
  to[0] = end.position.x;
  to[1] = end.position.y;
  to[2] = tf2::getYaw(end.orientation);

  double d = state_space_->distance(from(), to());
  // If this path is too long compared to the original, then this is probably
  // a loop-de-loop, treat as invalid as to not deviate too far from the original path.
  // 2.0 selected from prinicipled choice of boundary test points
  // r, 2 * r, r * PI, and 2 * PI * r. If there is a loop, it will be
  // approximately 2 * PI * r, which is 2 * PI > r, PI > 2 * r, and 2 > r * PI.
  // For all but the last backup test point, a loop would be approximately
  // 2x greater than any of the selections.
  if (d > 2.0 * expansion.original_path_length) {
    return;
  }

  std::vector<double> reals;
  double theta(0.0), x(0.0), y(0.0);
  double x_m = start.position.x;
  double y_m = start.position.y;

  // Get intermediary poses
  for (double i = 0; i <= expansion.path_end_idx; i++) {
    state_space_->interpolate(from(), to(), i / expansion.path_end_idx, s());
    reals = s.reals();
    // Make sure in range [0, 2PI)
    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    x = reals[0];
    y = reals[1];

    // Check for collision
    unsigned int mx, my;
    costmap->worldToMap(x, y, mx, my);
    if (static_cast<float>(costmap->getCost(mx, my)) >= INSCRIBED) {
      expansion.in_collision = true;
    }

    // Integrate path length
    expansion.expansion_path_length += hypot(x - x_m, y - y_m);
    x_m = x;
    y_m = y;

    // Store point
    expansion.pts.emplace_back(x, y, theta);
  }
}

template<typename IteratorT>
BoundaryExpansions Smoother::generateBoundaryExpansionPoints(IteratorT start, IteratorT end)
{
  std::vector<double> distances = {
    min_turning_rad_,  // Radius
    2.0 * min_turning_rad_,  // Diameter
    M_PI * min_turning_rad_,  // 50% Circumference
    2.0 * M_PI * min_turning_rad_  // Circumference
  };

  BoundaryExpansions boundary_expansions;
  boundary_expansions.resize(distances.size());
  double curr_dist = 0.0;
  double x_last = start->pose.position.x;
  double y_last = start->pose.position.y;
  geometry_msgs::msg::Point pt;
  unsigned int curr_dist_idx = 0;

  for (IteratorT iter = start; iter != end; iter++) {
    pt = iter->pose.position;
    curr_dist += hypot(pt.x - x_last, pt.y - y_last);
    x_last = pt.x;
    y_last = pt.y;

    if (curr_dist >= distances[curr_dist_idx]) {
      boundary_expansions[curr_dist_idx].path_end_idx = iter - start;
      boundary_expansions[curr_dist_idx].original_path_length = curr_dist;
      curr_dist_idx++;
    }

    if (curr_dist_idx == boundary_expansions.size()) {
      break;
    }
  }

  return boundary_expansions;
}

void Smoother::enforceStartBoundaryConditions(
  const geometry_msgs::msg::Pose & start_pose,  // 起始姿态
  nav_msgs::msg::Path & path,  // 路径
  const nav2_costmap_2d::Costmap2D * costmap,  // 代价地图
  const bool & reversing_segment)  // 是否为反向段
{
  // 找到用于测试的点范围
  BoundaryExpansions boundary_expansions =
    generateBoundaryExpansionPoints<PathIterator>(path.poses.begin(), path.poses.end());

  // 从起始点到测试点生成运动模型和元数据
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }

    if (!reversing_segment) {
      findBoundaryExpansion(
        start_pose, path.poses[expansion.path_end_idx].pose, expansion,
        costmap);
    } else {
      findBoundaryExpansion(
        path.poses[expansion.path_end_idx].pose, start_pose, expansion,
        costmap);
    }
  }

  // 找到最短的运动学可行的边界扩展
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }

  // 覆盖值以匹配曲线
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path.poses[i].pose.position.x = best_expansion.pts[i].x;
    path.poses[i].pose.position.y = best_expansion.pts[i].y;
    path.poses[i].pose.orientation = orientationAroundZAxis(best_expansion.pts[i].theta);
  }
}

void Smoother::enforceEndBoundaryConditions(
  const geometry_msgs::msg::Pose & end_pose,  // 结束姿态
  nav_msgs::msg::Path & path,  // 路径
  const nav2_costmap_2d::Costmap2D * costmap,  // 代价地图
  const bool & reversing_segment)  // 是否为反向段
{
  // 找到用于测试的点范围
  BoundaryExpansions boundary_expansions =
    generateBoundaryExpansionPoints<ReversePathIterator>(path.poses.rbegin(), path.poses.rend());

  // 从起始点到测试点生成运动模型和元数据
  unsigned int expansion_starting_idx;
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }
    expansion_starting_idx = path.poses.size() - expansion.path_end_idx - 1;
    if (!reversing_segment) {
      findBoundaryExpansion(path.poses[expansion_starting_idx].pose, end_pose, expansion, costmap);
    } else {
      findBoundaryExpansion(end_pose, path.poses[expansion_starting_idx].pose, expansion, costmap);
    }
  }

  // 找到最短的运动学可行的边界扩展
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }

  // 覆盖值以匹配曲线
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  expansion_starting_idx = path.poses.size() - best_expansion.path_end_idx - 1;
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path.poses[expansion_starting_idx + i].pose.position.x = best_expansion.pts[i].x;
    path.poses[expansion_starting_idx + i].pose.position.y = best_expansion.pts[i].y;
    path.poses[expansion_starting_idx + i].pose.orientation = orientationAroundZAxis(
      best_expansion.pts[i].theta);
  }
}
}  // namespace nav2_smac_planner
