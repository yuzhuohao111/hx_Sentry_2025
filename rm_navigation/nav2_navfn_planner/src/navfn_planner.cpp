// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// #define BENCHMARK_TESTING

#include "nav2_navfn_planner/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;  // 使用标准库中的时间字面量命名空间
using namespace std::chrono;  // 使用标准库中的时间命名空间，// NOLINT 是一个注释，表示忽略某些静态分析工具的警告
using nav2_util::declare_parameter_if_not_declared;  // 使用nav2_util库中的declare_parameter_if_not_declared函数
using rcl_interfaces::msg::ParameterType;  // 使用rcl_interfaces库中的ParameterType消息类型
using std::placeholders::_1;  // 使用标准库中的占位符_1，用于绑定函数参数

namespace nav2_navfn_planner  // 定义一个名为nav2_navfn_planner的命名空间
{

NavfnPlanner::NavfnPlanner()
: tf_(nullptr), costmap_(nullptr)  // 构造函数，初始化tf_和costmap_为nullptr
{
}

NavfnPlanner::~NavfnPlanner()  // 析构函数
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type NavfnPlanner",  // 输出日志信息，表示正在销毁NavfnPlanner插件
    name_.c_str());
}

void
NavfnPlanner::configure(  // 配置函数，用于初始化插件的参数和资源
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,  // 父节点的弱指针
  std::string name,  // 插件的名称
  std::shared_ptr<tf2_ros::Buffer> tf,  // tf2_ros的缓冲区指针
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)  // 代价地图ROS对象的指针
{
  tf_ = tf;  // 将传入的tf2_ros缓冲区指针赋值给成员变量tf_
  name_ = name;  // 将传入的插件名称赋值给成员变量name_
  costmap_ = costmap_ros->getCostmap();  // 从代价地图ROS对象中获取代价地图，并赋值给成员变量costmap_
  global_frame_ = costmap_ros->getGlobalFrameID();  // 从代价地图ROS对象中获取全局坐标系ID，并赋值给成员变量global_frame_

  node_ = parent;  // 将传入的父节点弱指针赋值给成员变量node_
  auto node = parent.lock();  // 将父节点的弱指针转换为强指针
  clock_ = node->get_clock();  // 从节点中获取时钟，并赋值给成员变量clock_
  logger_ = node->get_logger();  // 从节点中获取日志记录器，并赋值给成员变量logger_

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type NavfnPlanner",  // 输出日志信息，表示正在配置NavfnPlanner插件
    name_.c_str());

  // 初始化参数
  // 声明并获取插件的参数
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));  // 声明并获取容差参数
  node->get_parameter(name + ".tolerance", tolerance_);  // 获取容差参数的值，并赋值给成员变量tolerance_
  declare_parameter_if_not_declared(node, name + ".use_astar", rclcpp::ParameterValue(true));  // 声明并获取是否使用A*算法的参数
  node->get_parameter(name + ".use_astar", use_astar_);  // 获取是否使用A*算法的参数值，并赋值给成员变量use_astar_
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));  // 声明并获取是否允许未知区域的参数
  node->get_parameter(name + ".allow_unknown", allow_unknown_);  // 获取是否允许未知区域的参数值，并赋值给成员变量allow_unknown_
  declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));  // 声明并获取是否使用最终接近方向的参数
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);  // 获取是否使用最终接近方向的参数值，并赋值给成员变量use_final_approach_orientation_

  // 根据新的代价地图大小创建规划器
  planner_ = std::make_unique<NavFn>(
    costmap_->getSizeInCellsX(),  // 获取代价地图的X方向的单元格数量
    costmap_->getSizeInCellsY());  // 获取代价地图的Y方向的单元格数量
}

void
NavfnPlanner::activate()  // 激活函数，用于激活插件
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type NavfnPlanner",  // 输出日志信息，表示正在激活NavfnPlanner插件
    name_.c_str());
  // 添加动态参数的回调函数
  auto node = node_.lock();  // 将父节点的弱指针转换为强指针
  dyn_params_handler_ = node->add_on_set_parameters_callback(  // 添加动态参数的回调函数
    std::bind(&NavfnPlanner::dynamicParametersCallback, this, _1));  // 绑定动态参数回调函数
}

void
NavfnPlanner::deactivate()  // 停用函数，用于停用插件
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type NavfnPlanner",  // 输出日志信息，表示正在停用NavfnPlanner插件
    name_.c_str());
  dyn_params_handler_.reset();  // 重置动态参数回调处理程序
}

void
NavfnPlanner::cleanup()  // 清理函数，用于清理插件资源
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type NavfnPlanner",  // 输出日志信息，表示正在清理NavfnPlanner插件
    name_.c_str());
  planner_.reset();  // 重置规划器
}

nav_msgs::msg::Path NavfnPlanner::createPlan(  // 创建路径的函数
  const geometry_msgs::msg::PoseStamped & start,  // 起始位置
  const geometry_msgs::msg::PoseStamped & goal)  // 目标位置
{
#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();  // 如果定义了BENCHMARK_TESTING，记录当前时间点
#endif

  // 根据新的代价地图大小更新规划器
  if (isPlannerOutOfDate()) {  // 如果规划器过期
    planner_->setNavArr(
      costmap_->getSizeInCellsX(),  // 获取代价地图的X方向的单元格数量
      costmap_->getSizeInCellsY());  // 获取代价地图的Y方向的单元格数量
  }

  nav_msgs::msg::Path path;  // 创建路径对象

  // 起始位置和目标位置的(x,y)坐标相同的情况
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    unsigned int mx, my;
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);  // 将世界坐标转换为地图坐标
    if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {  // 如果该位置是致命障碍物
      RCLCPP_WARN(logger_, "Failed to create a unique pose path because of obstacles");  // 输出警告信息，表示由于障碍物无法创建唯一位置路径
      return path;  // 返回空路径
    }
    path.header.stamp = clock_->now();  // 设置路径的时间戳
    path.header.frame_id = global_frame_;  // 设置路径的坐标系
    geometry_msgs::msg::PoseStamped pose;  // 创建位置姿态对象
    pose.header = path.header;  // 设置位置姿态的时间戳和坐标系
    pose.pose.position.z = 0.0;  // 设置位置姿态的z坐标为0

    pose.pose = start.pose;  // 设置位置姿态为起始位置姿态
    // 如果起始位置和目标位置的方向不同，并且不使用最终接近方向，则将路径姿态的方向设置为目标方向
    // 除非use_final_approach_orientation=true，此时需要将路径姿态的方向设置为起始方向，以避免本地规划器的移动
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);  // 将位置姿态添加到路径中
    return path;  // 返回路径
  }

  if (!makePlan(start.pose, goal.pose, tolerance_, path)) {  // 如果创建路径失败
    RCLCPP_WARN(
      logger_, "%s: failed to create plan with "
      "tolerance %.2f.", name_.c_str(), tolerance_);  // 输出警告信息，表示创建路径失败
  }


#ifdef BENCHMARK_TESTING
  steady_clock::time_point b = steady_clock::now();  // 如果定义了BENCHMARK_TESTING，记录当前时间点
  duration<double> time_span = duration_cast<duration<double>>(b - a);  // 计算时间差
  std::cout << "It took " << time_span.count() * 1000 << std::endl;  // 输出时间差
#endif

  return path;  // 返回路径
}


bool
NavfnPlanner::isPlannerOutOfDate()
{
  // 检查规划器是否为空，或者规划器的网格大小是否与代价地图的网格大小不匹配
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return true;  // 如果规划器为空或网格大小不匹配，返回true表示规划器已过期
  }
  return false;  // 否则返回false，表示规划器未过期
}

bool
NavfnPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,  // 起点位置
  const geometry_msgs::msg::Pose & goal,  // 目标位置
  double tolerance,  // 目标位置的容忍度
  nav_msgs::msg::Path & plan)  // 输出的路径规划结果
{
  // 清空路径规划结果，以防万一
  plan.poses.clear();

  // 设置路径规划结果的时间戳和参考坐标系
  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  // 获取起点和目标点的世界坐标
  double wx = start.position.x;
  double wy = start.position.y;

  // 输出调试信息，显示起点和目标点的坐标
  RCLCPP_DEBUG(
    logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  // 将世界坐标转换为地图坐标
  if (!worldToMap(wx, wy, mx, my)) {
    // 如果起点位置超出代价地图范围，输出警告信息并返回false
    RCLCPP_WARN(
      logger_,
      "Cannot create a plan: the robot's start position is off the global"
      " costmap. Planning will always fail, are you sure"
      " the robot has been properly localized?");
    return false;
  }

  // 清除起点位置的代价地图单元格，因为该位置不可能有障碍物
  clearRobotCell(mx, my);

  // 获取代价地图的互斥锁，确保线程安全
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // 确保Navfn使用的底层数组大小与代价地图一致
  planner_->setNavArr(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  // 设置规划器的代价地图，并指定是否允许未知区域
  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  // 释放互斥锁
  lock.unlock();

  // 将起点位置转换为地图坐标
  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  // 获取目标点的世界坐标
  wx = goal.position.x;
  wy = goal.position.y;

  // 将目标点的世界坐标转换为地图坐标
  if (!worldToMap(wx, wy, mx, my)) {
    // 如果目标位置超出代价地图范围，输出警告信息并返回false
    RCLCPP_WARN(
      logger_,
      "The goal sent to the planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  // 将目标位置转换为地图坐标
  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // 设置规划器的起点和目标点
  planner_->setStart(map_goal);
  planner_->setGoal(map_start);

  // 根据配置选择使用A*算法还是Dijkstra算法进行路径规划
  if (use_astar_) {
    planner_->calcNavFnAstar();
  }
  else
  {
    planner_->calcNavFnAstar();
  }

  // 获取代价地图的分辨率
  double resolution = costmap_->getResolution();
  geometry_msgs::msg::Pose p, best_pose;

  bool found_legal = false;

  // 初始化当前位置为目标位置
  p = goal;
  // 获取目标位置的潜在代价
  double potential = getPointPotential(p.position);
  if (potential < POT_HIGH) {
    // 如果目标位置的潜在代价小于阈值，表示目标位置可达
    best_pose = p;
    found_legal = true;
  } else {
    // 如果目标位置不可达，尝试在目标位置的容忍区域内找到最近的可达点
    double best_sdist = std::numeric_limits<double>::max();

    // 在目标位置的容忍区域内遍历
    p.position.y = goal.position.y - tolerance;
    while (p.position.y <= goal.position.y + tolerance) {
      p.position.x = goal.position.x - tolerance;
      while (p.position.x <= goal.position.x + tolerance) {
        // 获取当前位置的潜在代价
        potential = getPointPotential(p.position);
        // 计算当前位置与目标位置的平方距离
        double sdist = squared_distance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          // 如果当前位置可达且距离目标位置更近，更新最佳位置
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
  }

  if (found_legal) {
    // 如果找到了合法的路径，提取路径
    if (getPlanFromPotential(best_pose, plan)) {
      // 平滑路径，使其更接近目标位置
      smoothApproachToGoal(best_pose, plan);

      // 如果使用最终接近方向，调整路径最后一个点的方向
      if (use_final_approach_orientation_) {
        size_t plan_size = plan.poses.size();
        if (plan_size == 1) {
          // 如果路径只有一个点，直接使用起点的方向
          plan.poses.back().pose.orientation = start.orientation;
        } else if (plan_size > 1) {
          double dx, dy, theta;
          auto last_pose = plan.poses.back().pose.position;
          auto approach_pose = plan.poses[plan_size - 2].pose.position;
          // 处理NavFn生成路径时最后两个点相同的情况
          if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
            std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
          {
            approach_pose = plan.poses[plan_size - 3].pose.position;
          }
          dx = last_pose.x - approach_pose.x;
          dy = last_pose.y - approach_pose.y;
          theta = atan2(dy, dx);
          // 设置路径最后一个点的方向
          plan.poses.back().pose.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(theta);
        }
      }
    } else {
      // 如果从潜在代价中提取路径失败，输出错误信息
      RCLCPP_ERROR(
        logger_,
        "Failed to create a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }

  // 返回路径是否为空，如果不为空则表示路径规划成功
  return !plan.poses.empty();
}

void
NavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal,  // 目标位置
  nav_msgs::msg::Path & plan)  // 路径规划结果
{
  // 如果路径的最后一个点比倒数第二个点离目标位置更远，则替换为目标位置
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
      squared_distance(goal, second_to_last_pose.pose))
    {
      plan.poses.back().pose = goal;
      return;
    }
  }
  // 如果路径的最后一个点已经是最优的，直接将目标位置添加到路径末尾
  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  plan.poses.push_back(goal_copy);
}

bool
NavfnPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,  // 目标位置
  nav_msgs::msg::Path & plan)  // 路径规划结果
{
  // 清空路径规划结果，以防万一
  plan.poses.clear();

  // 获取目标点的世界坐标
  double wx = goal.position.x;
  double wy = goal.position.y;

  // 潜在代价已经计算完成，因此不会更新代价地图
  unsigned int mx, my;
  // 将世界坐标转换为地图坐标
  if (!worldToMap(wx, wy, mx, my)) {
    // 如果目标位置超出代价地图范围，输出警告信息并返回false
    RCLCPP_WARN(
      logger_,
      "The goal sent to the navfn planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  // 将目标位置转换为地图坐标
  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // 设置规划器的起点
  planner_->setStart(map_goal);

  // 设置最大循环次数，以防止无限循环
  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

  // 计算路径长度
  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    // 如果没有找到路径，返回false
    return false;
  }

  // 获取路径的代价
  auto cost = planner_->getLastPathCost();
  // 输出调试信息，显示路径长度和代价
  RCLCPP_DEBUG(
    logger_,
    "Path found, %d steps, %f cost\n", path_len, cost);

  // 提取路径的x和y坐标
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  // 将路径从地图坐标转换为世界坐标
  for (int i = len - 1; i >= 0; --i) {
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    // 创建路径中的每个点
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  // 返回路径是否为空，如果不为空则表示路径规划成功
  return !plan.poses.empty();
}
double
NavfnPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  // 定义地图坐标系中的x和y坐标
  unsigned int mx, my;

  // 将世界坐标系中的点转换为地图坐标系中的点，如果转换失败则返回最大值
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  // 计算地图坐标系中的点的索引
  unsigned int index = my * planner_->nx + mx;

  // 返回该点的势能值
  return planner_->potarr[index];
}

// bool
// NavfnPlanner::validPointPotential(const geometry_msgs::msg::Point & world_point)
// {
//   return validPointPotential(world_point, tolerance_);
// }

// bool
// NavfnPlanner::validPointPotential(
//   const geometry_msgs::msg::Point & world_point, double tolerance)
// {
//   const double resolution = costmap_->getResolution();

//   geometry_msgs::msg::Point p = world_point;
//   double potential = getPointPotential(p);
//   if (potential < POT_HIGH) {
//     // world_point 本身是可达的
//     return true;
//   } else {
//     // world_point 不可达，尝试在其容差区域内找到任何可达的点
//     p.y = world_point.y - tolerance;
//     while (p.y <= world_point.y + tolerance) {
//       p.x = world_point.x - tolerance;
//       while (p.x <= world_point.x + tolerance) {
//         potential = getPointPotential(p);
//         if (potential < POT_HIGH) {
//           return true;
//         }
//         p.x += resolution;
//       }
//       p.y += resolution;
//     }
//   }

//   return false;
// }

bool
NavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  // 如果世界坐标系中的点在地图范围之外，则返回false
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  // 将世界坐标系中的点转换为地图坐标系中的点
  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  // 如果转换后的点在地图范围内，则返回true
  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  // 如果转换后的点超出地图范围，则记录错误信息并返回false
  RCLCPP_ERROR(
    logger_,
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void
NavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  // 将地图坐标系中的点转换为世界坐标系中的点
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void
NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): 检查此函数的使用情况，可能需要请求 world_model / map server
  // 将地图中的某个单元格设置为自由空间
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

rcl_interfaces::msg::SetParametersResult
NavfnPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  // 定义参数设置结果
  rcl_interfaces::msg::SetParametersResult result;

  // 遍历所有参数
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // 根据参数类型和名称更新相应的变量
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".tolerance") {
        tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_astar") {
        use_astar_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        allow_unknown_ = parameter.as_bool();
      } else if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      }
    }
  }

  // 设置参数设置结果为成功
  result.successful = true;
  return result;
}

}  // namespace nav2_navfn_planner

#include "pluginlib/class_list_macros.hpp"
// 导出 NavfnPlanner 类作为插件
PLUGINLIB_EXPORT_CLASS(nav2_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)
