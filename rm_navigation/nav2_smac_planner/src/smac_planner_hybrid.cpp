// Copyright (c) 2020, Samsung Research America
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

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

// 定义SmacPlannerHybrid类
SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),  // 初始化A*算法指针为空
  _collision_checker(nullptr, 1, nullptr),  // 初始化碰撞检测器，参数为空指针、1和空指针
  _smoother(nullptr),  // 初始化路径平滑器指针为空
  _costmap(nullptr),  // 初始化代价地图指针为空
  _costmap_downsampler(nullptr)  // 初始化代价地图降采样器指针为空
{
}

// 定义SmacPlannerHybrid类的析构函数
SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerHybrid",  // 输出日志信息，表示正在销毁插件
    _name.c_str());
}

// 定义SmacPlannerHybrid类的配置函数
void SmacPlannerHybrid::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,  // 父节点的弱指针
  std::string name,  // 插件名称
  std::shared_ptr<tf2_ros::Buffer>/*tf*/,  // TF缓冲区（未使用）
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)  // 代价地图ROS对象
{
  _node = parent;  // 保存父节点的弱指针
  auto node = parent.lock();  // 获取父节点的强指针
  _logger = node->get_logger();  // 获取日志记录器
  _clock = node->get_clock();  // 获取时钟
  _costmap = costmap_ros->getCostmap();  // 获取代价地图
  _costmap_ros = costmap_ros;  // 保存代价地图ROS对象
  _name = name;  // 保存插件名称
  _global_frame = costmap_ros->getGlobalFrameID();  // 获取全局坐标系ID

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerHybrid", name.c_str());  // 输出日志信息，表示正在配置插件

  int angle_quantizations;  // 角度量化数
  double analytic_expansion_max_length_m;  // 解析扩展的最大长度（米）
  bool smooth_path;  // 是否平滑路径

  // 通用规划器参数
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));  // 声明参数，是否降采样代价地图
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));  // 声明参数，降采样因子
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);  // 获取参数值

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));  // 声明参数，角度量化箱数
  node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);  // 获取参数值
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;  // 计算角度量化箱的大小
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);  // 转换为无符号整数

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.25));  // 声明参数，容差
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());  // 获取参数值并转换为浮点数
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));  // 声明参数，是否允许未知区域
  node->get_parameter(name + ".allow_unknown", _allow_unknown);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));  // 声明参数，最大迭代次数
  node->get_parameter(name + ".max_iterations", _max_iterations);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));  // 声明参数，接近目标时的最大迭代次数
  node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(true));  // 声明参数，是否平滑路径
  node->get_parameter(name + ".smooth_path", smooth_path);  // 获取参数值

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));  // 声明参数，最小转弯半径
  node->get_parameter(name + ".minimum_turning_radius", _minimum_turning_radius_global_coords);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));  // 声明参数，是否缓存障碍物启发式
  node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));  // 声明参数，反向惩罚
  node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.0));  // 声明参数，改变方向惩罚
  node->get_parameter(name + ".change_penalty", _search_info.change_penalty);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));  // 声明参数，非直线惩罚
  node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));  // 声明参数，代价惩罚
  node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));  // 声明参数，回顾惩罚
  node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));  // 声明参数，解析扩展比率
  node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));  // 声明参数，解析扩展最大长度
  node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);  // 获取参数值
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();  // 计算解析扩展最大长度（网格坐标）

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));  // 声明参数，最大规划时间
  node->get_parameter(name + ".max_planning_time", _max_planning_time);  // 获取参数值
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));  // 声明参数，查找表大小
  node->get_parameter(name + ".lookup_table_size", _lookup_table_size);  // 获取参数值

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));  // 声明参数，搜索的运动模型
  node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);  // 获取参数值
  _motion_model = fromString(_motion_model_for_search);  // 从字符串转换为运动模型
  if (_motion_model == MotionModel::UNKNOWN) {  // 如果运动模型未知
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());  // 输出警告信息
  }

  if (_max_on_approach_iterations <= 0) {  // 如果接近目标时的最大迭代次数小于等于0
    RCLCPP_INFO(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");  // 输出日志信息，表示禁用容差和接近目标时的迭代次数
    _max_on_approach_iterations = std::numeric_limits<int>::max();  // 设置为最大值
  }

  if (_max_iterations <= 0) {  // 如果最大迭代次数小于等于0
    RCLCPP_INFO(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");  // 输出日志信息，表示禁用最大迭代次数
    _max_iterations = std::numeric_limits<int>::max();  // 设置为最大值
  }

  // 转换为网格坐标
  if (!_downsample_costmap) {  // 如果不降采样代价地图
    _downsampling_factor = 1;  // 设置降采样因子为1
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);  // 计算最小转弯半径（网格坐标）
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);  // 计算查找表维度

  // 确保是整数
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // 确保是奇数
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {  // 如果是偶数
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);  // 输出日志信息，表示增加1以使其为奇数
    _lookup_table_dim += 1.0;
  }

  // 初始化碰撞检测器
  _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),  // 设置机器人足迹
    _costmap_ros->getUseRadius(),  // 设置使用半径
    findCircumscribedCost(_costmap_ros));  // 查找外接代价

  // 初始化A*模板
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,  // 是否允许未知区域
    _max_iterations,  // 最大迭代次数
    _max_on_approach_iterations,  // 接近目标时的最大迭代次数
    _max_planning_time,  // 最大规划时间
    _lookup_table_dim,  // 查找表维度
    _angle_quantizations);  // 角度量化数

  // 初始化路径平滑器
  if (smooth_path) {  // 如果需要平滑路径
    SmootherParams params;
    params.get(node, name);
    params.holonomic_ = false;    //添加这一行 适用全向小车在行走时旋转使得正方向是指定方向  去掉后是汽车模型会路径是带转弯角度的
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_minimum_turning_radius_global_coords);  // 初始化路径平滑器
  }

  // 初始化代价地图降采样器
  if (_downsample_costmap && _downsampling_factor > 1) {  // 如果需要降采样代价地图且降采样因子大于1
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);  // 配置代价地图降采样器
  }

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);  // 创建未平滑路径的发布者
  path_vehicles_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("pathVehicle", 1);  

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());  // 输出日志信息，表示插件配置完成
}

// 激活插件的函数
void SmacPlannerHybrid::activate()
{
  // 使用RCLCPP_INFO记录日志，显示正在激活的插件名称和类型
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  // 激活原始路径发布器
  _raw_plan_publisher->on_activate();
  path_vehicles_pub_->on_activate();
  // 如果存在代价地图降采样器，则激活它
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  // 获取节点指针
  auto node = _node.lock();
  // 添加动态参数的回调函数
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerHybrid::dynamicParametersCallback, this, _1));
}

// 停用插件的函数
void SmacPlannerHybrid::deactivate()
{
  // 使用RCLCPP_INFO记录日志，显示正在停用的插件名称和类型
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  // 停用原始路径发布器
  _raw_plan_publisher->on_deactivate();
  path_vehicles_pub_->on_deactivate();
  // 如果存在代价地图降采样器，则停用它
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  // 重置动态参数处理程序
  _dyn_params_handler.reset();
}

// 清理插件的函数
void SmacPlannerHybrid::cleanup()
{
  // 使用RCLCPP_INFO记录日志，显示正在清理的插件名称和类型
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  // 重置A*算法和路径平滑器
  _a_star.reset();
  _smoother.reset();
  // 如果存在代价地图降采样器，则清理并重置它
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
  // 重置原始路径发布器
  _raw_plan_publisher.reset();
  path_vehicles_pub_.reset();
}

// 创建路径的函数，输入起点和终点，输出路径
nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start, // 起点
  const geometry_msgs::msg::PoseStamped & goal) // 终点
{
  // 清除之前的可视化信息
  clearPathNodes();

  // 使用互斥锁保护重新初始化操作
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  // 记录开始时间
  steady_clock::time_point a = steady_clock::now();

  // 获取代价地图的互斥锁
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // 如果需要，对代价地图进行降采样
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // 设置碰撞检查器和代价地图信息
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(), // 获取机器人的足迹
    _costmap_ros->getUseRadius(), // 获取使用半径
    findCircumscribedCost(_costmap_ros)); // 查找外接圆的代价
  _a_star->setCollisionChecker(&_collision_checker); // 设置碰撞检查器

  // 设置起点，转换为A*算法使用的坐标
  unsigned int mx, my;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
    throw std::runtime_error("Start pose is out of costmap!"); // 如果起点超出代价地图范围，抛出异常
  }
  double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size; // 计算方向角
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations); // 处理负角度
  }
  // 处理精度问题
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin)); // 转换为整数
  _a_star->setStart(mx, my, orientation_bin_id); // 设置起点

  // 设置终点，转换为A*算法使用的坐标
  if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    throw std::runtime_error("Goal pose is out of costmap!"); // 如果终点超出代价地图范围，抛出异常
  }
  orientation_bin = tf2::getYaw(goal.pose.orientation) / _angle_bin_size; // 计算方向角
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations); // 处理负角度
  }
  // 处理精度问题
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin)); // 转换为整数
  _a_star->setGoal(mx, my, orientation_bin_id); // 设置终点

  // 初始化路径消息
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now(); // 设置时间戳
  plan.header.frame_id = _global_frame; // 设置全局坐标系
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0; // 设置高度为0
  pose.pose.orientation.x = 0.0; // 设置方向为默认值
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // 计算路径
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(
        path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution()))) // 计算路径
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found"); // 如果没有找到有效路径
      } else {
        error = std::string("exceeded maximum iterations"); // 如果超过最大迭代次数
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what(); // 捕获并记录异常信息
  }

  // 如果路径计算失败，记录警告并返回空路径
  if (!error.empty()) {
    RCLCPP_WARN(
      _logger,
      "%s: failed to create plan, %s.",
      _name.c_str(), error.c_str()); // 记录警告信息
    return plan; // 返回空路径
  }

  // 将路径转换为世界坐标
  plan.poses.reserve(path.size());
  std::vector<geometry_msgs::msg::PoseStamped> path_poses;
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap); // 转换为世界坐标
    pose.pose.orientation = getWorldOrientation(path[i].theta); // 设置方向
    plan.poses.push_back(pose); // 添加到路径中
    path_poses.push_back(pose); // 添加到可视化路径中
  }
  // 发布路径节点用于可视化
  publishPathNodes(path_poses);

  // 如果存在订阅者，发布原始路径用于调试
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan); // 发布原始路径
  }

  // 计算剩余时间用于路径平滑
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count()); // 计算剩余时间

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl; // 输出计算时间
#endif

  // 如果路径平滑器存在且迭代次数大于1，则进行路径平滑
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining); // 进行路径平滑
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl; // 输出平滑时间
#endif

  // 返回最终的路径
  return plan;
}

// 定义一个名为 `dynamicParametersCallback` 的函数，该函数接收一个 `std::vector<rclcpp::Parameter>` 类型的参数 `parameters`，并返回一个 `rcl_interfaces::msg::SetParametersResult` 类型的结果。
rcl_interfaces::msg::SetParametersResult
SmacPlannerHybrid::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  // 创建一个 `rcl_interfaces::msg::SetParametersResult` 类型的对象 `result`，用于存储参数设置的结果。
  rcl_interfaces::msg::SetParametersResult result;
  
  // 使用 `std::lock_guard` 锁定互斥锁 `_mutex`，以确保在修改共享资源时不会发生竞态条件。
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  // 初始化一些布尔变量，用于标记是否需要重新初始化某些组件。
  bool reinit_collision_checker = false;
  bool reinit_a_star = false;
  bool reinit_downsampler = false;
  bool reinit_smoother = false;

  // 遍历传入的参数列表 `parameters`。
  for (auto parameter : parameters) {
    // 获取当前参数的类型和名称。
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // 如果参数类型是 `PARAMETER_DOUBLE`（双精度浮点数）。
    if (type == ParameterType::PARAMETER_DOUBLE) {
      // 根据参数名称判断需要更新的变量，并设置相应的重新初始化标志。
      if (name == _name + ".max_planning_time") {
        reinit_a_star = true;
        _max_planning_time = parameter.as_double();
      } else if (name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".lookup_table_size") {
        reinit_a_star = true;
        _lookup_table_size = parameter.as_double();
      } else if (name == _name + ".minimum_turning_radius") {
        reinit_a_star = true;
        if (_smoother) {
          reinit_smoother = true;
        }
        _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".reverse_penalty") {
        reinit_a_star = true;
        _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".change_penalty") {
        reinit_a_star = true;
        _search_info.change_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".non_straight_penalty") {
        reinit_a_star = true;
        _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".cost_penalty") {
        reinit_a_star = true;
        _search_info.cost_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_ratio") {
        reinit_a_star = true;
        _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_max_length") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_length =
          static_cast<float>(parameter.as_double()) / _costmap->getResolution();
      }
    } 
    // 如果参数类型是 `PARAMETER_BOOL`（布尔值）。
    else if (type == ParameterType::PARAMETER_BOOL) {
      // 根据参数名称判断需要更新的变量，并设置相应的重新初始化标志。
      if (name == _name + ".downsample_costmap") {
        reinit_downsampler = true;
        _downsample_costmap = parameter.as_bool();
      } else if (name == _name + ".allow_unknown") {
        reinit_a_star = true;
        _allow_unknown = parameter.as_bool();
      } else if (name == _name + ".cache_obstacle_heuristic") {
        reinit_a_star = true;
        _search_info.cache_obstacle_heuristic = parameter.as_bool();
      } else if (name == _name + ".smooth_path") {
        if (parameter.as_bool()) {
          reinit_smoother = true;
        } else {
          _smoother.reset();
        }
      }
    } 
    // 如果参数类型是 `PARAMETER_INTEGER`（整数）。
    else if (type == ParameterType::PARAMETER_INTEGER) {
      // 根据参数名称判断需要更新的变量，并设置相应的重新初始化标志。
      if (name == _name + ".downsampling_factor") {
        reinit_a_star = true;
        reinit_downsampler = true;
        _downsampling_factor = parameter.as_int();
      } else if (name == _name + ".max_iterations") {
        reinit_a_star = true;
        _max_iterations = parameter.as_int();
        if (_max_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "maximum iteration selected as <= 0, "
            "disabling maximum iterations.");
          _max_iterations = std::numeric_limits<int>::max();
        }
      } else if (name == _name + ".max_on_approach_iterations") {
        reinit_a_star = true;
        _max_on_approach_iterations = parameter.as_int();
        if (_max_on_approach_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "On approach iteration selected as <= 0, "
            "disabling tolerance and on approach iterations.");
          _max_on_approach_iterations = std::numeric_limits<int>::max();
        }
      } else if (name == _name + ".angle_quantization_bins") {
        reinit_collision_checker = true;
        reinit_a_star = true;
        int angle_quantizations = parameter.as_int();
        _angle_bin_size = 2.0 * M_PI / angle_quantizations;
        _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
      }
    } 
    // 如果参数类型是 `PARAMETER_STRING`（字符串）。
    else if (type == ParameterType::PARAMETER_STRING) {
      // 根据参数名称判断需要更新的变量，并设置相应的重新初始化标志。
      if (name == _name + ".motion_model_for_search") {
        reinit_a_star = true;
        _motion_model = fromString(parameter.as_string());
        if (_motion_model == MotionModel::UNKNOWN) {
          RCLCPP_WARN(
            _logger,
            "Unable to get MotionModel search type. Given '%s', "
            "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
            _motion_model_for_search.c_str());
        }
      }
    }
  }

  // 如果需要重新初始化 A* 算法、降采样器、碰撞检测器或平滑器，则执行以下操作。
  if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
    // 如果不需要降采样，则将降采样因子设置为1。
    if (!_downsample_costmap) {
      _downsampling_factor = 1;
    }
    // 计算最小转弯半径和查找表尺寸，并确保它们是整数和奇数。
    _search_info.minimum_turning_radius =
      _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
    _lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution() * _downsampling_factor);

    // 确保查找表尺寸是整数。
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // 确保查找表尺寸是奇数。
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        _lookup_table_dim);
      _lookup_table_dim += 1.0;
    }

    // 获取当前节点。
    auto node = _node.lock();

    // 如果需要重新初始化 A* 算法，则重新初始化 A* 算法。
    if (reinit_a_star) {
      _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
      _a_star->initialize(
        _allow_unknown,
        _max_iterations,
        _max_on_approach_iterations,
        _max_planning_time,
        _lookup_table_dim,
        _angle_quantizations);
    }

    // 如果需要重新初始化降采样器，则重新初始化降采样器。
    if (reinit_downsampler) {
      if (_downsample_costmap && _downsampling_factor > 1) {
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler = std::make_unique<CostmapDownsampler>();
        _costmap_downsampler->on_configure(
          node, _global_frame, topic_name, _costmap, _downsampling_factor);
      }
    }

    // 如果需要重新初始化碰撞检测器，则重新初始化碰撞检测器。
    if (reinit_collision_checker) {
      _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
      _collision_checker.setFootprint(
        _costmap_ros->getRobotFootprint(),
        _costmap_ros->getUseRadius(),
        findCircumscribedCost(_costmap_ros));
    }

    // 如果需要重新初始化平滑器，则重新初始化平滑器。
    if (reinit_smoother) {
      SmootherParams params;
      params.get(node, _name);
      _smoother = std::make_unique<Smoother>(params);
      _smoother->initialize(_minimum_turning_radius_global_coords);
    }
  }
  
  // 设置参数设置结果为成功。
  result.successful = true;
  
  // 返回参数设置结果。
  return result;
}

// 将路径节点发布到RVIZ
void SmacPlannerHybrid::publishPathNodes(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
  visualization_msgs::msg::Marker pathVehicle;  // 创建一个用于可视化的标记消息
  int nodeSize = path.size();  // 获取路径的大小
  pathVehicle.header.stamp = _clock->now();  // 设置标记消息的时间戳
  pathVehicle.color.r = 52.f / 255.f;  // 设置标记的颜色（红色）
  pathVehicle.color.g = 250.f / 255.f;  // 设置标记的颜色（绿色）
  pathVehicle.color.b = 52.f / 255.f;  // 设置标记的颜色（蓝色）
  pathVehicle.type = visualization_msgs::msg::Marker::SPHERE;  // 设置标记的类型为箭头
  pathVehicle.header.frame_id = _global_frame;  // 设置标记的帧ID
  pathVehicle.scale.x = 0.5;  // 设置标记的缩放比例（x轴）
  pathVehicle.scale.y = 0.4;  // 设置标记的缩放比例（y轴）
  pathVehicle.scale.z = 0.12;  // 设置标记的缩放比例（z轴）
  pathVehicle.color.a = 0.1;  // 设置标记的透明度
  // 转换路径中的节点，并添加时间戳等信息
  for(int i = 0; i < nodeSize; i++) {
    pathVehicle.header.stamp = _clock->now();  // 设置标记的时间戳
    pathVehicle.pose = path[i].pose;  // 设置标记的位姿
    pathVehicle.id = i;  // 设置标记的ID
    pathNodes.markers.push_back(pathVehicle);  // 将标记添加到标记数组中
  }
  // 发布这些车辆位置标记点
  path_vehicles_pub_->publish(pathNodes);
}

// 清理可视化信息的标记点
void SmacPlannerHybrid::clearPathNodes() {
  // 初始化并配置节点为全清空模式
  visualization_msgs::msg::Marker node;
  pathNodes.markers.clear();  // 清空标记数组
  node.action = visualization_msgs::msg::Marker::DELETEALL;  // 设置标记的动作为删除所有
  node.header.frame_id = _global_frame;  // 设置标记的帧ID
  node.header.stamp = _clock->now();  // 设置标记的时间戳
  node.id = 0;  // 设置标记的ID
  node.action = 3;  // 设置标记的动作为删除
  pathNodes.markers.push_back(node);  // 将标记添加到标记数组中
  path_vehicles_pub_->publish(pathNodes);  // 发布清空标记的消息
  // RCLCPP_INFO(logger_, "Clean the path nodes");
}


}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)
