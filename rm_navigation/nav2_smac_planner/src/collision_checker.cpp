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

#include "nav2_smac_planner/collision_checker.hpp"

namespace nav2_smac_planner
{

// 构造函数，初始化GridCollisionChecker对象
GridCollisionChecker::GridCollisionChecker(
  nav2_costmap_2d::Costmap2D * costmap,  // 指向Costmap2D对象的指针
  unsigned int num_quantizations,       // 量化角度的数量
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)  // 指向LifecycleNode的共享指针
: FootprintCollisionChecker(costmap)  // 调用基类构造函数
{
  if (node) {  // 如果node不为空
    clock_ = node->get_clock();  // 获取节点的时钟
    logger_ = node->get_logger();  // 获取节点的日志记录器
  }

  // 将常规的量化数量转换为角度
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);  // 计算每个量化角度的尺寸
  angles_.reserve(num_quantizations);  // 预留角度数组的空间
  for (unsigned int i = 0; i != num_quantizations; i++) {  // 遍历每个量化角度
    angles_.push_back(bin_size * i);  // 将角度添加到数组中
  }
}

// GridCollisionChecker::GridCollisionChecker(
//   nav2_costmap_2d::Costmap2D * costmap,
//   std::vector<float> & angles)
// : FootprintCollisionChecker(costmap),
//   angles_(angles)
// {
// }

// 设置机器人的足迹
void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,  // 机器人的足迹
  const bool & radius,  // 是否使用半径进行碰撞检测
  const double & possible_inscribed_cost)  // 可能的内切成本
{
  possible_inscribed_cost_ = possible_inscribed_cost;  // 设置可能的内切成本
  footprint_is_radius_ = radius;  // 设置是否使用半径进行碰撞检测

  // 如果使用半径，则不需要缓存
  if (radius) {
    return;
  }

  // 如果没有变化，则不需要更新
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.clear();  // 清空已有的定向足迹
  oriented_footprints_.reserve(angles_.size());  // 预留定向足迹的空间
  double sin_th, cos_th;  // 存储sin和cos值
  geometry_msgs::msg::Point new_pt;  // 存储新的点
  const unsigned int footprint_size = footprint.size();  // 获取足迹的大小

  // 预计算用于检查的定向足迹
  for (unsigned int i = 0; i != angles_.size(); i++) {  // 遍历每个角度
    sin_th = sin(angles_[i]);  // 计算sin值
    cos_th = cos(angles_[i]);  // 计算cos值
    nav2_costmap_2d::Footprint oriented_footprint;  // 创建新的定向足迹
    oriented_footprint.reserve(footprint_size);  // 预留空间

    for (unsigned int j = 0; j < footprint_size; j++) {  // 遍历足迹中的每个点
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;  // 计算新的x坐标
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;  // 计算新的y坐标
      oriented_footprint.push_back(new_pt);  // 将新的点添加到定向足迹中
    }

    oriented_footprints_.push_back(oriented_footprint);  // 将定向足迹添加到列表中
  }

  unoriented_footprint_ = footprint;  // 更新未定向的足迹
}

// 检查是否发生碰撞
bool GridCollisionChecker::inCollision(
  const float & x,  // x坐标
  const float & y,  // y坐标
  const float & angle_bin,  // 角度量化
  const bool & traverse_unknown)  // 是否穿越未知区域
{
  // 检查单元格是否在地图范围内
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;  // 如果超出范围，则认为发生碰撞
  }

  // 假设setFootprint已经设置
  double wx, wy;  // 世界坐标
  costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);  // 将地图坐标转换为世界坐标

  if (!footprint_is_radius_) {  // 如果不使用半径
    // 如果使用足迹，则检查足迹的点，但首先检查机器人是否可能在内切碰撞中
    footprint_cost_ = costmap_->getCost(
      static_cast<unsigned int>(x), static_cast<unsigned int>(y));  // 获取足迹的成本

    if (footprint_cost_ < possible_inscribed_cost_) {  // 如果成本小于可能的内切成本
      if (possible_inscribed_cost_ > 0) {
        return false;  // 如果可能的内切成本大于0，则认为没有碰撞
      } else {
        RCLCPP_ERROR_THROTTLE(
          logger_, *clock_, 1000,
          "Inflation layer either not found or inflation is not set sufficiently for "
          "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
          " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
          "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
          " for full instructions. This will substantially impact run-time performance.");  // 记录错误信息
      }
    }

    // 如果中间是内切、碰撞或未知，则不需要检查足迹，它是无效的
    if (footprint_cost_ == UNKNOWN && !traverse_unknown) {
      return true;  // 如果成本是未知且不穿越未知区域，则认为发生碰撞
    }

    if (footprint_cost_ == INSCRIBED || footprint_cost_ == OCCUPIED) {
      return true;  // 如果成本是内切或占用，则认为发生碰撞
    }

    // 如果可能内切，则需要检查实际的足迹姿态
    // 使用预计算的定向足迹，偏移平移值以进行碰撞检查
    geometry_msgs::msg::Point new_pt;  // 存储新的点
    const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];  // 获取定向足迹
    nav2_costmap_2d::Footprint current_footprint;  // 创建当前足迹
    current_footprint.reserve(oriented_footprint.size());  // 预留空间
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {  // 遍历定向足迹中的每个点
      new_pt.x = wx + oriented_footprint[i].x;  // 计算新的x坐标
      new_pt.y = wy + oriented_footprint[i].y;  // 计算新的y坐标
      current_footprint.push_back(new_pt);  // 将新的点添加到当前足迹中
    }

    footprint_cost_ = footprintCost(current_footprint);  // 获取当前足迹的成本

    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;  // 如果成本是未知且穿越未知区域，则认为没有碰撞
    }

    // 如果占用或未知且不穿越未知区域
    return footprint_cost_ >= OCCUPIED;  // 如果成本大于等于占用，则认为发生碰撞
  } else {  // 如果使用半径
    // 如果使用半径，则可以检查成本的中心，假设使用了膨胀
    footprint_cost_ = costmap_->getCost(
      static_cast<unsigned int>(x), static_cast<unsigned int>(y));  // 获取足迹的成本

    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;  // 如果成本是未知且穿越未知区域，则认为没有碰撞
    }

    // 如果占用或未知且不穿越未知区域
    return static_cast<double>(footprint_cost_) >= INSCRIBED;  // 如果成本大于等于内切，则认为发生碰撞
  }
}

// 检查是否发生碰撞
bool GridCollisionChecker::inCollision(
  const unsigned int & i,  // 单元格索引
  const bool & traverse_unknown)  // 是否穿越未知区域
{
  footprint_cost_ = costmap_->getCost(i);  // 获取足迹的成本
  if (footprint_cost_ == UNKNOWN && traverse_unknown) {
    return false;  // 如果成本是未知且穿越未知区域，则认为没有碰撞
  }

  // 如果占用或未知且不穿越未知区域
  return footprint_cost_ >= INSCRIBED;  // 如果成本大于等于内切，则认为发生碰撞
}

// 获取成本
float GridCollisionChecker::getCost()
{
  // 假设在调用之前已经调用了inCollision
  return static_cast<float>(footprint_cost_);  // 返回足迹的成本
}

// 检查值是否超出范围
bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;  // 如果值小于0或大于最大值，则认为超出范围
}

}  // namespace nav2_smac_planner