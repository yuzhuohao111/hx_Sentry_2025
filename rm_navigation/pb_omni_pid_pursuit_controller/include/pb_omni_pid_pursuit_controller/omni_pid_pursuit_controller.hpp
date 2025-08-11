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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_core/controller.hpp"
#include "pb_omni_pid_pursuit_controller/pid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <rm_decision_interfaces/msg/mode.hpp>
#include <rm_decision_interfaces/msg/nav_flag.hpp>
namespace pb_omni_pid_pursuit_controller
{

/**
 * @class pb_omni_pid_pursuit_controller::OmniPidPursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class OmniPidPursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for
   * pb_omni_pid_pursuit_controller::OmniPidPursuitController
   */
  OmniPidPursuitController() = default;

  /**
   * @brief Destrructor for
   * pb_omni_pid_pursuit_controller::OmniPidPursuitController
   */
  ~OmniPidPursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with
   * possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful
   * in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses
   * ineligible for lookaheadPoint Points ineligible to be selected as a
   * lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief Gets the maximum extent of the costmap
   * @return Maximum costmap extent in meters
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Creates a Carrot Point Marker message for visualization
   * @param carrot_pose Lookahead point pose
   * @return Unique pointer to the Carrot Point Marker message
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Gets the lookahead point on the transformed plan
   * @param lookahead_dist Lookahead distance
   * @param transformed_plan Transformed local plan
   * @return Lookahead point pose
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan);

  /**
   * @brief Calculates the intersection point of a circle and a line segment
   * @param p1 Start point of the line segment
   * @param p2 End point of the line segment
   * @param r Radius of the circle
   * @return Intersection point (geometry_msgs::msg::Point)
   */
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

  /**
   * @brief Callback function for dynamic parameter updates
   * @param parameters Vector of updated parameters
   * @return Result of parameter setting
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Calculates the lookahead distance based on current velocity
   * @param speed Current robot velocity
   * @return Lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

  /**
   * @brief Calculates the approach velocity scaling factor based on remaining path distance
   * @param path Transformed local path
   * @return Velocity scaling factor
   */
  double approachVelocityScalingFactor(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  // double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Applies velocity scaling based on approach distance to the goal
   * @param path Transformed local path
   * @param linear_vel Linear velocity command (in out)
   */
  // void applyApproachVelocityScaling(const nav_msgs::msg::Path & path, double & linear_vel) const;

  void applyApproachVelocityScaling(const geometry_msgs::msg::PoseStamped & robot_pose, double & linear_vel) const;
// 
  /**
   * @brief Checks if collision is detected along the given path
   * @param path Local path to check for collisions
   * @return True if collision detected, false otherwise
   */
  bool isCollisionDetected(const nav_msgs::msg::Path & path);

private:
  /**
   * @brief Applies curvature based speed limitation
   * @param path Transformed local path
   * @param lookahead_pose Lookahead point pose
   * @param linear_vel Linear velocity command (in out)
   */
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);
  
  void modeCallback(const rm_decision_interfaces::msg::Mode::SharedPtr msg);
  void mode1Callback(const rm_decision_interfaces::msg::Mode::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg); // 添加里程计回调声明
  /**
   * @brief Calculates curvature using three-point circle fitting
   * @param path Transformed local path
   * @param lookahead_pose Lookahead pose (current point)
   * @param forward_dist
   * @param backward_dist
   * @return Curvature value
   */
  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;

  /**
   * @brief Calculates the radius of curvature using three points
   * @param near_point Pose before the current point
   * @param current_point Current pose (lookahead pose)
   * @param far_point Pose after the current point
   * @return Radius of curvature
   */
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;

  /**
   * @brief Visualizes near and far points used for curvature calculation
   * @param backward_pose Near point pose
   * @param forward_pose Far point pose
   */
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;

  /**
   * @brief Calculates cumulative distances along the path
   * @param path The path to calculate distances for
   * @return Vector of cumulative distances
   */
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Finds a pose on the path at a given distance
   * @param path The path to search on
   * @param cumulative_distances Vector of cumulative distances along the path
   * @param target_distance The target distance to find the pose at
   * @return Pose at the target distance, or empty pose if not found
   */
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;
  double calculateDistanceToGoal(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  double calculatePathDistanceToGoal(
    const nav_msgs::msg::Path & transformed_plan) const;

  double calculatePathCurvature(
        const nav_msgs::msg::Path & path) const;
    
  double getPathSegmentCurvature(
        const geometry_msgs::msg::PoseStamped & p1,
        const geometry_msgs::msg::PoseStamped & p2,
        const geometry_msgs::msg::PoseStamped & p3) const;

    
  double calculateLocalCurvature(
    const nav_msgs::msg::Path & path, 
    const geometry_msgs::msg::PoseStamped & lookahead_pose) const ;
  double calculateThreePointCurvature(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    const geometry_msgs::msg::Point & p3) const;
  void applyAdaptiveSpeedScaling(
    double curvature, double & linear_vel);
private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  double last_velocity_scaling_factor_;
  rm_decision_interfaces::msg::Mode current_mode_;
  rm_decision_interfaces::msg::Mode current_mode1_;
  std::mutex mode_mutex_;//互斥锁
  std::mutex mode1_mutex_;//互斥锁

  std::shared_ptr<PID> move_pid_;
  std::shared_ptr<PID> bihuanv_pid_;
  std::shared_ptr<PID> bihuanw_pid_;
  std::shared_ptr<PID> heading_pid_;
  int escape_phase_; // 0 = 前进, 1 = 后退
  double escape_backward_start_time_; // 后退阶段开始时间
  int escape_count_ = 0;              // 脱困尝试次数
  bool in_escape_phase_ = false;       // 是否正在脱困
  rclcpp::Time escape_start_time_;     // 脱困开始时间
  rclcpp::Time first_escape_start_time_;     // 脱困开始时间
  double escape_duration_ = 0.5;       // 每次脱困持续时间
  double min_actual_speed_threshold_ = 0.05; // 实际速度阈值
  double max_command_speed_threshold_ = 0.1; // 命令速度阈值

  mutable std::mutex odom_mutex_;      // 保护里程计数据的互斥锁
  rclcpp::Time stuck_start_time_;      // 记录卡住开始时间
  bool is_stuck_ = false;              // 是否处于卡住状态
  double stuck_duration_threshold_ = 2.0; // 卡住持续时间阈值（秒）
  // Controller parameters
  double translation_kp_, translation_ki_, translation_kd_;
  double bihuanv_kp_, bihuanv_ki_, bihuanv_kd_;
  double bihuanw_kp_, bihuanw_ki_, bihuanw_kd_;
  bool enable_rotation_;
  double rotation_kp_, rotation_ki_, rotation_kd_;
  double min_max_sum_error_;
  double control_duration_;
  double max_robot_pose_search_dist_;
  bool use_interpolation_;
  double lookahead_dist_;
  bool use_velocity_scaled_lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_rotate_to_heading_;
  double use_rotate_to_heading_treshold_;
  double current_linear_speed_ = 0.0;
  double current_angular_speed_ = 0.0;
  double v_linear_min_;
  double v_linear_max_;
  double v_angular_min_;
  double v_angular_max_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;
  double curvature_min_;
  double curvature_max_;
  double reduction_ratio_at_high_curvature_;
  double curvature_forward_dist_;
  double curvature_backward_dist_;
  double max_velocity_scaling_factor_rate_;
  tf2::Duration transform_tolerance_;
  double limit_i_v;
  double limit_i_v1;
  double limit_i_w;
  double limit_i_w1;
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr curvature_points_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rm_decision_interfaces::msg::NavFlag>::SharedPtr flag_pub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::Mode>::SharedPtr mode1_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // 添加里程计订阅

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
