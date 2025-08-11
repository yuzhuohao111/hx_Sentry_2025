
#include "icp_registration/icp_registration.hpp"  // ICP配准算法的头文件
#include <Eigen/src/Geometry/Quaternion.h>  // Eigen库四元数相关头文件
#include <Eigen/src/Geometry/Transform.h>  // Eigen库变换矩阵头文件
#include <chrono>  // 时间相关头文件
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>  // ROS位姿消息头文件
#include <iostream>  // 标准输入输出头文件
#include <pcl/features/normal_3d.h>  // PCL点云法线估计头文件
#include <pcl/filters/passthrough.h>  // PCL直通滤波器头文件
#include <pcl/io/pcd_io.h>  // PCL点云文件IO头文件
#include <pcl_conversions/pcl_conversions.h>  // PCL与ROS消息转换头文件
#include <rclcpp/qos.hpp>  // ROS2 QoS策略头文件
#include <stdexcept>  // 异常处理头文件
#include <tf2/LinearMath/Quaternion.h>  // TF2四元数头文件
#include <tf2_ros/create_timer_ros.h>  // TF2定时器创建头文件

namespace icp {

// ICP节点构造函数
IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(10), refine_iter_(5),
      first_scan_(true) {
  is_ready_ = false;  // 初始化标志位为false
  // 创建输入点云指针
  cloud_in_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  
  // 声明并获取粗糙体素滤波的叶子大小参数
  double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
  // 声明并获取精细体素滤波的叶子大小参数
  double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
  // 设置粗糙体素滤波器的叶子大小
  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  // 设置精细体素滤波器的叶子大小
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  // 声明并获取PCD文件路径参数
  pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
  // 检查PCD文件路径是否存在
  if (!std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
    throw std::runtime_error("Invalid pcd path");
  }
  // 读取PCD文件
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  reader.read(pcd_path_, *cloud);
  RCLCPP_INFO(this->get_logger(), "Loaded PCD points: %ld", cloud->size());

  // 设置高度滤波器，去除地面和过高点
  pcl::PassThrough<pcl::PointXYZI> height_filter;
  height_filter.setInputCloud(cloud);
  height_filter.setFilterFieldName("z");  // 设置过滤字段为Z轴
  height_filter.setFilterLimits(-0.6, 10.0);  // 设置高度范围
  height_filter.setNegative(false);  // 不过滤范围内的点
  height_filter.filter(*cloud);  // 执行滤波

  // 对点云进行精细体素滤波
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);

  // 为点云添加法线信息
  refine_map_ = addNorm(cloud);
  // 创建粗糙点云指针
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  // 创建滤波后的粗糙点云指针
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  // 复制点云数据
  pcl::copyPointCloud(*refine_map_, *point_rough);
  // 对点云进行粗糙体素滤波
  voxel_rough_filter_.setInputCloud(point_rough);
  voxel_rough_filter_.filter(*filterd_point_rough);
  // 为粗糙点云添加法线信息
  rough_map_ = addNorm(filterd_point_rough);

  // 设置粗糙ICP的最大迭代次数
  icp_rough_.setMaximumIterations(rough_iter_);
  // 设置粗糙ICP的目标点云
  icp_rough_.setInputTarget(rough_map_);

  // 设置精细ICP的最大迭代次数
  icp_refine_.setMaximumIterations(refine_iter_);
  // 设置精细ICP的目标点云
  icp_refine_.setInputTarget(refine_map_);

  RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
              refine_map_->size(), rough_map_->size());

  // 声明并获取坐标系ID参数
  map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ =
      this->declare_parameter("odom_frame_id", std::string("odom"));
  laser_frame_id_ =
      this->declare_parameter("laser_frame_id", std::string("laser"));
  // 声明并获取ICP配准阈值参数
  thresh_ = this->declare_parameter("thresh", 0.15);
  // 声明并获取XY偏移量参数
  xy_offset_ = this->declare_parameter("xy_offset", 0.2);
  // 声明并获取Yaw偏移量参数(转换为弧度)
  yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
  // 声明并获取Yaw分辨率参数(转换为弧度)
  yaw_resolution_ =
      this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;
  // 声明并获取初始位姿参数
  std::vector<double> initial_pose_vec = this->declare_parameter(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  try {
    // 设置初始位姿的位置
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    // 设置初始位姿的旋转(欧拉角转四元数)
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
             initial_pose_vec.at(5));
  } catch (const std::out_of_range &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "initial_pose is not a vector with 6 elements, what():%s",
                 ex.what());
  }

  // 设置点云订阅器
  std::string pointcloud_topic = this->declare_parameter(
      "pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // 设置QoS策略
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));
  // 设置初始位姿订阅器
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
          });
  // 设置TF广播器
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  // 设置TF缓冲区
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  // 设置TF监听器
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建TF发布线程
  tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::Rate rate(100);  // 设置发布频率为100Hz
    while (rclcpp::ok()) {
      {
        std::lock_guard lock(mutex_);
        if (is_ready_) {
          // 更新TF时间戳和坐标系ID
          map_to_odom_.header.stamp = now();
          map_to_odom_.header.frame_id = map_frame_id_;
          map_to_odom_.child_frame_id = odom_frame_id_;
          // 发布TF变换
          tf_broadcaster_->sendTransform(map_to_odom_);
        }
      }
      rate.sleep();
    }
  });

  RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

// ICP节点析构函数
IcpNode::~IcpNode() {
  if (tf_publisher_thread_->joinable()) {
    tf_publisher_thread_->join();  // 等待TF发布线程结束
  }
}

// 点云回调函数
void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // 将ROS点云消息转换为PCL点云
  pcl::fromROSMsg(*msg, *cloud_in_);
  // 设置高度滤波器
  pcl::PassThrough<pcl::PointXYZI> height_filter;
  height_filter.setInputCloud(cloud_in_);
  height_filter.setFilterFieldName("z");  // 设置过滤字段为Z轴
  height_filter.setFilterLimits(0.01, 10.0);  // 设置高度范围
  height_filter.setNegative(false);  // 不过滤范围内的点
  height_filter.filter(*cloud_in_);  // 执行滤波
  // 如果是第一帧扫描，设置初始位姿
  if (first_scan_) {
    auto pose_msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_msg->header = msg->header;
    pose_msg->pose.pose = initial_pose_;
    RCLCPP_INFO(this->get_logger(), "??????????????????????");

    initialPoseCallback(pose_msg);
    first_scan_ = false;  // 标记已处理第一帧
  }
}

// 初始位姿回调函数
void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

  // 从消息中提取位置信息
  Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  // 从消息中提取四元数信息
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  // 构建初始猜测变换矩阵
  Eigen::Matrix4d initial_guess;
  initial_guess.block<3, 3>(0, 0) = q.toRotationMatrix();  // 旋转部分
  initial_guess.block<3, 1>(0, 3) = pos;  // 平移部分
  initial_guess(3, 3) = 1;  // 齐次坐标

  // 执行点云配准
  RCLCPP_INFO(this->get_logger(), "Aligning the pointcloud");
  Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, initial_guess);
  // 如果配准失败，使用初始猜测
  if (!success_) {
    map_to_laser = initial_guess;
    RCLCPP_ERROR(this->get_logger(), "ICP failed");
  }

  // 获取激光雷达到里程计的变换
  Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
  try {
    // 从TF树中查找变换
    auto transform =
        tf_buffer_->lookupTransform(laser_frame_id_, odom_frame_id_,rclcpp::Time(0));
    // 提取平移和旋转信息
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
    Eigen::Quaterniond q(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);
    // 构建变换矩阵
    laser_to_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
    laser_to_odom.block<3, 1>(0, 3) = t;
  } catch (tf2::TransformException &ex) {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    is_ready_ = false;
    return;
  }
  // 计算地图到里程计的最终变换
  Eigen::Matrix4d result = map_to_laser * laser_to_odom.matrix().cast<double>();
  {
    std::lock_guard lock(mutex_);
    // 设置平移部分
    map_to_odom_.transform.translation.x = result(0, 3);
    map_to_odom_.transform.translation.y = result(1, 3);
    map_to_odom_.transform.translation.z = 0.0;

    // 设置旋转部分
    Eigen::Matrix3d rotation = result.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rotation);

    map_to_odom_.transform.rotation.w = q.w();
    map_to_odom_.transform.rotation.x = q.x();
    map_to_odom_.transform.rotation.y = q.y();
    map_to_odom_.transform.rotation.z = q.z();
    is_ready_ = true;  // 标记可以发布TF
  }
}

// 多候选点云配准函数
// 输入参数：
//   source: 待配准的源点云
//   init_guess: 初始变换矩阵
// 返回值：最终配准后的变换矩阵
Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                        const Eigen::Matrix4d &init_guess) {
  // 局部函数：将旋转矩阵转换为欧拉角（roll, pitch, yaw）
  // 输入参数：3x3旋转矩阵
  // 返回值：包含三个欧拉角的向量
  static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
    double roll = std::atan2(rot(2, 1), rot(2, 2));  // 计算roll角（绕X轴旋转）
    double pitch = asin(-rot(2, 0));  // 计算pitch角（绕Y轴旋转）
    double yaw = std::atan2(rot(1, 0), rot(0, 0));  // 计算yaw角（绕Z轴旋转）
    return Eigen::Vector3d(roll, pitch, yaw);
  };

  success_ = false;  // 初始化配准成功标志为false

  // 从初始变换矩阵中提取位置和旋转部分
  Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);  // 提取平移向量
  Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);  // 提取旋转矩阵
  Eigen::Vector3d rpy = rotate2rpy(rotation);  // 将旋转矩阵转换为欧拉角

  // 创建roll和pitch角轴表示
  Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());  // X轴旋转
  Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());  // Y轴旋转

  std::vector<Eigen::Matrix4f> candidates;  // 存储候选变换矩阵的容器
  Eigen::Matrix4f temp_pose;  // 临时变换矩阵变量

  // 打印初始猜测的位置和欧拉角信息
  RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
              xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));

  // 生成候选变换矩阵
  // 在X和Y方向生成偏移候选（-1,0,1）
  // 在Yaw方向生成旋转候选（从-yaw_offset_到yaw_offset_）
  for (int i = -1; i <= 1; i++) {  // X方向偏移
    for (int j = -1; j <= 1; j++) {  // Y方向偏移
      for (int k = -yaw_offset_; k <= yaw_offset_; k++) {  // Yaw方向旋转
        // 计算候选位置（X,Y,Z）
        Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
                            xyz(2));
        // 计算候选Yaw角
        Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_,
                                   Eigen::Vector3f::UnitZ());
        // 构建候选变换矩阵
        temp_pose.setIdentity();  // 初始化为单位矩阵
        // 设置旋转部分：roll * pitch * yaw
        temp_pose.block<3, 3>(0, 0) =
            (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
        // 设置平移部分
        temp_pose.block<3, 1>(0, 3) = pos;
        // 添加到候选列表
        candidates.push_back(temp_pose);
      }
    }
  }

  // 创建两个不同分辨率的点云用于配准
  pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
      new pcl::PointCloud<pcl::PointXYZI>);  // 粗糙配准使用的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
      new pcl::PointCloud<pcl::PointXYZI>);  // 精细配准使用的点云

  // 对源点云进行下采样
  voxel_rough_filter_.setInputCloud(source);  // 设置粗糙下采样输入
  voxel_rough_filter_.filter(*rough_source);  // 执行下采样
  voxel_refine_filter_.setInputCloud(source);  // 设置精细下采样输入
  voxel_refine_filter_.filter(*refine_source);  // 执行下采样

  // 为下采样后的点云计算法线
  PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);  // 粗糙点云法线
  PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);  // 精细点云法线
  PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);  // 配准结果点云

  // 粗糙配准阶段变量
  Eigen::Matrix4f best_rough_transform;  // 最佳粗糙变换矩阵
  double best_rough_score = 10.0;  // 最佳粗糙配准分数（初始设为较大值）
  bool rough_converge = false;  // 粗糙配准是否收敛标志
  auto tic = std::chrono::system_clock::now();  // 记录开始时间

  // 遍历所有候选变换矩阵进行粗糙配准
  for (Eigen::Matrix4f &init_pose : candidates) {
    icp_rough_.setInputSource(rough_source_norm);  // 设置粗糙配准输入源
    icp_rough_.align(*align_point, init_pose);  // 执行粗糙配准
    if (!icp_rough_.hasConverged())  // 检查是否收敛
      continue;
    double rough_score = icp_rough_.getFitnessScore();  // 获取配准分数
    if (rough_score > 2 * thresh_)  // 分数超过阈值则跳过
      continue;
    if (rough_score < best_rough_score) {  // 更新最佳配准结果
      best_rough_score = rough_score;
      rough_converge = true;
      best_rough_transform = icp_rough_.getFinalTransformation();
    }
  }

  // 如果粗糙配准未收敛，返回零矩阵
  if (!rough_converge)
    return Eigen::Matrix4d::Zero();

  // 精细配准阶段
  icp_refine_.setInputSource(refine_source_norm);  // 设置精细配准输入源
  icp_refine_.align(*align_point, best_rough_transform);  // 执行精细配准
  score_ = icp_refine_.getFitnessScore();  // 获取最终配准分数

  // 检查精细配准是否收敛和分数是否合格
  if (!icp_refine_.hasConverged())
    return Eigen::Matrix4d::Zero();
  if (score_ > thresh_)
    return Eigen::Matrix4d::Zero();

  success_ = true;  // 标记配准成功
  auto toc = std::chrono::system_clock::now();  // 记录结束时间
  std::chrono::duration<double> duration = toc - tic;  // 计算耗时
  // 打印配准耗时和分数
  RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() * 1000);
  RCLCPP_INFO(this->get_logger(), "score: %f", score_);

  // 返回最终变换矩阵（转换为double类型）
  return icp_refine_.getFinalTransformation().cast<double>();
}

// 为点云添加法线信息的函数
// 输入参数：原始点云
// 返回值：包含法线信息的点云
PointCloudXYZIN::Ptr
IcpNode::addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);  // 法线点云
  pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZI>);  // KD树搜索对象
  searchTree->setInputCloud(cloud);  // 设置KD树输入点云

  // 法线估计器
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);  // 设置输入点云
  normalEstimator.setSearchMethod(searchTree);  // 设置搜索方法
  normalEstimator.setKSearch(15);  // 设置近邻点数
  normalEstimator.compute(*normals);  // 计算法线

  // 合并原始点云和法线信息
  PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
  pcl::concatenateFields(*cloud, *normals, *out);  // 拼接点云和法线
  return out;  // 返回带法线的点云
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
