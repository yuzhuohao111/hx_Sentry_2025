#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
class MergeCloudNode : public rclcpp::Node
{
public:
  MergeCloudNode() : Node("merge_cloud_node")
  {
    // 订阅两个Livox点云话题（请修改为实际话题名）
    cloud1_sub_.subscribe(this, "/livox/lidar_192_168_1_125");
    cloud2_sub_.subscribe(this, "/livox/lidar_192_168_1_142");

    // 使用ApproximateTime同步器
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), cloud1_sub_, cloud2_sub_);
    sync_->registerCallback(
      std::bind(&MergeCloudNode::syncCallback, this, 
      std::placeholders::_1, std::placeholders::_2));

    // 初始化发布器
    merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_cloud", 10);
    merged_custom_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("merged_custom", 10);

    // 声明时间差阈值参数（单位：毫秒）
    this->declare_parameter<int64_t>("max_time_diff_ms", 1);
    
    // 初始化变换参数（根据实际标定结果修改）
    initTransformParams();

    RCLCPP_INFO(this->get_logger(), "MergeCloudNode initialized");
  }

private:
  void initTransformParams()
  {
    // 雷达1的变换参数（示例值，需要根据实际标定修改）
    roll1_ = 0.0f;    // X轴旋转弧度
    pitch1_ = 0.0f;   // Y轴旋转弧度
    yaw1_ = 0.0f;     // Z轴旋转弧度
    tx1_ = 0.0f;      // X轴平移（米）
    ty1_ = 0.0f;      // Y轴平移（米）
    tz1_ = -0.0f;      // Z轴平移（米）

    // 雷达2的变换参数（示例值）
    roll2_ = 0.0f;
    pitch2_ = 0.0f;
    yaw2_ = 0.0f;
    tx2_ = 0.0f;
    ty2_ = 0.0f;
    tz2_ = 0.0f;

    // 构建变换矩阵
    transform1_ = Eigen::Matrix4f::Identity();
    transform2_ = Eigen::Matrix4f::Identity();
    setTransformMatrix(transform1_, roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_);
    setTransformMatrix(transform2_, roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_);
  }

  void syncCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& cloud1_msg,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& cloud2_msg)
  {
    // 获取时间差阈值（转换为纳秒）
    const int64_t MAX_TIME_DIFF_NS = this->get_parameter("max_time_diff_ms").as_int() * 1000000LL;

    // 提取时间戳信息
    const auto& t1 = cloud1_msg->header.stamp;
    const auto& t2 = cloud2_msg->header.stamp;
    const int64_t t1_ns = t1.sec * 1000000000LL + t1.nanosec;
    const int64_t t2_ns = t2.sec * 1000000000LL + t2.nanosec;
    const int64_t diff_ns = std::abs(t1_ns - t2_ns);

    // 打印详细时间戳信息
    RCLCPP_INFO(this->get_logger(), "\nCloud1 timestamp: %d.%09d\nCloud2 timestamp: %d.%09d\nTime difference: %+.3fms", 
               t1.sec, t1.nanosec,
               t2.sec, t2.nanosec,
               diff_ns / 1e6);

    // 时间差超过阈值时丢弃
    if (diff_ns > MAX_TIME_DIFF_NS) {
      RCLCPP_WARN(this->get_logger(), 
        "Dropping frames with large time difference: %lld ns (t1: %ld.%09ld, t2: %ld.%09ld)", 
        diff_ns, t1.sec, t1.nanosec, t2.sec, t2.nanosec);
      return;
    }

    RCLCPP_INFO(this->get_logger(), 
      "Merging frames with time difference: %lld ns", diff_ns);

    // 选择较早的时间戳作为合并后的时间戳
    auto merged_stamp = 
      ((t1.sec < t2.sec) || (t1.sec == t2.sec && t1.nanosec <= t2.nanosec)) ? t1 : t2;

    // auto merged_stamp = cloud1_msg->header.stamp;
    // 转换第一个点云
    auto cloud1_transformed = processCloud(cloud1_msg, transform1_);
    // 转换第二个点云
    auto cloud2_transformed = processCloud(cloud2_msg, transform2_);

    // 合并点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *merged_cloud = *cloud1_transformed + *cloud2_transformed;

    // 发布PointCloud2格式
    publishPointCloud2(merged_cloud, merged_stamp);
    
    // 发布CustomMsg格式
    publishCustomMsg(cloud1_msg, cloud2_msg, cloud1_transformed, cloud2_transformed, merged_stamp);

    // livox_ros_driver2::msg::CustomMsg pub_msg;

    // pub_msg.header = cloud1_msg->header;
    // pub_msg.timebase = cloud1_msg->timebase;
    // pub_msg.point_num = cloud1_msg->point_num + cloud2_msg->point_num;
    // pub_msg.lidar_id = cloud1_msg->lidar_id;
    // pub_msg.rsvd = cloud1_msg->rsvd;
    // pub_msg.points.resize(pub_msg.point_num);
    
    // for (int i = 0; i < cloud1_msg->point_num; ++i) {
    //     pub_msg.points[i] = cloud1_msg->points[i];
    // }
    // // 复制第二个点云
    // for (int i = 0; i < cloud2_msg->point_num; ++i) {
    //     pub_msg.points[cloud1_msg->point_num + i] = cloud2_msg->points[i];
    // }

    // merged_custom_pub_->publish(pub_msg);

  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr processCloud(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg,
    const Eigen::Matrix4f& transform)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->reserve(msg->points.size());

    // 转换到PCL格式并处理强度值
    for (const auto& point : msg->points) {
      pcl::PointXYZI pcl_point;
      pcl_point.x = point.x;
      pcl_point.y = point.y;
      pcl_point.z = point.z;
      // 强度值四舍五入处理（与原始转换逻辑一致）
      pcl_point.intensity = static_cast<uint8_t>(point.reflectivity + 0.5f);
      cloud->push_back(pcl_point);
    }

    // 应用坐标变换
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    return transformed_cloud;
  }

  void publishPointCloud2(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const builtin_interfaces::msg::Time& stamp)
  {
      // ======================== 新增 CropBox 滤波 ========================
      // 创建临时点云存储滤波结果（避免修改原始数据）
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      
      // 初始化 CropBox 滤波器
      pcl::CropBox<pcl::PointXYZI> crop_filter;
      
      // 设置输入点云（使用原始输入）
      crop_filter.setInputCloud(cloud); 

      /* 包围盒参数设置（需根据机体实际尺寸调整）
        --------------------------------------------
        | 参数格式 | Eigen::Vector4f(x, y, z, 1.0) |
        --------------------------------------------
        - X轴范围：机体的左右方向（正右/负左）
          - 示例：±0.29 表示保留左右29厘米范围外的点
          - 测量方法：量取机体边缘到传感器的水平距离
          
        - Y轴范围：机体的前后方向（正前/负后）
          - 示例：±0.30 表示保留前后30厘米范围外的点
          - 测量方法：量取机头/机尾到传感器的纵向距离
          
        - Z轴范围：机体的高度方向（正上/负下）
          - 示例：-0.30~0.53 表示保留地面下方30cm到上方53cm外的点
          - 测量方法：量取机体底部到地面的距离（下界）和顶部高度（上界）*/
      
      // 最小顶点坐标（左下后）
      Eigen::Vector4f min_pt(-0.4, -0.40, -0.3, 1.0); 
      
      // 最大顶点坐标（右上前）
      Eigen::Vector4f max_pt(0.4, 0.40, 0.30, 1.0);  
      
      crop_filter.setMin(min_pt);    // 设置包围盒下界
      crop_filter.setMax(max_pt);    // 设置包围盒上界
      crop_filter.setNegative(true); // true=删除包围盒内的点（保留外部点）

      // 执行滤波（结果存储到 filtered_cloud）
      crop_filter.filter(*filtered_cloud); 
      // RCLCPP_INFO(get_logger(), "立方体滤波后点数: %zu", filtered_cloud->size());

      // 第二阶段：障碍物高度过滤（排除barrier高度区间）
      pcl::PassThrough<pcl::PointXYZI> height_filter;
      height_filter.setInputCloud(filtered_cloud);
      height_filter.setFilterFieldName("z");
      height_filter.setFilterLimits(2.0, 4.0);
      height_filter.setNegative(true);     // 排除该Z区间
      height_filter.filter(*filtered_cloud);
      // RCLCPP_INFO(get_logger(), "高度滤波后点数: %zu", filtered_cloud->size());
      
      // pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
      // ror.setInputCloud(filtered_cloud);
      // ror.setRadiusSearch(0.5);       // 搜索半径0.5米
      // ror.setMinNeighborsInRadius(8);  // 半径内最少需要5个邻居
      // ror.filter(*filtered_cloud);

      // pcl::VoxelGrid<pcl::PointXYZI> voxel;
      // voxel.setInputCloud(filtered_cloud);
      // voxel.setLeafSize(0.05f, 0.05f, 0.05f);
      // voxel.setMinimumPointsNumberPerVoxel(2); // 避免孤立点
      // voxel.filter(*filtered_cloud);
      // RCLCPP_INFO(get_logger(), "体素滤波后点数: %zu", filtered_cloud->size());

      // 转换为ROS消息（使用滤波后的点云）
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*filtered_cloud, msg);  // 关键修改：使用 filtered_cloud
      
      // 设置消息头
      msg.header.stamp = stamp;
      msg.header.frame_id = "livox_frame";  // 确保与TF树中的坐标系一致
      
      // 发布点云
      merged_cloud_pub_->publish(msg);
  }

  void publishCustomMsg(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg1,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg2,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2,
    const builtin_interfaces::msg::Time& stamp)
  {
    auto merged_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  
    // 设置消息头
    merged_msg->header.stamp = stamp;
    merged_msg->header.frame_id = "livox_frame";
    merged_msg->timebase = stamp.sec * 1e9 + stamp.nanosec; // 计算纳秒级时间基准
    merged_msg->point_num = cloud1->size() + cloud2->size();
    merged_msg->lidar_id = 88; // 合并后的虚拟雷达ID
    merged_msg->rsvd = std::max(msg1->rsvd, msg2->rsvd);
    merged_msg->points.reserve(merged_msg->point_num);

    // 添加第一个雷达的点
    for (size_t i = 0; i < cloud1->size(); ++i) {
      livox_ros_driver2::msg::CustomPoint point;
      point.x = cloud1->points[i].x;
      point.y = cloud1->points[i].y;
      point.z = cloud1->points[i].z;
      point.reflectivity = cloud1->points[i].intensity;
      point.offset_time = msg1->points[i].offset_time;
      point.tag = msg1->points[i].tag;
      point.line = msg1->points[i].line;
      merged_msg->points.push_back(point);
    }

    // 添加第二个雷达的点
    for (size_t i = 0; i < cloud2->size(); ++i) {
      livox_ros_driver2::msg::CustomPoint point;
      point.x = cloud2->points[i].x;
      point.y = cloud2->points[i].y;
      point.z = cloud2->points[i].z;
      point.reflectivity = cloud2->points[i].intensity;
      point.offset_time = msg2->points[i].offset_time;
      point.tag = msg2->points[i].tag;
      point.line = msg2->points[i].line;
      merged_msg->points.push_back(point);
    }

    merged_custom_pub_->publish(*merged_msg);
    // RCLCPP_INFO(this->get_logger(), "Published CustomMsg with %zu points", merged_msg->points.size());
  }

  // 计算相对于新时间基准的偏移时间
  uint32_t calculateOffsetTime(uint64_t src_timebase, uint32_t src_offset, uint64_t dst_timebase) {
    uint64_t absolute_time = src_timebase + src_offset;
    return static_cast<uint32_t>(absolute_time - dst_timebase);
  }

  void setTransformMatrix(Eigen::Matrix4f& transform, 
                         float roll, float pitch, float yaw,
                         float tx, float ty, float tz)
  {
    // 创建旋转矩阵（ZYX顺序：yaw -> pitch -> roll）
    Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
    
    Eigen::Quaternionf q = yaw_angle * pitch_angle * roll_angle;
    transform.block<3,3>(0,0) = q.matrix();
    transform(0,3) = tx;
    transform(1,3) = ty;
    transform(2,3) = tz;
  }

  // 同步策略和订阅器
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    livox_ros_driver2::msg::CustomMsg, 
    livox_ros_driver2::msg::CustomMsg>;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> cloud1_sub_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> cloud2_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // 发布器
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr merged_custom_pub_;

  // 变换参数
  Eigen::Matrix4f transform1_, transform2_;
  float roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_;
  float roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MergeCloudNode>());
  rclcpp::shutdown();
  return 0;
}
