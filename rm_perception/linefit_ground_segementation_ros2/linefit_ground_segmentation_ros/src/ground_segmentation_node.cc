#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include "ground_segmentation/ground_segmentation.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <rm_decision_interfaces/msg/mode.hpp>
class SegmentationNode : public rclcpp::Node {
public:
  SegmentationNode(const rclcpp::NodeOptions &node_options);
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void modeCallback(const rm_decision_interfaces::msg::Mode::SharedPtr msg);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::Mode>::SharedPtr mode_sub_;
  GroundSegmentationParams params_;
  std::shared_ptr<GroundSegmentation> segmenter_;
  std::string gravity_aligned_frame_;
  rm_decision_interfaces::msg::Mode mode_;
  std::mutex mode_mutex_;//互斥锁
};

SegmentationNode::SegmentationNode(const rclcpp::NodeOptions &node_options)
    : Node("ground_segmentation", node_options) {
  gravity_aligned_frame_ =
      this->declare_parameter("gravity_aligned_frame", "gravity_aligned");

  params_.visualize = this->declare_parameter("visualize", params_.visualize);
  params_.n_bins = this->declare_parameter("n_bins", params_.n_bins);
  params_.n_segments =
      this->declare_parameter("n_segments", params_.n_segments);
  params_.max_dist_to_line =
      this->declare_parameter("max_dist_to_line", params_.max_dist_to_line);
  params_.max_slope = this->declare_parameter("max_slope", params_.max_slope);
  params_.min_slope = this->declare_parameter("min_slope", params_.min_slope);
  params_.long_threshold =
      this->declare_parameter("long_threshold", params_.long_threshold);
  params_.max_long_height =
      this->declare_parameter("max_long_height", params_.max_long_height);
  params_.max_start_height =
      this->declare_parameter("max_start_height", params_.max_start_height);
  params_.sensor_height =
      this->declare_parameter("sensor_height", params_.sensor_height);
  params_.line_search_angle =
      this->declare_parameter("line_search_angle", params_.line_search_angle);
  params_.n_threads = this->declare_parameter("n_threads", params_.n_threads);
  // Params that need to be squared.
  double r_min, r_max, max_fit_error;
  if (this->get_parameter("r_min", r_min)) {
    params_.r_min_square = r_min * r_min;
  }
  if (this->get_parameter("r_max", r_max)) {
    params_.r_max_square = r_max * r_max;
  }
  if (this->get_parameter("max_fit_error", max_fit_error)) {
    params_.max_error_square = max_fit_error * max_fit_error;
  }
  segmenter_ = std::make_shared<GroundSegmentation>(params_);
  std::string ground_topic, obstacle_topic, input_topic;
  ground_topic = this->declare_parameter("ground_output_topic", "ground_cloud");
  obstacle_topic =
      this->declare_parameter("obstacle_output_topic", "obstacle_cloud");
  input_topic = this->declare_parameter("input_topic", "input_cloud");
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1));
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        obstacle_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  RCLCPP_INFO(this->get_logger(), "Segmentation node initialized");
  mode_sub_ = this->create_subscription<rm_decision_interfaces::msg::Mode>(
        "/mode",  // 确保与发送方的主题一致
        rclcpp::SensorDataQoS(),
  std::bind(&SegmentationNode::modeCallback, this, std::placeholders::_1));
}

void SegmentationNode::scanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;

  std::vector<int> labels;

  bool is_original_pc = true;
  if (!gravity_aligned_frame_.empty()) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
          gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp);
      // Remove translation part.
      tf_stamped.transform.translation.x = 0;
      tf_stamped.transform.translation.y = 0;
      tf_stamped.transform.translation.z = 0;
      Eigen::Affine3d tf;
      tf.translate(Eigen::Vector3d(0, 0, 0));
      tf.rotate(Eigen::Quaterniond(
          tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z));
      // tf::transformMsgToEigen(tf_stamped.transform, tf);
      pcl::transformPointCloud(cloud, cloud_transformed, tf);
      is_original_pc = false;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform point cloud into "
                  "gravity frame: %s",
                  ex.what());
    }
  }

  // Trick to avoid PC copy if we do not transform.
  const pcl::PointCloud<pcl::PointXYZ> &cloud_proc =
      is_original_pc ? cloud : cloud_transformed;

  segmenter_->segment(cloud_proc, &labels);
  pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (labels[i] == 1)
      ground_cloud.push_back(cloud[i]);
    else
      obstacle_cloud.push_back(cloud[i]);
  }

  // 转换为智能指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_ptr = obstacle_cloud.makeShared();

  // 执行CropBox过滤
  // pcl::CropBox<pcl::PointXYZ> crop_filter;
  // crop_filter.setInputCloud(obstacle_ptr);
  // crop_filter.setMin(Eigen::Vector4f(-0.5, -0.5, -0.3, 1.0));
  // crop_filter.setMax(Eigen::Vector4f(0.5, 0.5, 0.7, 1.0));
  // crop_filter.setNegative(true);
  // crop_filter.filter(*obstacle_ptr);

  
    
  if(mode_.mode==1)
  {
  
    // 执行高度过滤
    pcl::PassThrough<pcl::PointXYZ> height_filter;
    height_filter.setInputCloud(obstacle_ptr);
    height_filter.setFilterFieldName("z");
    height_filter.setFilterLimits(0.1,0.45);
    height_filter.setNegative(true);
    height_filter.filter(*obstacle_ptr);

    pcl::PassThrough<pcl::PointXYZ> height_filter1;
    height_filter1.setInputCloud(obstacle_ptr);
    height_filter1.setFilterFieldName("z");
    height_filter1.setFilterLimits(-0.1,0.0);
    height_filter1.setNegative(true);
    height_filter1.filter(*obstacle_ptr);
  }

  // 统计离群点滤除（新增部分）
  // pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
  // ror.setInputCloud(obstacle_ptr);
  // ror.setRadiusSearch(0.5);       // 搜索半径0.5米
  // ror.setMinNeighborsInRadius(10);  // 半径内最少需要5个邻居
  // ror.filter(*obstacle_ptr);
  

  // 将过滤后的点云转换回普通格式
  obstacle_cloud = *obstacle_ptr;

  auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  auto obstacle_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(ground_cloud, *ground_msg);
  pcl::toROSMsg(obstacle_cloud, *obstacle_msg);
  ground_msg->header = msg->header;
  obstacle_msg->header = msg->header;
  ground_pub_->publish(*ground_msg);
  obstacle_pub_->publish(*obstacle_msg);
}

void SegmentationNode::modeCallback(const rm_decision_interfaces::msg::Mode::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    mode_ = *msg;
    // RCLCPP_INFO(get_logger(), "云台模式 updated: %d", current_mode_.mode);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SegmentationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
