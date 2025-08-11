#ifndef CLOUD_PROCESS__POINT_CLOUD_PREPROCESSOR_HPP_
#define CLOUD_PROCESS__POINT_CLOUD_PREPROCESSOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/common/transforms.h> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/latest_time.h"
#include "message_filters/time_synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"

namespace cloud_process
{

// 定义多边形区域结构体
struct PolygonArea {
    std::vector<double> x;  // X坐标数组
    std::vector<double> y;  // Y坐标数组
    double min_z;           // 区域内过滤的最小高度
    double max_z;           // 区域内过滤的最大高度
};

class PointCloudPreprocessor : public rclcpp::Node
{
public:
    PointCloudPreprocessor(const rclcpp::NodeOptions &options);
    ~PointCloudPreprocessor() = default;

private:
    // 同步策略类型
    using CloudOdomSyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        nav_msgs::msg::Odometry>;
    
    // 回调函数
    void cloud_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);

    //点云坐标系转换函数
    bool transform_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                      const builtin_interfaces::msg::Time& stamp);
    
    //点云预处理函数
    void apply_filters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud); 
    
    // 判断点是否在多边形内
    bool pointInPolygon(double x, double y, const PolygonArea& area);
    
    void load_regions_from_string(const std::string& config_str);
    // 互斥锁
    std::mutex tf_mutex_;

    //参数
    std::string target_frame_; //目标坐标系
    std::string lidar_frame_;
    std::string base_frame_;

    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    double z_min_;
    double z_max_;
    double barrier_min_z_;
    double barrier_max_z_;
    double outlier_radius_;
    int outlier_min_neighbors_;
    int outlier_mean_k_;
    double outlier_std_dev_;
    
    // 多边形区域列表
    std::vector<PolygonArea> polygon_areas_;
    
    //tf
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //发布方、订阅方
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_pub_;
};

}  // namespace cloud_process

#endif  // CLOUD_PROCESS__POINT_CLOUD_PREPROCESSOR_HPP_