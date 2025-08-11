#include "cloud_process/cloud_process.hpp"
#include <pcl/filters/conditional_removal.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <cmath>
#include <rclcpp/parameter_value.hpp>  // 添加此头文件
#include <pcl/filters/crop_box.h>
namespace cloud_process
{

PointCloudPreprocessor::PointCloudPreprocessor
            (const rclcpp::NodeOptions &options) : Node("cloud_process_node", options)
{   
    // 声明参数
    this->declare_parameter("x_min", -0.5);
    this->declare_parameter("x_max", 0.5);
    this->declare_parameter("y_min", -0.5);
    this->declare_parameter("y_max", 0.5);
    this->declare_parameter("z_min", -1.0);
    this->declare_parameter("z_max", 1.0);
    this->declare_parameter("barrier_min_z", 0.4);
    this->declare_parameter("barrier_max_z", 0.5);
    this->declare_parameter("outlier_radius", 0.5);
    this->declare_parameter("outlier_min_neighbors", 5);
    this->declare_parameter("outlier_mean_k", 50);
    this->declare_parameter("outlier_std_dev", 1.0);
    this->declare_parameter("target_frame", "odom");
    this->declare_parameter("lidar_frame", "livox_frame");
    this->declare_parameter("base_frame", "base_link");
    
    // 获取参数值
    target_frame_ = this->get_parameter("target_frame").get_value<std::string>();
    lidar_frame_ = this->get_parameter("lidar_frame").get_value<std::string>(); 
    base_frame_ = this->get_parameter("base_frame").get_value<std::string>();
    x_min_ = this->get_parameter("x_min").get_value<double>();
    x_max_ = this->get_parameter("x_max").get_value<double>();
    y_min_ = this->get_parameter("y_min").get_value<double>();
    y_max_ = this->get_parameter("y_max").get_value<double>();
    z_min_ = this->get_parameter("z_min").get_value<double>();
    z_max_ = this->get_parameter("z_max").get_value<double>();
    barrier_min_z_ = this->get_parameter("barrier_min_z").get_value<double>();
    barrier_max_z_ = this->get_parameter("barrier_max_z").get_value<double>();
    outlier_radius_ = this->get_parameter("outlier_radius").get_value<double>();
    outlier_min_neighbors_= this->get_parameter("outlier_min_neighbors").get_value<int>();
    outlier_mean_k_ = this->get_parameter("outlier_mean_k").get_value<int>();
    outlier_std_dev_ = this->get_parameter("outlier_std_dev").get_value<double>();

    RCLCPP_INFO(get_logger(), "开始创建硬编码多边形区域...");
    
    // rmuc
    // PolygonArea area1;
    // area1.min_z = -0.2;
    // area1.max_z = 0.4;
    // area1.x = {8.0, 8.0, 11.5, 11.5, 8.0};
    // area1.y = {6.0,5.0, 5.0, 6.0,6.0};
    // polygon_areas_.push_back(area1);
    // RCLCPP_INFO(get_logger(), "区域1: %zu个顶点, 高度范围 [%.2f, %.2f]",
    //            area1.x.size(), area1.min_z, area1.max_z);

    // PolygonArea area2;
    // area2.min_z = -0.2;
    // area2.max_z = 0.4;
    // area2.x = {8.3, 8.5, 6.3, 6.0, 8.3};
    // area2.y = {-2.2, -3.0,-4.0, -3.3,-2.2};
    // polygon_areas_.push_back(area2);
    // RCLCPP_INFO(get_logger(), "区域2: %zu个顶点, 高度范围 [%.2f, %.2f]",
    //            area2.x.size(), area2.min_z, area2.max_z);
    
    // PolygonArea area3;
    // area3.min_z = -0.2;
    // area3.max_z = 0.4;
    // area3.x = {9.5, 9.5, 13.0, 13.0, 9.5};
    // area3.y = {-6.0,-7.0, -7.0, -6.0,-6.0};
    // polygon_areas_.push_back(area3);
    // RCLCPP_INFO(get_logger(), "区域3: %zu个顶点, 高度范围 [%.2f, %.2f]",
    //            area3.x.size(), area3.min_z, area3.max_z);

    // PolygonArea area4;
    // area4.min_z = -0.2;
    // area4.max_z = 0.4;
    // area4.x = {12.0, 12.2, 14.7, 14.3, 12.0};
    // area4.y = {2.2, 1.2, 2.2, 3.0, 2,2};
    // polygon_areas_.push_back(area4);
    // RCLCPP_INFO(get_logger(), "区域4: %zu个顶点, 高度范围 [%.2f, %.2f]",
    //            area4.x.size(), area4.min_z, area4.max_z);      


    //homered
    PolygonArea area1;
    area1.min_z = -3.0;
    area1.max_z = 3.0;
    area1.x = {9.0, 11.0, 11.0, 9.0, 9.0};
    area1.y = {-5.0,-5.0, -6.0, -6.0,-5.0};
    polygon_areas_.push_back(area1);
    RCLCPP_INFO(get_logger(), "区域1: %zu个顶点, 高度范围 [%.2f, %.2f]",
               area1.x.size(), area1.min_z, area1.max_z);

    PolygonArea area2;
    area2.min_z = -3.0;
    area2.max_z = 3.0;
    area2.x = {5.0, 5.0, 6.5, 6.5, 5.0};
    area2.y = {-2.0,-2.5, -2.5, -1.5,-2.0};
    polygon_areas_.push_back(area2);
    RCLCPP_INFO(get_logger(), "区域2: %zu个顶点, 高度范围 [%.2f, %.2f]",
               area2.x.size(), area2.min_z, area2.max_z);
    
    PolygonArea area3;
    area3.min_z = -3.0;
    area3.max_z = 3.0;
    area3.x = {12.7, 13.7, 13.6, 12.5,12.7};
    area3.y = {3.5, 3.9, 4.5, 4.5,3.5};
    polygon_areas_.push_back(area3);
    RCLCPP_INFO(get_logger(), "区域3: %zu个顶点, 高度范围 [%.2f, %.2f]",
               area3.x.size(), area3.min_z, area3.max_z);

    RCLCPP_INFO(get_logger(), "共创建 %zu 个多边形区域", polygon_areas_.size());
    // 创建点云订阅器
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_registered", 10,
        std::bind(&PointCloudPreprocessor::cloud_callback, this, std::placeholders::_1));

    // 创建处理后的点云发布器
    processed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_cloud", 10);

    // 初始化TF2缓冲器和监听器
    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
} 

void PointCloudPreprocessor::cloud_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
{   
    std::lock_guard<std::mutex> lock(tf_mutex_);
    
    // 转换ROS点云到PCL格式
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 执行坐标系变换（可选，根据需求启用）
    // if (!transform_cloud(cloud, cloud_msg->header.stamp)) {
    //     RCLCPP_ERROR(get_logger(), "点云坐标系转换失败");
    //     return;
    // }
    
    // 应用过滤算法
    apply_filters(cloud);

    // 转换回ROS格式并发布
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = cloud_msg->header;
    output.header.frame_id = "odom"; // 使用目标坐标系
    processed_pub_->publish(output);
}

bool PointCloudPreprocessor::transform_cloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
    const builtin_interfaces::msg::Time& stamp) 
{
    try {
        // 获取从雷达坐标系到目标坐标系的变换
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(
                target_frame_,    // 目标坐标系
                lidar_frame_,     // 源坐标系
                stamp,            // 使用点云时间戳
                rclcpp::Duration::from_seconds(0.1)); // 等待时间

        // 应用坐标变换
        Eigen::Matrix4f transform_matrix = 
            tf2::transformToEigen(transform.transform).matrix().cast<float>();
        pcl::transformPointCloud(*cloud, *cloud, transform_matrix);

        // 移除NaN点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "TF转换异常: %s", ex.what());
        return false;
    }
}

// 判断点是否在多边形内（射线法）
bool PointCloudPreprocessor::pointInPolygon(double x, double y, const PolygonArea& area) {
    int i, j;
    bool inside = false;
    int n = area.x.size();
    
    for (i = 0, j = n-1; i < n; j = i++) {
        if (((area.y[i] > y) != (area.y[j] > y)) &&
            (x < (area.x[j] - area.x[i]) * (y - area.y[i]) / 
                 (area.y[j] - area.y[i]) + area.x[i])) {
            inside = !inside;
        }
    }
    
    return inside;
}

void PointCloudPreprocessor::apply_filters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{   
    if (cloud->empty()) {
        RCLCPP_WARN(get_logger(), "输入点云为空");
        return;
    }
    size_t original_size = cloud->size();

    // 第一阶段：包围盒滤波（移除机体周围的点）

    pcl::CropBox<pcl::PointXYZI> crop_filter;
    crop_filter.setInputCloud(cloud);
    crop_filter.setMin(Eigen::Vector4f(-0.5, -0.5, -0.5, 1.0));
    crop_filter.setMax(Eigen::Vector4f(0.5, 0.5, 0.7, 1.0));
    crop_filter.setNegative(true);
    crop_filter.filter(*cloud);
    // RCLCPP_INFO(get_logger(), "包围盒滤波: 移除 %zu 点, 剩余: %zu", 
        //         original_size - cloud->size(), cloud->size());
    
    // 第二阶段：多个多边形区域内高度滤波
    if (!polygon_areas_.empty()) {
        // 创建新的点云，只保留不在任何多边形区域内的点
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        filtered_cloud->reserve(cloud->size());
        
        size_t removed_count = 0;
        for (const auto& point : *cloud) {
            bool in_any_area = false;
            
            // 检查点是否在任何一个多边形区域内
            for (const auto& area : polygon_areas_) {
                // 计算多边形的外接矩形，用于快速筛选
                double min_x = *std::min_element(area.x.begin(), area.x.end());
                double max_x = *std::max_element(area.x.begin(), area.x.end());
                double min_y = *std::min_element(area.y.begin(), area.y.end());
                double max_y = *std::max_element(area.y.begin(), area.y.end());
                
                // 快速检查：如果点在外接矩形外，跳过
                if (point.x < min_x || point.x > max_x || 
                    point.y < min_y || point.y > max_y) {
                    continue;
                }
                
                // 精确检查：点是否在多边形内
                if (pointInPolygon(point.x, point.y, area)) {
                    // 如果在多边形内且高度在过滤范围内，则标记为需要移除
                    if (point.z >= area.min_z && point.z <= area.max_z) {
                        in_any_area = true;
                        break; // 点在一个区域内，无需检查其他区域
                    }
                }
            }
            
            if (in_any_area) {
                removed_count++;
            } else {
                filtered_cloud->push_back(point);
            }
        }
        
        *cloud = *filtered_cloud;
        RCLCPP_INFO(get_logger(), "多边形区域高度滤波: 移除 %zu 点, 剩余: %zu",
                   removed_count, cloud->size());
    } else {
        RCLCPP_INFO(get_logger(), "无多边形区域配置，跳过区域高度滤波");
    }

    // 第三阶段：高度滤波（移除障碍物区域）
    // pcl::PassThrough<pcl::PointXYZI> height_filter;
    // height_filter.setInputCloud(cloud);
    // height_filter.setFilterFieldName("z");
    // height_filter.setFilterLimits(barrier_min_z_, barrier_max_z_);
    // height_filter.setNegative(true);
    
    // size_t before_height = cloud->size();
    // height_filter.filter(*cloud);
    // RCLCPP_INFO(get_logger(), "障碍物高度滤波: 移除高度[%.2f-%.2f]的点: 移除 %zu 点, 剩余: %zu",
    //            barrier_min_z_, barrier_max_z_, before_height - cloud->size(), cloud->size());
    
    // 第四阶段：统计离群点去除
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    // sor.setInputCloud(cloud);
    // sor.setMeanK(outlier_mean_k_);
    // sor.setStddevMulThresh(outlier_std_dev_);
    // sor.filter(*cloud);
    // RCLCPP_INFO(get_logger(), "统计离群点去除后点云大小: %zu", cloud->size());
    // // 第四阶段：半径离群点去除
    // pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
    // ror.setInputCloud(cloud);
    // ror.setRadiusSearch(outlier_radius_);
    // ror.setMinNeighborsInRadius(outlier_min_neighbors_);
    // ror.filter(*cloud);
    // RCLCPP_INFO(get_logger(), "半径离群点去除后点云大小: %zu", cloud->size());


    // 第四阶段：体素降采样（提升处理效率）
    //     pcl::VoxelGrid<pcl::PointXYZI> voxel;
    //     voxel.setInputCloud(cloud);
    //     voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    //     voxel.setMinimumPointsNumberPerVoxel(2); // 避免孤立点
        
    //     size_t before_voxel = cloud->size();
    //     voxel.filter(*cloud);
    //     RCLCPP_INFO(get_logger(), "体素降采样: 移除 %zu 点, 剩余: %zu", 
    //                before_voxel - cloud->size(), cloud->size());
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cloud_process::PointCloudPreprocessor)