// #include <so3_math.h>
#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "li_initialization.h"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false, flg_exit = false;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

int sleep_time = 0;

auto LOGGER = rclcpp::get_logger("laserMapping");

void SigHandle(int sig)
{
  flg_exit = true;
  RCLCPP_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string & file_path)
{
  auto pcd_ptr = std::make_shared<PointCloudXYZI>();

  if (pcl::io::loadPCDFile(file_path, *pcd_ptr) == -1) {
    RCLCPP_ERROR(LOGGER, "Couldn't read pcd file %s", file_path.c_str());
    return nullptr;
  }

  RCLCPP_INFO(LOGGER, "Loaded %zu points from %s", pcd_ptr->size(), file_path.c_str());
  return pcd_ptr;
}

inline void dump_lio_state_to_log(FILE * fp)
{
  V3D rot_ang;
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  if (use_imu_as_input) {
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // Acc
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
      kf_input.x_.gravity(2));  // Bias_a
  } else {
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // omega
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // Acc
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));  // Bias_g
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
      kf_output.x_.gravity(2));  // Bias_a
  }
  fprintf(fp, "\r\n");
  fflush(fp);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void MapIncremental()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) *
        filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (
          fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);
}

void publish_init_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  int size_init_map = init_feats_world->size();

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "odom";
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_down_world, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudFullRes->publish(laserCloudmsg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (pcd_save_en) {
      *pcl_wait_save += *feats_down_world;

      static int scan_wait_num = 0;
      scan_wait_num++;
      if (!pcl_wait_save->empty() && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
        pcd_index++;
        string all_points_dir(
          string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_dir << '\n';
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        pcl_wait_save->clear();
        scan_wait_num = 0;
      }
    }
  }
}

// void publish_frame_body(
//   const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
// {
//   int size = feats_undistort->points.size();
//   PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

//   for (int i = 0; i < size; i++) {
//     pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
//   }

//   sensor_msgs::msg::PointCloud2 laserCloudmsg;
//   pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
//   laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
//   laserCloudmsg.header.frame_id = "body";
//   pubLaserCloudFull_body->publish(laserCloudmsg);
// }
void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1)); // 直接存储世界坐标系点云
    
    // 将点云从lidar body直接转换到世界坐标系
    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom"; // 直接使用世界坐标系标识
    pubLaserCloudFull_body->publish(laserCloudmsg);
}
template <typename T>
void set_posestamp(T & out)
{
  // Static variable, initialized to true, only effective on the first call
  static bool is_first_kf = true;

  auto set_output_from_kf = [&](const auto & kf) {
    out.position.x = kf.x_.pos(0);
    out.position.y = kf.x_.pos(1);
    out.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  };

  if (!use_imu_as_input) {
    if (enable_prior_pcd && is_first_kf) {
      // Execute only on the first call
      kf_output.x_.pos(0) = init_pose[0];
      kf_output.x_.pos(1) = init_pose[1];
      kf_output.x_.pos(2) = init_pose[2];
      set_output_from_kf(kf_output);
      is_first_kf = false;  // Set is_first_kf to false after the first call
    } else {
      set_output_from_kf(kf_output);
    }
  } else {
    set_output_from_kf(kf_input);
  }
}

void publish_odometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,
  std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
  V3D body_vel;
  V3D angular_vel;
  if (!use_imu_as_input) {
    // 使用kf_output的状态
    V3D world_vel = kf_output.x_.vel;
    
    // 转换到body坐标系
    M3D R_inv = kf_output.x_.rot.inverse();
    body_vel = R_inv * world_vel;
    
    // 使用状态中的角速度
    angular_vel = kf_output.x_.omg;
  } else {
    // 使用kf_input的状态
    V3D world_vel = kf_input.x_.vel;
    
    // 转换到body坐标系
    M3D R_inv = kf_input.x_.rot.inverse();
    body_vel = R_inv * world_vel;
    
    // 获取角速度 (使用最新IMU数据)
    angular_vel << latest_imu_.angular_velocity.x,
                  latest_imu_.angular_velocity.y,
                  latest_imu_.angular_velocity.z;
    angular_vel -= kf_input.x_.bg;  // 减去零偏
  }
  // 填充twist字段
  odomAftMapped.twist.twist.linear.x = body_vel.x();
  odomAftMapped.twist.twist.linear.y = body_vel.y();
  odomAftMapped.twist.twist.linear.z = body_vel.z();
  
  odomAftMapped.twist.twist.angular.x = angular_vel.x();
  odomAftMapped.twist.twist.angular.y = angular_vel.y();
  odomAftMapped.twist.twist.angular.z = angular_vel.z();

  odomAftMapped.header.frame_id = "odom";
  odomAftMapped.child_frame_id = "base_footprint";
  if (publish_odometry_without_downsample) {
    odomAftMapped.header.stamp = get_ros_time(time_current);
  } else {
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  }
  set_posestamp(odomAftMapped.pose.pose);

  pubOdomAftMapped->publish(odomAftMapped);

  if (tf_send_en) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform);
  }


  
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  // msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  msg_body_pose.header.frame_id = "odom";
  static int jjj = 0;
  jjj++;
  // if (jjj % 2 == 0) // if path is too large, the rvis will crash
  {
    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);

  readParameters(nh);
  std::cout << "lidar_type: " << lidar_type << '\n';
  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  path.header.stamp = get_ros_time(lidar_end_time);
  path.header.frame_id = "odom";

  /*** variables definition for counting ***/
  int frame_num = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
         aver_time_solve = 0, aver_time_propag = 0;

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else {
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }

  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en = imu_en;

  kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
  kf_output.init_dyn_share_modified_3h(
    get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
  Eigen::Matrix<double, 24, 24> P_init;  // = MD(18, 18)::Identity() * 0.1;
  reset_cov(P_init);
  kf_input.change_P(P_init);
  Eigen::Matrix<double, 30, 30> P_init_output;  // = MD(24, 24)::Identity() * 0.01;
  reset_cov_output(P_init_output);
  kf_output.change_P(P_init_output);
  Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
  /*** debug record ***/
  FILE * fp;
  string pos_log_dir = root_dir + "/Log/pos_log.txt";
  fp = fopen(pos_log_dir.c_str(), "w");
  open_file();

  /*** ROS subscribe initialization ***/
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lid_topic, rclcpp::SensorDataQoS(),
      [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livox_pcl_cbk(msg); });
  } else {
    sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      lid_topic, rclcpp::SensorDataQoS(),
      [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { standard_pcl_cbk(msg); });
  }
  auto sub_imu =
    nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
  auto pub_laser_cloud_full_res =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 20);
  auto pub_laser_cloud_full_res_body =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", 20);
  auto pub_laser_cloud_effect =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_effected", 20);
  auto pub_laser_cloud_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
  auto pub_odom_aft_mapped =
    nh->create_publisher<nav_msgs::msg::Odometry>("Odometry", 20);
  auto pub_path = nh->create_publisher<nav_msgs::msg::Path>("path", 20);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

  //------------------------------------------------------------------------------------------------------
  signal(SIGINT, SigHandle);
  rclcpp::Rate rate(500);
  while (rclcpp::ok()) {
    if (flg_exit) break;
    executor.spin_some();
    if (sync_packages(Measures)) {
      if (flg_reset) {
        RCLCPP_WARN(LOGGER, "reset when rosbag play back");
        p_imu->Reset();
        feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          // state_in = kf_input.get_x();
          state_in = state_input();
          kf_input.change_P(P_init);
        } else {
          // state_out = kf_output.get_x();
          state_out = state_output();
          kf_output.change_P(P_init_output);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          // imu_next = *(imu_deque.front());
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc *= -1;

          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_deque.pop();
            }
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;
          // p_imu->after_imu_init_ = true;
        }
        G_m_s2 =
          std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      double t0, t1, t2, t3, t4, t5, match_start, solve_start;
      match_time = 0;
      solve_time = 0;
      propag_time = 0;
      update_time = 0;
      t0 = omp_get_wtime();

      /*** downsample the feature points in a scan ***/
      t1 = omp_get_wtime();
      p_imu->Process(Measures, feats_undistort);
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      {
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();
      }

      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          // kf_input.x_.rot; //.normalize();
          // kf_output.x_.rot; //.normalize();
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      /*** initialize the map ***/
      if (!init_map) {
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        for (const auto & point : *feats_down_world) {
          init_feats_world->points.emplace_back(point);
        }

        if (init_feats_world->size() >= init_map_size) {
          if (enable_prior_pcd) {
            auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
            ivox_->AddPoints(map_cloud->points);
          } else {
            ivox_->AddPoints(init_feats_world->points);
          }
          publish_init_map(pub_laser_cloud_map);
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        } else {
          init_map = false;
        }
        continue;
      }

      /*** ICP and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      t2 = omp_get_wtime();

      /*** iterated state estimation ***/
      crossmat_list.reserve(feats_down_size);
      pbody_list.reserve(feats_down_size);
      // pbody_ext_list.reserve(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        // {
        //     if (!use_imu_as_input)
        //     {
        //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
        //     }
        //     else
        //     {
        //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
        //     }
        // }
        // else
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }
      if (!use_imu_as_input) {
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
              }
              is_first_frame = false;
              imu_upda_cov = true;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (imu_en && !imu_deque.empty()) {
              bool last_imu = get_time_sec(imu_next.header.stamp) ==
                              get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                     !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                imu_upda_cov = true;
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                kf_output.predict(dt, Q_output, input_in, true, false);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    double propag_imu_start = omp_get_wtime();

                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                    propag_time += omp_get_wtime() - propag_imu_start;
                    double solve_imu_start = omp_get_wtime();
                    kf_output.update_iterated_dyn_share_IMU();
                    solve_time += omp_get_wtime() - solve_imu_start;
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            double propag_state_start = omp_get_wtime();
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output, input_in, true, false);
            propag_time += omp_get_wtime() - propag_state_start;
            time_predict_last_const = time_current;
            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            if (!kf_output.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }
            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_output.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose()
                         << " " << kf_output.x_.acc.transpose() << " "
                         << kf_output.x_.gravity.transpose() << " " << kf_output.x_.bg.transpose()
                         << " " << kf_output.x_.ba.transpose() << " "
                         << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx += time_seq[k];
            // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                  if (dt_cov > 0.0) {
                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                    time_update_last = time_current;
                  }
                  kf_output.predict(dt, Q_output, input_in, true, false);
                }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                kf_output.update_iterated_dyn_share_IMU();
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else {
        bool imu_prop_cov = false;
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
              imu_prop_cov = true;

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
            {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = get_time_sec(imu_last.header.stamp);  //time_current;
              }
              kf_input.predict(dt, Q_input, input_in, true, false);
              t_last = get_time_sec(imu_last.header.stamp);
              imu_prop_cov = true;

              if (imu_deque.empty()) break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_upda_cov = true;
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;
            double propag_start = omp_get_wtime();

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input, input_in, true, false);

            propag_time += omp_get_wtime() - propag_start;

            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            if (!kf_input.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }

            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_input.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose()
                         << " " << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }
            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx = idx + time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }
                imu_prop_cov = true;

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - t_last;

                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                  time_update_last = get_time_sec(imu_next.header.stamp);  //time_current;
                }
                // kf_input.predict(dt, Q_input, input_in, true, false);

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      // M3D rot_cur_lidar;
      // {
      //     rot_cur_lidar = state.rot_end;
      // }
      // euler_cur = RotMtoEuler(rot_cur_lidar);
      // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      //                     (euler_cur(0), euler_cur(1), euler_cur(2));
      /******* Publish odometry downsample *******/
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
      }

      /*** add the feature points to map ***/
      t3 = omp_get_wtime();
      if (feats_down_size > 4) {
        if (enable_prior_pcd) {
          sleep_time++;
          if (sleep_time > 200) {
            MapIncremental();
          }
        } else {
          MapIncremental();
        }
      }
      t5 = omp_get_wtime();
      /******* Publish points *******/
      if (path_en) publish_path(pub_path);
      if (scan_pub_en || pcd_save_en) publish_frame_world(pub_laser_cloud_full_res);
      if (scan_pub_en && scan_body_pub_en) publish_frame_body(pub_laser_cloud_full_res_body);

      /*** Debug variables Logging ***/
      if (runtime_pos_log) {
        frame_num++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        {
          aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
        }
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
        aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] = feats_undistort->points.size();
        s_plot3[time_log_counter] = aver_time_consu;
        time_log_counter++;
        printf(
          "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
          "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
          t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
          aver_time_icp, aver_time_propag);
        if (!publish_odometry_without_downsample) {
          if (!use_imu_as_input) {
            euler_cur = SO3ToEuler(kf_output.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                     << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                     << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose()
                     << " " << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose()
                     << " " << feats_undistort->points.size() << '\n';
          } else {
            euler_cur = SO3ToEuler(kf_input.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                     << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                     << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                     << feats_undistort->points.size() << '\n';
          }
        }
        dump_lio_state_to_log(fp);
      }
    }
    rate.sleep();
  }
  //--------------------------save map-----------------------------------
  // 1. make sure you have enough memories
  // 2. noted that pcd save will influence the real-time performances
  if (!pcl_wait_save->empty() && pcd_save_en) {
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }
  fout_out.close();
  fout_imu_pbp.close();
  return 0;
}