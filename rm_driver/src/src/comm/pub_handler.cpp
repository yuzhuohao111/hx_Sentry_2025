//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "pub_handler.h"  // 包含PubHandler类的头文件

#include <chrono>    // 用于时间相关操作
#include <cstdlib>   // 标准库头文件
#include <iostream>  // 输入输出流
#include <limits>    // 数值极限相关

namespace livox_ros  // 定义命名空间livox_ros
{

std::atomic<bool> PubHandler::is_timestamp_sync_;  // 静态原子变量，用于时间戳同步状态

// 获取PubHandler单例实例
PubHandler & pub_handler()
{
  static PubHandler handler;  // 静态局部变量，确保单例
  return handler;
}

// 全局外参变量，初始化为零位移和单位矩阵旋转
static ExtParameterDetailed extrinsic_global = {{0, 0, 0}, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

// 初始化函数，目前为空
void PubHandler::Init() {}

// 反初始化函数，清理资源
void PubHandler::Uninit()
{
  // 如果监听ID有效，则移除点云观察者
  if (lidar_listen_id_ > 0) {
    LivoxLidarRemovePointCloudObserver(lidar_listen_id_);
    lidar_listen_id_ = 0;
  }

  RequestExit();  // 请求退出

  // 如果点云处理线程存在且可加入，则等待线程结束
  if (point_process_thread_ && point_process_thread_->joinable()) {
    point_process_thread_->join();
    point_process_thread_ = nullptr;
  } else {
    /* */
  }
}

// 设置退出标志
void PubHandler::RequestExit() { is_quit_.store(true); }

// 设置点云发布频率
void PubHandler::SetPointCloudConfig(const double publish_freq)
{
  // 计算发布间隔（纳秒）
  publish_interval_ = (kNsPerSecond / (publish_freq * 10)) * 10;
  // 计算容忍的帧时间偏差
  publish_interval_tolerance_ = publish_interval_ - kNsTolerantFrameTimeDeviation;
  // 转换为毫秒
  publish_interval_ms_ = publish_interval_ / kRatioOfMsToNs;
  // 如果点云处理线程不存在，则创建
  if (!point_process_thread_) {
    point_process_thread_ = std::make_shared<std::thread>(&PubHandler::RawDataProcess, this);
  }
  return;
}

// 设置IMU数据回调函数
void PubHandler::SetImuDataCallback(ImuDataCallback cb, void * client_data)
{
  imu_client_data_ = client_data;  // 保存客户端数据
  imu_callback_ = cb;              // 保存回调函数
}

// 添加雷达外参
void PubHandler::AddLidarsExtParam(LidarExtParameter & lidar_param)
{
  std::unique_lock<std::mutex> lock(packet_mutex_);  // 加锁
  uint32_t id = 0;
  GetLidarId(lidar_param.lidar_type, lidar_param.handle, id);  // 获取雷达ID
  lidar_extrinsics_[id] = lidar_param;                         // 保存外参
}

// 清除所有雷达外参
void PubHandler::ClearAllLidarsExtrinsicParams()
{
  std::unique_lock<std::mutex> lock(packet_mutex_);  // 加锁
  lidar_extrinsics_.clear();                         // 清空外参容器
}

// 设置点云回调函数
void PubHandler::SetPointCloudsCallback(PointCloudsCallback cb, void * client_data)
{
  pub_client_data_ = client_data;  // 保存客户端数据
  points_callback_ = cb;           // 保存回调函数
  // 添加点云观察者
  lidar_listen_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this);
}

ExtParameterDetailed PubHandler::GetLidarExtrinsic(uint32_t lidar_id) {
    std::unique_lock<std::mutex> lock(packet_mutex_);
    auto it = lidar_extrinsics_.find(lidar_id);
    if (it != lidar_extrinsics_.end()) {
        LidarExtParameter &param = it->second;
        ExtParameterDetailed ext;
        ext.trans[0] = param.param.x;
        ext.trans[1] = param.param.y;
        ext.trans[2] = param.param.z;

        double roll = param.param.roll;
        double pitch = param.param.pitch;
        double yaw = param.param.yaw;
        double cos_roll = cos(roll * M_PI / 180.0);
        double sin_roll = sin(roll * M_PI / 180.0);
        double cos_pitch = cos(pitch * M_PI / 180.0);
        double sin_pitch = sin(pitch * M_PI / 180.0);
        double cos_yaw = cos(yaw * M_PI / 180.0);
        double sin_yaw = sin(yaw * M_PI / 180.0);

        ext.rotation[0][0] = cos_pitch * cos_yaw;
        ext.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
        ext.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

        ext.rotation[1][0] = cos_pitch * sin_yaw;
        ext.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
        ext.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

        ext.rotation[2][0] = -sin_pitch;
        ext.rotation[2][1] = sin_roll * cos_pitch;
        ext.rotation[2][2] = cos_roll * cos_pitch;

        return ext;
    } else {
        // 返回默认外参（单位矩阵，零平移）
        return extrinsic_global;
    }
}

// 雷达点云数据回调函数
void PubHandler::OnLivoxLidarPointCloudCallback(
  uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket * data, void * client_data)
{
  PubHandler * self = (PubHandler *)client_data;  // 获取PubHandler实例
  if (!self) {
    return;
  }

  // 根据时间戳类型设置同步状态
  if (data->time_type != kTimestampTypeNoSync) {
    is_timestamp_sync_.store(true);
  } else {
    is_timestamp_sync_.store(false);
  }

  // 处理IMU数据
  if (data->data_type == kLivoxLidarImuData) {
    if (self->imu_callback_) {
      RawImuPoint * imu = (RawImuPoint *)data->data;
      ImuData imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LidarProtoType::kLivoxLidarType);
      imu_data.handle = handle;

      // 获取雷达ID
      uint32_t lidar_id = 0;
      GetLidarId(LidarProtoType::kLivoxLidarType, handle, lidar_id);

      // 获取对应的外参
      ExtParameterDetailed imu_extrinsic = self->GetLidarExtrinsic(lidar_id);

      imu_data.time_stamp = GetEthPacketTimestamp(data->time_type, data->timestamp, sizeof(data->timestamp));

      // 应用外参到陀螺仪数据
      imu_data.gyro_x = imu->gyro_x * imu_extrinsic.rotation[0][0] +
                        imu->gyro_y * imu_extrinsic.rotation[0][1] +
                        imu->gyro_z * imu_extrinsic.rotation[0][2];
      imu_data.gyro_y = imu->gyro_x * imu_extrinsic.rotation[1][0] +
                        imu->gyro_y * imu_extrinsic.rotation[1][1] +
                        imu->gyro_z * imu_extrinsic.rotation[1][2];
      imu_data.gyro_z = imu->gyro_x * imu_extrinsic.rotation[2][0] +
                        imu->gyro_y * imu_extrinsic.rotation[2][1] +
                        imu->gyro_z * imu_extrinsic.rotation[2][2];

      // 应用外参到加速度数据
      imu_data.acc_x = imu->acc_x * imu_extrinsic.rotation[0][0] +
                      imu->acc_y * imu_extrinsic.rotation[0][1] +
                      imu->acc_z * imu_extrinsic.rotation[0][2];
      imu_data.acc_y = imu->acc_x * imu_extrinsic.rotation[1][0] +
                      imu->acc_y * imu_extrinsic.rotation[1][1] +
                      imu->acc_z * imu_extrinsic.rotation[1][2];
      imu_data.acc_z = imu->acc_x * imu_extrinsic.rotation[2][0] +
                      imu->acc_y * imu_extrinsic.rotation[2][1] +
                      imu->acc_z * imu_extrinsic.rotation[2][2];

      //不进行处理
      // imu_data.time_stamp = GetEthPacketTimestamp(data->time_type,
      //                                             data->timestamp, sizeof(data->timestamp));
      // imu_data.gyro_x = imu->gyro_x;
      // imu_data.gyro_y = imu->gyro_y;
      // imu_data.gyro_z = imu->gyro_z;
      // imu_data.acc_x = imu->acc_x;
      // imu_data.acc_y = imu->acc_y;
      // imu_data.acc_z = imu->acc_z;

      // 进行处理 适用单雷达
      // imu_data.time_stamp =
      //   GetEthPacketTimestamp(data->time_type, data->timestamp, sizeof(data->timestamp));
      // // 计算旋转后的陀螺仪数据
      // imu_data.gyro_x = imu->gyro_x * extrinsic_global.rotation[0][0] +
      //                   imu->gyro_y * extrinsic_global.rotation[0][1] +
      //                   imu->gyro_z * extrinsic_global.rotation[0][2];
      // imu_data.gyro_y = imu->gyro_x * extrinsic_global.rotation[1][0] +
      //                   imu->gyro_y * extrinsic_global.rotation[1][1] +
      //                   imu->gyro_z * extrinsic_global.rotation[1][2];
      // imu_data.gyro_z = imu->gyro_x * extrinsic_global.rotation[2][0] +
      //                   imu->gyro_y * extrinsic_global.rotation[2][1] +
      //                   imu->gyro_z * extrinsic_global.rotation[2][2];
      // // 计算旋转后的加速度数据
      // imu_data.acc_x = imu->acc_x * extrinsic_global.rotation[0][0] +
      //                  imu->acc_y * extrinsic_global.rotation[0][1] +
      //                  imu->acc_z * extrinsic_global.rotation[0][2];
      // imu_data.acc_y = imu->acc_x * extrinsic_global.rotation[1][0] +
      //                  imu->acc_y * extrinsic_global.rotation[1][1] +
      //                  imu->acc_z * extrinsic_global.rotation[1][2];
      // imu_data.acc_z = imu->acc_x * extrinsic_global.rotation[2][0] +
      //                  imu->acc_y * extrinsic_global.rotation[2][1] +
      //                  imu->acc_z * extrinsic_global.rotation[2][2];
      // 调用IMU回调函数
      self->imu_callback_(&imu_data, self->imu_client_data_);
    }
    return;
  }
  // 处理点云数据
  RawPacket packet = {};
  packet.handle = handle;
  packet.lidar_type = LidarProtoType::kLivoxLidarType;
  packet.extrinsic_enable = false;
  // 根据设备类型设置线数
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP) {
    packet.line_num = kLineNumberHAP;
  } else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360) {
    packet.line_num = kLineNumberMid360;
  } else {
    packet.line_num = kLineNumberDefault;
  }
  packet.data_type = data->data_type;
  packet.point_num = data->dot_num;
  packet.point_interval = data->time_interval * 100 / data->dot_num;  // 计算点间隔时间（ns）
  // 获取时间戳
  packet.time_stamp =
    GetEthPacketTimestamp(data->time_type, data->timestamp, sizeof(data->timestamp));
  // 复制原始数据
  uint32_t length = data->length - sizeof(LivoxLidarEthernetPacket) + 1;
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);  // 加锁
    self->raw_packet_queue_.push_back(packet);               // 添加到原始数据队列
  }
  self->packet_condition_.notify_one();  // 通知处理线程

  return;
}

// 发布点云数据
void PubHandler::PublishPointCloud()
{
  // 调用点云回调函数
  if (points_callback_) {
    points_callback_(&frame_, pub_client_data_);
  }
  return;
}

// 检查定时器，决定是否发布点云
void PubHandler::CheckTimer(uint32_t id)
{
  if (PubHandler::is_timestamp_sync_.load()) {  // 时间同步已启用
    auto & process_handler = lidar_process_handlers_[id];
    uint64_t recent_time_ms = process_handler->GetRecentTimeStamp() / kRatioOfMsToNs;
    // 检查时间是否符合发布间隔
    if ((recent_time_ms % publish_interval_ms_ != 0) || recent_time_ms == 0) {
      return;
    }

    uint64_t diff = process_handler->GetRecentTimeStamp() - process_handler->GetLidarBaseTime();
    if (diff < publish_interval_tolerance_) {
      return;
    }

    // 设置帧基础时间
    frame_.base_time[frame_.lidar_num] = process_handler->GetLidarBaseTime();
    points_[id].clear();
    // 获取点云数据
    process_handler->GetLidarPointClouds(points_[id]);
    if (points_[id].empty()) {
      return;
    }
    // 设置点云数据
    PointPacket & lidar_point = frame_.lidar_point[frame_.lidar_num];
    lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;  // TODO:
    lidar_point.handle = id;
    lidar_point.points_num = points_[id].size();
    lidar_point.points = points_[id].data();
    frame_.lidar_num++;

    // 如果雷达数量不为0，则发布点云
    if (frame_.lidar_num != 0) {
      PublishPointCloud();
      frame_.lidar_num = 0;
    }
  } else {  // 时间同步未启用
    auto now_time = std::chrono::high_resolution_clock::now();
    // 第一次设置
    static bool first = true;
    if (first) {
      last_pub_time_ = now_time;
      first = false;
      return;
    }
    // 检查是否达到发布间隔
    if (now_time - last_pub_time_ < std::chrono::nanoseconds(publish_interval_)) {
      return;
    }
    last_pub_time_ += std::chrono::nanoseconds(publish_interval_);
    // 遍历所有雷达处理句柄
    for (auto & process_handler : lidar_process_handlers_) {
      frame_.base_time[frame_.lidar_num] = process_handler.second->GetLidarBaseTime();
      uint32_t handle = process_handler.first;
      points_[handle].clear();
      process_handler.second->GetLidarPointClouds(points_[handle]);
      if (points_[handle].empty()) {
        continue;
      }
      // 设置点云数据
      PointPacket & lidar_point = frame_.lidar_point[frame_.lidar_num];
      lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;  // TODO:
      lidar_point.handle = handle;
      lidar_point.points_num = points_[handle].size();
      lidar_point.points = points_[handle].data();
      frame_.lidar_num++;
    }
    PublishPointCloud();  // 发布点云
    frame_.lidar_num = 0;
  }
  return;
}

// 原始数据处理线程函数
void PubHandler::RawDataProcess()
{
  RawPacket raw_data;
  while (!is_quit_.load()) {  // 循环直到退出标志被设置
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);  // 加锁
      // 如果原始数据队列为空，则等待
      if (raw_packet_queue_.empty()) {
        packet_condition_.wait_for(lock, std::chrono::milliseconds(500));
        if (raw_packet_queue_.empty()) {
          continue;
        }
      }
      raw_data = raw_packet_queue_.front();  // 获取队列头部数据
      raw_packet_queue_.pop_front();         // 移除队列头部数据
    }
    uint32_t id = 0;
    GetLidarId(raw_data.lidar_type, raw_data.handle, id);  // 获取雷达ID
    // 如果处理句柄不存在，则创建
    if (lidar_process_handlers_.find(id) == lidar_process_handlers_.end()) {
      lidar_process_handlers_[id].reset(new LidarPubHandler());
    }
    auto & process_handler = lidar_process_handlers_[id];
    // 如果存在外参，则设置
    if (lidar_extrinsics_.find(id) != lidar_extrinsics_.end()) {
      lidar_process_handlers_[id]->SetLidarsExtParam(lidar_extrinsics_[id]);
    }
    process_handler->PointCloudProcess(raw_data);  // 处理点云数据
    CheckTimer(id);                               // 检查定时器
  }
}

// 获取雷达ID
bool PubHandler::GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t & id)
{
  if (lidar_type == kLivoxLidarType) {
    id = handle;  // Livox雷达直接使用handle作为ID
    return true;
  }
  return false;
}

// 获取以太网数据包时间戳
uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t * time_stamp, uint8_t size)
{
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);  // 复制时间戳数据

  // 根据时间戳类型返回对应的时间戳
  if (timestamp_type == kTimestampTypeGptpOrPtp || timestamp_type == kTimestampTypeGps) {
    return time.stamp;
  }

  // 其他情况返回当前时间
  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

/*******************************/
/*  LidarPubHandler Definitions*/
// 构造函数，初始化外参设置标志为false
LidarPubHandler::LidarPubHandler() : is_set_extrinsic_params_(false) {}

// 获取雷达基础时间（第一个点的时间戳）
uint64_t LidarPubHandler::GetLidarBaseTime()
{
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.at(0).offset_time;
}

// 获取雷达点云数据
void LidarPubHandler::GetLidarPointClouds(std::vector<PointXyzlt> & points_clouds)
{
  std::lock_guard<std::mutex> lock(mutex_);  // 加锁
  points_clouds.swap(points_clouds_);        // 交换数据
}

// 获取最近的时间戳（最后一个点的时间戳）
uint64_t LidarPubHandler::GetRecentTimeStamp()
{
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.back().offset_time;
}

// 获取点云数据大小
uint32_t LidarPubHandler::GetLidarPointCloudsSize()
{
  std::lock_guard<std::mutex> lock(mutex_);  // 加锁
  return points_clouds_.size();              // 返回大小
}

// 点云数据处理入口
void LidarPubHandler::PointCloudProcess(RawPacket & pkt)
{
  if (pkt.lidar_type == LidarProtoType::kLivoxLidarType) {
    LivoxLidarPointCloudProcess(pkt);  // 处理Livox雷达数据
  } else {
    static bool flag = false;
    if (!flag) {
      std::cout << "error, unsupported protocol type: " << static_cast<int>(pkt.lidar_type)
                << std::endl;
      flag = true;
    }
  }
}

// 激光雷达点云数据处理类的方法实现

// 处理Livox激光雷达点云数据的主函数
// 参数pkt: 包含原始点云数据的RawPacket结构体
void LidarPubHandler::LivoxLidarPointCloudProcess(RawPacket & pkt)
{
  // 根据数据包中的数据类型字段进行不同的处理
  switch (pkt.data_type) {
    // 处理高精度笛卡尔坐标系数据
    case kLivoxLidarCartesianCoordinateHighData:
      ProcessCartesianHighPoint(pkt);
      break;
    // 处理低精度笛卡尔坐标系数据  
    case kLivoxLidarCartesianCoordinateLowData:
      ProcessCartesianLowPoint(pkt);
      break;
    // 处理球坐标系数据
    case kLivoxLidarSphericalCoordinateData:
      ProcessSphericalPoint(pkt);
      break;
    // 未知数据类型处理
    default:
      std::cout << "unknown data type: " << static_cast<int>(pkt.data_type) << " !!" << std::endl;
      break;
  }
}

// 设置激光雷达外参参数
// 参数lidar_param: 包含外参参数的结构体
void LidarPubHandler::SetLidarsExtParam(LidarExtParameter lidar_param)
{
  // 如果已经设置过外参则直接返回
  if (is_set_extrinsic_params_) {
    return;
  }
  
  // 设置平移参数(x,y,z)
  extrinsic_.trans[0] = lidar_param.param.x;
  extrinsic_.trans[1] = lidar_param.param.y;
  extrinsic_.trans[2] = lidar_param.param.z;

  // 计算旋转角度的三角函数值(将角度转换为弧度)
  double cos_roll = cos(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double cos_pitch = cos(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double cos_yaw = cos(static_cast<double>(lidar_param.param.yaw * PI / 180.0));
  double sin_roll = sin(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double sin_pitch = sin(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double sin_yaw = sin(static_cast<double>(lidar_param.param.yaw * PI / 180.0));

  // 打印外参的旋转角度(roll, pitch, yaw)
  std::cout << "extrinsic rpy"
            << " " << lidar_param.param.roll << " " << lidar_param.param.pitch << " "
            << lidar_param.param.yaw << "\n";

  // 计算并设置旋转矩阵(3x3)
  // 第一行
  extrinsic_.rotation[0][0] = cos_pitch * cos_yaw;
  extrinsic_.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  extrinsic_.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
  // 第二行
  extrinsic_.rotation[1][0] = cos_pitch * sin_yaw;
  extrinsic_.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  extrinsic_.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
  // 第三行
  extrinsic_.rotation[2][0] = -sin_pitch;
  extrinsic_.rotation[2][1] = sin_roll * cos_pitch;
  extrinsic_.rotation[2][2] = cos_roll * cos_pitch;

  // 同时设置全局外参的旋转矩阵(与局部外参相同)
  // extrinsic_global.rotation[0][0] = cos_pitch * cos_yaw;
  // extrinsic_global.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  // extrinsic_global.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  // extrinsic_global.rotation[1][0] = cos_pitch * sin_yaw;
  // extrinsic_global.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  // extrinsic_global.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  // extrinsic_global.rotation[2][0] = -sin_pitch;
  // extrinsic_global.rotation[2][1] = sin_roll * cos_pitch;
  // extrinsic_global.rotation[2][2] = cos_roll * cos_pitch;
  
  // 标记外参已设置
  is_set_extrinsic_params_ = true;
}

// 处理高精度笛卡尔坐标点云数据
// 参数pkt: 包含原始点云数据的RawPacket结构体
void LidarPubHandler::ProcessCartesianHighPoint(RawPacket & pkt)
{
  // 将原始数据转换为高精度笛卡尔坐标点结构体指针
  LivoxLidarCartesianHighRawPoint * raw = (LivoxLidarCartesianHighRawPoint *)pkt.raw_data.data();
  // 创建XYZI点结构体并初始化
  PointXyzlt point = {};
  
  // 遍历数据包中的所有点
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    // 如果启用了外参补偿
    if (pkt.extrinsic_enable) {
      // 直接使用原始坐标数据(单位从毫米转换为米)
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      // 应用外参变换(旋转+平移)
      point.x = (raw[i].x * extrinsic_.rotation[0][0] + raw[i].y * extrinsic_.rotation[0][1] +
                 raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) /
                1000.0;
      point.y = (raw[i].x * extrinsic_.rotation[1][0] + raw[i].y * extrinsic_.rotation[1][1] +
                 raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) /
                1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] + raw[i].y * extrinsic_.rotation[2][1] +
                 raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) /
                1000.0;
    }
    // 设置点云的其他属性
    point.intensity = raw[i].reflectivity;  // 反射强度
    point.line = i % pkt.line_num;          // 线号(激光线编号)
    point.tag = raw[i].tag;                 // 点标签
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;  // 时间戳
    
    // 加锁保护点云数据容器
    std::lock_guard<std::mutex> lock(mutex_);
    // 将处理后的点添加到点云容器中
    points_clouds_.push_back(point);
  }
}

// 处理低精度笛卡尔坐标点云数据(与高精度处理类似，只是单位转换不同)
// 参数pkt: 包含原始点云数据的RawPacket结构体
void LidarPubHandler::ProcessCartesianLowPoint(RawPacket & pkt)
{
  LivoxLidarCartesianLowRawPoint * raw = (LivoxLidarCartesianLowRawPoint *)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      // 单位从厘米转换为米
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      // 应用外参变换(旋转+平移)
      point.x = (raw[i].x * extrinsic_.rotation[0][0] + raw[i].y * extrinsic_.rotation[0][1] +
                 raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) /
                100.0;
      point.y = (raw[i].x * extrinsic_.rotation[1][0] + raw[i].y * extrinsic_.rotation[1][1] +
                 raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) /
                100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] + raw[i].y * extrinsic_.rotation[2][1] +
                 raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) /
                100.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

// 处理球坐标系点云数据
// 参数pkt: 包含原始点云数据的RawPacket结构体
void LidarPubHandler::ProcessSphericalPoint(RawPacket & pkt)
{
  // 将原始数据转换为球坐标点结构体指针
  LivoxLidarSpherPoint * raw = (LivoxLidarSpherPoint *)pkt.raw_data.data();
  PointXyzlt point = {};
  
  // 遍历数据包中的所有点
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    // 将球坐标转换为笛卡尔坐标
    double radius = raw[i].depth / 1000.0;  // 深度(单位从毫米转换为米)
    double theta = raw[i].theta / 100.0 / 180 * PI;  // 俯仰角(转换为弧度)
    double phi = raw[i].phi / 100.0 / 180 * PI;      // 方位角(转换为弧度)
    
    // 球坐标转笛卡尔坐标公式
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    
    // 如果启用了外参补偿
    if (pkt.extrinsic_enable) {
      // 直接使用转换后的坐标
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      // 应用外参变换(旋转+平移)
      point.x = src_x * extrinsic_.rotation[0][0] + src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] + src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] + src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    // 设置点云的其他属性
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    
    // 加锁保护点云数据容器
    std::lock_guard<std::mutex> lock(mutex_);
    // 将处理后的点添加到点云容器中
    points_clouds_.push_back(point);
  }
}
}  // namespace livox_ros