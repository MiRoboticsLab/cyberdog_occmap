// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef OCC_GRIDMAP__CREATE_GRIDMAP_HPP_
#define OCC_GRIDMAP__CREATE_GRIDMAP_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <vector>
#include <string>
#include <memory>

#include "cyberdog_visions_interfaces/msg/range_data.hpp"
#include "cyberdog_visions_interfaces/srv/finish_map.hpp"

#include "occ_gridmap/grid_mapper.hpp"
#include "occ_gridmap/common.hpp"

typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan> mySyncPolicy;

struct TopicParam
{
  std::string frame_id;
  std::string create_service;
  std::string finish_service;
  std::string start_nav_service;
  std::string stop_nav_service;
  std::string get_map_path;
  std::string odom_topic;
  std::string laser_topic;
  std::string range_data_topic;
  std::string visualization_topic;
};

class CreateGridMap : public rclcpp::Node
{
public:
  CreateGridMap();
  ~CreateGridMap();
  /**
   * @brief 解析yaml文件参数
   *
   * @param config_file 输入为yaml文件路径
   */
  void ParseParameters(std::string & config_file);

  /**
   * @brief 基于订阅的里程计位姿和单线激光雷达扫描数据
   *
   * @param odom 里程计数据
   * @param scan 单线激光雷达数据
   */
  void SubCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  /**
   * @brief 开始建图服务的CallBack，接受到开始建图服务后调用此函数对成员变量进行初始化
   *
   * @param request 开始建图传入参数
   * @param response 是否成功开始建图，成功则response为true，否则为fasle
   */
  void CreateServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
  );

  /**
  * @brief 结束建图服务的CallBack，接受到结束建图服务后调用此函数对成员变量进行回收，并保存栅格地图
  *
  * @param request 结束建图传入参数
  * @param response 是否成功结束建图，成功则response为true，否则为fasle
  */
  void FinishServiceCallback(
    const std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Request> request,
    std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Response> response
  );

  /**
  * @brief 发布子图点云数据，由publish_gridmap_node实时显示建图过程
  *
  */
  void PublishVisualizationCallback();

  /**
  * @brief 与其他cyberdog程序进行交互，获取栅格地图的保存路径
  *
  * @param future 地图保存路径
  */
  void GetMapPathCallback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  /**
  * @brief 将激光雷达的单帧扫描转化为点云数据
  *
  * @param scan 激光雷达单帧扫描数据
  *
  * @return 返回转换后的点云数据
  */
  sensor::PointCloudCarto ScanToPointCloud(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  /**
  * @brief 从标定文件读取外参
  *
  * @param calibr_file 标定文件路径
  */
  void LoadExtrinsicsFromYaml(const std::string & calibr_file);

  /**
  * @brief 将vector类型数组转换为Eigen存储
  *
  * @param data vector类型数组
  *
  * @return Eigen数组
  */
  Eigen::MatrixXd ConvertToEigenMatrix(std::vector<std::vector<double>> & data);

  /**
  * @brief 在重新建图之前，移除原先所建地图
  */
  void RemoveMap();

private:
  // 订阅里程计位置姿态
  message_filters::Subscriber<nav_msgs::msg::Odometry>
  odom_subscription_;

  // 订阅激光雷达扫描数据
  message_filters::Subscriber<sensor_msgs::msg::LaserScan>
  laser_subscription_;

  // 里程计与激光雷达进行时间戳近似同步
  std::shared_ptr<message_filters::Synchronizer<mySyncPolicy>>
  approximate_sync_;

  // 发布是否进行可视化
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    visualization_publisher_;

  // 每隔1s发布一次可视化
  rclcpp::TimerBase::SharedPtr pub_visualization_timer_;

  // 发布子图点云数据
  rclcpp::Publisher<cyberdog_visions_interfaces::msg::RangeData>::SharedPtr
    range_data_publisher_;

  // 开始建图服务
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    create_srv_;

  // 结束建图服务
  rclcpp::Service<cyberdog_visions_interfaces::srv::FinishMap>::SharedPtr
    finish_srv_;

  // 获取地图保存路径
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_map_path_client_;

private:
  bool create_map_flag_ = false;
  bool visualization_flag_ = false;
  GridMapStatus grid_map_status_ = GridMapStatus::HANG_MODEL;

private:
  TopicParam topic_param_;
  SubMapParam submap_param_;
  ProbabilityParam probability_param_;
  FilesSaveParam files_save_param_;
  FilterParam filter_param_;
  CeresScanMatchingParam ceres_param_;
  transform::Eigentf eigentf_;
  std::shared_ptr<mapping::GridMapper> g_mapper_;
  transform::Rigid3d laser2imu_;
  transform::Rigid3d laser2odom_;
  std::string config_file_;
  std::string calibr_file_;
  MapBeautiParam mapbeauti_param_;
  std::string map_mode_;
};  // class CreateGridMap

#endif  // OCC_GRIDMAP__CREATE_GRIDMAP_HPP_
