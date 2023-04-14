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

#ifndef OCC_GRIDMAP__PUBLISH_GRIDMAP_HPP_
#define OCC_GRIDMAP__PUBLISH_GRIDMAP_HPP_

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>
#include <memory>

#include "cyberdog_visions_interfaces/msg/range_data.hpp"

#include "map2d/grid_2d.hpp"
#include "map2d/grid_inserter_2d.hpp"
#include "map2d/probability_value.hpp"
#include "map2d/value_conversion_tables.hpp"
#include "occ_gridmap/map_beauti.hpp"

class PublishGridMap : public rclcpp::Node
{
public:
  explicit PublishGridMap(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~PublishGridMap() = default;

  /**
   * @brief 解析yaml文件参数
   *
   * @param config_file 输入为yaml文件路径
   */
  void ParseParameters(std::string & config_file);

  /**
   * @brief 订阅create_gridmap_node发布的子图点云数据，插入到栅格地图中
   *
   * @param range_data_msg PointCloud2类型的点云数据
   */
  void RangeDataCallback(
    const cyberdog_visions_interfaces::msg::RangeData::ConstSharedPtr range_data_msg);

  /**
   * @brief 与其他cyberdog程序进行交互，判断是否进行可视化操作
   *
   * @param visualization_flag_msg 交互参数，为true则进行可视化，否则不对建图过程进行可视化
   */
  void VisualizationCallback(
    const std_msgs::msg::Bool::ConstSharedPtr visualization_flag_msg);

  /**
   * @brief 定时函数，每隔1s发布一次当前的栅格地图到/map话题
   */
  void PublishOccupancyGridCallback();

  /**
   * @brief 根据初始位置创建一个栅格地图
   *
   * @param origin 机器人开始建图时在world系下的位置
   */
  std::unique_ptr<mapping::Grid2D> CreateGrid(const Eigen::Vector2d & origin);

  /**
   * @brief 初始化相关成员变量
   */
  void PrepareCommunication();

  /**
   * @brief 释放相关成员变量
   */
  void ResetCommunication();

private:
  // 订阅create_gridmap发布的子图话题
  rclcpp::Subscription<cyberdog_visions_interfaces::msg::RangeData>::SharedPtr
    range_data_subscriber_;

  // 订阅是否可视化话题
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visualization_subscriber_;

  // 发布栅格地图
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occgrid_publisher_;

  // 定时器
  rclcpp::TimerBase::SharedPtr pub_map_timer_;

  // 是否是第一帧位姿
  bool first_pose_flag_ = true;

  // 是否进行可视化
  bool visualization_flag_ = true;

  // 配置文件路径
  std::string config_file_;

  // range_data_subscriber_订阅的子图话题
  std::string range_data_topic_;

  // 实时发布的栅格地图话题
  std::string gridmap_topic_;

  // 订阅是否进行可视化话题
  std::string visualization_topic_;

  // 地图分辨率，默认为0.05
  double submap_resolution_;

  // 子图的初始尺寸
  int submap_initsize_;

  // 地图美化的成员变量
  MapBeautiParam mapbeauti_param_;

  // 栅格地图成员变量
  std::unique_ptr<mapping::Grid2D> grid_2d_ = nullptr;

  // 将子图插入地图
  std::unique_ptr<mapping::GridInserter2D> grid_inserter_2d_ = nullptr;

  // 消息头
  std_msgs::msg::Header header_;

  // 查找表
  mapping::ValueConversionTables value_conversion_tables_;

  // 地图美化成员变量
  std::shared_ptr<MapBeauti> map_beauti_;
};  // class PublishGridMap

#endif  // OCC_GRIDMAP__PUBLISH_GRIDMAP_HPP_
