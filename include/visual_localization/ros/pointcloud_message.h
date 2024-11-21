// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_ROS_POINTCLOUD_MESSAGE_H_
#define INCLUDE_VISUAL_LOCALIZATION_ROS_POINTCLOUD_MESSAGE_H_

#include <sensor_msgs/msg/point_cloud.hpp>
#include <vector>

#include "visual_localization/ros/point32_message.h"

inline sensor_msgs::msg::PointCloud MakePointCloudMessage(
    const std::vector<Eigen::Vector3d>& landmarks,
    const std_msgs::msg::Header& header) {
  sensor_msgs::msg::PointCloud message;
  message.header = header;
  for (const Eigen::Vector3d& p : landmarks) {
    message.points.push_back(MakePoint32Message(p));
  }
  return message;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_ROS_POINTCLOUD_MESSAGE_H_
