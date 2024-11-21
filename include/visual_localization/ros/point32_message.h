// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_ROS_POINT32_MESSAGE_H_
#define INCLUDE_VISUAL_LOCALIZATION_ROS_POINT32_MESSAGE_H_

#include <Eigen/Core>
#include <geometry_msgs/msg/point32.hpp>

inline geometry_msgs::msg::Point32 MakePoint32Message(
    const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point32 message;
  message.x = point.x();
  message.y = point.y();
  message.z = point.z();
  return message;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_ROS_POINT32_MESSAGE_H_
