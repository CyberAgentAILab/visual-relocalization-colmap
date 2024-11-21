// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_ROS_POINT_MESSAGE_H_
#define INCLUDE_VISUAL_LOCALIZATION_ROS_POINT_MESSAGE_H_

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>

inline geometry_msgs::msg::Point MakePointMessage(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Point message;
  message.x = v.x();
  message.y = v.y();
  message.z = v.z();
  return message;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_ROS_POINT_MESSAGE_H_
