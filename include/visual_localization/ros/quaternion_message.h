// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_ROS_QUATERNION_MESSAGE_H_
#define INCLUDE_VISUAL_LOCALIZATION_ROS_QUATERNION_MESSAGE_H_

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>

inline geometry_msgs::msg::Quaternion MakeQuaternionMessage(
    const Eigen::Quaterniond& q) {
  geometry_msgs::msg::Quaternion message;
  message.w = q.w();
  message.x = q.x();
  message.y = q.y();
  message.z = q.z();
  return message;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_ROS_QUATERNION_MESSAGE_H_
