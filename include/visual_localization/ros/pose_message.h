// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_ROS_POSE_MESSAGE_H_
#define INCLUDE_VISUAL_LOCALIZATION_ROS_POSE_MESSAGE_H_

#include <geometry_msgs/msg/pose.hpp>

#include "visual_localization/ros/point_message.h"
#include "visual_localization/ros/quaternion_message.h"

inline geometry_msgs::msg::Pose MakePoseMessage(const Eigen::Isometry3d& pose) {
  const Eigen::Quaterniond q(pose.rotation());

  geometry_msgs::msg::Pose message;
  message.orientation = MakeQuaternionMessage(q);
  message.position = MakePointMessage(pose.translation());
  return message;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_ROS_POSE_MESSAGE_H_
