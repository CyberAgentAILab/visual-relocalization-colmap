// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_ROS_POSE_STAMPED_MESSAGE_H_
#define INCLUDE_VISUAL_LOCALIZATION_ROS_POSE_STAMPED_MESSAGE_H_

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "visual_localization/ros/pose_message.h"

geometry_msgs::msg::PoseStamped MakePoseStampedMessage(
    const Eigen::Isometry3d& pose, const std_msgs::msg::Header& header) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.pose = MakePoseMessage(pose);
  return pose_stamped;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_ROS_POSE_STAMPED_MESSAGE_H_
