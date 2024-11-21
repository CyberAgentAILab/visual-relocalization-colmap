// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_ROS_HEADER_MESSAGE_H_
#define INCLUDE_VISUAL_LOCALIZATION_ROS_HEADER_MESSAGE_H_

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

inline std_msgs::msg::Header MakeHeaderMessage(
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp) {
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = stamp;
  return header;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_ROS_HEADER_MESSAGE_H_
