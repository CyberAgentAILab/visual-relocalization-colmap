// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_ROS_IMAGE_MESSAGE_H_
#define INCLUDE_VISUAL_LOCALIZATION_ROS_IMAGE_MESSAGE_H_

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

inline sensor_msgs::msg::Image::SharedPtr MakeImageMessage(
    const cv::Mat& image,
    const std_msgs::msg::Header& header = std_msgs::msg::Header{}) {
  return cv_bridge::CvImage(header, "rgb8", image).toImageMsg();
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_ROS_IMAGE_MESSAGE_H_
