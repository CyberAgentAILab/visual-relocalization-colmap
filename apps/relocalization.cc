// Copyright [2024] <CyberAgent AI Lab>

#include <fmt/core.h>
#include <pcl/io/auto_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
// Don't include opencv2/opencv.hpp or it will not compile
// #include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <unordered_set>
#include <utility>

#include "colmap/estimators/absolute_pose.h"
#include "colmap/estimators/pose.h"
#include "colmap/feature/sift.h"
#include "colmap/feature/types.h"
#include "colmap/geometry/rigid3.h"
#include "colmap/optim/loransac.h"
#include "colmap/retrieval/visual_index.h"
#include "colmap/scene/correspondence_graph.h"
#include "colmap/scene/database.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/sensor/bitmap.h"
#include "colmap/sfm/incremental_mapper.h"
#include "visual_localization/camera_parameter.h"
#include "visual_localization/colmap/bitmap.h"
#include "visual_localization/colmap/cv_colmap.h"
#include "visual_localization/colmap/feature.h"
#include "visual_localization/pnp.h"
#include "visual_localization/ros/header_message.h"
#include "visual_localization/ros/image_message.h"
#include "visual_localization/ros/pointcloud_message.h"
#include "visual_localization/ros/pose_message.h"
#include "visual_localization/ros/pose_stamped_message.h"
#include "visual_localization/triangulation.h"

enum PoseEstimationStatus {
  kSucecss,
  kEstimationFailed,
  kRetrievalFailed,
  kInsufficientInliers,
  kRefinementFailed,
};

struct PoseEstimationResult {
  PoseEstimationStatus status;
  colmap::image_t image_id;
  colmap::Rigid3d pose;
  colmap::FeatureMatches matches;
};

std::string ToString(const PoseEstimationStatus& status) {
  if (status == kSucecss) {
    return "kSucecss";
  }
  if (status == kEstimationFailed) {
    return "kEstimationFailed";
  }
  if (status == kRetrievalFailed) {
    return "kRetrievalFailed";
  }
  if (status == kInsufficientInliers) {
    return "kInsufficientInliers";
  }
  if (status == kRefinementFailed) {
    return "kRefinementFailed";
  }
  throw std::runtime_error("No such status");
}

std::shared_ptr<colmap::retrieval::VisualIndex<>> MakeVisualIndex(
    const rclcpp::Logger& logger, const std::string& vocab_tree_path,
    const colmap::Database& database) {
  auto visual_index = std::make_shared<colmap::retrieval::VisualIndex<>>();
  visual_index->Read(vocab_tree_path);

  const colmap::retrieval::VisualIndex<>::IndexOptions option;
  const auto all_images = database.ReadAllImages();
  for (size_t i = 0; i < all_images.size(); i++) {
    if (i % 50 != 0) {
      continue;
    }
    RCLCPP_INFO(logger, "Indexing image %6lu / %6lu", i, all_images.size());

    const colmap::Image& image = all_images[i];
    if (visual_index->ImageIndexed(image.ImageId())) {
      continue;
    }

    const auto keypoints = database.ReadKeypoints(image.ImageId());
    const auto descriptors = database.ReadDescriptors(image.ImageId());
    visual_index->Add(option, image.ImageId(), keypoints, descriptors);
  }
  visual_index->Prepare();
  return visual_index;
}

void AddCorrespondences(
    const Eigen::Vector2d& keypoint_xy,
    const std::shared_ptr<const colmap::Reconstruction>& reconstruction,
    const colmap::CorrespondenceGraph::CorrespondenceRange& corr_range,
    std::vector<Eigen::Vector2d>* tri_points2D,
    std::vector<Eigen::Vector3d>* tri_points3D) {
  std::unordered_set<colmap::point3D_t> corr_point3D_ids;
  for (const auto* corr = corr_range.beg; corr < corr_range.end; ++corr) {
    const colmap::Image& corr_image = reconstruction->Image(corr->image_id);
    if (!corr_image.IsRegistered()) {
      continue;
    }

    const colmap::Point2D& corr_point2D = corr_image.Point2D(corr->point2D_idx);
    if (!corr_point2D.HasPoint3D()) {
      continue;
    }

    // Avoid duplicate correspondences.
    if (corr_point3D_ids.contains(corr_point2D.point3D_id)) {
      continue;
    }

    const colmap::Point3D& point3D =
        reconstruction->Point3D(corr_point2D.point3D_id);

    corr_point3D_ids.insert(corr_point2D.point3D_id);
    tri_points2D->push_back(keypoint_xy);
    tri_points3D->push_back(point3D.xyz);
  }
}

colmap::Rigid3d MakeRigid3d(const Eigen::Matrix3x4d& matrix) {
  const Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
  const Eigen::Vector3d t = matrix.block<3, 1>(0, 3);
  return colmap::Rigid3d(q, t);
}

using AbsolutePoseRANSAC =
    colmap::LORANSAC<colmap::P3PEstimator, colmap::EPNPEstimator>;
bool EstimateAbsolutePoseKernel(const colmap::Camera& camera,
                                const std::vector<Eigen::Vector2d>& points2D,
                                const std::vector<Eigen::Vector3d>& points3D,
                                const colmap::RANSACOptions& options,
                                AbsolutePoseRANSAC::Report* report) {
  // Normalize image coordinates with current camera hypothesis.
  std::vector<Eigen::Vector2d> points2D_in_cam(points2D.size());
  for (size_t i = 0; i < points2D.size(); ++i) {
    points2D_in_cam[i] = camera.CamFromImg(points2D[i]);
  }

  // Estimate pose for given focal length.
  auto custom_options = options;
  custom_options.max_error = camera.CamFromImgThreshold(options.max_error);
  AbsolutePoseRANSAC ransac(custom_options);
  *report = ransac.Estimate(points2D_in_cam, points3D);
  return report->support.num_inliers > 0;
}

colmap::Camera ToColmapCamera(const CameraParameter& camera_parameter) {
  const double w = camera_parameter.cx * 2.;
  const double h = camera_parameter.cy * 2.;
  const double f = 0.5 * (camera_parameter.fx + camera_parameter.fy);
  return colmap::Camera::CreateFromModelId(
      colmap::kInvalidCameraId, colmap::CameraModelId::kPinhole, f, w, h);
}

PoseEstimationResult EstimateCamFromWorld(
    const std::shared_ptr<CameraParameter>& camera_parameter,
    const colmap::IncrementalMapper::Options& options,
    const std::shared_ptr<const colmap::Reconstruction>& reconstruction,
    const std::shared_ptr<const colmap::CorrespondenceGraph>& graph,
    const colmap::image_t image_id,
    const colmap::FeatureKeypoints& query_keypoints,
    const colmap::FeatureMatches& matches) {
  assert(reconstruction->NumRegImages() >= 2);
  assert(options.Check());

  std::vector<Eigen::Vector2d> tri_points2D;
  std::vector<Eigen::Vector3d> tri_points3D;
  for (const colmap::FeatureMatch& match : matches) {
    const colmap::point2D_t query_index = match.point2D_idx1;
    const colmap::point2D_t retrieved_index = match.point2D_idx2;
    const auto range = graph->FindCorrespondences(image_id, retrieved_index);
    const colmap::FeatureKeypoint& p = query_keypoints[query_index];
    const Eigen::Vector2d point2d(p.x, p.y);
    AddCorrespondences(point2d, reconstruction, range, &tri_points2D,
                       &tri_points3D);
  }

  //////////////////////////////////////////////////////////////////////////////
  // 2D-3D estimation
  //////////////////////////////////////////////////////////////////////////////

  colmap::RANSACOptions ransac_options;
  ransac_options.max_error = options.abs_pose_max_error;
  ransac_options.min_inlier_ratio = options.abs_pose_min_inlier_ratio;
  // Use high confidence to avoid preemptive termination of P3P RANSAC
  // - too early termination may lead to bad registration.
  ransac_options.min_num_trials = 100;
  ransac_options.max_num_trials = 10000;
  ransac_options.confidence = 0.99999;

  const colmap::Camera camera = ToColmapCamera(*camera_parameter);

  AbsolutePoseRANSAC::Report report;
  if (!EstimateAbsolutePoseKernel(camera, tri_points2D, tri_points3D,
                                  ransac_options, &report)) {
    return PoseEstimationResult{PoseEstimationStatus::kEstimationFailed,
                                image_id, colmap::Rigid3d(), matches};
  }
  const colmap::Rigid3d cam_from_world = MakeRigid3d(report.model);

  if (report.support.num_inliers <
      static_cast<size_t>(options.abs_pose_min_num_inliers)) {
    return PoseEstimationResult{PoseEstimationStatus::kInsufficientInliers,
                                image_id, colmap::Rigid3d(), matches};
  }

  const colmap::Rigid3d pose = Inverse(cam_from_world);
  return PoseEstimationResult{PoseEstimationStatus::kSucecss, image_id, pose,
                              matches};
}

cv::Mat Background(const bool success, const cv::Size& size,
                   const size_t padding) {
  const cv::Size s = size + cv::Size(padding * 2, padding * 2);
  return cv::Mat(s, CV_8UC3,
                 success ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0));
}

std::shared_ptr<const colmap::CorrespondenceGraph> GetCorrespondenceGraph(
    const colmap::Database& database) {
  const size_t min_num_matches = 15;
  const bool ignore_watermarks = false;
  const auto cache = colmap::DatabaseCache::Create(database, min_num_matches,
                                                   ignore_watermarks, {});
  return cache->CorrespondenceGraph();
}

colmap::IncrementalMapper::Options InitPoseEstimationOptions() {
  colmap::IncrementalMapper::Options option;
  option.abs_pose_min_num_inliers = 20;
  return option;
}

class PoseEstimator {
 public:
  PoseEstimator(
      const std::shared_ptr<CameraParameter>& camera_parameter,
      const std::shared_ptr<colmap::Database>& database,
      const std::shared_ptr<colmap::retrieval::VisualIndex<>>& visual_index,
      const std::shared_ptr<colmap::Reconstruction>& reconstruction)
      : camera_parameter_(camera_parameter),
        pose_estimation_options_(InitPoseEstimationOptions()),
        database_(database),
        visual_index_(visual_index),
        reconstruction_(reconstruction),
        graph_(GetCorrespondenceGraph(*database)),
        matcher_(std::make_unique<FeatureMatcher>()) {}

  PoseEstimationResult Estimate(
      const std::shared_ptr<colmap::FeatureKeypoints>& query_keypoints,
      const std::shared_ptr<colmap::FeatureDescriptors>& query_descriptors)
      const {
    std::vector<colmap::retrieval::ImageScore> image_scores;
    visual_index_->Query(query_options_, *query_keypoints, *query_descriptors,
                         &image_scores);
    if (image_scores.empty()) {
      return PoseEstimationResult{PoseEstimationStatus::kRetrievalFailed, 0U,
                                  colmap::Rigid3d(), colmap::FeatureMatches{}};
    }

    const auto& image_score = image_scores[0];
    const colmap::image_t image_id = image_score.image_id;

    const auto descriptors = database_->ReadDescriptors(image_id);
    const auto descriptors_ptr =
        std::make_shared<colmap::FeatureDescriptors>(descriptors);
    const auto matches = matcher_->Match(query_descriptors, descriptors_ptr);
    return EstimateCamFromWorld(camera_parameter_, pose_estimation_options_,
                                reconstruction_, graph_, image_id,
                                *query_keypoints, matches);
  }

 private:
  const std::shared_ptr<CameraParameter> camera_parameter_;
  const colmap::IncrementalMapper::Options pose_estimation_options_;
  const std::shared_ptr<colmap::Database> database_;
  const colmap::retrieval::VisualIndex<>::QueryOptions query_options_;
  const std::shared_ptr<const colmap::retrieval::VisualIndex<>> visual_index_;
  const std::shared_ptr<const colmap::Reconstruction> reconstruction_;
  const std::shared_ptr<const colmap::CorrespondenceGraph> graph_;
  const std::unique_ptr<FeatureMatcher> matcher_;
};

geometry_msgs::msg::Pose MakePoseMessage(const Eigen::Quaterniond& rotation,
                                         const Eigen::Vector3d& translation) {
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = rotation.w();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  return pose;
}

cv::Mat AddSuccessFlagFrame(const bool success, const cv::Mat& image) {
  const size_t padding = 20;
  cv::Mat frame = Background(success, image.size(), padding);
  image.copyTo(frame(cv::Rect(padding, padding, image.cols, image.rows)));
  return frame;
}

namespace fs = std::filesystem;

using OptionalVector3d = std::optional<Eigen::Vector3d>;

std::vector<fs::path> SortedPaths(const fs::path& dirpath) {
  std::set<fs::path> paths;  // Use set to sort paths
  for (const fs::directory_entry& entry : fs::directory_iterator(dirpath)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    paths.insert(entry.path());
  }
  return std::vector<fs::path>(paths.begin(), paths.end());
}

std::string IndexToFilename(const int index) {
  return fmt::format("{:06d}.png", index);
}

void ThrowIfNotExist(const std::string& path) {
  if (!fs::exists(path)) {
    throw std::runtime_error(fmt::format("File {} does not exist.", path));
  }
}

Eigen::Isometry3d ToWorldPose(const Eigen::Isometry3d& pose) {
  const Eigen::AngleAxisd dx(0.5 * std::numbers::pi, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd dz(0.5 * std::numbers::pi, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond r(pose.rotation() * dx * dz);

  return eigen_utils::MakeIsometry3d(r, pose.translation());
}

std::optional<sensor_msgs::msg::PointCloud2> ReadPointCloudAsMsg(
    const std::string& ply_path, const std::string& base_frame) {
  pcl::PCLPointCloud2 pointcloud2;
  if (pcl::io::loadPLYFile(ply_path, pointcloud2) == -1) {
    return std::nullopt;
  }
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl_conversions::fromPCL(pointcloud2, pointcloud_msg);
  pointcloud_msg.header.frame_id = base_frame;
  return std::make_optional(pointcloud_msg);
}

std::shared_ptr<colmap::Reconstruction> ReadReconstruction(
    const std::string& reconstruction_dir) {
  auto reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(reconstruction_dir);
  return reconstruction;
}

std::tuple<cv::Mat, cv::Mat> ReadIntrinsics(const std::string& intrinsic_path) {
  cv::FileStorage fs(intrinsic_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error(
        fmt::format("Failed to read file '{}'", intrinsic_path));
  }

  cv::Mat intrinsic;
  fs["intrinsic"] >> intrinsic;
  cv::Mat distortion;
  fs["distortion"] >> distortion;
  return {intrinsic, distortion};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("relocalization");

  const auto map_cloud_publisher =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/pointcloud",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  const auto world_pose_publisher =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("/world_pose",
                                                              10);
  const auto query_image_publisher =
      node->create_publisher<sensor_msgs::msg::Image>("/query_image", 10);
  const auto image_publisher =
      node->create_publisher<sensor_msgs::msg::Image>("/image", 10);

  const std::string vocab_tree_path =
      node->declare_parameter("vocab_tree_path", "");
  ThrowIfNotExist(vocab_tree_path);

  const std::string database_path =
      node->declare_parameter("database_path", "");
  ThrowIfNotExist(database_path);

  const std::string reconstruction_dir =
      node->declare_parameter("reconstruction_dir", "");
  ThrowIfNotExist(reconstruction_dir);

  const std::string intrinsic_path =
      node->declare_parameter<std::string>("intrinsic_path", "");

  const auto [intrinsic, distortion] = ReadIntrinsics(intrinsic_path);
  const auto camera_parameter = std::make_shared<CameraParameter>(intrinsic);

  const fs::path input_image_dirname =
      node->declare_parameter("input_image_dirname", "images");
  ThrowIfNotExist(input_image_dirname);

  const std::string ply_path = node->declare_parameter("ply_path", "");
  ThrowIfNotExist(ply_path);

  RCLCPP_INFO(node->get_logger(), "current_directory = %s",
              fs::current_path().string().c_str());
  RCLCPP_INFO(node->get_logger(), "vocab_tree_path = %s",
              vocab_tree_path.c_str());
  RCLCPP_INFO(node->get_logger(), "reconstruction_dir = %s",
              reconstruction_dir.c_str());
  RCLCPP_INFO(node->get_logger(), "input_image_dirname = %s",
              input_image_dirname.string().c_str());
  RCLCPP_INFO(node->get_logger(), "ply_path = %s", ply_path.c_str());
  RCLCPP_INFO(node->get_logger(), "intrinsic = ");
  RCLCPP_INFO(node->get_logger(), "%lf %lf %lf", intrinsic.at<double>(0, 0),
              intrinsic.at<double>(0, 1), intrinsic.at<double>(0, 2));
  RCLCPP_INFO(node->get_logger(), "%lf %lf %lf", intrinsic.at<double>(1, 0),
              intrinsic.at<double>(1, 1), intrinsic.at<double>(1, 2));
  RCLCPP_INFO(node->get_logger(), "%lf %lf %lf", intrinsic.at<double>(2, 0),
              intrinsic.at<double>(2, 1), intrinsic.at<double>(2, 2));

  auto read_image = [&](const fs::path& path) {
    RCLCPP_INFO(node->get_logger(), "path = %s", path.string().c_str());
    cv::Mat image = cv::imread(path);
    assert(!image.empty());
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    return image;
  };

  const auto reconstruction = ReadReconstruction(reconstruction_dir);

  const auto pointcloud_msg = ReadPointCloudAsMsg(ply_path, "map");
  if (pointcloud_msg.has_value()) {
    map_cloud_publisher->publish(pointcloud_msg.value());
  }

  const auto database = std::make_shared<colmap::Database>(database_path);
  RCLCPP_INFO(node->get_logger(), "Reading vocab tree");
  const auto visual_index =
      MakeVisualIndex(node->get_logger(), vocab_tree_path, *database);
  const PoseEstimator pose_estimator(camera_parameter, database, visual_index,
                                     reconstruction);
  const std_msgs::msg::Header header =
      MakeHeaderMessage("map", builtin_interfaces::msg::Time());

  FeatureExtractor extractor;
  const auto paths = SortedPaths(input_image_dirname);
  for (size_t i = 0; i < paths.size(); i += 10) {
    const fs::path& image_path = paths.at(i);

    const cv::Mat image = read_image(image_path);
    image_publisher->publish(*MakeImageMessage(image));

    const colmap::Bitmap query_bitmap = ReadBitmap(image_path);
    const auto [keypoints, descriptors] = extractor.Extract(query_bitmap);
    const auto result = pose_estimator.Estimate(keypoints, descriptors);
    const Eigen::Isometry3d pose = eigen_utils::MakeIsometry3d(
        result.pose.rotation, result.pose.translation);
    const Eigen::Isometry3d pose_world = ToWorldPose(pose);

    RCLCPP_INFO(node->get_logger(), "Relocalization status = %s",
                ToString(result.status).c_str());

    const bool success = result.status == PoseEstimationStatus::kSucecss;
    const cv::Mat framed = AddSuccessFlagFrame(success, image);
    query_image_publisher->publish(*MakeImageMessage(framed));

    if (success) {
      world_pose_publisher->publish(MakePoseStampedMessage(pose_world, header));
    }
  }

  rclcpp::shutdown();

  return 0;
}
