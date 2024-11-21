// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/pnp.h"

#include <gtest/gtest.h>

#include <ranges>

#include "visual_localization/eigen_utils.h"
#include "visual_localization/projection.h"

inline std::vector<Eigen::Vector2d> LocalProjection(
    const Eigen::Isometry3d& transform_cw,
    const std::vector<Eigen::Vector3d>& ps) {
  std::vector<Eigen::Vector2d> xs(ps.size());
  for (size_t i = 0; i < ps.size(); i++) {
    const std::optional<Eigen::Vector2d> maybe_x =
        Projection(transform_cw, ps[i]);
    assert(maybe_x.has_value());
    xs[i] = maybe_x.value();
  }
  return xs;
}

TEST(PnP, SmokeTest) {
  const double pi = std::numbers::pi;
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  const Eigen::Isometry3d transform_wc = eigen_utils::MakeIsometry3d(
      Eigen::AngleAxisd(-0.5 * pi, y_axis), Eigen::Vector3d(3., 0., 2.));

  const Eigen::Isometry3d transform_cw = transform_wc.inverse();

  const std::vector<Eigen::Vector3d> ps = {
      Eigen::Vector3d(1., 2., 3.),
      Eigen::Vector3d(1., 4., 0.),
      Eigen::Vector3d(0., 4., 2.),
      Eigen::Vector3d(-1., -1., 1.),
  };

  const std::vector<Eigen::Vector2d> xs = LocalProjection(transform_cw, ps);
  const std::optional<Eigen::Isometry3d> transform_cw_pred = SolvePnp(ps, xs);
  ASSERT_TRUE(transform_cw_pred.has_value());

  const Eigen::Matrix3d pred_r = transform_cw_pred.value().rotation();
  const Eigen::Vector3d pred_t = transform_cw_pred.value().translation();

  EXPECT_TRUE(pred_r.isApprox(transform_cw.rotation(), 1e-4));
  EXPECT_TRUE(pred_t.isApprox(transform_cw.translation(), 1e-4));
}

TEST(PnP, InsufficientInput) {
  const double pi = std::numbers::pi;
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  const Eigen::Isometry3d transform_wc = eigen_utils::MakeIsometry3d(
      Eigen::AngleAxisd(-0.5 * pi, y_axis), Eigen::Vector3d(3., 0., 2.));

  const Eigen::Isometry3d transform_cw = transform_wc.inverse();

  const std::vector<Eigen::Vector3d> ps = {
      Eigen::Vector3d(1., 2., 3.),
      Eigen::Vector3d(1., 4., 0.),
      Eigen::Vector3d(0., 4., 2.),
  };

  const std::vector<Eigen::Vector2d> xs = LocalProjection(transform_cw, ps);
  const std::optional<Eigen::Isometry3d> transform_cw_pred = SolvePnp(ps, xs);
  ASSERT_FALSE(transform_cw_pred.has_value());
}

TEST(PnP, NoisyInput) {
  const double pi = std::numbers::pi;
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  const Eigen::Isometry3d transform_wc = eigen_utils::MakeIsometry3d(
      Eigen::AngleAxisd(-0.5 * pi, y_axis), Eigen::Vector3d(3., 0., 2.));

  const Eigen::Isometry3d transform_cw = transform_wc.inverse();

  const std::vector<Eigen::Vector3d> ps = {
      Eigen::Vector3d(1., 2., 3.),  Eigen::Vector3d(1., 4., 0.),
      Eigen::Vector3d(0., 4., 2.),  Eigen::Vector3d(-1., 2., 4.),
      Eigen::Vector3d(-3., 5., 1.), Eigen::Vector3d(-2., 1., 6.),
      Eigen::Vector3d(-8., 2., 3.), Eigen::Vector3d(1., 2., 5.),
  };

  std::vector<Eigen::Vector2d> xs = LocalProjection(transform_cw, ps);
  xs[3] += Eigen::Vector2d(0.3, 0.8);

  const std::optional<Eigen::Isometry3d> transform_cw_pred = SolvePnp(ps, xs);
  ASSERT_TRUE(transform_cw_pred.has_value());

  const Eigen::Matrix3d pred_r = transform_cw_pred.value().rotation();
  const Eigen::Vector3d pred_t = transform_cw_pred.value().translation();

  EXPECT_TRUE(pred_r.isApprox(transform_cw.rotation(), 1e-4));
  EXPECT_TRUE(pred_t.isApprox(transform_cw.translation(), 1e-4));
}
