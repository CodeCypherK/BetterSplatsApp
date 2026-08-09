#include <gtest/gtest.h>

#include "calib/lut_fit.h"

namespace bs {
namespace {

const PinholeIntrinsics kIntrinsics{1456.0, 1456.0, 959.5, 719.5, 1920, 1440};

double MaxRadiusPx() {
  const double dx = std::max(kIntrinsics.cx, 1920 - kIntrinsics.cx);
  const double dy = std::max(kIntrinsics.cy, 1440 - kIntrinsics.cy);
  return std::hypot(dx, dy);
}

// Builds Apple-style tables from a known OPENCV model.
DistortionLut MakeLutFromModel(double k1, double k2, int entries = 42) {
  DistortionLut lut;
  lut.center_x = kIntrinsics.cx;
  lut.center_y = kIntrinsics.cy;
  const double r_max = MaxRadiusPx();
  const double f = kIntrinsics.fx;

  // Inverse table: index by undistorted radius fraction; entry = r_d / r_u.
  for (int i = 0; i < entries; ++i) {
    const double rf = static_cast<double>(i) / (entries - 1);
    const double ru = rf * r_max / f;
    const double scale = 1.0 + k1 * ru * ru + k2 * ru * ru * ru * ru;
    lut.inverse.push_back(static_cast<float>(scale));
  }
  // Forward table: index by distorted radius fraction; entry = r_u / r_d.
  // Invert numerically.
  for (int i = 0; i < entries; ++i) {
    const double rf = static_cast<double>(i) / (entries - 1);
    const double rd = rf * r_max / f;
    double ru = rd;
    for (int it = 0; it < 25; ++it) {
      const double scale = 1.0 + k1 * ru * ru + k2 * ru * ru * ru * ru;
      ru = rd / scale;
    }
    lut.magnification.push_back(
        static_cast<float>(rd > 1e-12 ? ru / rd : 1.0));
  }
  return lut;
}

TEST(LutFit, RecoversKnownModelFromInverseTable) {
  const double k1 = 0.06, k2 = -0.11;
  DistortionLut lut = MakeLutFromModel(k1, k2);
  lut.magnification.clear();  // force the inverse-table path

  const auto fit = FitOpencvModelFromLut(lut, kIntrinsics);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->k1, k1, 2e-3);
  EXPECT_NEAR(fit->k2, k2, 5e-3);
  EXPECT_LT(fit->max_residual_px, 0.25);
}

TEST(LutFit, RecoversKnownModelFromForwardTable) {
  const double k1 = 0.045, k2 = -0.08;
  DistortionLut lut = MakeLutFromModel(k1, k2);
  lut.inverse.clear();  // force the forward-table path

  const auto fit = FitOpencvModelFromLut(lut, kIntrinsics);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->k1, k1, 4e-3);
  EXPECT_NEAR(fit->k2, k2, 1e-2);
  EXPECT_LT(fit->max_residual_px, 0.35);
}

TEST(LutFit, EmptyLutIsExactPinhole) {
  const auto fit = FitOpencvModelFromLut(DistortionLut{}, kIntrinsics);
  ASSERT_TRUE(fit.has_value());
  EXPECT_DOUBLE_EQ(fit->k1, 0.0);
  EXPECT_DOUBLE_EQ(fit->k2, 0.0);
  EXPECT_DOUBLE_EQ(fit->max_residual_px, 0.0);
}

TEST(LutFit, RejectsDegenerateIntrinsics) {
  const DistortionLut lut = MakeLutFromModel(0.05, -0.1);
  PinholeIntrinsics bad = kIntrinsics;
  bad.fx = 0;
  EXPECT_FALSE(FitOpencvModelFromLut(lut, bad).has_value());
}

TEST(LutFit, UndistortInvertsDistort) {
  const double k1 = 0.06, k2 = -0.11;
  for (const auto& px : {Eigen::Vector2d(100, 100), Eigen::Vector2d(959, 719),
                         Eigen::Vector2d(1800, 1350), Eigen::Vector2d(40, 1400)}) {
    // Treat px as undistorted; distort it, then undistort back.
    const Eigen::Vector2d xy_u((px.x() - kIntrinsics.cx) / kIntrinsics.fx,
                               (px.y() - kIntrinsics.cy) / kIntrinsics.fy);
    const Eigen::Vector2d xy_d = DistortNormalized(xy_u, k1, k2);
    const Eigen::Vector2d px_d(xy_d.x() * kIntrinsics.fx + kIntrinsics.cx,
                               xy_d.y() * kIntrinsics.fy + kIntrinsics.cy);
    const Eigen::Vector2d roundtrip = UndistortPixel(px_d, kIntrinsics, k1, k2);
    EXPECT_LT((roundtrip - px).norm(), 5e-3) << px.transpose();
  }
}

}  // namespace
}  // namespace bs
