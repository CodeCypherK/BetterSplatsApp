#include <gtest/gtest.h>

#include <cmath>

#include "io/float16.h"
#include "lidar/depth_processing.h"

namespace bs {
namespace {

const Intrinsics kDepthK{242.0, 242.0, 159.5, 119.5, 320, 240};

// Depth map of a tilted plane: n . X = dist, camera at origin looking +Z.
// z(u, v) = dist / (n . ray(u, v)).
DepthImage MakePlaneDepth(const Eigen::Vector3d& n_unit, double dist,
                          const Intrinsics& K) {
  DepthImage depth;
  depth.width = K.width;
  depth.height = K.height;
  depth.f16.resize(static_cast<size_t>(K.width) * K.height);
  for (int y = 0; y < K.height; ++y) {
    for (int x = 0; x < K.width; ++x) {
      const Eigen::Vector3d ray = K.Unproject(x, y, 1.0);
      const double denom = n_unit.dot(ray);
      const double z = denom > 1e-9 ? dist / denom : NAN;
      depth.f16[static_cast<size_t>(y) * K.width + x] =
          F32ToF16(static_cast<float>(z));
    }
  }
  return depth;
}

TEST(DepthFrame, NormalsMatchPlaneNormal) {
  const Eigen::Vector3d n = Eigen::Vector3d(0.25, -0.1, 1.0).normalized();
  const DepthFrame frame(MakePlaneDepth(n, 3.0, kDepthK), kDepthK);

  int checked = 0;
  for (int y = 20; y < 220; y += 40) {
    for (int x = 20; x < 300; x += 40) {
      const auto normal = frame.NormalAt(x, y);
      ASSERT_TRUE(normal.has_value()) << x << "," << y;
      // DepthFrame orients normals toward the camera: plane normal with
      // positive z faces away, so compare against -n. Tolerance reflects
      // float16 depth quantization (~2 deg of normal noise at 3 m).
      const double dot = normal->dot(-n);
      EXPECT_GT(dot, 0.998) << "at " << x << "," << y << ": "
                            << normal->transpose();
      ++checked;
    }
  }
  EXPECT_GT(checked, 20);
}

TEST(DepthFrame, UnprojectRoundTrip) {
  const Eigen::Vector3d n(0, 0, 1);
  const DepthFrame frame(MakePlaneDepth(n, 2.5, kDepthK), kDepthK);
  const auto p = frame.Unproject(160, 120);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->z(), 2.5, 2e-3);  // float16 quantization
  const Eigen::Vector2d px = kDepthK.Project(*p);
  EXPECT_NEAR(px.x(), 160, 1e-6);
  EXPECT_NEAR(px.y(), 120, 1e-6);
}

TEST(DepthFrame, ConfidenceHighOnCleanInterior) {
  const DepthFrame frame(
      MakePlaneDepth(Eigen::Vector3d(0, 0, 1), 2.0, kDepthK), kDepthK);
  EXPECT_GT(frame.ConfidenceAt(160, 120), 0.9);
}

TEST(DepthFrame, ConfidenceKilledAtDepthEdges) {
  // Two fronto-parallel planes with a step discontinuity down the middle.
  DepthImage depth = MakePlaneDepth(Eigen::Vector3d(0, 0, 1), 1.5, kDepthK);
  for (int y = 0; y < kDepthK.height; ++y) {
    for (int x = 160; x < kDepthK.width; ++x) {
      depth.f16[static_cast<size_t>(y) * kDepthK.width + x] = F32ToF16(2.8f);
    }
  }
  const DepthFrame frame(depth, kDepthK);
  EXPECT_GT(frame.ConfidenceAt(80, 120), 0.9);   // interior left
  EXPECT_GT(frame.ConfidenceAt(240, 120), 0.9);  // interior right
  EXPECT_LT(frame.ConfidenceAt(160, 120), 0.05); // on the step
  EXPECT_LT(frame.ConfidenceAt(159, 120), 0.05);
}

TEST(DepthFrame, ConfidenceRangeRolloff) {
  LidarConfidenceOptions options;
  const DepthFrame near_frame(
      MakePlaneDepth(Eigen::Vector3d(0, 0, 1), 0.1, kDepthK), kDepthK, options);
  EXPECT_DOUBLE_EQ(near_frame.ConfidenceAt(160, 120), 0.0);  // < range_min

  const DepthFrame mid(
      MakePlaneDepth(Eigen::Vector3d(0, 0, 1), 4.0, kDepthK), kDepthK, options);
  const double w_mid = mid.ConfidenceAt(160, 120);
  EXPECT_GT(w_mid, 0.3);
  EXPECT_LT(w_mid, 0.7);  // halfway through the 3..5 m rolloff

  const DepthFrame far(
      MakePlaneDepth(Eigen::Vector3d(0, 0, 1), 5.5, kDepthK), kDepthK, options);
  EXPECT_DOUBLE_EQ(far.ConfidenceAt(160, 120), 0.0);
}

TEST(DepthFrame, ObliqueIncidenceDeratesConfidence) {
  // A strongly tilted plane seen fronto-parallel pixels: high incidence.
  const Eigen::Vector3d steep = Eigen::Vector3d(2.2, 0, 1.0).normalized();
  const DepthFrame frame(MakePlaneDepth(steep, 2.0, kDepthK), kDepthK);
  const double w_center = frame.ConfidenceAt(160, 120);
  const DepthFrame flat(
      MakePlaneDepth(Eigen::Vector3d(0, 0, 1), 2.0, kDepthK), kDepthK);
  EXPECT_LT(w_center, flat.ConfidenceAt(160, 120) * 0.9);
}

TEST(DepthFrame, HolesBlockInterpolationAndBicubic) {
  DepthImage depth = MakePlaneDepth(Eigen::Vector3d(0, 0, 1), 2.0, kDepthK);
  depth.f16[120 * 320 + 160] = F32ToF16(NAN);
  const DepthFrame frame(depth, kDepthK);

  EXPECT_FALSE(frame.Valid(160, 120));
  EXPECT_FALSE(frame.DepthBilinear(159.5, 119.5).has_value());
  EXPECT_FALSE(frame.BicubicSafe(159.2, 119.8));
  EXPECT_FALSE(frame.BicubicSafe(161.0, 121.0));  // hole in 4x4 support
  EXPECT_TRUE(frame.BicubicSafe(170.0, 130.0));
  EXPECT_TRUE(frame.DepthBilinear(170.3, 130.7).has_value());

  // Borders are never bicubic-safe.
  EXPECT_FALSE(frame.BicubicSafe(0.5, 100.0));
  EXPECT_FALSE(frame.BicubicSafe(318.9, 100.0));
}

TEST(DepthFrame, SigmaModel) {
  const DepthFrame frame(
      MakePlaneDepth(Eigen::Vector3d(0, 0, 1), 2.0, kDepthK), kDepthK);
  EXPECT_NEAR(frame.Sigma(1.0), 0.018, 1e-9);
  EXPECT_NEAR(frame.Sigma(3.0), 0.082, 1e-9);
  EXPECT_GT(frame.Sigma(3.0), frame.Sigma(1.0));
}


// --- sensor-reported confidence (ARKit's confidenceMap, when available) ---

namespace {

// A slanted plane, so the geometric model produces an interesting spread
// rather than a constant.
DepthImage PlaneDepth(int w, int h) {
  DepthImage d;
  d.width = w;
  d.height = h;
  d.f16.resize(static_cast<size_t>(w) * h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const float metres = 1.5f + 0.004f * static_cast<float>(x);
      d.f16[static_cast<size_t>(y) * w + x] = F32ToF16(metres);
    }
  }
  return d;
}

Intrinsics PlaneK(int w, int h) {
  return {static_cast<double>(w), static_cast<double>(w),
          w * 0.5 - 0.5, h * 0.5 - 0.5, w, h};
}

}  // namespace

TEST(LidarSensorConfidence, AbsentChangesNothing) {
  // The load-bearing property: every existing session, and every capture
  // backend that cannot supply confidence, must behave EXACTLY as before.
  // Absent means "no opinion", never "zero confidence".
  const int w = 24, h = 18;
  const DepthImage plain = PlaneDepth(w, h);
  DepthFrame frame(plain, PlaneK(w, h));
  EXPECT_FALSE(frame.has_sensor_confidence());

  DepthImage with_full = plain;
  with_full.confidence.assign(plain.f16.size(), 255);
  DepthFrame full(with_full, PlaneK(w, h));
  ASSERT_TRUE(full.has_sensor_confidence());

  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      EXPECT_NEAR(frame.ConfidenceAt(x, y), full.ConfidenceAt(x, y), 1e-12)
          << "at " << x << "," << y;
    }
  }
}

TEST(LidarSensorConfidence, LowSensorConfidenceDeratesAPixelTheModelTrusts) {
  const int w = 24, h = 18;
  DepthImage plain = PlaneDepth(w, h);
  DepthFrame baseline(plain, PlaneK(w, h));
  const double trusted = baseline.ConfidenceAt(12, 9);
  ASSERT_GT(trusted, 0.5) << "pick a pixel the geometric model likes";

  // ARKit reports low confidence here — the sensor knows something the
  // depth map's own geometry does not show.
  plain.confidence.assign(plain.f16.size(), 255);
  plain.confidence[static_cast<size_t>(9) * w + 12] = 26;  // ~0.1
  DepthFrame derated(plain, PlaneK(w, h));
  EXPECT_NEAR(derated.ConfidenceAt(12, 9), trusted * 26.0 / 255.0, 1e-9);
}

TEST(LidarSensorConfidence, CanOnlyLowerNeverRaise) {
  // A sensor claiming high confidence must not rescue depth the geometric
  // model rejects — that is the safety property that makes this additive.
  const int w = 24, h = 18;
  DepthImage plain = PlaneDepth(w, h);
  // Force an invalid pixel: the model returns 0 regardless.
  plain.f16[static_cast<size_t>(9) * w + 12] = F32ToF16(0.0f);
  plain.confidence.assign(plain.f16.size(), 255);
  DepthFrame frame(plain, PlaneK(w, h));
  EXPECT_DOUBLE_EQ(frame.ConfidenceAt(12, 9), 0.0);
}

TEST(LidarSensorConfidence, WrongSizedConfidenceIsIgnored) {
  // A backend bug that supplies a mismatched plane must not index out of
  // range or silently derate half the frame.
  const int w = 24, h = 18;
  DepthImage plain = PlaneDepth(w, h);
  DepthFrame baseline(plain, PlaneK(w, h));
  plain.confidence.assign(10, 0);  // nonsense
  DepthFrame frame(plain, PlaneK(w, h));
  EXPECT_FALSE(frame.has_sensor_confidence());
  EXPECT_NEAR(frame.ConfidenceAt(12, 9), baseline.ConfidenceAt(12, 9), 1e-12);
}

}  // namespace
}  // namespace bs
