// Measuring a plane from one LiDAR frame — the calibration step that makes
// the floor a fact rather than an inference.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>

#include "io/float16.h"
#include "lidar/plane_fit.h"

namespace bs {
namespace {

const Intrinsics kDepthK{242.0, 242.0, 159.5, 119.5, 320, 240};

// Renders a depth map of one plane, given in camera coordinates as
// normal.dot(X) + offset = 0 with the normal facing the camera.
DepthImage PlaneDepth(const Eigen::Vector3d& normal, double offset,
                      double noise_sigma = 0.0, double clutter_frac = 0.0,
                      uint32_t seed = 11) {
  DepthImage image;
  image.width = kDepthK.width;
  image.height = kDepthK.height;
  image.f16.assign(static_cast<size_t>(image.width) * image.height, 0);

  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, std::max(1e-9, noise_sigma));
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  const Eigen::Vector3d n = normal.normalized();
  for (int y = 0; y < image.height; ++y) {
    for (int x = 0; x < image.width; ++x) {
      // Ray through the pixel; depth is where it meets the plane.
      const Eigen::Vector3d ray((x - kDepthK.cx) / kDepthK.fx,
                                (y - kDepthK.cy) / kDepthK.fy, 1.0);
      const double denominator = n.dot(ray);
      if (std::abs(denominator) < 1e-9) continue;
      double t = -offset / denominator;
      if (t <= 0.05 || t > 8.0) continue;
      if (noise_sigma > 0) t += gauss(rng);
      // Clutter: objects standing on the surface, closer than it.
      if (clutter_frac > 0 && uniform(rng) < clutter_frac) t *= 0.55;
      image.f16[static_cast<size_t>(y) * image.width + x] =
          F32ToF16(static_cast<float>(t));
    }
  }
  return image;
}

TEST(PlaneFitTest, RecoversAFloorUnderTheCamera) {
  // Phone held 1.4 m up, tipped 20 degrees off straight-down. The floor's
  // normal in camera coordinates points back up toward the lens.
  const double tip = DegToRad(20.0);
  const Eigen::Vector3d normal(0.0, -std::sin(tip), -std::cos(tip));
  const DepthImage image = PlaneDepth(normal, 1.4, /*noise_sigma=*/0.004);
  const DepthFrame frame(image, kDepthK);

  const DepthPlane plane = FitDepthPlane(frame);
  ASSERT_TRUE(plane.valid);
  EXPECT_NEAR(plane.offset, 1.4, 0.02) << "camera height above the surface";
  EXPECT_LT(RadToDeg(std::acos(std::clamp(
                plane.normal.dot(normal.normalized()), -1.0, 1.0))),
            1.0)
      << "normal is off by more than a degree";
  EXPECT_LT(plane.rmse_m, 0.01);
  EXPECT_GT(plane.inlier_frac, 0.9);
  EXPECT_NEAR(plane.incidence_deg, 20.0, 2.0)
      << "should report how far off square the phone was held";
}

TEST(PlaneFitTest, NormalAlwaysFacesTheCamera) {
  // Whichever way the sample geometry happens to wind, `offset` has to come
  // out as the camera's distance above the surface — a caller reads it as a
  // height, and a sign flip would put the floor above the phone.
  for (double tip_deg : {0.0, 15.0, 35.0}) {
    const double tip = DegToRad(tip_deg);
    const Eigen::Vector3d normal(0.0, -std::sin(tip), -std::cos(tip));
    const DepthImage image = PlaneDepth(normal, 1.2);
    const DepthFrame frame(image, kDepthK);
    const DepthPlane plane = FitDepthPlane(frame);
    ASSERT_TRUE(plane.valid) << "tip " << tip_deg;
    EXPECT_GT(plane.offset, 0.0) << "tip " << tip_deg;
    EXPECT_NEAR(plane.offset, 1.2, 0.03) << "tip " << tip_deg;
  }
}

TEST(PlaneFitTest, SurvivesClutterStandingOnTheSurface) {
  // A quarter of the frame is objects sitting on the floor. The floor is
  // still the majority surface, so it should still be the answer.
  const Eigen::Vector3d normal(0.0, 0.0, -1.0);  // phone aimed at the floor
  const DepthImage image =
      PlaneDepth(normal, 1.5, /*noise_sigma=*/0.004, /*clutter_frac=*/0.25);
  const DepthFrame frame(image, kDepthK);

  const DepthPlane plane = FitDepthPlane(frame);
  ASSERT_TRUE(plane.valid);
  EXPECT_NEAR(plane.offset, 1.5, 0.03);
  EXPECT_LT(plane.inlier_frac, 0.95) << "clutter should not count as inliers";
}

TEST(PlaneFitTest, RefusesWhenNoSurfaceDominates) {
  // Most of the frame is clutter at random depths: there is no plane worth
  // calibrating against, and saying so beats returning a confident answer
  // about nothing.
  DepthImage image;
  image.width = kDepthK.width;
  image.height = kDepthK.height;
  image.f16.assign(static_cast<size_t>(image.width) * image.height, 0);
  std::mt19937 rng(4);
  std::uniform_real_distribution<double> depth(0.6, 3.0);
  for (auto& sample : image.f16) {
    sample = F32ToF16(static_cast<float>(depth(rng)));
  }
  const DepthFrame frame(image, kDepthK);

  const DepthPlane plane = FitDepthPlane(frame);
  EXPECT_FALSE(plane.valid) << "random depth is not a plane";
}

TEST(PlaneFitTest, RefusesAnEmptyFrame) {
  DepthImage image;
  image.width = kDepthK.width;
  image.height = kDepthK.height;
  image.f16.assign(static_cast<size_t>(image.width) * image.height, 0);
  const DepthFrame frame(image, kDepthK);
  EXPECT_FALSE(FitDepthPlane(frame).valid);
}

TEST(PlaneFitTest, DeterministicForTheSameFrame) {
  const Eigen::Vector3d normal(0.1, -0.15, -0.98);
  const DepthImage image = PlaneDepth(normal, 1.3, 0.004);
  const DepthFrame frame(image, kDepthK);

  const DepthPlane a = FitDepthPlane(frame);
  const DepthPlane b = FitDepthPlane(frame);
  ASSERT_TRUE(a.valid && b.valid);
  // Calibrating the same frame twice must not move the floor.
  EXPECT_LT((a.normal - b.normal).norm(), 1e-12);
  EXPECT_NEAR(a.offset, b.offset, 1e-12);
}

// A plane measured in camera coordinates plus that frame's pose gives the
// world plane. This is the whole reason the calibration is stored in the
// sensor's frame: the world plane is derived whenever it is needed, from
// whatever the pose estimate currently is, so the final solve moving that
// frame does not leave a stale floor behind.
TEST(PlaneFitTest, CameraPlaneAndPoseGiveTheWorldFloor) {
  // Aimed straight at the floor: its normal comes back along the lens axis.
  const Eigen::Vector3d camera_normal(0.0, 0.0, -1.0);
  const DepthImage image = PlaneDepth(camera_normal, 1.45, 0.004);
  const DepthFrame frame(image, kDepthK);
  const DepthPlane plane = FitDepthPlane(frame);
  ASSERT_TRUE(plane.valid);

  // A camera 1.45 m up, looking down, in a world where up is +Y.
  const Eigen::Vector3d centre(2.0, 1.45, -1.0);
  const Eigen::Matrix3d cam_to_world =
      (Eigen::Matrix3d() << 1, 0, 0, 0, 0, -1, 0, 1, 0).finished();
  const SE3 pose =
      SE3::FromCamToWorld(Eigen::Quaterniond(cam_to_world), centre);

  // n_world = R_cw * n_cam; the plane passes through the camera centre
  // displaced by `offset` along -n_world.
  const Eigen::Vector3d world_normal =
      (pose.q.conjugate() * plane.normal).normalized();
  const double world_offset = -world_normal.dot(centre) + plane.offset;

  EXPECT_LT((world_normal - Eigen::Vector3d::UnitY()).norm(), 0.01)
      << "a floor seen from a level camera should come out as world up";
  EXPECT_NEAR(world_offset, 0.0, 0.02) << "floor should sit at y = 0";
}

// The verdict the capture prompt renders. It lives in the engine so the
// thresholds — which are claims about how a phone is held and how flat a
// floor is — sit with the geometry rather than in a view model.
TEST(PlaneFitTest, VerdictGuidesTheUserToAUsableFloor) {
  auto at = [](double height, double incidence, double rmse) {
    DepthPlane p;
    p.valid = true;
    p.offset = height;
    p.incidence_deg = incidence;
    p.rmse_m = rmse;
    return p;
  };

  EXPECT_EQ(CheckFloorPlane(at(1.45, 8.0, 0.008)), FloorVerdict::kGood);

  // A phone at arm's length over a table, not the floor underfoot.
  EXPECT_EQ(CheckFloorPlane(at(0.35, 5.0, 0.008)), FloorVerdict::kTooClose);
  // Aimed across the room at the far floor, where depth is unreliable.
  EXPECT_EQ(CheckFloorPlane(at(3.4, 5.0, 0.008)), FloorVerdict::kTooFar);
  // Held nearly level: a glancing view, poor depth, poor leverage.
  EXPECT_EQ(CheckFloorPlane(at(1.45, 70.0, 0.008)), FloorVerdict::kTooOblique);
  // Planar enough to fit, too rough to trust — gravel, a rug pile, clutter.
  EXPECT_EQ(CheckFloorPlane(at(1.45, 8.0, 0.09)), FloorVerdict::kTooRough);

  DepthPlane nothing;
  EXPECT_EQ(CheckFloorPlane(nothing), FloorVerdict::kNoSurface);

  // Every verdict has to say something, including one the compiler cannot
  // prove is reachable — the prompt must never render an empty string.
  for (const FloorVerdict v :
       {FloorVerdict::kGood, FloorVerdict::kNoSurface, FloorVerdict::kTooClose,
        FloorVerdict::kTooFar, FloorVerdict::kTooOblique,
        FloorVerdict::kTooRough}) {
    EXPECT_GT(std::string(FloorVerdictAdvice(v)).size(), 8u);
  }
}

}  // namespace
}  // namespace bs
