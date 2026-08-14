// Which way to send someone who has lost tracking.
//
// These are sign-and-frame tests. The maths is three lines, and every one of
// those lines has a plausible-looking wrong version: world instead of camera
// coordinates, target-minus-here instead of here-minus-target, the pose's
// translation instead of its camera centre. All of them produce a confident
// arrow, and on a device a confident arrow pointing the wrong way is worse
// than no arrow — it walks the user further from the map while telling them
// they are going back to it.

#include <gtest/gtest.h>

#include "live/live_system.h"

namespace bs {
namespace {

// A keyframe whose camera sits at `centre` in world coordinates, looking
// along +Z. `pose` is world-to-camera, so t = -R * centre.
Keyframe KeyframeAt(uint32_t kf_id, const Eigen::Vector3d& centre) {
  Keyframe kf;
  kf.kf_id = kf_id;
  kf.frame_id = kf_id;
  kf.pose.q = Eigen::Quaterniond::Identity();
  kf.pose.t = -centre;
  return kf;
}

// A viewer at `centre` looking along +Z, i.e. the same convention.
SE3 PoseAt(const Eigen::Vector3d& centre) {
  SE3 pose;
  pose.q = Eigen::Quaterniond::Identity();
  pose.t = -centre;
  return pose;
}

TEST(GuideVector, EmptyMapGivesNothing) {
  const std::deque<Keyframe> none;
  const GuideVector guide =
      GuideToNearestKeyframe(PoseAt({0, 0, 0}), none);
  EXPECT_FALSE(guide.valid);
}

TEST(GuideVector, PointsAtTheNearestKeyframeNotTheMostRecent) {
  std::deque<Keyframe> kfs;
  kfs.push_back(KeyframeAt(1, {2, 0, 0}));   // nearest
  kfs.push_back(KeyframeAt(2, {-9, 0, 0}));  // most recent, far away
  const GuideVector guide = GuideToNearestKeyframe(PoseAt({0, 0, 0}), kfs);
  ASSERT_TRUE(guide.valid);
  EXPECT_EQ(guide.kf_id, 1u);
  EXPECT_NEAR(guide.distance_m, 2.0, 1e-9);
}

TEST(GuideVector, KeyframeToTheRightReadsAsRight) {
  // Camera looks along +Z; the map is at +X, which is the user's right.
  std::deque<Keyframe> kfs;
  kfs.push_back(KeyframeAt(1, {3, 0, 0}));
  const GuideVector guide = GuideToNearestKeyframe(PoseAt({0, 0, 0}), kfs);
  ASSERT_TRUE(guide.valid);
  EXPECT_NEAR(guide.dir_camera.x(), 1.0, 1e-9);
  EXPECT_NEAR(guide.dir_camera.z(), 0.0, 1e-9);
}

TEST(GuideVector, KeyframeBehindReadsAsBehind) {
  // The case that matters most in practice: walked forward into an unmapped
  // room, everything mapped is now behind. -Z in camera coordinates.
  std::deque<Keyframe> kfs;
  kfs.push_back(KeyframeAt(1, {0, 0, -4}));
  const GuideVector guide = GuideToNearestKeyframe(PoseAt({0, 0, 0}), kfs);
  ASSERT_TRUE(guide.valid);
  EXPECT_NEAR(guide.dir_camera.z(), -1.0, 1e-9);
  EXPECT_NEAR(guide.distance_m, 4.0, 1e-9);
}

TEST(GuideVector, DirectionIsRelativeToWhereTheCameraIsPointing) {
  // Same world geometry as KeyframeToTheRightReadsAsRight — target at world
  // +X — but the camera has been turned to face it. The reported direction
  // has to follow the camera, not the world.
  //
  // pose.q is world-to-camera, so the optical axis in world coordinates is
  // R^T * (0,0,1). With R = Ry(-90 deg) that is (1,0,0): the camera now
  // looks straight at the target, and "on your right" becomes "ahead".
  std::deque<Keyframe> kfs;
  kfs.push_back(KeyframeAt(1, {3, 0, 0}));
  SE3 turned;
  turned.q = Eigen::Quaterniond(
      Eigen::AngleAxisd(DegToRad(-90.0), Eigen::Vector3d::UnitY()));
  turned.t = Eigen::Vector3d::Zero();  // camera centre still at the origin
  ASSERT_NEAR((turned.q.conjugate() * Eigen::Vector3d::UnitZ()).x(), 1.0, 1e-9);

  const GuideVector guide = GuideToNearestKeyframe(turned, kfs);
  ASSERT_TRUE(guide.valid);
  EXPECT_NEAR(guide.dir_camera.z(), 1.0, 1e-9);   // now straight ahead
  EXPECT_NEAR(guide.dir_camera.x(), 0.0, 1e-9);

  // ...and the opposite turn puts it behind, which is the check that would
  // have caught this test having the sign backwards the first time.
  SE3 away = turned;
  away.q = Eigen::Quaterniond(
      Eigen::AngleAxisd(DegToRad(90.0), Eigen::Vector3d::UnitY()));
  EXPECT_NEAR(GuideToNearestKeyframe(away, kfs).dir_camera.z(), -1.0, 1e-9);
}

TEST(GuideVector, StandingOnAKeyframeGivesNoArrow) {
  // Lost while essentially on top of a mapped spot means the camera is
  // pointed wrong, not the feet. An arrow here would walk the user away from
  // the exact place they need to look at.
  std::deque<Keyframe> kfs;
  kfs.push_back(KeyframeAt(1, {0.1, 0, 0.1}));
  const GuideVector guide = GuideToNearestKeyframe(PoseAt({0, 0, 0}), kfs);
  EXPECT_FALSE(guide.valid);
}

TEST(GuideVector, DirectionIsUnitLength) {
  std::deque<Keyframe> kfs;
  kfs.push_back(KeyframeAt(1, {3, -2, 5}));
  const GuideVector guide = GuideToNearestKeyframe(PoseAt({-1, 1, 0}), kfs);
  ASSERT_TRUE(guide.valid);
  EXPECT_NEAR(guide.dir_camera.norm(), 1.0, 1e-9);
  // ...and the distance is the real one, not folded into the direction.
  EXPECT_NEAR(guide.distance_m,
              (Eigen::Vector3d(3, -2, 5) - Eigen::Vector3d(-1, 1, 0)).norm(),
              1e-9);
}

}  // namespace
}  // namespace bs
