// Synthetic session generator: geometric consistency of rendered depth
// against ground-truth poses, determinism, and reader integration.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <functional>
#include <utility>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include "common/config.h"
#include "common/geometry.h"
#include "generate.h"
#include "io/session_reader.h"
#include "synth_scene.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

class SynthTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Per-test directory, so ctest -j does not have two cases generating
    // into the same session path.
    dir_ = (fs::temp_directory_path() /
            (std::string("bs_synth_test_") +
             ::testing::UnitTest::GetInstance()->current_test_info()->name()))
               .string();
    fs::remove_all(dir_);
  }
  void TearDown() override {
    fs::remove_all(dir_);
    fs::remove_all(dir_ + "_b");
  }
  std::string dir_;
};

synth::GenerateOptions SmallOptions(const std::string& dir) {
  synth::GenerateOptions o;
  o.out_dir = dir;
  o.frame_count = 6;
  o.image_width = 480;
  o.image_height = 360;
  o.depth_width = 160;
  o.depth_height = 120;
  o.seed = 21;
  return o;
}

TEST_F(SynthTest, GeneratesReadableSession) {
  ASSERT_TRUE(synth::GenerateSession(SmallOptions(dir_)));

  auto reader = SessionReader::Open(dir_);
  ASSERT_TRUE(reader.has_value());
  EXPECT_EQ(reader->frame_ids().size(), 6u);
  EXPECT_EQ(reader->info().frame_count, 6u);
  EXPECT_EQ(reader->info().device_model, "synthetic");

  const auto gt = reader->ReadGroundTruth();
  ASSERT_TRUE(gt.has_value());
  EXPECT_EQ(gt->poses.size(), 6u);

  // Every frame is readable and depth is mostly valid indoor-range data.
  for (const uint32_t id : reader->frame_ids()) {
    const auto meta = reader->ReadMeta(id);
    const auto depth = reader->ReadDepth(id);
    const auto jpeg = reader->ReadImageBytes(id);
    ASSERT_TRUE(meta && depth && jpeg) << "frame " << id;

    const cv::Mat img = cv::imdecode(*jpeg, cv::IMREAD_COLOR);
    ASSERT_FALSE(img.empty());
    EXPECT_EQ(img.cols, 480);
    EXPECT_EQ(img.rows, 360);

    int valid = 0;
    double sum = 0;
    for (int y = 0; y < depth->height; ++y) {
      for (int x = 0; x < depth->width; ++x) {
        if (!depth->ValidAt(x, y)) continue;
        const float m = depth->MetersAt(x, y);
        EXPECT_GT(m, 0.05f);
        EXPECT_LT(m, 12.0f);
        ++valid;
        sum += m;
      }
    }
    const double frac = static_cast<double>(valid) / (depth->width * depth->height);
    EXPECT_GT(frac, 0.55) << "frame " << id;
    EXPECT_GT(sum / std::max(valid, 1), 0.4);
  }
}

TEST_F(SynthTest, DepthMatchesGroundTruthGeometry) {
  auto options = SmallOptions(dir_);
  options.depth_noise_scale = 0.0;  // perfect depth for this check
  ASSERT_TRUE(synth::GenerateSession(options));

  auto reader = SessionReader::Open(dir_);
  ASSERT_TRUE(reader.has_value());
  const auto gt = reader->ReadGroundTruth();
  ASSERT_TRUE(gt.has_value());

  const synth::Scene scene = synth::MakeRoomScene(options.seed, options.blank_wall);

  int checked = 0;
  for (size_t i = 0; i < gt->poses.size(); ++i) {
    const auto& gtp = gt->poses[i];
    const auto depth = reader->ReadDepth(gtp.frame_id);
    const auto meta = reader->ReadMeta(gtp.frame_id);
    ASSERT_TRUE(depth && meta);

    SE3 pose;
    pose.q = Eigen::Quaterniond(gtp.q[0], gtp.q[1], gtp.q[2], gtp.q[3]);
    pose.t = Eigen::Vector3d(gtp.t[0], gtp.t[1], gtp.t[2]);

    const Eigen::Vector3d center = pose.CameraCenter();
    const Eigen::Matrix3d R_cw = pose.q.conjugate().toRotationMatrix();
    const auto& K = meta->depth_intrinsics;

    // Independently re-cast rays for a grid of pixels and compare depths.
    for (int y = 10; y < depth->height; y += 25) {
      for (int x = 10; x < depth->width; x += 25) {
        if (!depth->ValidAt(x, y)) continue;
        const Eigen::Vector3d dir_cam((x - K.cx) / K.fx, (y - K.cy) / K.fy, 1.0);
        const synth::RayHit hit =
            synth::CastRay(scene, center, R_cw * dir_cam);
        if (hit.plane_index < 0) continue;
        // float16 quantization at ~5 m is ~4 mm; allow 1 cm.
        EXPECT_NEAR(depth->MetersAt(x, y), hit.t, 0.01)
            << "frame " << gtp.frame_id << " px (" << x << "," << y << ")";
        ++checked;
      }
    }
  }
  EXPECT_GT(checked, 100);
}

TEST_F(SynthTest, ImagesHaveDetectableFeatures) {
  ASSERT_TRUE(synth::GenerateSession(SmallOptions(dir_)));
  auto reader = SessionReader::Open(dir_);
  ASSERT_TRUE(reader.has_value());

  const auto jpeg = reader->ReadImageBytes(1);
  ASSERT_TRUE(jpeg.has_value());
  const cv::Mat gray = cv::imdecode(*jpeg, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(gray.empty());

  auto orb = cv::ORB::create(1200);
  std::vector<cv::KeyPoint> kps;
  cv::Mat desc;
  orb->detectAndCompute(gray, cv::noArray(), kps, desc);
  // The procedural texture must give the live tracker plenty to work with.
  EXPECT_GT(kps.size(), 300u);
}

TEST_F(SynthTest, DeterministicForSameSeed) {
  ASSERT_TRUE(synth::GenerateSession(SmallOptions(dir_)));
  ASSERT_TRUE(synth::GenerateSession(SmallOptions(dir_ + "_b")));

  auto a = SessionReader::Open(dir_);
  auto b = SessionReader::Open(dir_ + "_b");
  ASSERT_TRUE(a && b);
  ASSERT_EQ(a->frame_ids().size(), b->frame_ids().size());

  for (const uint32_t id : a->frame_ids()) {
    const auto img_a = a->ReadImageBytes(id);
    const auto img_b = b->ReadImageBytes(id);
    const auto d_a = a->ReadDepth(id);
    const auto d_b = b->ReadDepth(id);
    ASSERT_TRUE(img_a && img_b && d_a && d_b);
    EXPECT_EQ(*img_a, *img_b) << "frame " << id << " image bytes differ";
    EXPECT_EQ(d_a->f16, d_b->f16) << "frame " << id << " depth differs";
  }
}

TEST_F(SynthTest, GroundTruthPosesAreValidRotations) {
  ASSERT_TRUE(synth::GenerateSession(SmallOptions(dir_)));
  auto reader = SessionReader::Open(dir_);
  const auto gt = reader->ReadGroundTruth();
  ASSERT_TRUE(gt.has_value());

  for (const auto& p : gt->poses) {
    const Eigen::Quaterniond q(p.q[0], p.q[1], p.q[2], p.q[3]);
    EXPECT_NEAR(q.norm(), 1.0, 1e-9);
    // Camera stays inside the room at eye height.
    SE3 pose;
    pose.q = q;
    pose.t = Eigen::Vector3d(p.t[0], p.t[1], p.t[2]);
    const Eigen::Vector3d c = pose.CameraCenter();
    EXPECT_GT(c.y(), 0.5);
    EXPECT_LT(c.y(), 2.4);
    EXPECT_LT(std::abs(c.x()), 3.0);
    EXPECT_LT(std::abs(c.z()), 4.0);
  }
}

TEST_F(SynthTest, WalkthroughClosesItsLoopAcrossBothRooms) {
  // The two-room harness is only meaningful if the walk actually visits both
  // rooms and comes back to where it started — that revisit is what loop
  // closure has to recognize.
  const std::vector<SE3> poses =
      synth::WalkthroughTrajectory(200, 1.5, /*seed=*/4);
  ASSERT_EQ(poses.size(), 200u);

  bool in_room_a = false, in_room_b = false, through_door = false;
  for (const auto& p : poses) {
    const Eigen::Vector3d c = p.CameraCenter();
    EXPECT_NEAR(c.y(), 1.5, 0.25);                 // eye height, mild jitter
    EXPECT_GT(p.q.toRotationMatrix().determinant(), 0.99);  // right-handed
    if (c.x() < 2.0) in_room_a = true;
    if (c.x() > 4.5) in_room_b = true;
    // The doorway is the only opening in the divider (z within +-0.55).
    if (std::abs(c.x() - 3.0) < 0.35) {
      through_door = true;
      EXPECT_LT(std::abs(c.z()), 0.55) << "walked through the dividing wall";
    }
  }
  EXPECT_TRUE(in_room_a);
  EXPECT_TRUE(in_room_b);
  EXPECT_TRUE(through_door);

  // Closed loop: the last pose returns to the first.
  const double gap =
      (poses.back().CameraCenter() - poses.front().CameraCenter()).norm();
  EXPECT_LT(gap, 0.35) << "walkthrough must revisit its starting viewpoint";
}

TEST_F(SynthTest, ScoutCircuitHugsWallsAndFacesInward) {
  // The scout pass exists to see as much of each room as possible from as
  // far apart as possible — back to the wall, camera across the space. That
  // is the opposite of the capture walk, and the reason it makes a good
  // localization scaffold and a bad reconstruction input.
  const std::vector<SE3> poses = synth::ScoutTrajectory(120, 1.5, /*seed=*/4);
  ASSERT_EQ(poses.size(), 120u);

  int facing_inward = 0;
  int near_perimeter = 0;
  for (const auto& p : poses) {
    const Eigen::Vector3d c = p.CameraCenter();
    // Perimeter: within arm's reach of a wall of the room you are in (room A
    // spans x in [-3,3], room B x in [3,9], both z in [-4,4]). Crossing from
    // a doorway to the next wall is a short diagonal through open floor, so
    // this is a strong majority rather than every single frame.
    const double x_lo = c.x() < 3.0 ? -3.0 : 3.0;
    const double x_hi = c.x() < 3.0 ? 3.0 : 9.0;
    const double to_wall =
        std::min({std::abs(c.x() - x_lo), std::abs(c.x() - x_hi),
                  std::abs(c.z() - (-4.0)), std::abs(c.z() - 4.0)});
    if (to_wall < 1.2) ++near_perimeter;

    // Optical axis points at the centre of the room being stood in.
    const Eigen::Vector3d centre =
        c.x() < 3.0 ? Eigen::Vector3d(0, 1.2, 0) : Eigen::Vector3d(6, 1.2, 0);
    const Eigen::Vector3d forward = p.Inverse().q * Eigen::Vector3d(0, 0, 1);
    if (forward.dot((centre - c).normalized()) > 0.9) ++facing_inward;
  }
  EXPECT_GT(near_perimeter, 100) << "scout should hug the perimeter";
  EXPECT_GT(facing_inward, 100) << "scout frames should look across the room";

  // Both rooms visited, and the circuit closes.
  bool room_a = false, room_b = false;
  for (const auto& p : poses) {
    if (p.CameraCenter().x() < 0.0) room_a = true;
    if (p.CameraCenter().x() > 6.0) room_b = true;
  }
  EXPECT_TRUE(room_a);
  EXPECT_TRUE(room_b);
  EXPECT_LT((poses.back().CameraCenter() - poses.front().CameraCenter()).norm(),
            0.5);
}

TEST_F(SynthTest, ScoutFramesAreTaggedAndSeparableFromCapture) {
  synth::GenerateOptions o;
  o.out_dir = dir_;
  o.frame_count = 8;
  o.scout_frames = 5;
  o.two_room = true;
  o.image_width = 160;
  o.image_height = 120;
  o.depth_width = 80;
  o.depth_height = 60;
  o.seed = 4;
  ASSERT_TRUE(synth::GenerateSession(o));

  const auto session = SessionReader::Open(dir_);
  ASSERT_TRUE(session.has_value());
  int scout = 0, capture = 0;
  for (const uint32_t id : session->frame_ids()) {
    const auto meta = session->ReadMeta(id);
    ASSERT_TRUE(meta.has_value());
    if (meta->is_scout()) {
      ++scout;
      EXPECT_LE(id, 5u) << "scout frames come first";
    } else {
      ++capture;
    }
  }
  EXPECT_EQ(scout, 5);
  EXPECT_EQ(capture, 8);
}

TEST_F(SynthTest, TwoRoomSceneHasAnOpenDoorway) {
  const synth::Scene scene = synth::MakeTwoRoomScene(4);
  // A ray straight through the doorway from room A must reach room B's far
  // wall (x = 9), not stop at the divider (x = 3).
  const Eigen::Vector3d from(0.0, 1.4, 0.0);
  const synth::RayHit open =
      synth::CastRay(scene, from, Eigen::Vector3d(1, 0, 0));
  ASSERT_GT(open.t, 0.0);
  EXPECT_GT(open.t, 8.0) << "doorway is blocked";

  // Off to the side of the doorway the divider must block the ray.
  const Eigen::Vector3d blocked_from(0.0, 1.4, 2.5);
  const synth::RayHit blocked =
      synth::CastRay(scene, blocked_from, Eigen::Vector3d(1, 0, 0));
  ASSERT_GT(blocked.t, 0.0);
  EXPECT_LT(blocked.t, 3.5) << "divider should block off-doorway rays";
}

// A frame count over a fixed path IS a speed. Choosing the count for render
// cost produced a 9 m/s "walkthrough" that the tracker's own motion gate
// rejects — which looked for a long time like a tracker defect rather than a
// harness defect. Speed-derived counts must stay inside hand-held motion.
TEST_F(SynthTest, SpeedDerivedFrameCountsStayWithinHandHeldMotion) {
  const EngineConfig limits;
  const double fps = 30.0, speed = 1.0, pan = 40.0;

  struct Case {
    const char* name;
    std::vector<SE3> (*shape)(int);
  };
  const auto walk = [](int n) {
    return synth::WalkthroughTrajectory(n, 1.5, 4u);
  };
  const auto scout = [](int n) { return synth::ScoutTrajectory(n, 1.5, 4u); };
  const auto orbit = [](int n) {
    return synth::OrbitTrajectory(n, 1.6, 2.2, 1.5, 4u, 140.0);
  };

  for (const auto& [name, shape] :
       std::vector<std::pair<const char*, std::function<std::vector<SE3>(int)>>>{
           {"walkthrough", walk}, {"scout", scout}, {"orbit", orbit}}) {
    const synth::TrajectoryMotion probe = synth::MeasureMotion(shape(256));
    const double seconds =
        std::max(probe.path_m / speed, probe.turn_deg / pan);
    const int frames = static_cast<int>(std::llround(seconds * fps)) + 1;
    ASSERT_GE(frames, 2) << name;

    const synth::TrajectoryMotion m = synth::MeasureMotion(shape(frames));
    EXPECT_LE(m.mean_step_m * fps, limits.track_max_speed_mps)
        << name << " mean speed";
    EXPECT_LE(m.mean_turn_deg * fps, limits.track_max_rot_dps)
        << name << " mean rotation rate";
    // The WORST frame is what decides this, not the average or the 95th
    // percentile. A look target that switched rooms at the doorway produced
    // exactly one 174 deg frame in an otherwise clean 0.7 deg/frame circuit;
    // the mean and p95 both looked healthy, and that one frame ended
    // tracking for the following 800.
    EXPECT_LE(m.max_step_m * fps, limits.track_max_speed_mps)
        << name << " peak speed";
    EXPECT_LE(m.max_turn_deg * fps, limits.track_max_rot_dps)
        << name << " peak rotation rate";
  }
}

// The measurement itself: a straight 1 m walk sampled 11 times is 10 cm and
// no rotation per step, whatever the units elsewhere claim.
TEST_F(SynthTest, MeasureMotionReportsPathAndTurn) {
  std::vector<SE3> poses;
  for (int i = 0; i <= 10; ++i) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(DegToRad(i * 2.0), Eigen::Vector3d::UnitY()));
    poses.push_back(SE3::FromCamToWorld(
        q, Eigen::Vector3d(0.1 * i, 0.0, 0.0)));
  }
  const synth::TrajectoryMotion m = synth::MeasureMotion(poses);
  EXPECT_NEAR(m.path_m, 1.0, 1e-9);
  EXPECT_NEAR(m.mean_step_m, 0.1, 1e-9);
  EXPECT_NEAR(m.turn_deg, 20.0, 1e-6);
  EXPECT_NEAR(m.mean_turn_deg, 2.0, 1e-6);
  EXPECT_NEAR(m.max_step_m, 0.1, 1e-9);
  EXPECT_NEAR(m.max_turn_deg, 2.0, 1e-6);
  EXPECT_EQ(synth::MeasureMotion({}).path_m, 0.0);
}

// The view direction is rate-limited, so a trajectory whose look target
// jumps still turns the camera at a speed a wrist can produce.
TEST_F(SynthTest, ScoutCircuitDoesNotTeleportItsViewAtTheDoorway) {
  const std::vector<SE3> poses = synth::ScoutTrajectory(1303, 1.5, 4u);
  const synth::TrajectoryMotion m = synth::MeasureMotion(poses);
  EXPECT_LT(m.max_turn_deg, 6.0)
      << "single-frame view flip at the room boundary";

  // And the camera never ends up staring at the floor: the doorway blend
  // once put the look target on top of the camera, which both aimed the lens
  // down and made the roll ill-conditioned.
  double worst_tilt = 0;
  for (const SE3& p : poses) {
    const Eigen::Vector3d forward = p.Inverse().q * Eigen::Vector3d(0, 0, 1);
    worst_tilt = std::max(worst_tilt, std::abs(forward.y()));
  }
  EXPECT_LT(worst_tilt, 0.5) << "view tilted past 30 deg from horizontal";
}

}  // namespace
}  // namespace bs
