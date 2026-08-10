// Persisting the live map so a later pass can localize against it. The
// scaffold is only useful if it survives the trip byte-for-byte in the
// parts relocalization depends on: descriptors, 3D points, and the
// cross-references between them.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "live/map_io.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

class MapIoTest : public ::testing::Test {
 protected:
  void SetUp() override {
    path_ = (fs::temp_directory_path() /
              (std::string("bs_map_io_") +
               ::testing::UnitTest::GetInstance()->current_test_info()->name() +
               ".bin"))
                 .string();
    fs::remove(path_);
  }
  void TearDown() override { fs::remove(path_); }

  // A small map with the structure relocalization actually walks: several
  // keyframes, points observed from more than one of them, and per-feature
  // associations pointing back at those points.
  LiveMap MakeMap() {
    LiveMap map;
    std::vector<uint32_t> kf_ids;
    for (int i = 0; i < 4; ++i) {
      Keyframe kf;
      kf.frame_id = static_cast<uint32_t>(100 + i);
      kf.t_capture = 10.0 + i;
      kf.pose.q = Eigen::Quaterniond(
          Eigen::AngleAxisd(0.1 * i, Eigen::Vector3d::UnitY()));
      kf.pose.t = Eigen::Vector3d(0.2 * i, -0.1, 1.0);
      kf.K = {600, 601, 319.5, 239.5, 640, 480};
      kf.features.type = FeatureType::kOrb;
      const int n = 6;
      kf.features.descriptors.create(n, 32, CV_8U);
      for (int r = 0; r < n; ++r) {
        for (int c = 0; c < 32; ++c) {
          kf.features.descriptors.at<uint8_t>(r, c) =
              static_cast<uint8_t>((r * 31 + c * 7 + i * 13) & 0xFF);
        }
        kf.features.keypoints.emplace_back(
            cv::Point2f(10.0f * r + i, 20.0f * r - i), 7.0f);
        kf.undistorted.emplace_back(10.0f * r + i + 0.5f, 20.0f * r - i - 0.5f);
      }
      kf.point_ids.assign(n, -1);
      kf_ids.push_back(map.AddKeyframe(std::move(kf)).kf_id);
    }

    for (int p = 0; p < 5; ++p) {
      MapPoint mp;
      mp.X = Eigen::Vector3d(0.3 * p, 0.5, 2.0 + 0.1 * p);
      mp.descriptor.create(1, 32, CV_8U);
      for (int c = 0; c < 32; ++c) {
        mp.descriptor.at<uint8_t>(0, c) = static_cast<uint8_t>((p * 17 + c) & 0xFF);
      }
      mp.mean_gradient = 12.5f + p;
      mp.max_tri_angle_deg = 3.5f + p;
      mp.last_reproj_err_px = 0.6f;
      mp.rgb[0] = static_cast<uint8_t>(10 * p);
      // Observed by two keyframes, at feature slot p.
      mp.observations.emplace_back(kf_ids[p % 4], p);
      mp.observations.emplace_back(kf_ids[(p + 1) % 4], p);
      const int32_t id = map.AddPoint(std::move(mp)).id;
      map.FindKeyframe(kf_ids[p % 4])->point_ids[p] = id;
      map.FindKeyframe(kf_ids[(p + 1) % 4])->point_ids[p] = id;
    }
    return map;
  }

  std::string path_;
};

TEST_F(MapIoTest, RoundTripPreservesScaffold) {
  const LiveMap original = MakeMap();
  ASSERT_TRUE(WriteLiveMap(original, path_, /*scale_locked=*/true));

  LiveMap loaded;
  bool metric = false;
  ASSERT_TRUE(ReadLiveMap(path_, loaded, &metric));
  EXPECT_TRUE(metric);

  ASSERT_EQ(loaded.keyframes().size(), original.keyframes().size());
  ASSERT_EQ(loaded.points().size(), original.points().size());

  for (size_t i = 0; i < loaded.keyframes().size(); ++i) {
    const Keyframe& a = original.keyframes()[i];
    const Keyframe& b = loaded.keyframes()[i];
    EXPECT_EQ(b.frame_id, a.frame_id);
    EXPECT_DOUBLE_EQ(b.t_capture, a.t_capture);
    EXPECT_LT((b.pose.t - a.pose.t).norm(), 1e-12);
    EXPECT_LT(std::abs(b.pose.q.dot(a.pose.q)) - 1.0, 1e-12);
    EXPECT_DOUBLE_EQ(b.K.fx, a.K.fx);
    EXPECT_EQ(b.K.width, a.K.width);
    EXPECT_TRUE(b.from_scaffold);

    // Descriptors are what relocalization matches against — they must be
    // bit-identical, not merely close.
    ASSERT_EQ(b.features.descriptors.rows, a.features.descriptors.rows);
    EXPECT_EQ(cv::norm(b.features.descriptors, a.features.descriptors,
                       cv::NORM_L1),
              0.0);
    ASSERT_EQ(b.features.keypoints.size(), a.features.keypoints.size());
    EXPECT_EQ(b.features.keypoints[2].pt, a.features.keypoints[2].pt);
    ASSERT_EQ(b.undistorted.size(), a.undistorted.size());
    EXPECT_EQ(b.undistorted[3], a.undistorted[3]);
  }

  // Cross-references survive id reassignment: every feature association
  // names a point that exists, and that point names this keyframe back.
  int linked = 0;
  for (const auto& kf : loaded.keyframes()) {
    for (size_t f = 0; f < kf.point_ids.size(); ++f) {
      const int32_t pid = kf.point_ids[f];
      if (pid < 0) continue;
      const auto it = loaded.points().find(pid);
      ASSERT_NE(it, loaded.points().end()) << "dangling point id " << pid;
      EXPECT_TRUE(it->second.from_scaffold);
      bool back_reference = false;
      for (const auto& [kf_id, feat] : it->second.observations) {
        if (kf_id == kf.kf_id && feat == static_cast<int>(f)) {
          back_reference = true;
        }
      }
      EXPECT_TRUE(back_reference) << "point does not observe its keyframe";
      ++linked;
    }
  }
  EXPECT_EQ(linked, 10);  // 5 points x 2 observing keyframes
}

TEST_F(MapIoTest, RejectsGarbageAndTruncation) {
  LiveMap empty;
  EXPECT_FALSE(ReadLiveMap(path_ + "_missing", empty));

  std::ofstream(path_, std::ios::binary) << "not a map file at all";
  LiveMap loaded;
  EXPECT_FALSE(ReadLiveMap(path_, loaded));

  // A valid file cut short must fail rather than yield a partial scaffold.
  const LiveMap original = MakeMap();
  ASSERT_TRUE(WriteLiveMap(original, path_, /*scale_locked=*/true));
  const auto full = fs::file_size(path_);
  std::string bytes(full, '\0');
  { std::ifstream(path_, std::ios::binary).read(bytes.data(), full); }
  std::ofstream(path_, std::ios::binary | std::ios::trunc)
      .write(bytes.data(), static_cast<std::streamsize>(full / 2));
  LiveMap truncated;
  EXPECT_FALSE(ReadLiveMap(path_, truncated));
}

}  // namespace
}  // namespace bs
