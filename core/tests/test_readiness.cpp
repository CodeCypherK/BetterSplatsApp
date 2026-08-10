// Splat-readiness scoring: crafted maps -> expected scores and guidance.

#include <gtest/gtest.h>

#include "readiness/patch_grid.h"

namespace bs {
namespace {

// Builds a map with keyframes on an arc facing a wall segment at z = 3 and
// a grid of points on that wall, with controllable per-point stats.
struct MapBuilder {
  LiveMap map;

  std::vector<uint32_t> AddArcKeyframes(int count, double spread_x,
                                        double x_offset = 0.0) {
    std::vector<uint32_t> ids;
    for (int i = 0; i < count; ++i) {
      Keyframe kf;
      kf.frame_id = static_cast<uint32_t>(i + 1);
      const double x =
          (count == 1 ? 0.0 : -spread_x / 2 + spread_x * i / (count - 1)) +
          x_offset;
      kf.pose.q = Eigen::Quaterniond::Identity();
      kf.pose.t = -Eigen::Vector3d(x, 0, 0);  // camera at (x,0,0) facing +Z
      kf.K = {600, 600, 479.5, 359.5, 960, 720};
      ids.push_back(map.AddKeyframe(std::move(kf)).kf_id);
    }
    return ids;
  }

  void AddWallPoints(double x0, double x1, double y0, double y1, int nx, int ny,
                     const std::vector<uint32_t>& observers, float gradient,
                     float angle_deg, float err_px) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        MapPoint p;
        p.X = Eigen::Vector3d(x0 + (x1 - x0) * ix / std::max(1, nx - 1),
                              y0 + (y1 - y0) * iy / std::max(1, ny - 1), 3.0);
        p.mean_gradient = gradient;
        p.max_tri_angle_deg = angle_deg;
        p.last_reproj_err_px = err_px;
        for (const uint32_t kf : observers) p.observations.emplace_back(kf, 0);
        map.AddPoint(std::move(p));
      }
    }
  }
};

TEST(Readiness, WellObservedPatchScoresHigh) {
  MapBuilder b;
  const auto kfs = b.AddArcKeyframes(6, 1.6);
  // Dense, textured, well-triangulated points inside one 0.35 m patch.
  b.AddWallPoints(0.02, 0.32, 0.02, 0.32, 7, 7, kfs, /*gradient=*/30,
                  /*angle=*/6.0, /*err=*/0.5);

  PatchGrid grid;
  grid.Build(b.map);
  ASSERT_FALSE(grid.patches().empty());
  EXPECT_GT(grid.OverallScore(), 60.0f);
  EXPECT_GT(grid.SubScore(0), 80.0f);  // geometry
  EXPECT_GT(grid.SubScore(1), 80.0f);  // pose
  // Texture at 3 m viewing distance: gradient saturated but sampling
  // density (px/cm) is mid-range — an honest "move closer" signal.
  EXPECT_GT(grid.SubScore(2), 45.0f);
  EXPECT_LT(grid.SubScore(2), 80.0f);
  // LiDAR-less map: the axis scores neutrally (mean of the others).
  EXPECT_GT(grid.SubScore(3), 50.0f);
  EXPECT_GT(grid.SubScore(4), 80.0f);  // 30 deg spread, ample baseline
}

TEST(Readiness, SparseWeakPatchScoresLow) {
  MapBuilder b;
  const auto kfs = b.AddArcKeyframes(1, 0.0);  // single view, no baseline
  b.AddWallPoints(0.05, 0.25, 0.05, 0.25, 2, 2, kfs, /*gradient=*/4,
                  /*angle=*/0.4, /*err=*/1.8);

  PatchGrid grid;
  grid.Build(b.map);
  ASSERT_FALSE(grid.patches().empty());
  EXPECT_LT(grid.OverallScore(), 30.0f);
  EXPECT_FALSE(grid.weak_areas().empty());
}

TEST(Readiness, MinAxisCapsAggregate) {
  MapBuilder b;
  const auto kfs = b.AddArcKeyframes(6, 1.6);
  // Everything strong except texture (blank wall, gradient ~0).
  b.AddWallPoints(0.02, 0.32, 0.02, 0.32, 7, 7, kfs, /*gradient=*/0.5,
                  /*angle=*/6.0, /*err=*/0.4);

  PatchGrid grid;
  grid.Build(b.map);
  const auto& patch = grid.patches().begin()->second;
  const float min_sub = *std::min_element(patch.sub, patch.sub + 5);
  EXPECT_LE(patch.score, min_sub + 35.0f + 1e-4f);
}

TEST(Readiness, WeakAreaDiagnosesTextureDeficiency) {
  MapBuilder b;
  const auto kfs = b.AddArcKeyframes(6, 1.6);
  // Strong geometry/view but near-zero texture: the weak area must call
  // out the texture axis (index 2), classify the surface as a wall, and
  // suggest a sideways move perpendicular to the wall normal.
  b.AddWallPoints(0.02, 0.32, 1.32, 1.62, 7, 7, kfs, /*gradient=*/0.5,
                  /*angle=*/6.0, /*err=*/0.4);

  PatchGrid grid;
  grid.Build(b.map);
  ASSERT_FALSE(grid.weak_areas().empty());
  const WeakArea& area = grid.weak_areas().front();
  EXPECT_EQ(area.deficiency, 2);
  EXPECT_EQ(area.surface_kind, 0);  // wall (points span x-y plane, normal z)
  // Move direction horizontal and perpendicular to the wall normal (z).
  EXPECT_LT(std::abs(area.move_dir.z()), 0.3);
  EXPECT_NEAR(area.move_dir.norm(), 1.0, 1e-6);
  EXPECT_GE(area.move_dist_m, 0.3);
  EXPECT_LE(area.move_dist_m, 1.0);
}

TEST(Readiness, FloorClassification) {
  MapBuilder b;
  // Cameras looking down at the floor from above: floor points at y=0
  // spanning x-z, normal along +Y, below eye height.
  const auto kf_ids = b.AddArcKeyframes(4, 1.0);
  for (auto& kf : b.map.keyframes()) {
    // Reposition: camera at y=1.6 looking straight down (-Y): camera +Z
    // maps to world -Y.
    const Eigen::Vector3d center(kf.pose.CameraCenter().x(), 1.6, 0.5);
    Eigen::Matrix3d R_cw;
    R_cw.col(0) = Eigen::Vector3d(1, 0, 0);    // cam x -> world x
    R_cw.col(1) = Eigen::Vector3d(0, 0, 1);    // cam y(down) -> world z
    R_cw.col(2) = Eigen::Vector3d(0, -1, 0);   // cam z(fwd) -> world -y
    kf.pose = SE3::FromCamToWorld(Eigen::Quaterniond(R_cw), center);
  }
  for (int ix = 0; ix < 6; ++ix) {
    for (int iz = 0; iz < 6; ++iz) {
      MapPoint p;
      p.X = Eigen::Vector3d(0.02 + 0.05 * ix, 0.0, 0.4 + 0.05 * iz);
      p.mean_gradient = 1.0f;  // blank floor -> weak texture
      p.max_tri_angle_deg = 5.0f;
      p.last_reproj_err_px = 0.5f;
      for (const uint32_t kf : kf_ids) p.observations.emplace_back(kf, 0);
      b.map.AddPoint(std::move(p));
    }
  }

  PatchGrid grid;
  grid.Build(b.map);
  ASSERT_FALSE(grid.weak_areas().empty());
  EXPECT_EQ(grid.weak_areas().front().surface_kind, 1);  // floor
}

TEST(Readiness, EmptyMapIsSafe) {
  LiveMap empty;
  PatchGrid grid;
  grid.Build(empty);
  EXPECT_EQ(grid.OverallScore(), 0.0f);
  EXPECT_TRUE(grid.patches().empty());
  EXPECT_TRUE(grid.weak_areas().empty());

  std::vector<bs_snap_patch> patches;
  std::vector<bs_snap_region> regions;
  std::vector<bs_snap_weak_area> weak;
  grid.FillSnapshot(patches, regions, weak, {});
  EXPECT_TRUE(patches.empty());
  EXPECT_TRUE(regions.empty());
}

TEST(Readiness, SnapshotCarriesRegionAndScores) {
  MapBuilder b;
  const auto kfs = b.AddArcKeyframes(6, 1.6);
  b.AddWallPoints(0.02, 0.32, 0.02, 0.32, 7, 7, kfs, 30, 6.0, 0.5);

  PatchGrid grid;
  grid.Build(b.map);
  std::vector<bs_snap_patch> patches;
  std::vector<bs_snap_region> regions;
  std::vector<bs_snap_weak_area> weak;
  grid.FillSnapshot(patches, regions, weak, {{1, "Kitchen"}});

  ASSERT_EQ(regions.size(), 1u);
  EXPECT_STREQ(regions[0].name, "Kitchen");
  EXPECT_EQ(regions[0].region_id, 1u);
  EXPECT_EQ(regions[0].patch_count, patches.size());
  EXPECT_NEAR(regions[0].score, grid.OverallScore(), 1e-4);
  ASSERT_FALSE(patches.empty());
  EXPECT_GT(patches[0].extent, 0.0f);
  EXPECT_EQ(patches[0].region_id, 1u);
}

TEST(Readiness, DistantClustersFormSeparateRegions) {
  MapBuilder b;
  // Room A: strong wall near the origin. Room B: 10 m away (beyond the
  // 2 m clustering distance, zero covisibility), blank wall -> weak.
  const auto kfs_a = b.AddArcKeyframes(3, 1.0);
  const auto kfs_b = b.AddArcKeyframes(3, 1.0, /*x_offset=*/10.0);
  b.AddWallPoints(0.02, 0.32, 0.02, 0.32, 7, 7, kfs_a, /*gradient=*/30,
                  /*angle=*/6.0, /*err=*/0.5);
  b.AddWallPoints(10.02, 10.12, 0.02, 0.32, 7, 7, kfs_b, /*gradient=*/0.5,
                  /*angle=*/6.0, /*err=*/0.5);

  PatchGrid grid;
  grid.Build(b.map);
  ASSERT_EQ(grid.regions().size(), 2u);
  // Discovery order: earliest keyframe's component is region 1.
  EXPECT_EQ(grid.regions()[0].id, 1u);
  EXPECT_EQ(grid.regions()[1].id, 2u);
  // Scores stay separated per room instead of averaging together.
  EXPECT_GT(grid.regions()[0].score, 60.0f);
  EXPECT_LT(grid.regions()[1].score, 45.0f);
  uint32_t patch_total = 0;
  for (const auto& r : grid.regions()) patch_total += r.patch_count;
  EXPECT_EQ(patch_total, grid.patches().size());

  // The weak area lands in room B.
  ASSERT_FALSE(grid.weak_areas().empty());
  EXPECT_EQ(grid.weak_areas().front().region_id, 2u);

  // Renames apply per id; unnamed regions fall back to "Room N".
  std::vector<bs_snap_patch> patches;
  std::vector<bs_snap_region> regions;
  std::vector<bs_snap_weak_area> weak;
  grid.FillSnapshot(patches, regions, weak, {{2, "Bedroom"}});
  ASSERT_EQ(regions.size(), 2u);
  EXPECT_STREQ(regions[0].name, "Room 1");
  EXPECT_STREQ(regions[1].name, "Bedroom");
}

}  // namespace
}  // namespace bs
