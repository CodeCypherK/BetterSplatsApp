// Splitting one reconstruction into several COLMAP models that share a
// world frame — the property that lets a facility be trained a room at a
// time and reassembled without registration.

#include <gtest/gtest.h>

#include <map>
#include <set>

#include "final/model_split.h"

namespace bs {
namespace {

// Two rooms of cameras joined by a narrow doorway, in the shape the splitter
// is meant to find: dense covisibility inside each room, a handful of tracks
// across the opening. Room A images 1..N, room B images N+1..2N.
ColmapModel TwoRoomModel(int per_room, int doorway_tracks = 4) {
  ColmapModel model;
  model.camera.width = 640;
  model.camera.height = 480;
  model.camera.params = {500, 500, 320, 240};

  for (int i = 0; i < 2 * per_room; ++i) {
    ColmapImage image;
    image.image_id = static_cast<uint32_t>(i + 1);
    const bool room_b = i >= per_room;
    // Cameras sit either side of x = 3; coordinates are what the test later
    // checks survive the split untouched.
    image.pose.t = Eigen::Vector3d(room_b ? 6.0 : 0.0, 1.5, 0.1 * i);
    image.pose.q = Eigen::Quaterniond::Identity();
    image.name = "img" + std::to_string(image.image_id) + ".jpg";
    model.images.push_back(image);
  }

  uint64_t next_id = 1;
  // Interior structure: each point seen by a run of consecutive images
  // inside one room, so the room is densely connected.
  for (int room = 0; room < 2; ++room) {
    const int base = room * per_room;
    for (int p = 0; p < per_room * 6; ++p) {
      ColmapPoint point;
      point.point3d_id = next_id++;
      point.xyz = Eigen::Vector3d(room * 6.0 + 0.01 * p, 1.0, 0.02 * p);
      const int start = base + (p % std::max(1, per_room - 3));
      for (int k = 0; k < 4 && start + k < base + per_room; ++k) {
        point.track.push_back(
            {static_cast<uint32_t>(start + k + 1), -1, 10.0 * k, 20.0});
      }
      if (point.track.size() >= 2) model.points.push_back(point);
    }
  }
  // The doorway: a few points seen from both sides.
  for (int p = 0; p < doorway_tracks; ++p) {
    ColmapPoint point;
    point.point3d_id = next_id++;
    point.xyz = Eigen::Vector3d(3.0, 1.2, 0.05 * p);
    point.track.push_back(
        {static_cast<uint32_t>(per_room - p % 2), -1, 100.0, 200.0});
    point.track.push_back(
        {static_cast<uint32_t>(per_room + 1 + p % 2), -1, 110.0, 210.0});
    model.points.push_back(point);
  }
  return model;
}

TEST(ModelSplitTest, SmallModelStaysWhole) {
  const ColmapModel model = TwoRoomModel(20);
  SplitOptions options;
  options.max_images = 500;  // comfortably larger than the model
  const auto parts = SplitModelByCovisibility(model, options);

  ASSERT_EQ(parts.size(), 1u) << "nothing to split";
  EXPECT_EQ(parts[0].model.images.size(), model.images.size());
  EXPECT_EQ(parts[0].model.points.size(), model.points.size());
}

// The property the whole feature rests on: a part's cameras and points are
// the combined model's, bit for bit. Any re-centring or re-scaling here and
// the separately trained splats would not line up, which is the one thing
// splitting is supposed to buy.
TEST(ModelSplitTest, PartsShareTheCombinedWorldFrame) {
  const ColmapModel model = TwoRoomModel(40);
  SplitOptions options;
  options.max_images = 45;
  options.min_images = 5;
  const auto parts = SplitModelByCovisibility(model, options);
  ASSERT_GE(parts.size(), 2u) << "expected the doorway to be cut";

  std::map<uint32_t, Eigen::Vector3d> centre_of;
  for (const auto& image : model.images) {
    centre_of[image.image_id] = image.pose.CameraCenter();
  }
  std::map<uint64_t, Eigen::Vector3d> xyz_of;
  for (const auto& point : model.points) xyz_of[point.point3d_id] = point.xyz;

  for (const auto& part : parts) {
    for (const auto& image : part.model.images) {
      ASSERT_TRUE(centre_of.count(image.image_id));
      EXPECT_LT((image.pose.CameraCenter() - centre_of[image.image_id]).norm(),
                1e-12)
          << "image " << image.image_id << " moved";
    }
    for (const auto& point : part.model.points) {
      ASSERT_TRUE(xyz_of.count(point.point3d_id));
      EXPECT_LT((point.xyz - xyz_of[point.point3d_id]).norm(), 1e-12)
          << "point " << point.point3d_id << " moved";
      // Ids survive too, so a reader can tell that two parts are describing
      // the same piece of structure at a seam.
      EXPECT_GT(point.track.size(), 1u) << "a track needs two observations";
    }
  }
}

TEST(ModelSplitTest, EveryImageLandsInExactlyOnePartAsPrimary) {
  const ColmapModel model = TwoRoomModel(40);
  SplitOptions options;
  options.max_images = 45;
  options.min_images = 5;
  const auto parts = SplitModelByCovisibility(model, options);

  // Primary counts partition the images; membership may overlap on top.
  uint32_t primary_total = 0;
  for (const auto& part : parts) primary_total += part.primary_images;
  EXPECT_EQ(primary_total, model.images.size())
      << "images must be owned exactly once, whatever the overlap";

  std::set<uint32_t> covered;
  for (const auto& part : parts) {
    for (const auto& image : part.model.images) covered.insert(image.image_id);
  }
  EXPECT_EQ(covered.size(), model.images.size()) << "no image may be dropped";
}

// Overlap at the seams is the point, not a side effect: each part has to
// cover its own side of a doorway, so the images that see through it belong
// to both.
TEST(ModelSplitTest, SeamImagesAppearInBothParts) {
  const ColmapModel model = TwoRoomModel(40, /*doorway_tracks=*/40);
  SplitOptions options;
  options.max_images = 45;
  options.min_images = 5;
  options.overlap_min_shared = 4;
  const auto parts = SplitModelByCovisibility(model, options);
  ASSERT_GE(parts.size(), 2u);

  std::map<uint32_t, int> appearances;
  for (const auto& part : parts) {
    for (const auto& image : part.model.images) ++appearances[image.image_id];
  }
  int shared = 0;
  for (const auto& [id, count] : appearances) {
    if (count > 1) ++shared;
  }
  EXPECT_GT(shared, 0) << "no image is shared, so the seam has no overlap";
}

TEST(ModelSplitTest, PartsRespectTheImageCap) {
  const ColmapModel model = TwoRoomModel(60);
  SplitOptions options;
  options.max_images = 30;
  options.min_images = 5;
  const auto parts = SplitModelByCovisibility(model, options);
  ASSERT_GE(parts.size(), 2u);

  for (const auto& part : parts) {
    // The cap governs a part's OWN images; context borrowed from a
    // neighbour rides on top, which is what overlap means.
    //
    // The bound is the cap PLUS the minimum size, and deliberately so: a
    // part below min_images is absorbed into its strongest neighbour rather
    // than left stranded, and the absorbed part can be up to min_images - 1
    // in size. A handful of orphaned images is not a training run, and
    // dropping them would lose coverage the capture paid for — so a part
    // slightly over capacity is accepted as the lesser problem. The
    // overshoot is bounded, which is what makes it safe to size against.
    EXPECT_LE(part.primary_images, 30u + 5u)
        << "part " << part.index << " over cap by more than one absorption";
  }
  // ...and the cap still has to bite, or the test proves nothing.
  EXPECT_GE(parts.size(), 3u) << "120 images at a cap of 30 should make parts";
}

TEST(ModelSplitTest, DeterministicForTheSameModel) {
  const ColmapModel model = TwoRoomModel(40);
  SplitOptions options;
  options.max_images = 45;
  options.min_images = 5;
  const auto a = SplitModelByCovisibility(model, options);
  const auto b = SplitModelByCovisibility(model, options);

  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    ASSERT_EQ(a[i].model.images.size(), b[i].model.images.size());
    for (size_t k = 0; k < a[i].model.images.size(); ++k) {
      EXPECT_EQ(a[i].model.images[k].image_id, b[i].model.images[k].image_id);
    }
  }
}

}  // namespace
}  // namespace bs
