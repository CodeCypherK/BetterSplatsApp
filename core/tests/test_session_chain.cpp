// Chained sessions: a facility too big for one capture, walked as several.
//
// The whole design rests on one claim — that a chain reads exactly like a
// single session, so nothing downstream of SessionReader had to change to
// reconstruct across one. These tests hold that claim up, and pin the three
// ways a chain can be malformed on disk, all of which are silent corruption
// rather than crashes if they get through:
//
//   - two sessions claiming the same frame id (every downstream id, from
//     COLMAP image ids to the exported filenames, would describe the wrong
//     picture)
//   - a parent that points back into the chain (a hang)
//   - a parent that is not there (a reconstruction covering less of the
//     building than the chain claims, with nothing saying so)

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "io/session_reader.h"
#include "io/session_writer.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

class SessionChainTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = (fs::temp_directory_path() /
             (std::string("bs_session_chain_") +
              ::testing::UnitTest::GetInstance()->current_test_info()->name()))
                .string();
    fs::remove_all(root_);
    fs::create_directories(root_);
  }
  void TearDown() override { fs::remove_all(root_); }

  static SessionInfo MakeInfo(const std::string& id,
                              const std::string& parent = "") {
    SessionInfo info;
    info.session_id = id;
    info.created_utc = "2026-08-14T12:00:00Z";
    info.device_model = "iPhone16,1";
    info.device_ios = "18.6";
    info.video_w = 1920;
    info.video_h = 1440;
    info.video_fps = 30;
    info.video_pixel_format = "420f";
    info.depth_w = 320;
    info.depth_h = 240;
    info.depth_format = "hdep";
    info.parent_session = parent;
    info.app_version = "0.1.0";
    return info;
  }

  static CalibrationInfo MakeCalibration() {
    CalibrationInfo c;
    c.ref_width = 1920;
    c.ref_height = 1440;
    c.intrinsics_session = {1456.0, 1456.0, 959.5, 719.5, 1920, 1440};
    return c;
  }

  static FrameMeta MakeMeta(uint32_t id) {
    FrameMeta m;
    m.frame_id = id;
    m.t_capture = 0.033 * id;
    m.t_depth = m.t_capture;
    m.intrinsics = {1456.0, 1456.0, 959.5, 719.5, 1920, 1440};
    m.depth_intrinsics = {242.7, 242.7, 159.5, 119.5, 320, 240};
    return m;
  }

  // Depth values encode the frame id, so a frame resolved from the wrong
  // session in the chain is caught rather than passing as "some depth".
  static DepthImage MakeDepth(uint32_t id) {
    DepthImage d;
    d.width = 4;
    d.height = 4;
    d.f16.assign(16, static_cast<uint16_t>(0x3C00 + id));
    return d;
  }

  // Writes a session holding frames [first_id, first_id + count).
  std::string WriteSession(const std::string& name, uint32_t first_id,
                           uint32_t count, const std::string& parent = "") {
    const std::string dir = (fs::path(root_) / name).string();
    SessionWriter writer;
    EXPECT_TRUE(SessionWriter::Create(dir, MakeInfo(name, parent),
                                      MakeCalibration(), writer));
    for (uint32_t i = 0; i < count; ++i) {
      const uint32_t id = first_id + i;
      EXPECT_TRUE(writer.WriteFrame(MakeMeta(id), {0xFF, 0xD8}, MakeDepth(id)));
    }
    return dir;
  }

  std::string root_;
};

TEST_F(SessionChainTest, UnchainedSessionIsUnaffected) {
  const std::string only = WriteSession("s1", 1, 3);
  auto reader = SessionReader::Open(only);
  ASSERT_TRUE(reader.has_value());
  EXPECT_FALSE(reader->is_chained());
  EXPECT_EQ(reader->chain().size(), 1u);
  EXPECT_EQ(reader->frame_ids(), (std::vector<uint32_t>{1, 2, 3}));
}

TEST_F(SessionChainTest, ChainReadsAsOneSession) {
  WriteSession("s1", 1, 3);
  WriteSession("s2", 4, 2, "s1");
  const std::string third = WriteSession("s3", 6, 2, "s2");

  auto reader = SessionReader::Open(third);
  ASSERT_TRUE(reader.has_value());
  EXPECT_TRUE(reader->is_chained());
  // Oldest first, so chain() reads in capture order.
  ASSERT_EQ(reader->chain().size(), 3u);
  EXPECT_EQ(fs::path(reader->chain()[0]).filename(), "s1");
  EXPECT_EQ(fs::path(reader->chain()[2]).filename(), "s3");

  EXPECT_EQ(reader->frame_ids(),
            (std::vector<uint32_t>{1, 2, 3, 4, 5, 6, 7}));

  // Every id resolves to whichever session actually holds it — this is the
  // property the final solve relies on without knowing chains exist.
  for (uint32_t id = 1; id <= 7; ++id) {
    const auto meta = reader->ReadMeta(id);
    ASSERT_TRUE(meta.has_value()) << "frame " << id;
    EXPECT_EQ(meta->frame_id, id);
    const auto depth = reader->ReadDepth(id);
    ASSERT_TRUE(depth.has_value()) << "frame " << id;
    EXPECT_EQ(depth->f16[0], static_cast<uint16_t>(0x3C00 + id));
    EXPECT_TRUE(fs::exists(reader->ImagePath(id))) << "frame " << id;
  }
  // info() stays the newest session's, which is the one being reconstructed.
  EXPECT_EQ(reader->info().session_id, "s3");
}

TEST_F(SessionChainTest, DuplicateFrameIdsAreRejected) {
  // Both sessions number from 1 — a capture-side numbering bug. Resolving it
  // to one of the two would silently reconstruct from the wrong images.
  WriteSession("s1", 1, 3);
  const std::string second = WriteSession("s2", 1, 3, "s1");
  EXPECT_FALSE(SessionReader::Open(second).has_value());
}

TEST_F(SessionChainTest, MissingParentReconstructsWhatIsPresent) {
  // The user deleted the earlier session, or has not copied it across yet.
  // Refusing outright would strand a perfectly reconstructable session.
  const std::string orphan = WriteSession("s2", 4, 2, "s_does_not_exist");
  auto reader = SessionReader::Open(orphan);
  ASSERT_TRUE(reader.has_value());
  EXPECT_FALSE(reader->is_chained());
  EXPECT_EQ(reader->frame_ids(), (std::vector<uint32_t>{4, 5}));
}

TEST_F(SessionChainTest, SelfReferenceDoesNotHang) {
  const std::string dir = WriteSession("s1", 1, 2, "s1");
  auto reader = SessionReader::Open(dir);
  ASSERT_TRUE(reader.has_value());
  EXPECT_EQ(reader->frame_ids(), (std::vector<uint32_t>{1, 2}));
}

TEST_F(SessionChainTest, CycleDoesNotHang) {
  // s1 -> s2 -> s1. Written by hand-editing or a bug; must terminate.
  WriteSession("s1", 1, 2, "s2");
  const std::string second = WriteSession("s2", 3, 2, "s1");
  auto reader = SessionReader::Open(second);
  ASSERT_TRUE(reader.has_value());
  // It walks s2 -> s1 -> (s2 again, refused), so both sessions are present
  // exactly once and the ids stay unique.
  EXPECT_EQ(reader->frame_ids(), (std::vector<uint32_t>{1, 2, 3, 4}));
}

TEST_F(SessionChainTest, ParentIsANameNotAPathSoAChainSurvivesBeingMoved) {
  WriteSession("s1", 1, 2);
  WriteSession("s2", 3, 2, "s1");

  // Move the whole lot somewhere else, as unzipping on another machine does.
  const std::string moved = (fs::path(root_) / "elsewhere").string();
  fs::create_directories(moved);
  fs::rename(fs::path(root_) / "s1", fs::path(moved) / "s1");
  fs::rename(fs::path(root_) / "s2", fs::path(moved) / "s2");

  auto reader = SessionReader::Open((fs::path(moved) / "s2").string());
  ASSERT_TRUE(reader.has_value());
  EXPECT_TRUE(reader->is_chained());
  EXPECT_EQ(reader->frame_ids(), (std::vector<uint32_t>{1, 2, 3, 4}));
}

}  // namespace
}  // namespace bs
