// Session writer -> reader identity, schema round-trips, RAW write-once
// enforcement.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "io/session_reader.h"
#include "io/session_writer.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

class SessionIoTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = (fs::temp_directory_path() /
            (std::string("bs_session_io_test_") +
             ::testing::UnitTest::GetInstance()->current_test_info()->name()))
               .string();
    fs::remove_all(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  static SessionInfo MakeInfo() {
    SessionInfo info;
    info.session_id = "test_session";
    info.created_utc = "2026-08-09T12:00:00Z";
    info.device_model = "iPhone16,1";
    info.device_ios = "18.6";
    info.video_w = 1920;
    info.video_h = 1440;
    info.video_fps = 30;
    info.video_pixel_format = "420f";
    info.depth_w = 320;
    info.depth_h = 240;
    info.depth_format = "hdep";
    info.ae_locked = true;
    info.awb_locked = true;
    info.regions = {{1, "Room 1", false}, {2, "Hallway", true}};
    info.app_version = "0.1.0";
    return info;
  }

  static CalibrationInfo MakeCalibration() {
    CalibrationInfo c;
    c.ref_width = 1920;
    c.ref_height = 1440;
    c.intrinsics_session = {1456.0, 1456.0, 959.5, 719.5, 1920, 1440};
    c.distortion_lut.magnification = {1.0f, 1.001f, 1.004f, 1.009f};
    c.distortion_lut.inverse = {1.0f, 0.999f, 0.996f, 0.991f};
    c.distortion_lut.center_x = 959.5;
    c.distortion_lut.center_y = 719.5;
    ColmapCameraModel m;
    m.model = "OPENCV";
    m.params = {1456.0, 1456.0, 959.5, 719.5, 0.01, -0.02, 0.0, 0.0};
    m.fit_residual_px_max = 0.12;
    c.colmap_model = m;
    return c;
  }

  static FrameMeta MakeMeta(uint32_t id) {
    FrameMeta m;
    m.frame_id = id;
    m.t_capture = 100.0 + id * 0.033;
    m.t_depth = m.t_capture - 0.002;
    m.intrinsics = {1456.0, 1456.0, 959.5, 719.5, 1920, 1440};
    m.depth_intrinsics = {242.7, 242.7, 159.5, 119.5, 320, 240};
    m.exposure = {0.008, 200.0, -0.3};
    m.quality = {312.0, 0.004};
    m.is_keyframe = (id % 2) == 1;
    m.store_reason = m.is_keyframe ? "kf" : "gate";
    return m;
  }

  static DepthImage MakeDepth(uint32_t id) {
    DepthImage d;
    d.width = 32;
    d.height = 24;
    d.f16.resize(32 * 24);
    for (size_t i = 0; i < d.f16.size(); ++i) {
      d.f16[i] = static_cast<uint16_t>((i * 7 + id * 13) & 0x3BFF);
    }
    return d;
  }

  std::string dir_;
};

TEST_F(SessionIoTest, WriteThenReadBackIdentity) {
  SessionWriter writer;
  ASSERT_TRUE(SessionWriter::Create(dir_, MakeInfo(), MakeCalibration(), writer));

  const std::vector<uint8_t> jpeg1 = {0xFF, 0xD8, 0xFF, 0xE0, 1, 2, 3, 0xFF, 0xD9};
  const std::vector<uint8_t> jpeg2 = {0xFF, 0xD8, 0xFF, 0xE0, 9, 8, 7, 6, 0xFF, 0xD9};
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(1), jpeg1, MakeDepth(1)));
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(2), jpeg2, MakeDepth(2)));

  GroundTruth gt;
  gt.poses.push_back({1, {0.9, 0.1, -0.2, 0.3}, {0.5, -0.25, 1.75}});
  ASSERT_TRUE(writer.WriteGroundTruth(gt));
  ASSERT_TRUE(writer.Finalize("2026-08-09T12:05:00Z"));

  auto reader = SessionReader::Open(dir_);
  ASSERT_TRUE(reader.has_value());

  EXPECT_EQ(reader->info().session_id, "test_session");
  EXPECT_EQ(reader->info().end_utc, "2026-08-09T12:05:00Z");
  EXPECT_EQ(reader->info().frame_count, 2u);
  ASSERT_EQ(reader->info().regions.size(), 2u);
  EXPECT_EQ(reader->info().regions[1].name, "Hallway");
  EXPECT_TRUE(reader->info().regions[1].renamed);
  EXPECT_EQ(reader->info().keyframe_ids, std::vector<uint32_t>{1});
  // Photometric lock flags survive the round trip.
  EXPECT_TRUE(reader->info().af_locked);
  EXPECT_TRUE(reader->info().ae_locked);
  EXPECT_TRUE(reader->info().awb_locked);

  ASSERT_EQ(reader->frame_ids(), (std::vector<uint32_t>{1, 2}));

  const auto meta = reader->ReadMeta(1);
  ASSERT_TRUE(meta.has_value());
  EXPECT_EQ(meta->frame_id, 1u);
  EXPECT_DOUBLE_EQ(meta->t_capture, 100.033);
  EXPECT_DOUBLE_EQ(meta->intrinsics.fx, 1456.0);
  EXPECT_DOUBLE_EQ(meta->depth_intrinsics.cy, 119.5);
  EXPECT_TRUE(meta->is_keyframe);
  EXPECT_EQ(meta->store_reason, "kf");
  EXPECT_DOUBLE_EQ(meta->exposure.iso, 200.0);
  EXPECT_DOUBLE_EQ(meta->quality.lap_var, 312.0);

  const auto depth = reader->ReadDepth(2);
  ASSERT_TRUE(depth.has_value());
  EXPECT_EQ(depth->f16, MakeDepth(2).f16);

  const auto bytes = reader->ReadImageBytes(2);
  ASSERT_TRUE(bytes.has_value());
  EXPECT_EQ(*bytes, jpeg2);

  const auto calib = reader->calibration();
  EXPECT_EQ(calib.ref_width, 1920);
  ASSERT_TRUE(calib.colmap_model.has_value());
  EXPECT_EQ(calib.colmap_model->model, "OPENCV");
  ASSERT_EQ(calib.colmap_model->params.size(), 8u);
  EXPECT_DOUBLE_EQ(calib.colmap_model->params[4], 0.01);
  ASSERT_EQ(calib.distortion_lut.magnification.size(), 4u);
  EXPECT_FLOAT_EQ(calib.distortion_lut.magnification[3], 1.009f);

  const auto read_gt = reader->ReadGroundTruth();
  ASSERT_TRUE(read_gt.has_value());
  ASSERT_EQ(read_gt->poses.size(), 1u);
  EXPECT_EQ(read_gt->poses[0].frame_id, 1u);
  EXPECT_DOUBLE_EQ(read_gt->poses[0].q[3], 0.3);
  EXPECT_DOUBLE_EQ(read_gt->poses[0].t[2], 1.75);
}

TEST_F(SessionIoTest, LegacySessionReportsPhotometricLocksAbsent) {
  // A session.json written before the app locked AE/AWB has no such keys.
  // Reading it must NOT claim locks that never happened.
  SessionWriter writer;
  ASSERT_TRUE(SessionWriter::Create(dir_, MakeInfo(), MakeCalibration(), writer));
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(1), {0xFF, 0xD8}, MakeDepth(1)));
  ASSERT_TRUE(writer.Finalize("2026-08-09T12:05:00Z"));

  const fs::path path = fs::path(dir_) / "session.json";
  std::ifstream in(path);
  std::string text((std::istreambuf_iterator<char>(in)),
                   std::istreambuf_iterator<char>());
  in.close();
  auto strip = [&text](const std::string& key) {
    const size_t at = text.find("\"" + key + "\"");
    ASSERT_NE(at, std::string::npos);
    const size_t end = text.find(',', at);
    ASSERT_NE(end, std::string::npos);
    text.erase(at, end - at + 1);
  };
  strip("ae_locked");
  strip("awb_locked");
  std::ofstream(path, std::ios::trunc) << text;

  auto reader = SessionReader::Open(dir_);
  ASSERT_TRUE(reader.has_value());
  EXPECT_TRUE(reader->info().af_locked);    // long-standing key, defaults true
  EXPECT_FALSE(reader->info().ae_locked);   // absent -> not locked
  EXPECT_FALSE(reader->info().awb_locked);
}

TEST_F(SessionIoTest, RawIsWriteOnce) {
  SessionWriter writer;
  ASSERT_TRUE(SessionWriter::Create(dir_, MakeInfo(), MakeCalibration(), writer));
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(1), {0xFF, 0xD8}, MakeDepth(1)));

  // Duplicate frame is refused.
  EXPECT_FALSE(writer.WriteFrame(MakeMeta(1), {0xFF, 0xD8}, MakeDepth(1)));

  // A second writer cannot claim an existing session directory.
  SessionWriter second;
  EXPECT_FALSE(SessionWriter::Create(dir_, MakeInfo(), MakeCalibration(), second));
}

TEST_F(SessionIoTest, OpenRejectsMissingOrCorrupt) {
  EXPECT_FALSE(SessionReader::Open(dir_ + "_nope").has_value());

  fs::create_directories(fs::path(dir_) / "frames");
  EXPECT_FALSE(SessionReader::Open(dir_).has_value());  // no session.json

  WriteTextFile((fs::path(dir_) / "session.json").string(), "{not json");
  EXPECT_FALSE(SessionReader::Open(dir_).has_value());

  // Wrong schema version is rejected.
  WriteTextFile((fs::path(dir_) / "session.json").string(),
                R"({"schema_version": 99, "session_id": "x"})");
  EXPECT_FALSE(SessionReader::Open(dir_).has_value());
}

TEST_F(SessionIoTest, ReaderEnumeratesFromDiskNotJson) {
  SessionWriter writer;
  ASSERT_TRUE(SessionWriter::Create(dir_, MakeInfo(), MakeCalibration(), writer));
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(3), {0xFF}, MakeDepth(3)));
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(10), {0xFF}, MakeDepth(10)));
  // No Finalize: session.json still says frame_count = 0.

  auto reader = SessionReader::Open(dir_);
  ASSERT_TRUE(reader.has_value());
  EXPECT_EQ(reader->info().frame_count, 0u);
  EXPECT_EQ(reader->frame_ids(), (std::vector<uint32_t>{3, 10}));
}

// A frame directory whose write did not finish is not a frame. The device
// writes meta.json last, so its absence is the marker; the app removes the
// debris when the write throws, but a jetsam kill or a flat battery between
// the first file and the last leaves it with nothing running to tidy up.
// Taking it as real puts a frame with no intrinsics, no timestamp and no pass
// tag into the solve.
TEST_F(SessionIoTest, IncompleteFrameDirectoryIsSkipped) {
  SessionWriter writer;
  ASSERT_TRUE(SessionWriter::Create(dir_, MakeInfo(), MakeCalibration(), writer));
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(1), {0xFF}, MakeDepth(1)));
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(2), {0xFF}, MakeDepth(2)));

  // Frame 3 got as far as its image and then the disk filled.
  const fs::path partial = fs::path(dir_) / "frames" / "000003";
  fs::create_directories(partial);
  { std::ofstream(partial / "image.jpg", std::ios::binary) << "\xFF\xD8"; }

  auto reader = SessionReader::Open(dir_);
  ASSERT_TRUE(reader.has_value());
  EXPECT_EQ(reader->frame_ids(), (std::vector<uint32_t>{1, 2}));
}

// The `pass` key is a contract between two languages: the Swift capture code
// writes it into meta.json and this reader decides the final solve on it. A
// rename on either side would not fail to compile anywhere — it would just
// quietly stop excluding the scout circuit, and the first symptom would be a
// splat trained on fast blurry walk-through frames. So the key name and both
// of its values are pinned literally rather than through the struct.
TEST_F(SessionIoTest, PassKeyIsWrittenLiterallyAndRoundTrips) {
  SessionWriter writer;
  ASSERT_TRUE(SessionWriter::Create(dir_, MakeInfo(), MakeCalibration(), writer));

  FrameMeta scout = MakeMeta(1);
  scout.pass = "scout";
  ASSERT_TRUE(writer.WriteFrame(scout, {0xFF}, MakeDepth(1)));
  FrameMeta capture = MakeMeta(2);  // default
  ASSERT_TRUE(writer.WriteFrame(capture, {0xFF}, MakeDepth(2)));

  const std::string raw =
      ReadTextFile((fs::path(dir_) / "frames" / "000001" / "meta.json").string());
  EXPECT_NE(raw.find("\"pass\""), std::string::npos) << raw;
  EXPECT_NE(raw.find("\"scout\""), std::string::npos) << raw;

  auto reader = SessionReader::Open(dir_);
  ASSERT_TRUE(reader.has_value());
  const auto read_scout = reader->ReadMeta(1);
  const auto read_capture = reader->ReadMeta(2);
  ASSERT_TRUE(read_scout.has_value());
  ASSERT_TRUE(read_capture.has_value());
  EXPECT_TRUE(read_scout->is_scout());
  EXPECT_FALSE(read_capture->is_scout());
  EXPECT_EQ(read_capture->pass, "capture");
}

// A meta.json with no `pass` at all — every session written before the scout
// pass existed — has to read as capture. Defaulting the other way would
// exclude an entire old session from its own reconstruction.
TEST_F(SessionIoTest, MissingPassDefaultsToCapture) {
  SessionWriter writer;
  ASSERT_TRUE(SessionWriter::Create(dir_, MakeInfo(), MakeCalibration(), writer));
  ASSERT_TRUE(writer.WriteFrame(MakeMeta(1), {0xFF}, MakeDepth(1)));

  const auto path =
      (fs::path(dir_) / "frames" / "000001" / "meta.json").string();
  std::string raw = ReadTextFile(path);
  const auto at = raw.find("\"pass\"");
  ASSERT_NE(at, std::string::npos);
  const auto end = raw.find(',', at);
  raw.erase(at, (end == std::string::npos ? raw.size() : end + 1) - at);
  WriteTextFile(path, raw);

  const auto meta = FrameMeta::FromJson(ReadTextFile(path));
  ASSERT_TRUE(meta.has_value());
  EXPECT_FALSE(meta->is_scout());
  EXPECT_EQ(meta->pass, "capture");
}

}  // namespace
}  // namespace bs
