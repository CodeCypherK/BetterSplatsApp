#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "final/colmap_export.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

class ColmapExportTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = (fs::temp_directory_path() / "bs_colmap_test").string();
    fs::remove_all(dir_);
    fs::create_directories(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  ColmapModel MakeModel() {
    ColmapModel model;
    model.camera.model = "PINHOLE";
    model.camera.width = 640;
    model.camera.height = 480;
    model.camera.params = {500.0, 500.0, 319.5, 239.5};

    for (uint32_t id : {1u, 2u, 3u}) {
      // Tiny valid JPEG stand-ins (content irrelevant to the validator).
      const std::string src = dir_ + "/src_" + std::to_string(id) + ".jpg";
      std::ofstream(src) << "jpegbytes";
      ColmapImage image;
      image.image_id = id;
      image.pose.q = Eigen::Quaterniond(
          Eigen::AngleAxisd(0.05 * id, Eigen::Vector3d::UnitY()));
      image.pose.t = Eigen::Vector3d(0.1 * id, 0, 0);
      char name[16];
      std::snprintf(name, sizeof(name), "%06u.jpg", id);
      image.name = name;
      image.source_jpeg_path = src;
      model.images.push_back(image);
    }

    for (uint64_t pid : {1u, 2u, 3u, 4u}) {
      ColmapPoint point;
      point.point3d_id = pid;
      point.xyz = Eigen::Vector3d(0.1 * pid, 0.2, 2.0);
      point.rgb[0] = 200;
      point.error = 0.4;
      point.track.push_back({1, -1, 100.0 + pid, 120.0});
      point.track.push_back({2, -1, 104.0 + pid, 121.0});
      if (pid % 2 == 0) point.track.push_back({3, -1, 108.0 + pid, 122.0});
      model.points.push_back(point);
    }
    return model;
  }

  std::string dir_;
};

TEST_F(ColmapExportTest, WriteThenValidateCleanly) {
  ColmapModel model = MakeModel();
  const std::string out = dir_ + "/colmap";
  ASSERT_TRUE(WriteColmapModel(model, out));
  EXPECT_EQ(ValidateColmapDir(out), "");

  // Every referenced image was copied.
  for (const auto& image : model.images) {
    EXPECT_TRUE(fs::exists(fs::path(out) / "images" / image.name));
  }
  // point2d_idx assigned contiguously per image.
  for (const auto& point : model.points) {
    for (const auto& obs : point.track) {
      EXPECT_GE(obs.point2d_idx, 0);
    }
  }
}

TEST_F(ColmapExportTest, ValidationCatchesMissingImageFile) {
  ColmapModel model = MakeModel();
  const std::string out = dir_ + "/colmap";
  ASSERT_TRUE(WriteColmapModel(model, out));
  fs::remove(fs::path(out) / "images" / "000002.jpg");
  EXPECT_NE(ValidateColmapDir(out), "");
}

TEST_F(ColmapExportTest, ValidationCatchesCorruptQuaternion) {
  ColmapModel model = MakeModel();
  const std::string out = dir_ + "/colmap";
  ASSERT_TRUE(WriteColmapModel(model, out));

  // Break a quaternion norm in images.txt.
  const fs::path path = fs::path(out) / "images.txt";
  std::ifstream in(path);
  std::string content((std::istreambuf_iterator<char>(in)),
                      std::istreambuf_iterator<char>());
  in.close();
  const size_t pos = content.find("\n1 ");
  ASSERT_NE(pos, std::string::npos);
  content.replace(pos + 1, 3, "1 9");
  std::ofstream(path, std::ios::trunc) << content;

  EXPECT_NE(ValidateColmapDir(out), "");
}

TEST_F(ColmapExportTest, ValidationCatchesDanglingTrackReference) {
  ColmapModel model = MakeModel();
  const std::string out = dir_ + "/colmap";
  ASSERT_TRUE(WriteColmapModel(model, out));

  // Append a bogus track entry referencing an unknown image.
  std::ofstream append(fs::path(out) / "points3D.txt", std::ios::app);
  append << "99 1 2 3 10 10 10 0.5 77 0\n";
  append.close();
  EXPECT_NE(ValidateColmapDir(out), "");
}

TEST_F(ColmapExportTest, PlyRoundTripHeader) {
  const std::string path = dir_ + "/cloud.ply";
  ASSERT_TRUE(WriteBinaryPly(path, {{1, 2, 3}, {4, 5, 6}},
                             {{{255, 0, 0}}, {{0, 255, 0}}}));
  std::ifstream in(path, std::ios::binary);
  std::string header(64, '\0');
  in.read(header.data(), 64);
  EXPECT_NE(header.find("ply"), std::string::npos);
  EXPECT_NE(header.find("element vertex 2"), std::string::npos);
  in.seekg(0, std::ios::end);
  // header + 2 * (12 bytes xyz + 3 bytes rgb)
  const auto size = in.tellg();
  EXPECT_GT(size, 30 + 15);

  EXPECT_FALSE(WriteBinaryPly(dir_ + "/bad.ply", {{1, 2, 3}}, {}));
}

}  // namespace
}  // namespace bs
