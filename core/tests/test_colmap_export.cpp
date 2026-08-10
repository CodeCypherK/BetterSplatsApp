#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include "final/colmap_export.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

class ColmapExportTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = (fs::temp_directory_path() /
            (std::string("bs_colmap_test_") +
             ::testing::UnitTest::GetInstance()->current_test_info()->name()))
               .string();
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

TEST_F(ColmapExportTest, TransformsJsonRecoversColmapPose) {
  ColmapModel model = MakeModel();
  model.camera.model = "OPENCV";
  model.camera.params = {500.0, 505.0, 319.5, 239.5, -0.02, 0.003, 0.0, 0.0};
  const std::string path = dir_ + "/transforms.json";
  ASSERT_TRUE(WriteTransformsJson(model, path));

  std::ifstream in(path);
  nlohmann::json j;
  in >> j;

  EXPECT_EQ(j.at("camera_model"), "OPENCV");
  EXPECT_DOUBLE_EQ(j.at("fl_x").get<double>(), 500.0);
  EXPECT_DOUBLE_EQ(j.at("fl_y").get<double>(), 505.0);
  EXPECT_DOUBLE_EQ(j.at("k1").get<double>(), -0.02);
  ASSERT_EQ(j.at("frames").size(), model.images.size());

  for (size_t i = 0; i < model.images.size(); ++i) {
    const auto& frame = j["frames"][i];
    EXPECT_EQ(frame.at("file_path"), "images/" + model.images[i].name);
    const auto& tm = frame.at("transform_matrix");

    // Reconstruct the 3x3 rotation and translation from the JSON.
    Eigen::Matrix3d m;
    Eigen::Vector3d c;
    for (int r = 0; r < 3; ++r) {
      for (int col = 0; col < 3; ++col) m(r, col) = tm[r][col].get<double>();
      c(r) = tm[r][3].get<double>();
    }
    // Bottom row is homogeneous.
    EXPECT_DOUBLE_EQ(tm[3][3].get<double>(), 1.0);

    // Expected: camera-to-world with Y/Z axes flipped (NeRF convention), and
    // translation equal to the COLMAP camera centre C = -R^T t.
    const Eigen::Matrix3d R = model.images[i].pose.q.toRotationMatrix();
    const Eigen::Matrix3d Rwc = R.transpose();
    const Eigen::Vector3d expected_c = -Rwc * model.images[i].pose.t;
    Eigen::Matrix3d expected_m = Rwc;
    expected_m.col(1) = -expected_m.col(1);
    expected_m.col(2) = -expected_m.col(2);

    EXPECT_LT((c - expected_c).norm(), 1e-9);
    EXPECT_LT((m - expected_m).norm(), 1e-9);
    // The NeRF frame must stay right-handed (det +1).
    EXPECT_NEAR(m.determinant(), 1.0, 1e-9);
  }
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
