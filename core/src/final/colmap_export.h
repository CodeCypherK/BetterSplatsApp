#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/geometry.h"

namespace bs {

// COLMAP text-model writer (docs/FORMATS.md, "COLMAP export conventions").
// Only reliable, tracked structure is exported: points3D.txt holds feature
// tracks exclusively; fused LiDAR ships separately as dense.ply.

struct ColmapCamera {
  int camera_id = 1;
  std::string model = "PINHOLE";  // or "OPENCV"
  int width = 0;
  int height = 0;
  std::vector<double> params;  // fx fy cx cy [k1 k2 p1 p2]
};

struct ColmapObservation {
  uint32_t image_id = 0;
  // Index into that image's POINTS2D list — assigned by the writer.
  int point2d_idx = -1;
  // Pixel coordinates in the ORIGINAL (stored, distorted) image.
  double x = 0, y = 0;
};

struct ColmapPoint {
  uint64_t point3d_id = 0;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  uint8_t rgb[3] = {128, 128, 128};
  double error = 0;
  std::vector<ColmapObservation> track;  // point2d_idx filled by the writer
};

struct ColmapImage {
  uint32_t image_id = 0;
  SE3 pose;  // world-to-camera
  int camera_id = 1;
  std::string name;  // filename inside images/
  std::string source_jpeg_path;  // copied into images/
};

struct ColmapModel {
  ColmapCamera camera;
  std::vector<ColmapImage> images;
  std::vector<ColmapPoint> points;
};

// Writes cameras.txt, images.txt, points3D.txt into `out_dir` and copies
// the referenced JPEGs into out_dir/images/. Fills point2d_idx across the
// model (mutates `model`). Returns false on IO failure.
bool WriteColmapModel(ColmapModel& model, const std::string& out_dir);

// Structural self-check of a written model directory: parses the three
// files, verifies id uniqueness, track cross-references, and quaternion
// normalization. Returns an empty string when valid, else a description.
std::string ValidateColmapDir(const std::string& dir);

// Binary little-endian PLY point cloud writer (x y z r g b).
bool WriteBinaryPly(const std::string& path,
                    const std::vector<Eigen::Vector3f>& points,
                    const std::vector<std::array<uint8_t, 3>>& colors);

// Writes a nerfstudio/instant-ngp `transforms.json` beside the COLMAP model
// so the export drops straight into gsplat/nerfstudio with no conversion.
// Poses are camera-to-world in the OpenGL/NeRF convention (camera looks down
// -Z, +Y up); intrinsics use the shared OPENCV model. file_path entries point
// at images/<name>. Returns false on IO failure.
bool WriteTransformsJson(const ColmapModel& model, const std::string& path);

}  // namespace bs
