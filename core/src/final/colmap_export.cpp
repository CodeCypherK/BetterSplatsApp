#include "final/colmap_export.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include "common/log.h"

namespace bs {

namespace fs = std::filesystem;

bool WriteColmapModel(ColmapModel& model, const std::string& out_dir) {
  std::error_code ec;
  fs::create_directories(fs::path(out_dir) / "images", ec);
  if (ec) return false;

  // Assign per-image 2D point lists from the tracks. Only tracked features
  // are exported (valid COLMAP: POINTS2D need not cover every detection).
  struct Point2D {
    double x, y;
    uint64_t point3d_id;
  };
  std::map<uint32_t, std::vector<Point2D>> image_points;
  for (auto& point : model.points) {
    for (auto& obs : point.track) {
      auto& list = image_points[obs.image_id];
      obs.point2d_idx = static_cast<int>(list.size());
      list.push_back({obs.x, obs.y, point.point3d_id});
    }
  }

  // cameras.txt
  {
    std::ofstream out(fs::path(out_dir) / "cameras.txt", std::ios::trunc);
    if (!out) return false;
    out << "# Camera list with one line of data per camera:\n"
        << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
        << "# Number of cameras: 1\n";
    out << model.camera.camera_id << " " << model.camera.model << " "
        << model.camera.width << " " << model.camera.height;
    out.precision(12);
    for (const double p : model.camera.params) out << " " << p;
    out << "\n";
    if (!out) return false;
  }

  // images.txt
  {
    std::ofstream out(fs::path(out_dir) / "images.txt", std::ios::trunc);
    if (!out) return false;
    out << "# Image list with two lines of data per image:\n"
        << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
        << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n"
        << "# Number of images: " << model.images.size() << "\n";
    out.precision(12);
    for (const auto& image : model.images) {
      const Eigen::Quaterniond q = image.pose.q.normalized();
      out << image.image_id << " " << q.w() << " " << q.x() << " " << q.y()
          << " " << q.z() << " " << image.pose.t.x() << " " << image.pose.t.y()
          << " " << image.pose.t.z() << " " << image.camera_id << " "
          << image.name << "\n";
      const auto it = image_points.find(image.image_id);
      if (it != image_points.end()) {
        bool first = true;
        for (const auto& p : it->second) {
          if (!first) out << " ";
          first = false;
          out << p.x << " " << p.y << " " << p.point3d_id;
        }
      }
      out << "\n";
    }
    if (!out) return false;
  }

  // points3D.txt
  {
    std::ofstream out(fs::path(out_dir) / "points3D.txt", std::ios::trunc);
    if (!out) return false;
    out << "# 3D point list with one line of data per point:\n"
        << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, "
           "TRACK[] as (IMAGE_ID, POINT2D_IDX)\n"
        << "# Number of points: " << model.points.size() << "\n";
    out.precision(12);
    for (const auto& point : model.points) {
      out << point.point3d_id << " " << point.xyz.x() << " " << point.xyz.y()
          << " " << point.xyz.z() << " " << static_cast<int>(point.rgb[0])
          << " " << static_cast<int>(point.rgb[1]) << " "
          << static_cast<int>(point.rgb[2]) << " " << point.error;
      for (const auto& obs : point.track) {
        out << " " << obs.image_id << " " << obs.point2d_idx;
      }
      out << "\n";
    }
    if (!out) return false;
  }

  // images/ — byte-identical copies of the stored RAW JPEGs.
  for (const auto& image : model.images) {
    if (image.source_jpeg_path.empty()) continue;
    fs::copy_file(image.source_jpeg_path,
                  fs::path(out_dir) / "images" / image.name,
                  fs::copy_options::overwrite_existing, ec);
    if (ec) {
      BS_LOGE("colmap", "copy %s failed: %s", image.name.c_str(),
              ec.message().c_str());
      return false;
    }
  }
  return true;
}

std::string ValidateColmapDir(const std::string& dir) {
  auto fail = [](const std::string& why) { return why; };

  // cameras.txt
  {
    std::ifstream in(fs::path(dir) / "cameras.txt");
    if (!in) return fail("cameras.txt missing");
    std::string line;
    int cameras = 0;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      int id, w, h;
      std::string model;
      if (!(ss >> id >> model >> w >> h)) return fail("cameras.txt malformed");
      if (w <= 0 || h <= 0) return fail("cameras.txt bad dims");
      int params = 0;
      double v;
      while (ss >> v) ++params;
      if (model == "PINHOLE" && params != 4) return fail("PINHOLE needs 4 params");
      if (model == "OPENCV" && params != 8) return fail("OPENCV needs 8 params");
      ++cameras;
    }
    if (cameras != 1) return fail("expected exactly 1 camera");
  }

  // images.txt: collect per-image point counts + point3D references.
  std::map<uint32_t, int> image_point_counts;
  std::set<uint64_t> referenced_points;
  {
    std::ifstream in(fs::path(dir) / "images.txt");
    if (!in) return fail("images.txt missing");
    std::string line;
    std::set<uint32_t> ids;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      uint32_t image_id;
      double qw, qx, qy, qz, tx, ty, tz;
      int camera_id;
      std::string name;
      if (!(ss >> image_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >>
            camera_id >> name)) {
        return fail("images.txt pose line malformed");
      }
      if (!ids.insert(image_id).second) return fail("duplicate image id");
      const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
      if (std::abs(norm - 1.0) > 1e-6) return fail("quaternion not normalized");
      if (!(fs::exists(fs::path(dir) / "images" / name))) {
        return fail("missing image file: " + name);
      }

      if (!std::getline(in, line)) return fail("missing POINTS2D line");
      std::istringstream ps(line);
      double x, y;
      long long pid;
      int count = 0;
      while (ps >> x >> y >> pid) {
        if (pid >= 0) referenced_points.insert(static_cast<uint64_t>(pid));
        ++count;
      }
      image_point_counts[image_id] = count;
    }
    if (ids.empty()) return fail("no images");
  }

  // points3D.txt: ids unique; tracks reference valid image/point2d slots.
  {
    std::ifstream in(fs::path(dir) / "points3D.txt");
    if (!in) return fail("points3D.txt missing");
    std::string line;
    std::set<uint64_t> ids;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      uint64_t pid;
      double x, y, z, err;
      int r, g, b;
      if (!(ss >> pid >> x >> y >> z >> r >> g >> b >> err)) {
        return fail("points3D.txt line malformed");
      }
      if (!ids.insert(pid).second) return fail("duplicate point id");
      uint32_t image_id;
      int p2d;
      int track_len = 0;
      while (ss >> image_id >> p2d) {
        const auto it = image_point_counts.find(image_id);
        if (it == image_point_counts.end()) {
          return fail("track references unknown image");
        }
        if (p2d < 0 || p2d >= it->second) {
          return fail("track references out-of-range POINT2D_IDX");
        }
        ++track_len;
      }
      if (track_len < 2) return fail("point with track shorter than 2");
    }
    for (const uint64_t pid : referenced_points) {
      if (!ids.count(pid)) return fail("images.txt references unknown point");
    }
    if (ids.empty()) return fail("no points");
  }
  return "";
}

bool WriteTransformsJson(const ColmapModel& model, const std::string& path) {
  std::ofstream out(path, std::ios::trunc);
  if (!out) return false;
  out.setf(std::ios::fmtflags(0), std::ios::floatfield);
  out.precision(12);

  const ColmapCamera& cam = model.camera;
  auto param = [&](size_t i) { return i < cam.params.size() ? cam.params[i] : 0.0; };
  const double fx = param(0), fy = param(1), cx = param(2), cy = param(3);
  // OPENCV model carries k1 k2 p1 p2 after fx fy cx cy; PINHOLE has none.
  const bool opencv = cam.model == "OPENCV";
  const double k1 = opencv ? param(4) : 0.0;
  const double k2 = opencv ? param(5) : 0.0;
  const double p1 = opencv ? param(6) : 0.0;
  const double p2 = opencv ? param(7) : 0.0;

  out << "{\n";
  out << "  \"camera_model\": \"OPENCV\",\n";
  out << "  \"fl_x\": " << fx << ",\n  \"fl_y\": " << fy << ",\n";
  out << "  \"cx\": " << cx << ",\n  \"cy\": " << cy << ",\n";
  out << "  \"w\": " << cam.width << ",\n  \"h\": " << cam.height << ",\n";
  out << "  \"k1\": " << k1 << ",\n  \"k2\": " << k2 << ",\n";
  out << "  \"p1\": " << p1 << ",\n  \"p2\": " << p2 << ",\n";
  out << "  \"frames\": [\n";

  for (size_t i = 0; i < model.images.size(); ++i) {
    const ColmapImage& img = model.images[i];
    // world->camera (COLMAP, +Z forward/+Y down) -> camera->world, then flip
    // the Y and Z axes to reach the OpenGL/NeRF convention (+Y up, -Z fwd).
    const Eigen::Matrix3d R = img.pose.q.toRotationMatrix();
    const Eigen::Matrix3d Rwc = R.transpose();
    const Eigen::Vector3d c = -Rwc * img.pose.t;
    Eigen::Matrix3d m = Rwc;
    m.col(1) = -m.col(1);
    m.col(2) = -m.col(2);

    out << "    {\n";
    out << "      \"file_path\": \"images/" << img.name << "\",\n";
    out << "      \"transform_matrix\": [\n";
    for (int r = 0; r < 3; ++r) {
      out << "        [" << m(r, 0) << ", " << m(r, 1) << ", " << m(r, 2)
          << ", " << c(r) << "],\n";
    }
    out << "        [0.0, 0.0, 0.0, 1.0]\n";
    out << "      ]\n";
    out << "    }" << (i + 1 < model.images.size() ? "," : "") << "\n";
  }

  out << "  ]\n}\n";
  return static_cast<bool>(out);
}

bool WriteBinaryPly(const std::string& path,
                    const std::vector<Eigen::Vector3f>& points,
                    const std::vector<std::array<uint8_t, 3>>& colors) {
  if (points.size() != colors.size()) return false;
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  out << "ply\nformat binary_little_endian 1.0\n"
      << "element vertex " << points.size() << "\n"
      << "property float x\nproperty float y\nproperty float z\n"
      << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
      << "end_header\n";
  for (size_t i = 0; i < points.size(); ++i) {
    out.write(reinterpret_cast<const char*>(points[i].data()), 12);
    out.write(reinterpret_cast<const char*>(colors[i].data()), 3);
  }
  return static_cast<bool>(out);
}

}  // namespace bs
