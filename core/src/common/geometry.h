#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace bs {

// Rigid transform, stored world-to-camera in COLMAP convention throughout
// the entire codebase:  x_cam = q * x_world + t.
// (docs/FORMATS.md is normative; do not introduce camera-to-world storage.)
struct SE3 {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  static SE3 Identity() { return {}; }

  static SE3 FromWorldToCam(const Eigen::Quaterniond& q_wc,
                            const Eigen::Vector3d& t_wc) {
    return {q_wc.normalized(), t_wc};
  }

  // Convenience for building poses from a camera center + orientation.
  static SE3 FromCamToWorld(const Eigen::Quaterniond& q_cw,
                            const Eigen::Vector3d& center) {
    SE3 out;
    out.q = q_cw.conjugate().normalized();
    out.t = -(out.q * center);
    return out;
  }

  Eigen::Vector3d Apply(const Eigen::Vector3d& x_world) const {
    return q * x_world + t;
  }

  SE3 Inverse() const {
    SE3 out;
    out.q = q.conjugate();
    out.t = -(out.q * t);
    return out;
  }

  // this ∘ other: first apply `other`, then `this`.
  SE3 operator*(const SE3& other) const {
    SE3 out;
    out.q = (q * other.q).normalized();
    out.t = q * other.t + t;
    return out;
  }

  Eigen::Vector3d CameraCenter() const { return -(q.conjugate() * t); }

  // Optical axis direction in world coordinates (camera +Z).
  Eigen::Vector3d ViewDirWorld() const {
    return q.conjugate() * Eigen::Vector3d::UnitZ();
  }
};

// Pinhole intrinsics for a specific pixel resolution.
struct Intrinsics {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  int width = 0, height = 0;

  Eigen::Vector2d Project(const Eigen::Vector3d& x_cam) const {
    return {fx * x_cam.x() / x_cam.z() + cx, fy * x_cam.y() / x_cam.z() + cy};
  }

  Eigen::Vector3d Unproject(double u, double v, double depth) const {
    return {(u - cx) / fx * depth, (v - cy) / fy * depth, depth};
  }

  Intrinsics ScaledTo(int new_w, int new_h) const {
    const double sx = static_cast<double>(new_w) / width;
    const double sy = static_cast<double>(new_h) / height;
    return {fx * sx, fy * sy, cx * sx, cy * sy, new_w, new_h};
  }
};

inline double DegToRad(double deg) { return deg * M_PI / 180.0; }
inline double RadToDeg(double rad) { return rad * 180.0 / M_PI; }

// Rotation angle between two orientations, radians.
inline double AngularDistance(const Eigen::Quaterniond& a,
                              const Eigen::Quaterniond& b) {
  const double dot = std::min(1.0, std::abs(a.normalized().dot(b.normalized())));
  return 2.0 * std::acos(dot);
}

}  // namespace bs
