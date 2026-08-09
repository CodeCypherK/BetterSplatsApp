#pragma once

#include <memory>
#include <optional>

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include "common/geometry.h"
#include "lidar/depth_processing.h"

namespace bs {

// Ceres cost functors shared by live local BA (M4) and the final global BA
// (M6). Parameter-block layout, used identically everywhere:
//   q: double[4], Eigen coeff order (x, y, z, w), EigenQuaternionManifold,
//      world-to-camera rotation
//   t: double[3], world-to-camera translation
//   X: double[3], world point
// Helpers below convert to/from SE3.

inline void PoseToBlocks(const SE3& pose, double* q_xyzw, double* t) {
  q_xyzw[0] = pose.q.x();
  q_xyzw[1] = pose.q.y();
  q_xyzw[2] = pose.q.z();
  q_xyzw[3] = pose.q.w();
  t[0] = pose.t.x();
  t[1] = pose.t.y();
  t[2] = pose.t.z();
}

inline SE3 PoseFromBlocks(const double* q_xyzw, const double* t) {
  SE3 pose;
  pose.q = Eigen::Quaterniond(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2])
               .normalized();
  pose.t = Eigen::Vector3d(t[0], t[1], t[2]);
  return pose;
}

// Standard 2-D reprojection residual (pixels) for an undistorted
// observation under fixed intrinsics. Robustified by the caller (Huber).
struct ReprojectionResidual {
  ReprojectionResidual(double u, double v, const Intrinsics& K)
      : u_(u), v_(v), fx_(K.fx), fy_(K.fy), cx_(K.cx), cy_(K.cy) {}

  template <typename T>
  bool operator()(const T* const q_xyzw, const T* const t, const T* const X,
                  T* residual) const {
    const Eigen::Quaternion<T> q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
    const Eigen::Matrix<T, 3, 1> x_world(X[0], X[1], X[2]);
    const Eigen::Matrix<T, 3, 1> x_cam =
        q * x_world + Eigen::Matrix<T, 3, 1>(t[0], t[1], t[2]);

    // Behind-camera points produce a large, smooth penalty instead of a
    // division blow-up; pruning removes them between rounds.
    const T z = x_cam.z() < T(1e-6) ? T(1e-6) : x_cam.z();
    residual[0] = T(fx_) * x_cam.x() / z + T(cx_) - T(u_);
    residual[1] = T(fy_) * x_cam.y() / z + T(cy_) - T(v_);
    return true;
  }

  static ceres::CostFunction* Create(double u, double v, const Intrinsics& K) {
    return new ceres::AutoDiffCostFunction<ReprojectionResidual, 2, 4, 3, 3>(
        new ReprojectionResidual(u, v, K));
  }

  double u_, v_, fx_, fy_, cx_, cy_;
};

// Pose-only variant with the world point fixed (live tracking refinement:
// map points stay put while the current pose polishes).
struct ReprojectionPoseOnlyResidual {
  ReprojectionPoseOnlyResidual(double u, double v, const Eigen::Vector3d& X,
                               const Intrinsics& K)
      : u_(u), v_(v), X_(X), fx_(K.fx), fy_(K.fy), cx_(K.cx), cy_(K.cy) {}

  template <typename T>
  bool operator()(const T* const q_xyzw, const T* const t, T* residual) const {
    const Eigen::Quaternion<T> q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
    const Eigen::Matrix<T, 3, 1> x_cam =
        q * X_.cast<T>() + Eigen::Matrix<T, 3, 1>(t[0], t[1], t[2]);
    const T z = x_cam.z() < T(1e-6) ? T(1e-6) : x_cam.z();
    residual[0] = T(fx_) * x_cam.x() / z + T(cx_) - T(u_);
    residual[1] = T(fy_) * x_cam.y() / z + T(cy_) - T(v_);
    return true;
  }

  static ceres::CostFunction* Create(double u, double v,
                                     const Eigen::Vector3d& X,
                                     const Intrinsics& K) {
    return new ceres::AutoDiffCostFunction<ReprojectionPoseOnlyResidual, 2, 4,
                                           3>(
        new ReprojectionPoseOnlyResidual(u, v, X, K));
  }

  double u_, v_;
  Eigen::Vector3d X_;
  double fx_, fy_, cx_, cy_;
};

// Differentiable lookup into one frame's sanitized depth map. Owned by the
// engine per depth frame; shared by all residuals against that frame.
class DepthLookup {
 public:
  explicit DepthLookup(const DepthFrame& frame)
      : width_(frame.width()),
        height_(frame.height()),
        K_(frame.K()),
        grid_(frame.SanitizedDepth().data(), 0, frame.height(), 0,
              frame.width()),
        interp_(grid_) {}

  // Grid2D is indexed (row, col) = (y, x).
  template <typename T>
  void Evaluate(const T& x, const T& y, T* depth) const {
    interp_.Evaluate(y, x, depth);
  }

  int width() const { return width_; }
  int height() const { return height_; }
  const Intrinsics& K() const { return K_; }

 private:
  int width_, height_;
  Intrinsics K_;
  ceres::Grid2D<double, 1> grid_;
  ceres::BiCubicInterpolator<ceres::Grid2D<double, 1>> interp_;
};

// One vetted point<->depth association, produced OUTSIDE the solver each
// optimization round (docs/ARCHITECTURE.md: LiDAR is evidence, and the
// association gate re-evaluates as geometry improves).
struct LidarAssociation {
  double sqrt_weight = 0;  // sqrt(w_edge * w_range * w_angle * w_tex)
  double sigma_m = 0.02;   // fixed per association round
};

// Confidence-weighted LiDAR depth residual:
//   r = sqrt(w) * (z(T, X) - D(pi_depth(T, X))) / sigma
// Gates (leaving the depth image, approaching the camera plane) zero the
// residual smoothly-enough for a round; the association layer re-vets
// between rounds. Cauchy loss at the call site prevents surviving outliers
// from dominating.
struct LidarDepthResidual {
  LidarDepthResidual(const DepthLookup* lookup, const LidarAssociation& assoc)
      : lookup_(lookup), assoc_(assoc) {}

  template <typename T>
  bool operator()(const T* const q_xyzw, const T* const t, const T* const X,
                  T* residual) const {
    const Eigen::Quaternion<T> q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
    const Eigen::Matrix<T, 3, 1> x_world(X[0], X[1], X[2]);
    const Eigen::Matrix<T, 3, 1> x_cam =
        q * x_world + Eigen::Matrix<T, 3, 1>(t[0], t[1], t[2]);

    if (x_cam.z() < T(0.05)) {
      residual[0] = T(0);
      return true;
    }
    const auto& K = lookup_->K();
    const T u = T(K.fx) * x_cam.x() / x_cam.z() + T(K.cx);
    const T v = T(K.fy) * x_cam.y() / x_cam.z() + T(K.cy);
    // Bicubic support needs one pixel of margin.
    if (u < T(1.0) || v < T(1.0) || u > T(lookup_->width() - 2.0) ||
        v > T(lookup_->height() - 2.0)) {
      residual[0] = T(0);
      return true;
    }
    T measured;
    lookup_->Evaluate(u, v, &measured);
    residual[0] = T(assoc_.sqrt_weight) * (x_cam.z() - measured) /
                  T(assoc_.sigma_m);
    return true;
  }

  static ceres::CostFunction* Create(const DepthLookup* lookup,
                                     const LidarAssociation& assoc) {
    return new ceres::AutoDiffCostFunction<LidarDepthResidual, 1, 4, 3, 3>(
        new LidarDepthResidual(lookup, assoc));
  }

  const DepthLookup* lookup_;
  LidarAssociation assoc_;
};

// Builds a vetted association for world point X observed by the frame whose
// depth is `frame` under pose T (world-to-camera). Returns nullopt when the
// point projects outside the map, the neighborhood is not bicubic-safe, the
// depth disagrees beyond `gate_sigmas`, or confidence vanishes.
//
// `w_tex` is the adaptive fusion term in [tex_floor, 1]: near 1 for
// low-texture/weakly-triangulated points (full LiDAR anchoring), near
// tex_floor for strongly-supported visual points (fine detail is never
// flattened onto the coarse depth map).
std::optional<LidarAssociation> MakeLidarAssociation(
    const DepthFrame& frame, const SE3& pose, const Eigen::Vector3d& X,
    double w_tex, double gate_sigmas);

}  // namespace bs
