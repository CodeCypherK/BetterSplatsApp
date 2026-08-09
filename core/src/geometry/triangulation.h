#pragma once

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "common/geometry.h"

namespace bs {

// Multi-view DLT triangulation and the quality measures used everywhere a
// candidate point is evaluated (bootstrap, mapper, final solve, readiness).

struct Observation2D {
  SE3 pose;        // world-to-camera of the observing view
  Intrinsics K;    // intrinsics for the observed pixel coordinates
  cv::Point2f px;  // undistorted pixel observation
};

struct TriangulationResult {
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double max_reproj_error_px = 0;
  double mean_reproj_error_px = 0;
  // Largest pairwise angle between observing rays, degrees.
  double max_angle_deg = 0;
  int behind_camera_count = 0;

  bool InFrontOfAll() const { return behind_camera_count == 0; }
};

// Linear DLT over >= 2 observations. Returns nullopt when the system is
// numerically degenerate. Callers apply their own quality gates on the
// returned errors/angle.
std::optional<TriangulationResult> TriangulatePoint(
    const std::vector<Observation2D>& observations);

// Convenience for the two-view case.
std::optional<TriangulationResult> TriangulateTwoView(
    const SE3& pose_a, const SE3& pose_b, const Intrinsics& K,
    const cv::Point2f& px_a, const cv::Point2f& px_b);

// Angle between the rays from two camera centers to a world point, degrees.
double TriangulationAngleDeg(const Eigen::Vector3d& center_a,
                             const Eigen::Vector3d& center_b,
                             const Eigen::Vector3d& point);

}  // namespace bs
