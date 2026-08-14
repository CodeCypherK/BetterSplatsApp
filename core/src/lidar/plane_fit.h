#pragma once

#include "common/geometry.h"
#include "lidar/depth_processing.h"

namespace bs {

// Fitting a plane to a single LiDAR depth frame, in that frame's own camera
// coordinates.
//
// This exists so the floor can be *measured* rather than inferred. Working
// out which plane is the floor after the fact, from sparse reconstructed
// points, is a genuinely hard problem: floors are low on texture and seen at
// a glancing angle, a ceiling satisfies every geometric test a floor does,
// and a scan holding one big plane is ambiguous outright. Pointing the phone
// at the floor for a moment answers all of it at once — the user knows which
// surface it is, and the depth sensor sees a couple of hundred thousand
// samples of it from a metre away.
//
// The result is expressed in CAMERA coordinates and paired with a frame id,
// never in world coordinates. That keeps it a measurement: an immutable fact
// about what the sensor saw, which the current pose estimate turns into a
// world plane whenever one is needed. A calibration stored in world
// coordinates would silently bake in whatever the live tracker believed at
// the time, and go stale the moment the final solve moved that frame.

struct DepthPlaneOptions {
  double inlier_m = 0.03;
  int ransac_iterations = 400;
  double min_confidence = 0.5;   // per-sample LiDAR confidence floor
  double min_range_m = 0.3;      // ignore samples nearer than this
  double max_range_m = 4.0;      // ...and further than this
  int sample_stride = 2;         // subsample the depth map
  double min_inlier_frac = 0.55; // of usable samples, or the fit is refused
  int min_inliers = 400;
};

struct DepthPlane {
  bool valid = false;
  // Camera-space plane: points X satisfy normal.dot(X) + offset = 0. The
  // normal faces the camera, so offset is the camera's height above it.
  Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
  double offset = 0;

  int inliers = 0;
  double inlier_frac = 0;
  double rmse_m = 0;
  // Angle between the plane normal and the camera's optical axis. Near 0
  // means the phone was pointed straight down at the floor, which is what a
  // calibration prompt should be asking for.
  double incidence_deg = 0;
};

// Fits the dominant plane in `depth`. Returns valid = false when no plane
// takes a clear majority of the usable samples — pointing at a cluttered
// corner should refuse rather than return a confident answer about nothing.
DepthPlane FitDepthPlane(const DepthFrame& depth,
                         const DepthPlaneOptions& options = {});

}  // namespace bs
