#pragma once

#include <vector>

#include "common/geometry.h"

namespace bs {

// Aligning reconstruction components.
//
// When the live pass never posed a stretch of frames, the final solve
// rebuilds them as their own component in their own gauge, then has to put
// that component back into the main model's frame. Both gauges are metric
// once LiDAR has anchored them, so the transform between them is rigid; the
// scale is estimated only when a component's LiDAR anchoring failed.

struct Similarity {
  double scale = 1.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  Eigen::Vector3d Apply(const Eigen::Vector3d& x) const {
    return scale * (R * x) + t;
  }
};

// Closed-form (Umeyama) fit mapping `src` onto `dst`. With
// `allow_scale = false` the scale is held at 1 and only rotation and
// translation are fitted. Needs at least 3 non-degenerate correspondences.
bool SimilarityFromCorrespondences(const std::vector<Eigen::Vector3d>& src,
                                   const std::vector<Eigen::Vector3d>& dst,
                                   Similarity& out, bool allow_scale = true);

// RANSAC + local refinement around the above. Component overlaps carry many
// spurious correspondences (a track triangulated badly in either gauge), and
// a single outlier drags a closed-form fit arbitrarily far.
bool RobustSimilarity(const std::vector<Eigen::Vector3d>& src,
                      const std::vector<Eigen::Vector3d>& dst,
                      double inlier_radius_m, int min_inliers, bool allow_scale,
                      Similarity& out, int& inlier_count);

// Moves a world-to-camera pose from a component's gauge into the target
// gauge. The camera centre transforms like a point; the rotation composes
// with the similarity's rotation (a scale never enters a pose's rotation).
SE3 TransformPose(const SE3& pose_in_component, const Similarity& sim);

}  // namespace bs
