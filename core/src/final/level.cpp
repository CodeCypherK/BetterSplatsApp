#include "final/level.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

#include <Eigen/Eigenvalues>

#include "common/log.h"

namespace bs {

namespace {

struct Plane {
  Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
  double offset = 0;  // points on the plane satisfy normal.dot(X) + offset = 0
  double SignedDistance(const Eigen::Vector3d& x) const {
    return normal.dot(x) + offset;
  }
};

// Least-squares plane through a set of points (smallest-eigenvalue normal).
bool FitPlane(const std::vector<Eigen::Vector3d>& points,
              const std::vector<int>& indices, Plane& out) {
  if (indices.size() < 3) return false;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const int i : indices) centroid += points[i];
  centroid /= static_cast<double>(indices.size());

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const int i : indices) {
    const Eigen::Vector3d d = points[i] - centroid;
    cov += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) return false;
  out.normal = solver.eigenvectors().col(0).normalized();
  out.offset = -out.normal.dot(centroid);
  return true;
}

// Rotation taking `from` onto `to`, both unit.
Eigen::Quaterniond RotationBetween(const Eigen::Vector3d& from,
                                   const Eigen::Vector3d& to) {
  return Eigen::Quaterniond::FromTwoVectors(from, to);
}

}  // namespace

Leveling EstimateLeveling(const std::vector<Eigen::Vector3d>& points,
                          const std::vector<Eigen::Vector3d>& camera_centres,
                          const LevelingOptions& options) {
  Leveling out;
  out.transform = SE3::Identity();
  if (points.size() < static_cast<size_t>(options.min_floor_inliers) ||
      camera_centres.empty()) {
    return out;
  }

  Eigen::Vector3d camera_centroid = Eigen::Vector3d::Zero();
  for (const auto& c : camera_centres) camera_centroid += c;
  camera_centroid /= static_cast<double>(camera_centres.size());

  // --- an up-direction prior from the camera trajectory ---
  //
  // A phone carried around a room stays at roughly one height, so the plane
  // through the camera centres is horizontal and its normal is up. Only the
  // AXIS is used, never the sign — which way is up gets settled later by
  // which side the cameras are on. A trajectory too straight to define a
  // plane yields no prior, and the search falls back to unconstrained.
  Eigen::Vector3d prior_up = Eigen::Vector3d::Zero();
  if (camera_centres.size() >= 8) {
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& c : camera_centres) {
      const Eigen::Vector3d d = c - camera_centroid;
      cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() == Eigen::Success) {
      const Eigen::Vector3d eigenvalues = solver.eigenvalues();
      // Planar, not linear: the second-smallest spread has to be a real
      // fraction of the largest, or the path is a line and its "plane" is
      // whatever noise decided.
      if (eigenvalues(2) > 0 &&
          eigenvalues(1) / eigenvalues(2) > options.prior_min_planarity) {
        prior_up = solver.eigenvectors().col(0).normalized();
      }
    }
  }
  const double prior_min_cos =
      std::cos(DegToRad(std::clamp(options.prior_max_angle_deg, 1.0, 90.0)));

  // --- find the floor ---
  //
  // Fixed seed: the same reconstruction must level the same way every run,
  // or a re-export silently moves the world.
  std::mt19937 rng(0xF100Du);
  std::uniform_int_distribution<size_t> pick(0, points.size() - 1);

  // Two candidates are tracked, on opposite sides of the cameras: whichever
  // way the first qualifying plane happens to face, and its opposite. In a
  // room those are the floor and the ceiling, and telling them apart needs
  // both.
  Plane best, opposite;
  int best_inliers = 0, opposite_inliers = 0;
  double best_height = 0, opposite_height = 0;
  bool have_reference = false;
  Eigen::Vector3d reference_normal = Eigen::Vector3d::UnitY();
  for (int iter = 0; iter < options.ransac_iterations; ++iter) {
    const size_t a = pick(rng), b = pick(rng), c = pick(rng);
    if (a == b || b == c || a == c) continue;
    const Eigen::Vector3d e1 = points[b] - points[a];
    const Eigen::Vector3d e2 = points[c] - points[a];
    const Eigen::Vector3d n = e1.cross(e2);
    if (n.norm() < 1e-9) continue;

    Plane plane;
    plane.normal = n.normalized();
    plane.offset = -plane.normal.dot(points[a]);

    // Cheapest test first, and the one that does most of the work: a
    // candidate pointing nowhere near up cannot be a floor.
    if (prior_up.squaredNorm() > 0 &&
        std::abs(plane.normal.dot(prior_up)) < prior_min_cos) {
      continue;
    }

    // Orient "up": the cameras are above the floor, by definition of a floor.
    const double camera_side = plane.SignedDistance(camera_centroid);
    if (camera_side < 0) {
      plane.normal = -plane.normal;
      plane.offset = -plane.offset;
    }
    const double height = std::abs(camera_side);

    // A floor is the plane a walking phone stays a sensible distance above.
    // This is what separates it from the ceiling and from every wall, none
    // of which satisfy it, and it costs nothing to check before counting
    // inliers.
    if (height < options.min_camera_height_m ||
        height > options.max_camera_height_m) {
      continue;
    }
    // Every camera has to be above it, not just their average — a wall
    // catches the average of a trajectory that runs alongside it.
    bool all_above = true;
    for (const auto& centre : camera_centres) {
      if (plane.SignedDistance(centre) < 0.2) {
        all_above = false;
        break;
      }
    }
    if (!all_above) continue;

    int inliers = 0;
    int below = 0;
    for (const auto& p : points) {
      const double d = plane.SignedDistance(p);
      if (std::abs(d) < options.floor_inlier_m) {
        ++inliers;
      } else if (d < 0) {
        ++below;
      }
    }
    // Nothing lives under a floor. A plane raked through the room can catch
    // as many points as the floor does and still keep every camera above
    // it, but it always buries a large part of the scene — which the floor
    // never does.
    if (static_cast<double>(below) >
        options.max_below_frac * static_cast<double>(points.size())) {
      continue;
    }
    if (!have_reference) {
      have_reference = true;
      reference_normal = plane.normal;
    }
    if (plane.normal.dot(reference_normal) >= 0) {
      if (inliers > best_inliers) {
        best_inliers = inliers;
        best = plane;
        best_height = height;
      }
    } else if (inliers > opposite_inliers) {
      opposite_inliers = inliers;
      opposite = plane;
      opposite_height = height;
    }
  }

  // Of two opposed surfaces, the floor is the farther one — a phone is
  // carried above the mid-height of a room. Only swap when the gap is big
  // enough to mean something; when they are too alike, say so and stop.
  if (opposite_inliers >= options.min_floor_inliers &&
      best_inliers >= options.min_floor_inliers) {
    const double gap = std::abs(best_height - opposite_height);
    if (gap < options.min_floor_ceiling_gap_m) {
      BS_LOGI("level",
              "two opposed planes %.2f m and %.2f m from the cameras are too "
              "alike to call floor from ceiling; leaving the reconstruction "
              "as solved",
              best_height, opposite_height);
      return out;
    }
    if (opposite_height > best_height) {
      std::swap(best, opposite);
      std::swap(best_inliers, opposite_inliers);
      std::swap(best_height, opposite_height);
    }
  }

  if (best_inliers < options.min_floor_inliers) {
    BS_LOGI("level", "no floor plane found (best %d inliers); leaving the "
                     "reconstruction as solved",
            best_inliers);
    return out;
  }

  // Refit on the inliers so the normal is not hostage to three sample points.
  {
    std::vector<int> inlier_indices;
    inlier_indices.reserve(best_inliers);
    for (size_t i = 0; i < points.size(); ++i) {
      if (std::abs(best.SignedDistance(points[i])) < options.floor_inlier_m) {
        inlier_indices.push_back(static_cast<int>(i));
      }
    }
    Plane refined;
    if (FitPlane(points, inlier_indices, refined)) {
      if (refined.SignedDistance(camera_centroid) < 0) {
        refined.normal = -refined.normal;
        refined.offset = -refined.offset;
      }
      best = refined;
    }
    double sq = 0;
    for (const int i : inlier_indices) {
      const double d = best.SignedDistance(points[i]);
      sq += d * d;
    }
    out.floor_inliers = static_cast<int>(inlier_indices.size());
    out.floor_rmse_m =
        std::sqrt(sq / std::max<size_t>(1, inlier_indices.size()));
  }

  // --- level it: floor normal to +Y, floor to y = 0 ---
  const Eigen::Vector3d up(0, 1, 0);
  out.rotation_deg =
      RadToDeg(std::acos(std::clamp(best.normal.dot(up), -1.0, 1.0)));
  const Eigen::Quaterniond level_rotation = RotationBetween(best.normal, up);

  SE3 transform;
  transform.q = level_rotation;
  transform.t = Eigen::Vector3d(0, best.offset, 0);
  out.floor_found = true;

  // --- square the walls to the axes ---
  //
  // With the floor level, walls are vertical, so the problem collapses to
  // one angle. Points in the wall band project onto the ground plane as
  // lines; rotating the frame until those lines fall along the axes is a
  // Manhattan-frame fit. Scoring by the sum of squared histogram counts
  // rewards exactly that concentration — a wall square to the axis dumps
  // all its points into a handful of bins.
  if (options.align_walls) {
    std::vector<Eigen::Vector2d> ground;
    ground.reserve(points.size());
    for (const auto& p : points) {
      const Eigen::Vector3d q = transform.Apply(p);
      if (q.y() < options.wall_band_lo_m || q.y() > options.wall_band_hi_m) {
        continue;
      }
      ground.emplace_back(q.x(), q.z());
    }

    if (static_cast<int>(ground.size()) >= options.min_wall_points) {
      const double bin = std::max(0.01, options.wall_bin_m);
      double best_theta = 0, best_score = -1;
      for (double theta = 0; theta < 90.0; theta += options.wall_step_deg) {
        const double r = DegToRad(theta);
        const double cs = std::cos(r), sn = std::sin(r);
        std::unordered_map<int, int> hist_u, hist_v;
        for (const auto& g : ground) {
          const double u = g.x() * cs + g.y() * sn;
          const double v = -g.x() * sn + g.y() * cs;
          ++hist_u[static_cast<int>(std::floor(u / bin))];
          ++hist_v[static_cast<int>(std::floor(v / bin))];
        }
        double score = 0;
        for (const auto& [_, n] : hist_u) score += static_cast<double>(n) * n;
        for (const auto& [_, n] : hist_v) score += static_cast<double>(n) * n;
        if (score > best_score) {
          best_score = score;
          best_theta = theta;
        }
      }
      // Rotate the world by +theta about up, which is what carries the wall
      // direction onto the axis the search found it near.
      const Eigen::Quaterniond square(
          Eigen::AngleAxisd(DegToRad(best_theta), up));
      transform.q = (square * transform.q).normalized();
      transform.t = square * transform.t;
      out.walls_squared = true;
      out.wall_turn_deg = best_theta;
    }
  }

  out.transform = transform;

  // Camera heights above the levelled floor: the honest check on whether the
  // fit describes the scan or merely the largest plane in it.
  {
    std::vector<double> heights;
    heights.reserve(camera_centres.size());
    for (const auto& c : camera_centres) heights.push_back(transform.Apply(c).y());
    std::sort(heights.begin(), heights.end());
    out.camera_height_m = heights[heights.size() / 2];
    std::vector<double> deviation;
    deviation.reserve(heights.size());
    for (const double h : heights) {
      deviation.push_back(std::abs(h - out.camera_height_m));
    }
    std::sort(deviation.begin(), deviation.end());
    // Median absolute deviation, scaled to read like a standard deviation.
    out.camera_height_spread_m = 1.4826 * deviation[deviation.size() / 2];
  }

  BS_LOGI("level",
          "floor: %d inliers, rmse %.3f m; cameras %.2f m above it "
          "(spread %.3f m); world rotated %.1f deg%s",
          out.floor_inliers, out.floor_rmse_m, out.camera_height_m,
          out.camera_height_spread_m, out.rotation_deg,
          out.walls_squared ? "" : " (walls not squared)");
  if (out.walls_squared) {
    BS_LOGI("level", "walls squared by %.2f deg about up", out.wall_turn_deg);
  }
  return out;
}

}  // namespace bs
