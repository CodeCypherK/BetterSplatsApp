#include "final/final_solve.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <functional>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "calib/lut_fit.h"
#include "common/log.h"
#include "final/colmap_export.h"
#include "final/final_readiness.h"
#include "final/image_report.h"
#include "final/level.h"
#include "final/model_split.h"
#include "final/component_align.h"
#include "fusion/residuals.h"
#include "fusion/scale.h"
#include "geometry/triangulation.h"
#include "geometry/two_view.h"
#include "io/session_reader.h"
#include "vision/features.h"
#include "vision/matching.h"

namespace bs {

namespace fs = std::filesystem;

namespace {

struct FrameData {
  uint32_t frame_id = 0;
  int index = -1;  // position in the ordered frame list
  FeatureSet features;
  std::vector<cv::Point2f> undistorted;
  std::vector<std::array<uint8_t, 3>> colors;   // sampled at keypoints
  std::vector<float> gradients;                 // local texture per feature
  std::vector<int32_t> track_of_feature;        // -1 or track index
  Intrinsics K;  // full resolution
  SE3 pose;
  bool posed = false;
  std::shared_ptr<DepthFrame> depth;
  std::shared_ptr<DepthLookup> lookup;
  std::string jpeg_path;
  std::string export_name;
  // Carried from meta.json purely so the report can name the frames that
  // hurt the model. Capture-time measurements, never inputs to the solve.
  float lap_var = 0;
  float overexp_frac = 0;
};

struct TrackObs {
  int frame_index = -1;
  int feature = -1;
};

struct Track {
  std::vector<TrackObs> observations;
  Eigen::Vector3d X = Eigen::Vector3d::Zero();
  bool has_point = false;
  bool dead = false;
  double mean_err_px = 0;
  double max_angle_deg = 0;
  float mean_gradient = 0;
};

struct UnionFind {
  std::vector<int> parent;
  explicit UnionFind(size_t n) : parent(n) {
    std::iota(parent.begin(), parent.end(), 0);
  }
  int Find(int a) {
    while (parent[a] != a) {
      parent[a] = parent[parent[a]];
      a = parent[a];
    }
    return a;
  }
  void Union(int a, int b) { parent[Find(a)] = Find(b); }
};

double MeanGradientAt(const cv::Mat& gray, const cv::Point2f& pt) {
  const int x = std::clamp(static_cast<int>(pt.x), 2, gray.cols - 3);
  const int y = std::clamp(static_cast<int>(pt.y), 2, gray.rows - 3);
  double sum = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      sum += 0.5 * (std::abs(gray.at<uint8_t>(y + dy, x + dx + 1) -
                             gray.at<uint8_t>(y + dy, x + dx - 1)) +
                    std::abs(gray.at<uint8_t>(y + dy + 1, x + dx) -
                             gray.at<uint8_t>(y + dy - 1, x + dx)));
    }
  }
  return sum / 9.0;
}

// One geometrically verified image pair and its surviving correspondences.
struct PairMatches {
  int a, b;
  std::vector<Match> inliers;
};

// ------------------------------------------------------- resume caching
// Features and verified matches persist to final/cache/ as they are
// computed, keyed by a hash of the solve-relevant configuration and the
// frame list. A cancelled/killed solve resumes past the expensive stages;
// any config or session change invalidates the cache wholesale.

uint64_t Fnv1a(uint64_t h, const void* data, size_t n) {
  const auto* p = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 1099511628211ull;
  }
  return h;
}

// Bump whenever feature detection or matching changes what the cached bytes
// would contain. The cache stores *outputs* of these algorithms, so a code
// change with an unchanged config would otherwise be silently served stale
// results from a previous run (detector improvements would appear to do
// nothing). Config values alone cannot catch that — this must be manual.
//   1: initial ORB/SIFT extraction
//   2: yield-based retry in both detectors (blur/noise starvation fix)
constexpr int kFeatureAlgorithmVersion = 2;

uint64_t CacheConfigHash(const EngineConfig& c, bool use_sift, bool fast,
                         const std::vector<uint32_t>& frame_ids) {
  uint64_t h = 1469598103934665603ull;
  const int schema = 1;
  h = Fnv1a(h, &schema, sizeof(schema));
  h = Fnv1a(h, &kFeatureAlgorithmVersion, sizeof(kFeatureAlgorithmVersion));
  h = Fnv1a(h, &use_sift, sizeof(use_sift));
  h = Fnv1a(h, &fast, sizeof(fast));
  h = Fnv1a(h, &c.final_orb_features, sizeof(c.final_orb_features));
  h = Fnv1a(h, &c.final_sift_features, sizeof(c.final_sift_features));
  h = Fnv1a(h, &c.final_match_ratio, sizeof(c.final_match_ratio));
  h = Fnv1a(h, &c.final_ransac_px, sizeof(c.final_ransac_px));
  h = Fnv1a(h, &c.final_pair_min_inliers, sizeof(c.final_pair_min_inliers));
  h = Fnv1a(h, &c.final_seq_window, sizeof(c.final_seq_window));
  h = Fnv1a(h, &c.final_exhaustive_below, sizeof(c.final_exhaustive_below));
  h = Fnv1a(h, frame_ids.data(), frame_ids.size() * sizeof(uint32_t));
  return h;
}

template <typename T>
void PutPod(std::ofstream& out, const T& v) {
  out.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
bool GetPod(std::ifstream& in, T& v) {
  in.read(reinterpret_cast<char*>(&v), sizeof(T));
  return static_cast<bool>(in);
}

constexpr uint32_t kFeatureCacheMagic = 0x42534643;  // "BSFC"
constexpr uint32_t kMatchCacheMagic = 0x42534D43;    // "BSMC"

struct CachedFeatures {
  FeatureSet features;
  std::vector<cv::Point2f> undistorted;
  std::vector<std::array<uint8_t, 3>> colors;
  std::vector<float> gradients;
};

bool WriteFeatureCache(const fs::path& path, const CachedFeatures& data) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  PutPod(out, kFeatureCacheMagic);
  PutPod(out, static_cast<int32_t>(data.features.type));
  const int32_t n = data.features.size();
  PutPod(out, n);
  const int32_t desc_cols = data.features.descriptors.cols;
  const int32_t desc_type = data.features.descriptors.type();
  PutPod(out, desc_cols);
  PutPod(out, desc_type);
  for (const auto& kp : data.features.keypoints) {
    PutPod(out, kp.pt.x);
    PutPod(out, kp.pt.y);
    PutPod(out, kp.response);
    PutPod(out, kp.octave);
  }
  if (n > 0) {
    out.write(reinterpret_cast<const char*>(data.features.descriptors.data),
              static_cast<std::streamsize>(data.features.descriptors.total() *
                                           data.features.descriptors.elemSize()));
  }
  out.write(reinterpret_cast<const char*>(data.undistorted.data()),
            static_cast<std::streamsize>(n * sizeof(cv::Point2f)));
  out.write(reinterpret_cast<const char*>(data.colors.data()),
            static_cast<std::streamsize>(n * 3));
  out.write(reinterpret_cast<const char*>(data.gradients.data()),
            static_cast<std::streamsize>(n * sizeof(float)));
  return static_cast<bool>(out);
}

bool ReadFeatureCache(const fs::path& path, FeatureType expected_type,
                      CachedFeatures& data) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  uint32_t magic;
  int32_t type_raw, n, desc_cols, desc_type;
  if (!GetPod(in, magic) || magic != kFeatureCacheMagic) return false;
  if (!GetPod(in, type_raw) ||
      static_cast<FeatureType>(type_raw) != expected_type) {
    return false;
  }
  if (!GetPod(in, n) || n < 0 || n > 100000) return false;
  if (!GetPod(in, desc_cols) || !GetPod(in, desc_type)) return false;

  data.features.type = expected_type;
  data.features.keypoints.resize(n);
  for (auto& kp : data.features.keypoints) {
    if (!GetPod(in, kp.pt.x) || !GetPod(in, kp.pt.y) ||
        !GetPod(in, kp.response) || !GetPod(in, kp.octave)) {
      return false;
    }
  }
  data.features.descriptors.create(n, desc_cols, desc_type);
  if (n > 0) {
    in.read(reinterpret_cast<char*>(data.features.descriptors.data),
            static_cast<std::streamsize>(data.features.descriptors.total() *
                                         data.features.descriptors.elemSize()));
  }
  data.undistorted.resize(n);
  in.read(reinterpret_cast<char*>(data.undistorted.data()),
          static_cast<std::streamsize>(n * sizeof(cv::Point2f)));
  data.colors.resize(n);
  in.read(reinterpret_cast<char*>(data.colors.data()),
          static_cast<std::streamsize>(n * 3));
  data.gradients.resize(n);
  in.read(reinterpret_cast<char*>(data.gradients.data()),
          static_cast<std::streamsize>(n * sizeof(float)));
  return static_cast<bool>(in);
}

struct CachedPair {
  uint32_t frame_a, frame_b;
  std::vector<std::pair<int32_t, int32_t>> inliers;
};

bool WriteMatchCache(const fs::path& path,
                     const std::vector<CachedPair>& pairs) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  PutPod(out, kMatchCacheMagic);
  PutPod(out, static_cast<uint32_t>(pairs.size()));
  for (const auto& p : pairs) {
    PutPod(out, p.frame_a);
    PutPod(out, p.frame_b);
    PutPod(out, static_cast<uint32_t>(p.inliers.size()));
    out.write(reinterpret_cast<const char*>(p.inliers.data()),
              static_cast<std::streamsize>(p.inliers.size() * sizeof(int32_t) *
                                           2));
  }
  return static_cast<bool>(out);
}

bool ReadMatchCache(const fs::path& path, std::vector<CachedPair>& pairs) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  uint32_t magic, count;
  if (!GetPod(in, magic) || magic != kMatchCacheMagic) return false;
  if (!GetPod(in, count) || count > 10'000'000) return false;
  pairs.resize(count);
  for (auto& p : pairs) {
    uint32_t n;
    if (!GetPod(in, p.frame_a) || !GetPod(in, p.frame_b) || !GetPod(in, n) ||
        n > 1'000'000) {
      return false;
    }
    p.inliers.resize(n);
    in.read(reinterpret_cast<char*>(p.inliers.data()),
            static_cast<std::streamsize>(n * sizeof(int32_t) * 2));
    if (!in) return false;
  }
  return true;
}

std::unordered_map<uint32_t, SE3> LoadLivePoses(const std::string& session_dir) {
  std::unordered_map<uint32_t, SE3> poses;
  std::ifstream in(fs::path(session_dir) / "live" / "poses.jsonl");
  std::string line;
  while (std::getline(in, line)) {
    if (line.find("\"tracking\"") == std::string::npos) continue;
    uint32_t frame_id;
    double qw, qx, qy, qz, px, py, pz;
    const char* q = std::strstr(line.c_str(), "\"q\":[");
    const char* p = std::strstr(line.c_str(), "\"p\":[");
    if (std::sscanf(line.c_str(), "{\"frame_id\":%u", &frame_id) != 1 ||
        q == nullptr || p == nullptr ||
        std::sscanf(q, "\"q\":[%lf,%lf,%lf,%lf]", &qw, &qx, &qy, &qz) != 4 ||
        std::sscanf(p, "\"p\":[%lf,%lf,%lf]", &px, &py, &pz) != 3) {
      continue;
    }
    SE3 pose;
    pose.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
    pose.t = Eigen::Vector3d(px, py, pz);
    poses[frame_id] = pose;
  }
  return poses;
}

}  // namespace

FinalOutcome RunFinalSolve(const EngineConfig& config,
                           const std::string& session_dir,
                           const std::string& preset,
                           const FinalProgressFn& progress,
                           const std::atomic<bool>* cancel) {
  FinalOutcome outcome;
  FinalMetrics metrics;
  const bool fast = preset == "fast";
  const int ba_rounds = fast ? std::max(1, config.final_ba_rounds - 1)
                             : config.final_ba_rounds;
  const int orb_features = fast ? 2000 : config.final_orb_features;

  auto cancelled = [&] { return cancel != nullptr && cancel->load(); };
  auto report = [&](bs_final_stage stage, float pct) {
    if (progress) progress(stage, pct, metrics);
  };
  auto fail = [&](const std::string& why) {
    outcome.error = why;
    outcome.metrics = metrics;
    BS_LOGE("final", "%s", why.c_str());
    return outcome;
  };

  const auto session = SessionReader::Open(session_dir);
  if (!session) return fail("cannot open session " + session_dir);

  // Distortion model for undistortion + export.
  double k1 = 0, k2 = 0;
  std::string camera_model = "PINHOLE";
  const auto& calib = session->calibration();
  if (calib.colmap_model && calib.colmap_model->model == "OPENCV" &&
      calib.colmap_model->params.size() >= 6) {
    k1 = calib.colmap_model->params[4];
    k2 = calib.colmap_model->params[5];
    camera_model = "OPENCV";
  } else if (!calib.distortion_lut.empty()) {
    if (const auto fit =
            FitOpencvModelFromLut(calib.distortion_lut, calib.intrinsics_session)) {
      k1 = fit->k1;
      k2 = fit->k2;
      camera_model = "OPENCV";
    }
  }

  // Scout frames are a localization scaffold, not reconstruction input: the
  // opening circuit is walked fast, far from every surface, and covers each
  // room only briefly. Their images would drag a splat's appearance and
  // their geometry adds nothing the capture pass does not cover, so they are
  // excluded here — which also keeps them out of the exported images/.
  // Frames superseded by a later rescan are dropped on the same principle.
  // "Go back and redo that room" would be worthless if the old, worse frames
  // still trained the splat alongside the new ones — but deleting them would
  // break the immutable-RAW rule and make the decision irreversible. So the
  // old frames stay on disk exactly as recorded, and the SOLVE declines to
  // reconstruct from them.
  //
  // Position comes from the live poses, which share one world frame across
  // the whole chain — that is what makes a volume from one session mean
  // anything to another. A frame the live pass never posed cannot be
  // located, and is KEPT: excluding data we cannot place would throw away
  // frames on a guess, which is the more damaging mistake of the two.
  std::unordered_map<uint32_t, SE3> supersede_poses;
  if (!session->supersessions().empty()) {
    for (const auto& dir : session->chain()) {
      for (auto& [id, pose] : LoadLivePoses(dir)) {
        supersede_poses.emplace(id, pose);
      }
    }
  }

  std::vector<uint32_t> solve_frame_ids;
  uint32_t scout_skipped = 0;
  uint32_t superseded_skipped = 0;
  uint32_t superseded_unlocatable = 0;
  for (const uint32_t frame_id : session->frame_ids()) {
    if (!config.final_include_scout) {
      const auto meta = session->ReadMeta(frame_id);
      if (meta && meta->is_scout()) {
        ++scout_skipped;
        continue;
      }
    }
    if (!session->supersessions().empty()) {
      const auto at = supersede_poses.find(frame_id);
      if (at == supersede_poses.end()) {
        ++superseded_unlocatable;
      } else {
        const Eigen::Vector3d centre = at->second.CameraCenter();
        const double position[3] = {centre.x(), centre.y(), centre.z()};
        if (session->IsSuperseded(frame_id, position)) {
          ++superseded_skipped;
          continue;
        }
      }
    }
    solve_frame_ids.push_back(frame_id);
  }
  if (superseded_skipped > 0 || superseded_unlocatable > 0) {
    BS_LOGI("final",
            "rescan: excluding %u superseded frames; %u could not be located "
            "and were kept",
            superseded_skipped, superseded_unlocatable);
  }
  metrics.frames_superseded = superseded_skipped;
  if (scout_skipped > 0) {
    BS_LOGI("final", "excluding %u scout frames; solving on %zu capture frames",
            scout_skipped, solve_frame_ids.size());
  }
  metrics.scout_frames_excluded = scout_skipped;
  if (solve_frame_ids.size() < 2) return fail("session has <2 capture frames");

  // Feature choice: SIFT for the quality preset when the session fits the
  // transient descriptor budget (or forced by config), else ORB.
  const size_t session_frames = solve_frame_ids.size();
  const bool use_sift =
      !fast && (config.final_use_sift == 1 ||
                (config.final_use_sift == 2 &&
                 static_cast<int>(session_frames) <=
                     config.final_sift_max_frames));

  // Resume cache setup.
  const fs::path cache_dir = fs::path(session_dir) / "final" / "cache";
  {
    std::error_code ec;
    fs::create_directories(cache_dir, ec);
  }
  const uint64_t config_hash =
      CacheConfigHash(config, use_sift, fast, solve_frame_ids);
  bool cache_valid = false;
  {
    std::ifstream manifest(cache_dir / "manifest.txt");
    uint64_t stored = 0;
    if (manifest >> stored && stored == config_hash) cache_valid = true;
  }
  if (!cache_valid) {
    std::error_code ec;
    fs::remove_all(cache_dir, ec);
    fs::create_directories(cache_dir, ec);
    std::ofstream(cache_dir / "manifest.txt") << config_hash << "\n";
  }
  const FeatureType feature_type =
      use_sift ? FeatureType::kSift : FeatureType::kOrb;

  // ---------------------------------------------------------- S1 features
  std::vector<FrameData> frames;
  frames.reserve(session_frames);
  metrics.images_total = static_cast<uint32_t>(session_frames);
  {
    int done = 0;
    for (const uint32_t frame_id : solve_frame_ids) {
      if (cancelled()) {
        outcome.cancelled = true;
        return outcome;
      }
      const auto meta = session->ReadMeta(frame_id);
      if (!meta) continue;

      FrameData frame;
      frame.frame_id = frame_id;
      frame.jpeg_path = session->ImagePath(frame_id);
      char name[32];
      std::snprintf(name, sizeof(name), "%06u.jpg", frame_id);
      frame.export_name = name;
      frame.lap_var = static_cast<float>(meta->quality.lap_var);
      frame.overexp_frac = static_cast<float>(meta->quality.overexp_frac);

      char cache_name[32];
      std::snprintf(cache_name, sizeof(cache_name), "feat_%06u.bin", frame_id);
      const fs::path cache_path = cache_dir / cache_name;

      CachedFeatures cached;
      if (cache_valid && ReadFeatureCache(cache_path, feature_type, cached)) {
        frame.features = std::move(cached.features);
        frame.undistorted = std::move(cached.undistorted);
        frame.colors = std::move(cached.colors);
        frame.gradients = std::move(cached.gradients);
        frame.K = {meta->intrinsics.fx, meta->intrinsics.fy,
                   meta->intrinsics.cx, meta->intrinsics.cy,
                   meta->intrinsics.ref_w, meta->intrinsics.ref_h};
        ++metrics.features_cached;
      } else {
        const auto jpeg = session->ReadImageBytes(frame_id);
        if (!jpeg) continue;
        const cv::Mat color = cv::imdecode(*jpeg, cv::IMREAD_COLOR);
        if (color.empty()) continue;
        cv::Mat gray;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

        frame.K = {meta->intrinsics.fx, meta->intrinsics.fy,
                   meta->intrinsics.cx, meta->intrinsics.cy, gray.cols,
                   gray.rows};
        if (use_sift) {
          SiftOptions sift;
          sift.max_features = config.final_sift_features;
          frame.features = DetectSift(gray, sift);
        } else {
          OrbOptions orb;
          orb.max_features = orb_features;
          frame.features = DetectOrb(gray, orb);
        }

        const PinholeIntrinsics pin{frame.K.fx, frame.K.fy, frame.K.cx,
                                    frame.K.cy, frame.K.width, frame.K.height};
        frame.undistorted.reserve(frame.features.keypoints.size());
        frame.colors.reserve(frame.features.keypoints.size());
        frame.gradients.reserve(frame.features.keypoints.size());
        for (const auto& kp : frame.features.keypoints) {
          if (k1 == 0.0 && k2 == 0.0) {
            frame.undistorted.push_back(kp.pt);
          } else {
            const Eigen::Vector2d u =
                UndistortPixel({kp.pt.x, kp.pt.y}, pin, k1, k2);
            frame.undistorted.emplace_back(static_cast<float>(u.x()),
                                           static_cast<float>(u.y()));
          }
          const int px =
              std::clamp(static_cast<int>(kp.pt.x), 0, color.cols - 1);
          const int py =
              std::clamp(static_cast<int>(kp.pt.y), 0, color.rows - 1);
          const cv::Vec3b bgr = color.at<cv::Vec3b>(py, px);
          frame.colors.push_back({bgr[2], bgr[1], bgr[0]});
          frame.gradients.push_back(
              static_cast<float>(MeanGradientAt(gray, kp.pt)));
        }

        CachedFeatures to_cache;
        to_cache.features = frame.features;
        to_cache.undistorted = frame.undistorted;
        to_cache.colors = frame.colors;
        to_cache.gradients = frame.gradients;
        WriteFeatureCache(cache_path, to_cache);
      }

      frame.track_of_feature.assign(frame.features.keypoints.size(), -1);
      frame.index = static_cast<int>(frames.size());
      frames.push_back(std::move(frame));
      report(BS_STAGE_FEATURES,
             static_cast<float>(++done) / metrics.images_total);
    }
  }
  if (frames.size() < 3) return fail("too few readable frames");

  // ------------------------------------------------------------- S3 pairs
  std::set<std::pair<int, int>> pairs;
  {
    const int n = static_cast<int>(frames.size());
    const bool exhaustive = n <= config.final_exhaustive_below;
    for (int i = 0; i < n; ++i) {
      if (exhaustive) {
        for (int j = i + 1; j < n; ++j) pairs.emplace(i, j);
      } else {
        for (int d = 1; d <= config.final_seq_window && i + d < n; ++d) {
          pairs.emplace(i, i + d);
        }
        for (const int d : {12, 20, 32, 52, 84}) {
          if (i + d < n) pairs.emplace(i, i + d);
        }
      }
    }
    // Loop pairs from live pose proximity (initialization hints only).
    const auto live_poses = LoadLivePoses(session_dir);
    for (int i = 0; i < n; ++i) {
      const auto pi = live_poses.find(frames[i].frame_id);
      if (pi == live_poses.end()) continue;
      for (int j = i + 20; j < n; ++j) {
        const auto pj = live_poses.find(frames[j].frame_id);
        if (pj == live_poses.end()) continue;
        if ((pi->second.CameraCenter() - pj->second.CameraCenter()).norm() <
            1.5) {
          pairs.emplace(i, j);
        }
      }
    }
    report(BS_STAGE_PAIRS, 1.0f);
  }

  // --------------------------------------------------- S4 verified matching
  std::vector<PairMatches> verified;
  {
    std::unordered_map<uint32_t, int> index_of_frame;
    for (const auto& f : frames) index_of_frame[f.frame_id] = f.index;

    std::vector<CachedPair> cached_pairs;
    if (cache_valid &&
        ReadMatchCache(cache_dir / "matches.bin", cached_pairs)) {
      for (const auto& cp : cached_pairs) {
        const auto ia = index_of_frame.find(cp.frame_a);
        const auto ib = index_of_frame.find(cp.frame_b);
        if (ia == index_of_frame.end() || ib == index_of_frame.end()) continue;
        PairMatches pm;
        pm.a = ia->second;
        pm.b = ib->second;
        pm.inliers.reserve(cp.inliers.size());
        for (const auto& [fa, fb] : cp.inliers) {
          Match m;
          m.idx_a = fa;
          m.idx_b = fb;
          pm.inliers.push_back(m);
        }
        verified.push_back(std::move(pm));
      }
      metrics.matches_cached = static_cast<uint32_t>(verified.size());
      report(BS_STAGE_MATCHING, 1.0f);
    } else {
      int done = 0;
      for (const auto& [i, j] : pairs) {
        if (cancelled()) {
          outcome.cancelled = true;
          return outcome;
        }
        MatchOptions mo;
        mo.ratio = config.final_match_ratio;
        mo.cross_check = true;
        // Absolute cap applies to Hamming (ORB); SIFT relies on ratio.
        mo.max_distance = use_sift ? 0.0f : 64.0f;
        const std::vector<Match> matches =
            MatchFeatures(frames[i].features, frames[j].features, mo);
        if (static_cast<int>(matches.size()) < config.final_pair_min_inliers) {
          ++done;
          continue;
        }
        std::vector<cv::Point2f> pa, pb;
        for (const auto& m : matches) {
          pa.push_back(frames[i].undistorted[m.idx_a]);
          pb.push_back(frames[j].undistorted[m.idx_b]);
        }
        TwoViewOptions tv;
        tv.ransac_thresh_px = config.final_ransac_px;
        tv.min_median_tri_angle_deg = 0.0;  // verification only, keep pairs
        tv.estimate_homography = false;
        const TwoViewResult rel = EstimateRelativePose(pa, pb, frames[i].K, tv);
        if (rel.ok() || rel.failure == TwoViewFailure::kRotationDominant) {
          PairMatches pm;
          pm.a = i;
          pm.b = j;
          for (size_t m = 0; m < matches.size(); ++m) {
            if (m < rel.inlier_mask.size() && rel.inlier_mask[m]) {
              pm.inliers.push_back(matches[m]);
            }
          }
          if (static_cast<int>(pm.inliers.size()) >=
              config.final_pair_min_inliers) {
            verified.push_back(std::move(pm));
          }
        }
        report(BS_STAGE_MATCHING, static_cast<float>(++done) / pairs.size());
      }

      std::vector<CachedPair> to_cache;
      to_cache.reserve(verified.size());
      for (const auto& pm : verified) {
        CachedPair cp;
        cp.frame_a = frames[pm.a].frame_id;
        cp.frame_b = frames[pm.b].frame_id;
        cp.inliers.reserve(pm.inliers.size());
        for (const auto& m : pm.inliers) cp.inliers.emplace_back(m.idx_a, m.idx_b);
        to_cache.push_back(std::move(cp));
      }
      WriteMatchCache(cache_dir / "matches.bin", to_cache);
    }
  }
  if (verified.empty()) return fail("no verified pairs");

  // ------------------------------------------------------------- S5 tracks
  std::vector<Track> tracks;
  {
    // Global feature index: frame.index * stride + feature.
    size_t stride = 0;
    for (const auto& f : frames) {
      stride = std::max(stride, f.features.keypoints.size());
    }
    UnionFind uf(frames.size() * stride);
    for (const auto& pm : verified) {
      for (const auto& m : pm.inliers) {
        uf.Union(static_cast<int>(pm.a * stride + m.idx_a),
                 static_cast<int>(pm.b * stride + m.idx_b));
      }
    }
    std::unordered_map<int, int> root_to_track;
    for (const auto& pm : verified) {
      for (const auto& m : pm.inliers) {
        for (const auto& [fi, feat] :
             {std::pair<int, int>(pm.a, m.idx_a),
              std::pair<int, int>(pm.b, m.idx_b)}) {
          const int root = uf.Find(static_cast<int>(fi * stride + feat));
          auto [it, inserted] = root_to_track.emplace(
              root, static_cast<int>(tracks.size()));
          if (inserted) tracks.emplace_back();
          Track& track = tracks[it->second];
          if (frames[fi].track_of_feature[feat] == it->second) continue;
          if (frames[fi].track_of_feature[feat] != -1) continue;  // conflict
          frames[fi].track_of_feature[feat] = it->second;
          track.observations.push_back({fi, feat});
        }
      }
    }
    // Same-image conflicts: a track landing on two features of one frame is
    // already prevented above; short tracks die here.
    for (auto& track : tracks) {
      std::set<int> seen;
      bool conflict = false;
      for (const auto& obs : track.observations) {
        if (!seen.insert(obs.frame_index).second) conflict = true;
      }
      if (conflict || track.observations.size() < 2) track.dead = true;
      float grad = 0;
      for (const auto& obs : track.observations) {
        grad += frames[obs.frame_index].gradients[obs.feature];
      }
      track.mean_gradient =
          track.observations.empty()
              ? 0
              : grad / static_cast<float>(track.observations.size());
    }
    // Descriptors are no longer needed past track building — releasing
    // them bounds peak memory (matters for the SIFT quality path on-device;
    // the resume cache keeps them on disk).
    for (auto& frame : frames) frame.features.descriptors.release();
    report(BS_STAGE_TRACKS, 1.0f);
  }

  // Depth + solver lookup for one frame, built once. Component recovery and
  // the BA rounds both need these, and they must be created together: a
  // frame carrying `depth` without `lookup` silently contributes zero LiDAR
  // residuals, which un-anchors the geometry.
  auto ensure_depth = [&](FrameData& frame) {
    if (frame.depth && frame.lookup) return;
    const auto meta = session->ReadMeta(frame.frame_id);
    const auto depth_img = session->ReadDepth(frame.frame_id);
    if (!meta || !depth_img) return;
    const Intrinsics kd{meta->depth_intrinsics.fx, meta->depth_intrinsics.fy,
                        meta->depth_intrinsics.cx, meta->depth_intrinsics.cy,
                        depth_img->width, depth_img->height};
    LidarConfidenceOptions lo;
    lo.sigma_base_m = config.lidar_sigma_base_m;
    lo.sigma_quadratic = config.lidar_sigma_quadratic;
    lo.range_min_m = config.lidar_range_min_m;
    lo.range_full_m = config.lidar_range_full_m;
    lo.range_zero_m = config.lidar_range_zero_m;
    frame.depth = std::make_shared<DepthFrame>(*depth_img, kd, lo);
    frame.lookup = std::make_shared<DepthLookup>(*frame.depth);
  };

  // One reconstruction component in its own gauge: a seed pair grown by PnP
  // against its own points, optionally scaled to metres by LiDAR. Used both
  // to start the model from images alone and to rebuild stretches the live
  // pass never posed.
  struct Component {
    std::unordered_map<int, SE3> pose;                 // frame index -> pose
    std::unordered_map<int, Eigen::Vector3d> point;    // track index -> X
    bool metric = false;
    int seed_a = -1, seed_b = -1;
  };

  auto build_component =
      [&](const std::function<bool(int)>& eligible) -> std::optional<Component> {
    // --- seed: strongest verified pair with real parallax among candidates
    const PairMatches* seed = nullptr;
    TwoViewResult seed_rel;
    size_t seed_score = 0;
    for (const auto& pm : verified) {
      if (!eligible(pm.a) || !eligible(pm.b)) continue;
      if (pm.inliers.size() <= seed_score) continue;
      std::vector<cv::Point2f> pa, pb;
      pa.reserve(pm.inliers.size());
      pb.reserve(pm.inliers.size());
      for (const auto& m : pm.inliers) {
        pa.push_back(frames[pm.a].undistorted[m.idx_a]);
        pb.push_back(frames[pm.b].undistorted[m.idx_b]);
      }
      TwoViewOptions tv;
      tv.ransac_thresh_px = config.final_ransac_px;
      tv.min_median_tri_angle_deg = config.final_tri_min_angle_deg;
      const TwoViewResult rel =
          EstimateRelativePose(pa, pb, frames[pm.a].K, tv);
      // A plane-dominated pair carries the conjugate-plane ambiguity and may
      // return the wrong branch — never seed a component from one.
      if (!rel.ok() || rel.planar_ambiguous) continue;
      seed = &pm;
      seed_rel = rel;
      seed_score = pm.inliers.size();
    }
    if (seed == nullptr) return std::nullopt;

    Component comp;
    comp.seed_a = seed->a;
    comp.seed_b = seed->b;
    // Seed frame defines the component's world; the partner's pose is the
    // relative pose directly (unit baseline until LiDAR sets the scale).
    comp.pose[seed->a] = SE3::Identity();
    comp.pose[seed->b] = seed_rel.rel_pose;

    auto retriangulate = [&] {
      comp.point.clear();
      for (size_t t = 0; t < tracks.size(); ++t) {
        if (tracks[t].dead) continue;
        std::vector<Observation2D> obs;
        for (const auto& to : tracks[t].observations) {
          const auto it = comp.pose.find(to.frame_index);
          if (it == comp.pose.end()) continue;
          obs.push_back({it->second, frames[to.frame_index].K,
                         frames[to.frame_index].undistorted[to.feature]});
        }
        if (obs.size() < 2) continue;
        const auto tri = TriangulatePoint(obs);
        if (!tri || !tri->InFrontOfAll()) continue;
        if (tri->max_angle_deg < config.final_tri_min_angle_deg) continue;
        if (tri->max_reproj_error_px > config.final_tri_max_err_px) continue;
        comp.point[static_cast<int>(t)] = tri->point;
      }
    };
    retriangulate();

    // --- grow: PnP eligible frames against this component's own points
    for (int iter = 0; iter < config.final_component_grow_iters; ++iter) {
      int added = 0;
      for (size_t i = 0; i < frames.size(); ++i) {
        const int idx = static_cast<int>(i);
        if (!eligible(idx) || comp.pose.count(idx)) continue;
        FrameData& frame = frames[i];
        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> image_points;
        for (size_t feat = 0; feat < frame.track_of_feature.size(); ++feat) {
          const int32_t t = frame.track_of_feature[feat];
          if (t < 0) continue;
          const auto it = comp.point.find(t);
          if (it == comp.point.end()) continue;
          object_points.emplace_back(static_cast<float>(it->second.x()),
                                     static_cast<float>(it->second.y()),
                                     static_cast<float>(it->second.z()));
          image_points.push_back(frame.undistorted[feat]);
        }
        if (static_cast<int>(object_points.size()) <
            config.final_register_min_inliers) {
          continue;
        }
        cv::Mat camera = (cv::Mat_<double>(3, 3) << frame.K.fx, 0, frame.K.cx, 0,
                          frame.K.fy, frame.K.cy, 0, 0, 1);
        cv::Mat rvec, tvec;
        std::vector<int> inliers;
        if (!cv::solvePnPRansac(
                object_points, image_points, camera, cv::noArray(), rvec, tvec,
                false, 200, static_cast<float>(config.final_register_thresh_px),
                0.999, inliers, cv::SOLVEPNP_EPNP) ||
            static_cast<int>(inliers.size()) <
                config.final_register_min_inliers) {
          continue;
        }
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        Eigen::Matrix3d rot;
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) rot(r, c) = R.at<double>(r, c);
        }
        SE3 pose;
        pose.q = Eigen::Quaterniond(rot).normalized();
        pose.t = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1),
                                 tvec.at<double>(2));
        comp.pose[idx] = pose;
        ++added;
      }
      retriangulate();
      if (added == 0) break;
    }

    // --- metric anchor: LiDAR fixes this component's scale, exactly as it
    // fixes the live map's gauge at bootstrap.
    std::vector<double> ratios;
    for (const auto& [track_index, X] : comp.point) {
      for (const auto& to : tracks[track_index].observations) {
        const auto pit = comp.pose.find(to.frame_index);
        if (pit == comp.pose.end()) continue;
        FrameData& f = frames[to.frame_index];
        ensure_depth(f);
        if (!f.depth) continue;
        const Eigen::Vector3d xc = pit->second.Apply(X);
        if (xc.z() < 0.05) continue;
        const auto& dK = f.depth->K();
        const int u =
            static_cast<int>(std::lround(dK.fx * xc.x() / xc.z() + dK.cx));
        const int v =
            static_cast<int>(std::lround(dK.fy * xc.y() / xc.z() + dK.cy));
        if (!f.depth->Valid(u, v)) continue;
        const double d = f.depth->DepthAt(u, v);
        if (d > 0.05) ratios.push_back(d / xc.z());
      }
    }
    const ScaleEstimate est = EstimateScale(std::move(ratios));
    if (est.locked) {
      for (auto& [_, pose] : comp.pose) pose.t *= est.scale;
      for (auto& [_, X] : comp.point) X *= est.scale;
      comp.metric = true;
    }
    return comp;
  };

  // ---------------------------------------------------------- S6 pose init
  //
  // Live poses are an initialization hint, not a dependency. When the live
  // pass produced too few to start from — it failed, or the user skipped
  // straight to a capture pass — the model is bootstrapped from image
  // geometry instead, which is what "the final solve recomputes everything"
  // has to mean if a live failure is not to become a final failure.
  {
    const auto live_poses = LoadLivePoses(session_dir);
    int posed = 0;
    for (auto& frame : frames) {
      const auto it = live_poses.find(frame.frame_id);
      if (it != live_poses.end()) {
        frame.pose = it->second;
        frame.posed = true;
        ++posed;
      }
    }

    if (posed < 2) {
      BS_LOGI("final",
              "live initialization unusable (%d posed) — bootstrapping the "
              "model from image geometry",
              posed);
      auto unposed = [&](int i) { return !frames[i].posed; };
      const auto comp = build_component(unposed);
      if (!comp || comp->pose.size() < 2) {
        return fail("no usable initial image pair to bootstrap the solve");
      }
      // The first component defines the session's world gauge, so its poses
      // are adopted directly — there is nothing yet to align against.
      for (const auto& [frame_index, pose] : comp->pose) {
        frames[frame_index].pose = pose;
        frames[frame_index].posed = true;
        ++posed;
      }
      metrics.components_recovered = 1;
      metrics.frames_recovered = static_cast<uint32_t>(comp->pose.size());
    }

    metrics.images_registered = posed;
    report(BS_STAGE_INIT_POSES, 1.0f);
  }

  // -------------------------------------------------------- S7 triangulate
  auto triangulate_track = [&](Track& track) {
    std::vector<Observation2D> obs;
    for (const auto& to : track.observations) {
      const FrameData& f = frames[to.frame_index];
      if (!f.posed) continue;
      obs.push_back({f.pose, f.K, f.undistorted[to.feature]});
    }
    if (obs.size() < 2) return false;
    const auto tri = TriangulatePoint(obs);
    if (!tri || !tri->InFrontOfAll()) return false;
    if (tri->max_angle_deg < config.final_tri_min_angle_deg) return false;
    if (tri->max_reproj_error_px > config.final_tri_max_err_px) return false;
    track.X = tri->point;
    track.has_point = true;
    track.mean_err_px = tri->mean_reproj_error_px;
    track.max_angle_deg = tri->max_angle_deg;
    return true;
  };
  {
    int made = 0;
    for (auto& track : tracks) {
      if (!track.dead && triangulate_track(track)) ++made;
    }
    metrics.points = made;
    if (made < 50) return fail("triangulation produced too few points");
    report(BS_STAGE_TRIANGULATE, 1.0f);
  }

  // ------------------------------------- S7b multi-component recovery
  //
  // Frames the live pass never posed are unreachable by PnP when they see no
  // already-reconstructed structure — walk into the next room and every
  // frame there is invisible to the model. Rebuild them from image geometry
  // instead: bootstrap a component from its own best two-view pair, grow it
  // internally, anchor its scale on LiDAR, then align it back over the
  // tracks it shares with the main model. This is what lets the final solve
  // repair a live tracking failure rather than inherit it.
  if (config.final_multi_component) {
    std::vector<bool> exhausted(frames.size(), false);  // tried, cannot seed
    int components_merged = 0;
    int frames_recovered = 0;

    for (int attempt = 0; attempt < config.final_max_components; ++attempt) {
      if (cancelled()) {
        outcome.cancelled = true;
        return outcome;
      }
      auto eligible = [&](int i) {
        return !frames[i].posed && !exhausted[i];
      };
      int remaining = 0;
      for (size_t i = 0; i < frames.size(); ++i) remaining += eligible(i) ? 1 : 0;
      if (remaining < config.final_component_min_frames) break;

      const auto comp = build_component(eligible);
      if (!comp) {
        // Nothing left that can start a component.
        for (size_t i = 0; i < frames.size(); ++i) {
          if (!frames[i].posed) exhausted[i] = true;
        }
        break;
      }
      const auto& comp_pose = comp->pose;
      const auto& comp_point = comp->point;
      const bool component_is_metric = comp->metric;

      if (static_cast<int>(comp_pose.size()) < config.final_component_min_frames) {
        exhausted[comp->seed_a] = exhausted[comp->seed_b] = true;
        continue;
      }

      // --- align: tracks reconstructed in BOTH gauges give the transform
      std::vector<Eigen::Vector3d> src, dst;
      for (const auto& [track_index, X] : comp_point) {
        const Track& track = tracks[track_index];
        if (track.dead || !track.has_point) continue;
        src.push_back(X);
        dst.push_back(track.X);
      }
      Similarity sim;
      int align_inliers = 0;
      // LiDAR already put this component in metric units, and the main model
      // is metric too — so the only freedom left is rigid. Fitting a scale
      // here would let alignment noise resize a whole room.
      bool aligned = RobustSimilarity(
          src, dst, config.final_merge_inlier_m, config.final_merge_min_points,
          /*allow_scale=*/!component_is_metric, sim, align_inliers);
      // A handful of inliers among hundreds of candidates is a coincidence,
      // not an overlap — placing a room on that evidence is worse than
      // leaving its frames out.
      if (aligned && !src.empty() &&
          static_cast<double>(align_inliers) / static_cast<double>(src.size()) <
              config.final_merge_min_inlier_frac) {
        BS_LOGI("final",
                "component alignment rejected: only %d/%zu inliers (%.0f%%)",
                align_inliers, src.size(),
                100.0 * align_inliers / static_cast<double>(src.size()));
        aligned = false;
      }
      if (!aligned) {
        // Genuinely disconnected from the main model (no shared structure).
        // Leave the frames unposed and honest rather than inventing a pose.
        BS_LOGI("final",
                "component of %zu frames shares no alignable structure "
                "(%zu candidate points) — left unregistered",
                comp_pose.size(), src.size());
        for (const auto& [frame_index, _] : comp_pose) {
          exhausted[frame_index] = true;
        }
        continue;
      }

      for (const auto& [frame_index, pose] : comp_pose) {
        frames[frame_index].pose = TransformPose(pose, sim);
        frames[frame_index].posed = true;
        ++frames_recovered;
      }
      ++components_merged;

      // Fold the new views into the shared structure before the next pass.
      for (auto& track : tracks) {
        if (!track.dead) triangulate_track(track);
      }

      // Note: re-solving each merged frame by PnP against the freshly
      // triangulated structure was tried and measured slightly worse
      // (0.19 m -> 0.24 m aligned RMSE) — the re-triangulation already
      // absorbs the component's poses, so PnP just refits to points it
      // dragged. The BA rounds below are the right place to refine.
      BS_LOGI("final",
              "recovered component: %zu frames, scale %.3f, %d/%zu alignment "
              "inliers",
              comp_pose.size(), sim.scale, align_inliers, src.size());
    }

    metrics.components_recovered = static_cast<uint32_t>(components_merged);
    metrics.frames_recovered = static_cast<uint32_t>(frames_recovered);
    uint32_t posed_now = 0;
    for (const auto& f : frames) posed_now += f.posed ? 1 : 0;
    metrics.images_registered = posed_now;
  }

  // ------------------------------------------------- S8 global BA rounds
  const int first_posed = [&] {
    for (const auto& f : frames) {
      if (f.posed) return f.index;
    }
    return 0;
  }();

  for (int round = 1; round <= ba_rounds; ++round) {
    if (cancelled()) {
      outcome.cancelled = true;
      return outcome;
    }
    metrics.ba_round = round;

    // Lazily build depth frames for posed frames.
    for (auto& frame : frames) {
      if (frame.posed) ensure_depth(frame);
    }

    // Build the problem.
    ceres::Problem problem;
    std::unordered_map<int, std::array<double, 7>> pose_blocks;
    auto pose_block = [&](int frame_index) -> double* {
      auto it = pose_blocks.find(frame_index);
      if (it == pose_blocks.end()) {
        std::array<double, 7> block{};
        PoseToBlocks(frames[frame_index].pose, block.data(), block.data() + 4);
        it = pose_blocks.emplace(frame_index, block).first;
      }
      return it->second.data();
    };

    metrics.lidar_residuals = 0;
    int reproj_count = 0;
    std::vector<std::array<double, 3>> point_blocks(tracks.size());
    for (size_t t = 0; t < tracks.size(); ++t) {
      Track& track = tracks[t];
      if (track.dead || !track.has_point) continue;
      point_blocks[t] = {track.X.x(), track.X.y(), track.X.z()};
      const double w_tex =
          TextureWeight(track.mean_gradient, track.max_angle_deg, 25.0, 4.0,
                        config.lidar_tex_floor);
      for (const auto& obs : track.observations) {
        FrameData& f = frames[obs.frame_index];
        if (!f.posed) continue;
        double* pose = pose_block(obs.frame_index);
        const cv::Point2f& px = f.undistorted[obs.feature];
        problem.AddResidualBlock(
            ReprojectionResidual::Create(px.x, px.y, f.K),
            new ceres::HuberLoss(config.final_ba_huber_px), pose, pose + 4,
            point_blocks[t].data());
        ++reproj_count;

        if (f.depth && f.lookup) {
          const auto assoc = MakeLidarAssociation(
              *f.depth, f.pose, track.X, w_tex, config.lidar_gate_sigmas);
          if (assoc) {
            problem.AddResidualBlock(
                LidarDepthResidual::Create(f.lookup.get(), *assoc),
                new ceres::CauchyLoss(1.0), pose, pose + 4,
                point_blocks[t].data());
            ++metrics.lidar_residuals;
          }
        }
      }
    }
    if (reproj_count < 100) return fail("BA problem too small");

    for (auto& [frame_index, block] : pose_blocks) {
      problem.SetManifold(block.data(), new ceres::EigenQuaternionManifold);
      if (frame_index == first_posed) {
        problem.SetParameterBlockConstant(block.data());
        problem.SetParameterBlockConstant(block.data() + 4);
      }
    }

    ceres::Solver::Options options;
    options.max_num_iterations =
        fast ? config.final_ba_max_iterations * 2 / 3
             : config.final_ba_max_iterations;
    options.function_tolerance = 1e-5;
    options.num_threads = config.final_threads;
    options.linear_solver_type = pose_blocks.size() > 250
                                     ? ceres::ITERATIVE_SCHUR
                                     : ceres::SPARSE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.logging_type = ceres::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Write back.
    for (const auto& [frame_index, block] : pose_blocks) {
      frames[frame_index].pose = PoseFromBlocks(block.data(), block.data() + 4);
    }
    for (size_t t = 0; t < tracks.size(); ++t) {
      if (!tracks[t].dead && tracks[t].has_point) {
        tracks[t].X = Eigen::Vector3d(point_blocks[t][0], point_blocks[t][1],
                                      point_blocks[t][2]);
      }
    }

    // Prune observations and points; recompute stats.
    size_t pruned_obs = 0;
    size_t pruned_points = 0;
    double err_sq_sum = 0;
    size_t err_count = 0;
    for (auto& track : tracks) {
      if (track.dead || !track.has_point) continue;
      std::vector<TrackObs> kept;
      double err_sum = 0;
      for (const auto& obs : track.observations) {
        const FrameData& f = frames[obs.frame_index];
        if (!f.posed) {
          kept.push_back(obs);
          continue;
        }
        const Eigen::Vector3d xc = f.pose.Apply(track.X);
        if (xc.z() <= 0.01) {
          frames[obs.frame_index].track_of_feature[obs.feature] = -1;
          ++pruned_obs;
          continue;
        }
        const Eigen::Vector2d proj = f.K.Project(xc);
        const cv::Point2f& px = f.undistorted[obs.feature];
        const double err = std::hypot(proj.x() - px.x, proj.y() - px.y);
        if (err > config.final_prune_obs_px) {
          frames[obs.frame_index].track_of_feature[obs.feature] = -1;
          ++pruned_obs;
          continue;
        }
        err_sum += err;
        err_sq_sum += err * err;
        ++err_count;
        kept.push_back(obs);
      }
      track.observations = std::move(kept);

      int posed_obs = 0;
      for (const auto& obs : track.observations) {
        if (frames[obs.frame_index].posed) ++posed_obs;
      }
      if (posed_obs < 2) {
        track.dead = true;
        ++pruned_points;
        continue;
      }
      track.mean_err_px = err_sum / std::max(1, posed_obs);
      if (track.mean_err_px > config.final_prune_point_mean_px) {
        track.dead = true;
        ++pruned_points;
        continue;
      }
      // Refresh triangulation angle.
      track.max_angle_deg = 0;
      for (size_t i = 0; i < track.observations.size(); ++i) {
        for (size_t j = i + 1; j < track.observations.size(); ++j) {
          const FrameData& fa = frames[track.observations[i].frame_index];
          const FrameData& fb = frames[track.observations[j].frame_index];
          if (!fa.posed || !fb.posed) continue;
          track.max_angle_deg = std::max(
              track.max_angle_deg,
              TriangulationAngleDeg(fa.pose.CameraCenter(),
                                    fb.pose.CameraCenter(), track.X));
        }
      }
      if (track.max_angle_deg < config.final_tri_min_angle_deg) {
        track.dead = true;
        ++pruned_points;
      }
    }
    metrics.reproj_rmse_px =
        err_count > 0 ? static_cast<float>(std::sqrt(err_sq_sum / err_count))
                      : 0.0f;

    // Register frames the live pass missed via 2D-3D PnP.
    int newly_registered = 0;
    for (auto& frame : frames) {
      if (frame.posed) continue;
      std::vector<cv::Point3f> object_points;
      std::vector<cv::Point2f> image_points;
      for (size_t feat = 0; feat < frame.track_of_feature.size(); ++feat) {
        const int32_t t = frame.track_of_feature[feat];
        if (t < 0 || tracks[t].dead || !tracks[t].has_point) continue;
        object_points.emplace_back(static_cast<float>(tracks[t].X.x()),
                                   static_cast<float>(tracks[t].X.y()),
                                   static_cast<float>(tracks[t].X.z()));
        image_points.push_back(frame.undistorted[feat]);
      }
      if (static_cast<int>(object_points.size()) <
          config.final_register_min_inliers) {
        continue;
      }
      cv::Mat camera = (cv::Mat_<double>(3, 3) << frame.K.fx, 0, frame.K.cx, 0,
                        frame.K.fy, frame.K.cy, 0, 0, 1);
      cv::Mat rvec, tvec;
      std::vector<int> inliers;
      const bool ok = cv::solvePnPRansac(
          object_points, image_points, camera, cv::noArray(), rvec, tvec,
          false, 200, static_cast<float>(config.final_register_thresh_px),
          0.999, inliers, cv::SOLVEPNP_EPNP);
      if (!ok || static_cast<int>(inliers.size()) <
                     config.final_register_min_inliers) {
        continue;
      }
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      Eigen::Matrix3d rot;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) rot(r, c) = R.at<double>(r, c);
      }
      frame.pose.q = Eigen::Quaterniond(rot).normalized();
      frame.pose.t = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1),
                                     tvec.at<double>(2));
      frame.posed = true;
      ++newly_registered;
    }

    // Re-triangulate tracks that lost their point or gained posed views.
    int retriangulated = 0;
    for (auto& track : tracks) {
      if (track.dead) continue;
      if (!track.has_point && triangulate_track(track)) ++retriangulated;
    }

    uint32_t registered = 0;
    uint32_t live_points = 0;
    double track_len_sum = 0;
    for (const auto& f : frames) registered += f.posed ? 1 : 0;
    for (const auto& t : tracks) {
      if (!t.dead && t.has_point) {
        ++live_points;
        track_len_sum += static_cast<double>(t.observations.size());
      }
    }
    metrics.images_registered = registered;
    metrics.points = live_points;
    metrics.mean_track_len =
        live_points ? static_cast<float>(track_len_sum / live_points) : 0.0f;

    report(BS_STAGE_GLOBAL_BA, static_cast<float>(round) / ba_rounds);
    BS_LOGI("final",
            "round %d: %u/%u registered, %u points, rmse %.2fpx, pruned "
            "%zu obs %zu pts, +%d frames, +%d retriangulated, %u lidar terms",
            round, registered, metrics.images_total, live_points,
            metrics.reproj_rmse_px, pruned_obs, pruned_points,
            newly_registered, retriangulated, metrics.lidar_residuals);

    const double change_frac =
        static_cast<double>(pruned_obs + pruned_points + newly_registered) /
        std::max<size_t>(1, err_count);
    if (round >= 2 && newly_registered == 0 &&
        change_frac < config.final_early_stop_frac) {
      break;
    }
  }

  // ------------------------------------------------ S8b level the world
  //
  // Image-only structure from motion has no idea which way is down: the
  // world frame is anchored on whatever the first camera happened to be
  // doing, so an ordinary room routinely comes out on a slope with its walls
  // at some arbitrary bearing. Finding the floor and squaring the walls
  // fixes that with one rigid transform.
  //
  // Applied HERE, before the floater sweep and the dense cloud, so every
  // later stage simply works in the levelled frame — there is no second
  // copy of the world to keep in step, and the split parts inherit it for
  // free. Poses and points move together, so the reconstruction's geometry
  // is untouched: only where it sits changes.
  Leveling leveling;
  if (config.final_level_floor) {
    std::vector<Eigen::Vector3d> level_points;
    level_points.reserve(tracks.size());
    for (const auto& track : tracks) {
      if (!track.dead && track.has_point) level_points.push_back(track.X);
    }
    std::vector<Eigen::Vector3d> level_cameras;
    for (const auto& frame : frames) {
      if (frame.posed) level_cameras.push_back(frame.pose.CameraCenter());
    }

    LevelingOptions level_options;
    level_options.align_walls = config.final_square_walls;

    // A floor the user pointed at beats one found by searching. Every
    // ambiguity the search reasons around — floor against ceiling, a lone
    // plane, a floor too sparsely tracked to find — was settled at capture
    // time, and the depth sensor saw a dense sheet of it from a metre away.
    // The calibration is stored in its frame's camera coordinates, so it
    // needs that frame's FINAL pose to become a world plane, which is
    // exactly why it is resolved here rather than at capture.
    const SurfaceCalibration& floor_cal = session->info().floor_calibration;
    const FrameData* calibration_frame = nullptr;
    if (floor_cal.present) {
      for (const auto& frame : frames) {
        if (frame.posed && frame.frame_id == floor_cal.frame_id) {
          calibration_frame = &frame;
          break;
        }
      }
      if (calibration_frame == nullptr) {
        BS_LOGW("final",
                "floor calibration names frame %u, which this solve did not "
                "register; falling back to finding the floor",
                floor_cal.frame_id);
      }
    }

    if (calibration_frame != nullptr) {
      leveling = LevelingFromMeasuredFloor(
          Eigen::Vector3d(floor_cal.normal[0], floor_cal.normal[1],
                          floor_cal.normal[2]),
          floor_cal.offset_m, calibration_frame->pose, level_points,
          level_cameras, level_options);
    } else {
      leveling = EstimateLeveling(level_points, level_cameras, level_options);
    }

    if (leveling.floor_found) {
      const SE3 inverse = leveling.transform.Inverse();
      for (auto& track : tracks) {
        if (!track.dead && track.has_point) {
          track.X = leveling.transform.Apply(track.X);
        }
      }
      // A world-to-camera pose composed with the inverse world transform is
      // the same camera looking at the same thing from the same place, said
      // in the new frame.
      for (auto& frame : frames) {
        if (frame.posed) frame.pose = frame.pose * inverse;
      }
      BS_LOGI("final",
              "levelled: floor rmse %.3f m, cameras %.2f m above it%s",
              leveling.floor_rmse_m, leveling.camera_height_m,
              leveling.walls_squared ? ", walls squared" : "");
    }
  }

  // ------------------------------------------------------ S9 floater sweep
  {
    uint32_t removed = 0;
    for (auto& track : tracks) {
      if (track.dead || !track.has_point) continue;
      int front_votes = 0;
      for (const auto& obs : track.observations) {
        const FrameData& f = frames[obs.frame_index];
        if (!f.posed || !f.depth) continue;
        const Eigen::Vector3d xc = f.pose.Apply(track.X);
        if (xc.z() <= 0.05) continue;
        const auto& dK = f.depth->K();
        const double u = dK.fx * xc.x() / xc.z() + dK.cx;
        const double v = dK.fy * xc.y() / xc.z() + dK.cy;
        const int xi = static_cast<int>(std::lround(u));
        const int yi = static_cast<int>(std::lround(v));
        if (f.depth->ConfidenceAt(xi, yi) < 0.6) continue;
        const auto d = f.depth->DepthBilinear(u, v);
        if (!d) continue;
        const double sigma = f.depth->Sigma(*d);
        if (*d - xc.z() > config.floater_sigma_gate * sigma) ++front_votes;
      }
      if (front_votes >= config.floater_min_rays) {
        track.dead = true;
        ++removed;
      }
    }
    metrics.floaters_removed = removed;
    metrics.points -= std::min(metrics.points, removed);
    report(BS_STAGE_FLOATER_SWEEP, 1.0f);
    BS_LOGI("final", "floater sweep removed %u points", removed);
  }

  // ------------------------------------------- S10 final LiDAR alignment
  {
    std::vector<Eigen::Vector3f> cloud;
    std::vector<std::array<uint8_t, 3>> cloud_colors;
    std::unordered_set<uint64_t> voxel_seen;
    const double voxel = 0.01;
    int done = 0;
    int posed_total = 0;
    for (const auto& f : frames) posed_total += f.posed ? 1 : 0;

    for (const auto& frame : frames) {
      if (cancelled()) {
        outcome.cancelled = true;
        return outcome;
      }
      if (!frame.posed || !frame.depth) continue;
      const cv::Mat color_img = cv::imread(frame.jpeg_path, cv::IMREAD_COLOR);
      const SE3 cam_to_world = frame.pose.Inverse();
      for (int y = 0; y < frame.depth->height(); y += 2) {
        for (int x = 0; x < frame.depth->width(); x += 2) {
          if (frame.depth->ConfidenceAt(x, y) < 0.5) continue;
          const auto p_cam = frame.depth->Unproject(x, y);
          if (!p_cam) continue;
          const Eigen::Vector3d p_world = cam_to_world.Apply(*p_cam);
          const uint64_t key =
              (static_cast<uint64_t>(
                   static_cast<int32_t>(std::floor(p_world.x() / voxel)) &
                   0x1FFFFF)
               << 42) |
              (static_cast<uint64_t>(
                   static_cast<int32_t>(std::floor(p_world.y() / voxel)) &
                   0x1FFFFF)
               << 21) |
              (static_cast<uint64_t>(
                  static_cast<int32_t>(std::floor(p_world.z() / voxel)) &
                  0x1FFFFF));
          if (!voxel_seen.insert(key).second) continue;
          cloud.push_back(p_world.cast<float>());
          std::array<uint8_t, 3> rgb{170, 170, 170};
          if (!color_img.empty()) {
            const double scale_x =
                static_cast<double>(color_img.cols) / frame.depth->width();
            const double scale_y =
                static_cast<double>(color_img.rows) / frame.depth->height();
            const int cx = std::clamp(static_cast<int>(x * scale_x), 0,
                                      color_img.cols - 1);
            const int cy = std::clamp(static_cast<int>(y * scale_y), 0,
                                      color_img.rows - 1);
            const cv::Vec3b bgr = color_img.at<cv::Vec3b>(cy, cx);
            rgb = {bgr[2], bgr[1], bgr[0]};
          }
          cloud_colors.push_back(rgb);
        }
      }
      report(BS_STAGE_LIDAR_ALIGN,
             static_cast<float>(++done) / std::max(1, posed_total));
    }

    std::error_code ec;
    fs::create_directories(fs::path(session_dir) / "final", ec);
    if (!WriteBinaryPly((fs::path(session_dir) / "final" / "dense.ply").string(),
                        cloud, cloud_colors)) {
      return fail("dense.ply write failed");
    }
    BS_LOGI("final", "dense.ply: %zu points", cloud.size());
  }

  // ------------------------------------------------------------ S11 export
  {
    ColmapModel model;
    const FrameData& ref = frames[first_posed];
    model.camera.model = camera_model;
    model.camera.width = ref.K.width;
    model.camera.height = ref.K.height;
    model.camera.params = {ref.K.fx, ref.K.fy, ref.K.cx, ref.K.cy};
    if (camera_model == "OPENCV") {
      model.camera.params.push_back(k1);
      model.camera.params.push_back(k2);
      model.camera.params.push_back(0.0);
      model.camera.params.push_back(0.0);
    }

    for (const auto& frame : frames) {
      if (!frame.posed) continue;
      ColmapImage image;
      image.image_id = frame.frame_id;
      image.pose = frame.pose;
      image.name = frame.export_name;
      image.source_jpeg_path = frame.jpeg_path;
      model.images.push_back(std::move(image));
    }

    uint64_t next_point_id = 1;
    for (const auto& track : tracks) {
      if (track.dead || !track.has_point) continue;
      ColmapPoint point;
      point.point3d_id = next_point_id++;
      point.xyz = track.X;
      point.error = track.mean_err_px;
      int rgb_sum[3] = {0, 0, 0};
      int rgb_n = 0;
      for (const auto& obs : track.observations) {
        const FrameData& f = frames[obs.frame_index];
        if (!f.posed) continue;
        // COLMAP 2D points live in the ORIGINAL (distorted) image space —
        // the exported OPENCV/PINHOLE model reapplies distortion.
        const cv::Point2f& raw = f.features.keypoints[obs.feature].pt;
        point.track.push_back({f.frame_id, -1, raw.x, raw.y});
        for (int c = 0; c < 3; ++c) rgb_sum[c] += f.colors[obs.feature][c];
        ++rgb_n;
      }
      if (point.track.size() < 2) continue;
      for (int c = 0; c < 3; ++c) {
        point.rgb[c] = static_cast<uint8_t>(rgb_sum[c] / std::max(1, rgb_n));
      }
      model.points.push_back(std::move(point));
    }

    const std::string colmap_dir =
        (fs::path(session_dir) / "final" / "colmap").string();
    if (!WriteColmapModel(model, colmap_dir)) {
      return fail("COLMAP export failed");
    }
    const std::string invalid = ValidateColmapDir(colmap_dir);
    if (!invalid.empty()) return fail("COLMAP self-validation: " + invalid);

    // Optional split export. The combined model above is always written;
    // this adds parts beside it for facilities too large to train in one
    // pass. Every part carries the SAME coordinates — no re-centring, no
    // per-part gauge — so splats trained from them separately load back
    // together already aligned. That is the whole reason to split rather
    // than to re-solve each room on its own.
    if (config.final_split_max_images > 0) {
      SplitOptions split_options;
      split_options.max_images = config.final_split_max_images;
      split_options.min_images = config.final_split_min_images;
      split_options.overlap_min_shared = config.final_split_overlap_points;
      const std::vector<ModelPart> parts =
          SplitModelByCovisibility(model, split_options);

      const fs::path parts_dir = fs::path(session_dir) / "final" / "colmap_parts";
      std::error_code parts_ec;
      fs::remove_all(parts_dir, parts_ec);

      std::ostringstream manifest;
      manifest << "{\n  \"schema_version\": 1,\n"
               << "  \"shared_world_frame\": true,\n"
               << "  \"combined_model\": \"../colmap\",\n"
               << "  \"part_count\": " << parts.size() << ",\n"
               << "  \"parts\": [\n";

      for (size_t i = 0; i < parts.size(); ++i) {
        ModelPart part = parts[i];
        char name[32];
        std::snprintf(name, sizeof(name), "part_%02u", part.index);
        const std::string part_dir = (parts_dir / name).string();
        if (!WriteColmapModel(part.model, part_dir)) {
          return fail(std::string("split export failed for ") + name);
        }
        const std::string part_invalid = ValidateColmapDir(part_dir);
        if (!part_invalid.empty()) {
          return fail(std::string(name) + " self-validation: " + part_invalid);
        }
        if (!WriteTransformsJson(part.model,
                                 (fs::path(part_dir) / "transforms.json").string())) {
          return fail(std::string("transforms.json failed for ") + name);
        }
        manifest << "    {\"name\": \"" << name << "\", \"images\": "
                 << part.model.images.size() << ", \"primary_images\": "
                 << part.primary_images << ", \"points\": "
                 << part.model.points.size() << "}"
                 << (i + 1 < parts.size() ? "," : "") << "\n";
      }
      manifest << "  ]\n}\n";

      std::ofstream manifest_out(parts_dir / "parts.json", std::ios::trunc);
      manifest_out << manifest.str();
      if (!manifest_out) return fail("parts.json write failed");
      BS_LOGI("final", "split export: %zu parts sharing one world frame",
              parts.size());
    }

    // Drop a nerfstudio/instant-ngp transforms.json beside the model so the
    // export feeds gsplat/nerfstudio directly, no COLMAP conversion step.
    const std::string transforms_path =
        (fs::path(session_dir) / "final" / "transforms.json").string();
    if (!WriteTransformsJson(model, transforms_path)) {
      return fail("transforms.json export failed");
    }

    // Metrics reflect the exported model, not the last BA round's live map.
    metrics.points = static_cast<uint32_t>(model.points.size());
    double track_len_sum = 0;
    for (const auto& p : model.points) track_len_sum += p.track.size();
    metrics.mean_track_len =
        model.points.empty()
            ? 0.0f
            : static_cast<float>(track_len_sum / model.points.size());

    // Splat readiness, recomputed from the FINAL reconstruction.
    //
    // PatchGrid scores a LiveMap, so this assembles one from the solve's own
    // frames and tracks rather than growing a second scoring implementation
    // that would drift from the live one. What the user sees while walking
    // comes from the approximate live map and answers "is this room worth
    // more of your time"; this answers "is the data you are about to spend
    // GPU hours on any good", from globally adjusted geometry with the
    // outliers already pruned.
    ReadinessReport readiness;
    {
      LiveMap scored;
      std::unordered_map<int, uint32_t> kf_of_frame;
      for (size_t i = 0; i < frames.size(); ++i) {
        const FrameData& f = frames[i];
        if (!f.posed) continue;
        Keyframe kf;
        kf.frame_id = f.frame_id;
        kf.pose = f.pose;
        kf.K = f.K;
        kf.features = f.features;      // descriptors already released
        kf.undistorted = f.undistorted;
        kf.depth = f.depth;            // drives the LiDAR-coverage axis
        kf.depth_lookup = f.lookup;
        kf.point_ids.assign(f.features.keypoints.size(), -1);
        kf_of_frame[static_cast<int>(i)] = scored.AddKeyframe(std::move(kf)).kf_id;
      }
      for (const auto& track : tracks) {
        if (track.dead || !track.has_point) continue;
        MapPoint mp;
        mp.X = track.X;
        mp.mean_gradient = track.mean_gradient;
        mp.max_tri_angle_deg = static_cast<float>(track.max_angle_deg);
        mp.last_reproj_err_px = static_cast<float>(track.mean_err_px);
        int rgb_sum[3] = {0, 0, 0};
        int rgb_n = 0;
        for (const auto& obs : track.observations) {
          const auto at = kf_of_frame.find(obs.frame_index);
          if (at == kf_of_frame.end()) continue;   // frame never registered
          mp.observations.emplace_back(at->second, obs.feature);
          const FrameData& f = frames[obs.frame_index];
          for (int c = 0; c < 3; ++c) rgb_sum[c] += f.colors[obs.feature][c];
          ++rgb_n;
        }
        if (mp.observations.size() < 2) continue;
        for (int c = 0; c < 3; ++c) {
          mp.rgb[c] = static_cast<uint8_t>(rgb_sum[c] / std::max(1, rgb_n));
        }
        const int32_t id = scored.AddPoint(std::move(mp)).id;
        // Back-link so covisibility (and therefore the room split) sees the
        // same associations the solve ended with.
        for (const auto& obs : track.observations) {
          const auto at = kf_of_frame.find(obs.frame_index);
          if (at == kf_of_frame.end()) continue;
          if (Keyframe* kf = scored.FindKeyframe(at->second)) {
            if (obs.feature >= 0 &&
                obs.feature < static_cast<int>(kf->point_ids.size())) {
              kf->point_ids[obs.feature] = id;
            }
          }
        }
      }

      if (!scored.points().empty()) {
        PatchGrid grid;
        grid.Build(scored);
        readiness.present = true;
        readiness.overall = grid.OverallScore();
        for (int a = 0; a < 5; ++a) readiness.overall_sub[a] = grid.SubScore(a);

        // Region bounds come from the patches, which is the only place they
        // exist after a solve — and what a rescan of a room would need.
        std::map<uint32_t, RegionReport> by_region;
        for (const auto& [key, patch] : grid.patches()) {
          RegionReport& r = by_region[patch.region_id];
          if (r.id == 0) {
            r.id = patch.region_id;
            for (int c = 0; c < 3; ++c) {
              r.min[c] = r.max[c] = patch.centroid[c];
            }
          }
          for (int c = 0; c < 3; ++c) {
            r.min[c] = std::min(r.min[c], patch.centroid[c]);
            r.max[c] = std::max(r.max[c], patch.centroid[c]);
          }
        }
        for (const auto& aggregate : grid.regions()) {
          RegionReport& r = by_region[aggregate.id];
          r.id = aggregate.id;
          r.name = "Room " + std::to_string(aggregate.id);
          r.score = aggregate.score;
          for (int a = 0; a < 5; ++a) r.sub[a] = aggregate.sub[a];
          r.area_m2 = aggregate.area_m2;
          r.patch_count = aggregate.patch_count;
          r.weak_area_count = aggregate.weak_area_count;
          r.worst_deficiency = aggregate.worst_deficiency;
        }
        for (auto& [id, r] : by_region) readiness.regions.push_back(r);
        BS_LOGI("final", "readiness: %.0f%% overall across %zu regions",
                readiness.overall, readiness.regions.size());
      }
    }

    // Per-image accounting. Aggregate metrics can say a reconstruction is
    // healthy while a handful of smeared or blown-out frames put a soft
    // patch on one wall, and until now nothing told the user which frames
    // those were. Residuals use the solver's own convention — undistorted
    // coordinates, surviving tracks only — so the number reported is the one
    // the optimizer actually saw.
    std::vector<ImageQuality> image_stats;
    image_stats.reserve(frames.size());
    {
      std::unordered_map<uint32_t, std::pair<double, uint32_t>> err_by_frame;
      for (const auto& track : tracks) {
        if (track.dead || !track.has_point) continue;
        for (const auto& obs : track.observations) {
          const FrameData& f = frames[obs.frame_index];
          if (!f.posed) continue;
          const Eigen::Vector3d xc = f.pose.Apply(track.X);
          if (xc.z() <= 0.01) continue;
          const Eigen::Vector2d proj = f.K.Project(xc);
          const cv::Point2f& px = f.undistorted[obs.feature];
          const double dx = proj.x() - px.x, dy = proj.y() - px.y;
          auto& acc = err_by_frame[f.frame_id];
          acc.first += dx * dx + dy * dy;
          acc.second += 1;
        }
      }
      for (const auto& frame : frames) {
        ImageQuality q;
        q.frame_id = frame.frame_id;
        q.name = frame.export_name;
        q.registered = frame.posed;
        q.lap_var = frame.lap_var;
        q.overexp_frac = frame.overexp_frac;
        const auto it = err_by_frame.find(frame.frame_id);
        if (it != err_by_frame.end() && it->second.second > 0) {
          q.observations = it->second.second;
          q.reproj_rmse_px =
              std::sqrt(it->second.first / static_cast<double>(it->second.second));
        }
        image_stats.push_back(std::move(q));
      }
    }
    const ImageReport image_report = FlagImages(std::move(image_stats));
    if (image_report.blurry > 0 || image_report.overexposed > 0 ||
        image_report.weakly_observed > 0) {
      BS_LOGI("final",
              "image quality: %u blurry, %u overexposed, %u weakly observed "
              "of %zu",
              image_report.blurry, image_report.overexposed,
              image_report.weakly_observed, image_report.images.size());
    }

    // report.json
    std::ofstream report_out(fs::path(session_dir) / "final" / "report.json",
                             std::ios::trunc);
    report_out << "{\n  \"schema_version\": 1,\n"
               << "  \"images_total\": " << metrics.images_total << ",\n"
               << "  \"images_registered\": " << metrics.images_registered
               << ",\n"
               << "  \"points\": " << model.points.size() << ",\n"
               << "  \"reproj_rmse_px\": " << metrics.reproj_rmse_px << ",\n"
               << "  \"mean_track_len\": " << metrics.mean_track_len << ",\n"
               << "  \"frames_superseded\": " << metrics.frames_superseded
               << ",\n"
               << "  \"floaters_removed\": " << metrics.floaters_removed
               << ",\n"
               << "  \"lidar_residuals\": " << metrics.lidar_residuals << ",\n"
               << "  \"ba_rounds\": " << metrics.ba_round << ",\n"
               << "  \"levelled\": " << (leveling.floor_found ? "true" : "false")
               << ",\n"
               << "  \"floor_measured\": "
               << (leveling.floor_measured ? "true" : "false") << ",\n"
               << "  \"level_rotation_deg\": " << leveling.rotation_deg << ",\n"
               << "  \"level_floor_rmse_m\": " << leveling.floor_rmse_m << ",\n"
               << "  \"level_camera_height_m\": " << leveling.camera_height_m
               << ",\n"
               << "  \"level_camera_height_spread_m\": "
               << leveling.camera_height_spread_m << ",\n"
               << "  \"walls_squared\": "
               << (leveling.walls_squared ? "true" : "false") << ",\n"
               << ReadinessReportJson(readiness)
               << ImageReportJson(image_report)
               << "  \"preset\": \"" << (fast ? "fast" : "quality") << "\"\n"
               << "}\n";
    report(BS_STAGE_EXPORT, 1.0f);
  }

  outcome.ok = true;
  outcome.metrics = metrics;
  report(BS_STAGE_DONE, 1.0f);
  return outcome;
}

}  // namespace bs
