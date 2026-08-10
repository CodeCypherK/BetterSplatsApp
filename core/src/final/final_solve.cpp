#include "final/final_solve.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
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

uint64_t CacheConfigHash(const EngineConfig& c, bool use_sift, bool fast,
                         const std::vector<uint32_t>& frame_ids) {
  uint64_t h = 1469598103934665603ull;
  const int schema = 1;
  h = Fnv1a(h, &schema, sizeof(schema));
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

  // Feature choice: SIFT for the quality preset when the session fits the
  // transient descriptor budget (or forced by config), else ORB.
  const size_t session_frames = session->frame_ids().size();
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
      CacheConfigHash(config, use_sift, fast, session->frame_ids());
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
    for (const uint32_t frame_id : session->frame_ids()) {
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
  struct PairMatches {
    int a, b;
    std::vector<Match> inliers;
  };
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

  // ---------------------------------------------------------- S6 pose init
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
    metrics.images_registered = posed;
    if (posed < 2) return fail("live initialization has <2 posed frames");
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
      if (!frame.posed || frame.depth) continue;
      const auto meta = session->ReadMeta(frame.frame_id);
      const auto depth_img = session->ReadDepth(frame.frame_id);
      if (!meta || !depth_img) continue;
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
      const auto& dK = frame.depth->K();
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

    // Metrics reflect the exported model, not the last BA round's live map.
    metrics.points = static_cast<uint32_t>(model.points.size());
    double track_len_sum = 0;
    for (const auto& p : model.points) track_len_sum += p.track.size();
    metrics.mean_track_len =
        model.points.empty()
            ? 0.0f
            : static_cast<float>(track_len_sum / model.points.size());

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
               << "  \"floaters_removed\": " << metrics.floaters_removed
               << ",\n"
               << "  \"lidar_residuals\": " << metrics.lidar_residuals << ",\n"
               << "  \"ba_rounds\": " << metrics.ba_round << ",\n"
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
