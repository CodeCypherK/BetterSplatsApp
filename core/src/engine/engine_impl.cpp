#include "engine/engine_impl.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>

#include <opencv2/imgproc.hpp>

#include "calib/lut_fit.h"
#include "common/log.h"
#include "io/float16.h"
#include "io/session_schema.h"

namespace bs {

namespace fs = std::filesystem;

Engine::Engine(EngineConfig config) : config_(config) {
  log_level() = static_cast<LogLevel>(config_.log_level);
  BS_LOGI("engine", "created");
}

Engine::~Engine() {
  if (final_) {
    final_->cancel.store(true);
    if (final_->worker.joinable()) final_->worker.join();
  }
  if (live_ && live_->state() != BS_LIVE_FINISHED) {
    LiveEnd();
  }
}

bs_result Engine::Fail(bs_result code, const std::string& message) {
  last_error_ = message;
  BS_LOGE("engine", "%s", message.c_str());
  return code;
}

const char* Engine::LastError() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_.c_str();
}

bs_result Engine::LiveBegin(const char* session_dir) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_dir == nullptr || session_dir[0] == '\0') {
    return Fail(BS_ERR_INVALID_ARGUMENT, "LiveBegin: empty session_dir");
  }
  if (live_ && live_->state() != BS_LIVE_FINISHED &&
      live_->state() != BS_LIVE_IDLE) {
    return Fail(BS_ERR_INVALID_STATE, "LiveBegin: live session already active");
  }

  std::error_code ec;
  fs::create_directories(fs::path(session_dir) / "live", ec);
  if (ec) {
    return Fail(BS_ERR_IO, "LiveBegin: cannot create live/ in " +
                               std::string(session_dir) + ": " + ec.message());
  }

  session_dir_ = session_dir;
  frames_fed_ = 0;
  last_frame_id_ = 0;
  distortion_ready_ = false;
  k1_ = k2_ = 0;

  // Session calibration, when already on disk (replay path; the device app
  // writes it after the first frame, handled by MaybeFitDistortion).
  const std::string calib_text =
      ReadTextFile((fs::path(session_dir) / "calibration.json").string());
  if (!calib_text.empty()) {
    if (const auto calib = CalibrationInfo::FromJson(calib_text)) {
      if (calib->colmap_model && calib->colmap_model->params.size() >= 6 &&
          calib->colmap_model->model == "OPENCV") {
        k1_ = calib->colmap_model->params[4];
        k2_ = calib->colmap_model->params[5];
        distortion_ready_ = true;
      } else if (!calib->distortion_lut.empty()) {
        if (const auto fit = FitOpencvModelFromLut(
                calib->distortion_lut, calib->intrinsics_session)) {
          k1_ = fit->k1;
          k2_ = fit->k2;
          distortion_ready_ = true;
        }
      } else if (calib->colmap_model &&
                 calib->colmap_model->model == "PINHOLE") {
        distortion_ready_ = true;  // exact pinhole (synthetic sessions)
      }
    }
  }

  live_ = std::make_unique<LiveSystem>(config_);
  live_->Begin(session_dir_, k1_, k2_);
  BS_LOGI("engine", "live session started at %s", session_dir);
  return BS_OK;
}

void Engine::MaybeFitDistortion(const bs_frame_in& frame) {
  if (distortion_ready_) return;
  if (frame.lut == nullptr || frame.lut_count <= 0 || frame.lut_ref_width <= 0) {
    return;
  }
  DistortionLut lut;
  lut.magnification.assign(frame.lut, frame.lut + frame.lut_count);
  if (frame.lut_inverse != nullptr) {
    lut.inverse.assign(frame.lut_inverse, frame.lut_inverse + frame.lut_count);
  }
  lut.center_x = frame.lut_center_x;
  lut.center_y = frame.lut_center_y;

  PinholeIntrinsics pin;
  const double sx =
      static_cast<double>(frame.lut_ref_width) / frame.luma_width;
  pin.fx = frame.fx * sx;
  pin.fy = frame.fy * sx;
  pin.cx = frame.cx * sx;
  pin.cy = frame.cy * sx;
  pin.ref_w = frame.lut_ref_width;
  pin.ref_h = frame.lut_ref_height;

  if (const auto fit = FitOpencvModelFromLut(lut, pin)) {
    k1_ = fit->k1;
    k2_ = fit->k2;
    distortion_ready_ = true;
    live_->SetDistortion(k1_, k2_);
    BS_LOGI("engine", "distortion fit from frame LUT: k1=%.4f k2=%.4f (%.2fpx)",
            k1_, k2_, fit->max_residual_px);
  }
}

bs_result Engine::LiveFeed(const bs_frame_in& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!live_ || (live_->state() != BS_LIVE_INITIALIZING &&
                 live_->state() != BS_LIVE_TRACKING &&
                 live_->state() != BS_LIVE_LOST)) {
    return Fail(BS_ERR_INVALID_STATE, "LiveFeed: no live session");
  }
  if (frame.luma == nullptr || frame.luma_width <= 0 || frame.luma_height <= 0) {
    return Fail(BS_ERR_INVALID_ARGUMENT, "LiveFeed: missing luma plane");
  }
  if (frame.frame_id <= last_frame_id_ && frames_fed_ > 0) {
    return Fail(BS_ERR_INVALID_ARGUMENT, "LiveFeed: frame_id not increasing");
  }

  MaybeFitDistortion(frame);

  LiveFrameInput input;
  input.frame_id = frame.frame_id;
  input.t_capture = frame.t_capture;
  input.t_depth = frame.t_depth;

  // Wrap the borrowed luma, then normalize to tracking resolution (<=960
  // wide) with intrinsics scaled to match.
  const cv::Mat luma(frame.luma_height, frame.luma_width, CV_8UC1,
                     const_cast<uint8_t*>(frame.luma),
                     static_cast<size_t>(frame.luma_stride));
  Intrinsics K{frame.fx, frame.fy, frame.cx, frame.cy, frame.luma_width,
               frame.luma_height};
  if (frame.luma_width > 1280) {
    const int new_w = 960;
    const int new_h =
        static_cast<int>(std::lround(960.0 * frame.luma_height /
                                     frame.luma_width));
    cv::resize(luma, input.gray, cv::Size(new_w, new_h), 0, 0, cv::INTER_AREA);
    K = K.ScaledTo(new_w, new_h);
  } else {
    input.gray = luma.clone();
  }
  input.K = K;

  if (frame.depth != nullptr && frame.depth_width > 0 &&
      frame.depth_height > 0) {
    input.depth.width = frame.depth_width;
    input.depth.height = frame.depth_height;
    const size_t n =
        static_cast<size_t>(frame.depth_width) * frame.depth_height;
    input.depth.f16.resize(n);
    for (size_t i = 0; i < n; ++i) {
      input.depth.f16[i] = F32ToF16(frame.depth[i]);
    }
    input.Kd = {frame.dfx, frame.dfy, frame.dcx, frame.dcy,
                frame.depth_width, frame.depth_height};
  }

  if (frame.gyro_valid) {
    input.gyro = Eigen::Vector3f(frame.gyro_dx, frame.gyro_dy, frame.gyro_dz);
    input.gyro_valid = true;
  }

  ++frames_fed_;
  last_frame_id_ = frame.frame_id;
  live_->Feed(input);
  return BS_OK;
}

bs_result Engine::LivePollStatus(bs_live_status& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::memset(&out, 0, sizeof(out));
  out.q[0] = 1.0;
  if (!live_) {
    out.state = BS_LIVE_IDLE;
    return BS_OK;
  }
  live_->FillStatus(out);
  out.frames_fed = frames_fed_;

  const std::vector<bs_store_directive> directives = live_->DrainDirectives();
  out.directive_count = static_cast<int32_t>(directives.size());
  for (size_t i = 0; i < directives.size(); ++i) {
    out.directives[i] = directives[i];
  }
  return BS_OK;
}

bs_result Engine::LiveEnd() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!live_ || live_->state() == BS_LIVE_IDLE ||
      live_->state() == BS_LIVE_FINISHED) {
    return Fail(BS_ERR_INVALID_STATE, "LiveEnd: no live session");
  }
  const bool ok = live_->End();
  BS_LOGI("engine", "live session ended after %u frames", frames_fed_);
  return ok ? BS_OK : Fail(BS_ERR_IO, "LiveEnd: failed to flush live/ state");
}

bs_result Engine::SnapshotAcquire(bs_snapshot& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::memset(&out, 0, sizeof(out));

  auto* storage = new (std::nothrow) SnapshotStorage;
  if (storage == nullptr) {
    return Fail(BS_ERR_INTERNAL, "SnapshotAcquire: allocation failed");
  }
  if (live_) {
    live_->FillSnapshot(storage->points, storage->cameras);
    live_->readiness().FillSnapshot(storage->patches, storage->regions,
                                    storage->weak_areas,
                                    live_->region_name().c_str());
  }

  out._h = storage;
  out.points = storage->points.data();
  out.point_count = static_cast<uint32_t>(storage->points.size());
  out.cameras = storage->cameras.data();
  out.camera_count = static_cast<uint32_t>(storage->cameras.size());
  out.patches = storage->patches.data();
  out.patch_count = static_cast<uint32_t>(storage->patches.size());
  out.regions = storage->regions.data();
  out.region_count = static_cast<uint32_t>(storage->regions.size());
  out.weak_areas = storage->weak_areas.data();
  out.weak_area_count = static_cast<uint32_t>(storage->weak_areas.size());

  float lo[3] = {1e9f, 1e9f, 1e9f};
  float hi[3] = {-1e9f, -1e9f, -1e9f};
  for (const auto& p : storage->points) {
    const float v[3] = {p.x, p.y, p.z};
    for (int i = 0; i < 3; ++i) {
      lo[i] = std::min(lo[i], v[i]);
      hi[i] = std::max(hi[i], v[i]);
    }
  }
  for (int i = 0; i < 3; ++i) {
    out.bounds_min[i] = storage->points.empty() ? 0.0f : lo[i];
    out.bounds_max[i] = storage->points.empty() ? 0.0f : hi[i];
  }
  return BS_OK;
}

void Engine::SnapshotRelease(bs_snapshot& snap) {
  delete static_cast<SnapshotStorage*>(snap._h);
  std::memset(&snap, 0, sizeof(snap));
}

bs_result Engine::FinalStart(const char* session_dir, const char* preset) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_dir == nullptr || session_dir[0] == '\0') {
    return Fail(BS_ERR_INVALID_ARGUMENT, "FinalStart: empty session_dir");
  }
  if (final_ && final_->running.load()) {
    return Fail(BS_ERR_BUSY, "FinalStart: a final solve is already running");
  }
  if (final_ && final_->worker.joinable()) final_->worker.join();

  final_ = std::make_unique<FinalState>();
  final_->running.store(true);
  final_->stage.store(BS_STAGE_FEATURES);

  FinalState* state = final_.get();
  const std::string dir = session_dir;
  const std::string preset_str = preset != nullptr ? preset : "quality";
  const EngineConfig config = config_;
  state->worker = std::thread([state, dir, preset_str, config] {
    const FinalOutcome outcome = RunFinalSolve(
        config, dir, preset_str,
        [state](bs_final_stage stage, float pct, const FinalMetrics& metrics) {
          state->stage.store(stage);
          state->stage_progress.store(pct);
          std::lock_guard<std::mutex> lock(state->metrics_mutex);
          state->metrics = metrics;
        },
        &state->cancel);
    {
      std::lock_guard<std::mutex> lock(state->metrics_mutex);
      state->outcome = outcome;
      state->metrics = outcome.metrics;
    }
    state->stage.store(outcome.ok
                           ? BS_STAGE_DONE
                           : (outcome.cancelled ? BS_STAGE_IDLE
                                                : BS_STAGE_FAILED));
    state->running.store(false);
  });
  return BS_OK;
}

bs_result Engine::FinalPoll(bs_final_progress& out) {
  std::memset(&out, 0, sizeof(out));
  if (!final_) {
    out.stage = BS_STAGE_IDLE;
    return BS_OK;
  }
  out.stage = final_->stage.load();
  out.stage_progress = final_->stage_progress.load();
  out.running = final_->running.load() ? 1 : 0;
  out.paused_thermal = thermal_level_.load() >= 2 ? 1 : 0;
  // Rough overall progress: stages weighted equally through EXPORT.
  const float stage_index = static_cast<float>(
      std::clamp<int32_t>(out.stage, BS_STAGE_FEATURES, BS_STAGE_DONE) -
      BS_STAGE_FEATURES);
  const float stage_span =
      static_cast<float>(BS_STAGE_DONE - BS_STAGE_FEATURES);
  out.total_progress =
      out.stage == BS_STAGE_DONE
          ? 1.0f
          : std::min(0.99f, (stage_index + out.stage_progress) / stage_span);

  std::lock_guard<std::mutex> lock(final_->metrics_mutex);
  out.images_total = final_->metrics.images_total;
  out.images_registered = final_->metrics.images_registered;
  out.points = final_->metrics.points;
  out.reproj_rmse_px = final_->metrics.reproj_rmse_px;
  out.mean_track_len = final_->metrics.mean_track_len;
  out.ba_round = final_->metrics.ba_round;
  if (final_->stage.load() == BS_STAGE_FAILED &&
      !final_->outcome.error.empty()) {
    last_error_ = final_->outcome.error;
  }
  return BS_OK;
}

bs_result Engine::FinalCancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (final_) final_->cancel.store(true);
  return BS_OK;
}

bs_result Engine::ThermalHint(int32_t level) {
  if (level < 0 || level > 3) {
    std::lock_guard<std::mutex> lock(mutex_);
    return Fail(BS_ERR_INVALID_ARGUMENT, "ThermalHint: level out of range");
  }
  thermal_level_.store(level);
  return BS_OK;
}

bs_result Engine::RegionRename(uint32_t region_id, const char* utf8_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (utf8_name == nullptr || utf8_name[0] == '\0') {
    return Fail(BS_ERR_INVALID_ARGUMENT, "RegionRename: empty name");
  }
  if (!live_) {
    return Fail(BS_ERR_INVALID_STATE, "RegionRename: no live session");
  }
  if (region_id != 1) {
    return Fail(BS_ERR_INVALID_ARGUMENT,
                "RegionRename: unknown region id (auto-clustering into "
                "multiple regions is a later milestone)");
  }
  live_->RenameRegion(utf8_name);
  return BS_OK;
}

}  // namespace bs
