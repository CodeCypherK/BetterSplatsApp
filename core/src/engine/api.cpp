// C ABI implementation: thin translation onto bs::Engine. Every function is
// null-safe — the app must never be able to crash the process through this
// boundary with bad handles.

#include <cstring>
#include <new>

#include "bs/bs_api.h"
#include "engine/engine_impl.h"
#include "io/depth_codec.h"

struct bs_engine {
  bs::Engine impl;
  explicit bs_engine(bs::EngineConfig config) : impl(std::move(config)) {}
};

extern "C" {

bs_engine* bs_create(const char* config_json) {
  bool ok = true;
  bs::EngineConfig config = bs::EngineConfig::FromJson(config_json, &ok);
  return new (std::nothrow) bs_engine(config);
}

void bs_destroy(bs_engine* e) { delete e; }

const char* bs_version(void) { return BS_VERSION_NUM " (core " BS_GIT_HASH ")"; }

const char* bs_last_error(const bs_engine* e) {
  if (e == nullptr) return "null engine";
  return e->impl.LastError();
}

bs_result bs_live_begin(bs_engine* e, const char* session_dir) {
  if (e == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.LiveBegin(session_dir);
}

bs_result bs_live_feed(bs_engine* e, const bs_frame_in* frame) {
  if (e == nullptr || frame == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.LiveFeed(*frame);
}

bs_result bs_live_poll_status(bs_engine* e, bs_live_status* out) {
  if (e == nullptr || out == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.LivePollStatus(*out);
}

bs_result bs_live_end(bs_engine* e) {
  if (e == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.LiveEnd();
}

bs_result bs_snapshot_acquire(bs_engine* e, bs_snapshot* out) {
  if (e == nullptr || out == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.SnapshotAcquire(*out);
}

void bs_snapshot_release(bs_engine* e, bs_snapshot* snap) {
  if (e == nullptr || snap == nullptr) return;
  e->impl.SnapshotRelease(*snap);
}

bs_result bs_final_start(bs_engine* e, const char* session_dir, const char* preset) {
  if (e == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.FinalStart(session_dir, preset);
}

bs_result bs_final_poll(bs_engine* e, bs_final_progress* out) {
  if (e == nullptr || out == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.FinalPoll(*out);
}

bs_result bs_final_cancel(bs_engine* e) {
  if (e == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.FinalCancel();
}

bs_result bs_thermal_hint(bs_engine* e, int32_t level) {
  if (e == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.ThermalHint(level);
}

bs_result bs_region_rename(bs_engine* e, uint32_t region_id, const char* utf8_name) {
  if (e == nullptr) return BS_ERR_INVALID_ARGUMENT;
  return e->impl.RegionRename(region_id, utf8_name);
}

const uint8_t* bs_depth_encode(const uint16_t* f16, int32_t width,
                               int32_t height, size_t* out_len) {
  if (f16 == nullptr || width <= 0 || height <= 0 || width > 0xFFFF ||
      height > 0xFFFF || out_len == nullptr) {
    return nullptr;
  }
  bs::DepthImage depth;
  depth.width = width;
  depth.height = height;
  depth.f16.assign(f16, f16 + static_cast<size_t>(width) * height);
  const std::vector<uint8_t> encoded = bs::EncodeDepth(depth, /*compress=*/true);

  uint8_t* buf = new (std::nothrow) uint8_t[encoded.size()];
  if (buf == nullptr) return nullptr;
  std::memcpy(buf, encoded.data(), encoded.size());
  *out_len = encoded.size();
  return buf;
}

void bs_buffer_release(const uint8_t* buf) { delete[] buf; }

}  // extern "C"
