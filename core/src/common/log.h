#pragma once

#include <cstdarg>
#include <cstdio>

namespace bs {

enum class LogLevel { kDebug = 0, kInfo = 1, kWarn = 2, kError = 3 };

// Minimal logging: stderr with a level tag. On iOS stderr is captured by the
// unified logging console; the replay CLI prints directly. Level is a global
// set from EngineConfig.
inline LogLevel& log_level() {
  static LogLevel level = LogLevel::kInfo;
  return level;
}

inline void log_at(LogLevel level, const char* tag, const char* fmt, ...) {
  if (level < log_level()) return;
  static const char* names[] = {"D", "I", "W", "E"};
  std::fprintf(stderr, "[bs:%s] %s: ", names[static_cast<int>(level)], tag);
  va_list args;
  va_start(args, fmt);
  std::vfprintf(stderr, fmt, args);
  va_end(args);
  std::fprintf(stderr, "\n");
}

}  // namespace bs

#define BS_LOGD(tag, ...) ::bs::log_at(::bs::LogLevel::kDebug, tag, __VA_ARGS__)
#define BS_LOGI(tag, ...) ::bs::log_at(::bs::LogLevel::kInfo, tag, __VA_ARGS__)
#define BS_LOGW(tag, ...) ::bs::log_at(::bs::LogLevel::kWarn, tag, __VA_ARGS__)
#define BS_LOGE(tag, ...) ::bs::log_at(::bs::LogLevel::kError, tag, __VA_ARGS__)
