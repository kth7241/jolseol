// traffic_monitor/include/common/logging.hpp
#pragma once
/**
 * @file logging.hpp
 * @brief Minimal thread-safe logger.
 */

#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace traffic {

enum class LogLevel {
  Debug = 0,
  Info  = 1,
  Warn  = 2,
  Error = 3,
  None  = 4
};

class Logger {
public:
  static void setLevel(LogLevel lv);
  static LogLevel level();

  // printf-style
  static void log(LogLevel lv, const char* fmt, ...);

private:
  static const char* levelStr(LogLevel lv);
  static std::mutex& mutexRef();
  static LogLevel& levelRef();
};

} // namespace traffic

#define TM_LOGD(...) ::traffic::Logger::log(::traffic::LogLevel::Debug, __VA_ARGS__)
#define TM_LOGI(...) ::traffic::Logger::log(::traffic::LogLevel::Info,  __VA_ARGS__)
#define TM_LOGW(...) ::traffic::Logger::log(::traffic::LogLevel::Warn,  __VA_ARGS__)
#define TM_LOGE(...) ::traffic::Logger::log(::traffic::LogLevel::Error, __VA_ARGS__)

// -----------------------------------------------------------------------------
// Compatibility macros
// 일부 소스가 LOG_INFO/LOGW/LOGE 형태를 사용하므로 별칭을 제공한다.
// -----------------------------------------------------------------------------
#ifndef LOG_INFO
#define LOG_INFO(...) TM_LOGI(__VA_ARGS__)
#endif

#ifndef LOGW
#define LOGW(...) TM_LOGW(__VA_ARGS__)
#endif

#ifndef LOGE
#define LOGE(...) TM_LOGE(__VA_ARGS__)
#endif
