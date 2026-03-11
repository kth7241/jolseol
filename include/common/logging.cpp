// traffic_monitor/src/common/logging.cpp
#include "common/logging.hpp"

namespace traffic {

std::mutex& Logger::mutexRef() {
  static std::mutex m;
  return m;
}

LogLevel& Logger::levelRef() {
  static LogLevel lv = LogLevel::Info;
  return lv;
}

void Logger::setLevel(LogLevel lv) {
  std::lock_guard<std::mutex> lk(mutexRef());
  levelRef() = lv;
}

LogLevel Logger::level() {
  std::lock_guard<std::mutex> lk(mutexRef());
  return levelRef();
}

const char* Logger::levelStr(LogLevel lv) {
  switch (lv) {
    case LogLevel::Debug: return "D";
    case LogLevel::Info:  return "I";
    case LogLevel::Warn:  return "W";
    case LogLevel::Error: return "E";
    default:              return "-";
  }
}

void Logger::log(LogLevel lv, const char* fmt, ...) {
  // 레벨 필터
  {
    std::lock_guard<std::mutex> lk(mutexRef());
    if (lv < levelRef()) return;
  }

  std::lock_guard<std::mutex> lk(mutexRef());
  std::fprintf(stderr, "[%s] ", levelStr(lv));

  va_list args;
  va_start(args, fmt);
  std::vfprintf(stderr, fmt, args);
  va_end(args);

  std::fprintf(stderr, "\n");
  std::fflush(stderr);
}

} // namespace traffic