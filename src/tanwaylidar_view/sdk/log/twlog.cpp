#include "twlog.h"
#include <cstdarg>
#include <ctime>
#include <iostream>
#include <time.h>

TWLOG &TWLOG::GetInstance() {
  static TWLOG instance;
  return instance;
}

void TWLOG::SetFileName(const std::string &filename) { filename_ = filename; }
void TWLOG::SetLogLevel(LOGLEVEL level) { log_level_ = level; }
void TWLOG::Start() {
  if (running_)
    return;

  if (filename_.empty()) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::tm buf;
#if defined(_WIN32)
    localtime_s(&buf, &in_time_t); // Windows-specific
#else
    localtime_r(&in_time_t, &buf); // POSIX-compliant systems
#endif

    std::stringstream ss;
    ss << std::put_time(&buf, "%Y%m%d%H%M%S.log");

    filename_ = ss.str();
  }

  logfile_.open(filename_, std::ios::out | std::ios::trunc);
  if (!logfile_.is_open()) {
    std::cerr << "Failed to open log file: " << filename_ << std::endl;
    return;
  }
  // else std::cout<<"open file"<<std::endl;
  thread_ = std::thread(&TWLOG::ThreadFunc, this);
  running_ = true;
}

void TWLOG::Stop() {
  if (!running_)
    return;

  exit_ = true;
  cv_.notify_one();

  if (thread_.joinable()) {
    thread_.join();
  }

  logfile_.close();
  running_ = false;
}

std::string TWLOG::FormatLogMessage(LOGLEVEL level, const char *file, int line,
                                    const char *func, const char *msg) {
  const char *level_str;
  switch (level) {
  case LOG_DEBUG:
    level_str = "DEBUG";
    break;
  case LOG_INFO:
    level_str = "INFO";
    break;
  case LOG_TEST_POINT:
    level_str = "TEST_POINT";
    break;
  case LOG_TEST_TIME:
    level_str = "TEST_TIME";
    break;
  default:
    level_str = "OFF";
    break;
  }
  return " [" + GenerateTimestamp() + "] [line:" + std::to_string(line) +
         "] [" + std::string(level_str) + "]:" + msg;
  // return " ["+ GenerateTimestamp() + "] [" + std::string(file) + ":" +
  // std::to_string(line)  + "] [" + std::string(level_str) + "]:" + msg;
}

void TWLOG::Log(LOGLEVEL level, const char *file, int line, const char *func,
                const char *fmt, va_list args) {
  if (level > log_level_) {
    return; // 如果当前日志等级高于配置的日志等级，则不记录日志
  }
  if (level == LOG_TEST_POINT && log_level_ == LOG_TEST_TIME) {
    return; // 如果配置的等级是LOG_TEST_TIME（解算时间），则不输出LOG_TEST_POINT（解算点信息）
  }

  char msg[256];
  vsnprintf(msg, sizeof(msg), fmt, args);

  std::lock_guard<std::mutex> lock(mutex_);
  log_queue_.push(FormatLogMessage(level, file, line, func, msg));
  cv_.notify_one();
  // std::cerr << "Log message added to queue: " << msg << std::endl;
}
void TWLOG::LogDebug(const char *file, int line, const char *func,
                     const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Log(LOG_DEBUG, file, line, func, fmt, args);
  va_end(args);
}

void TWLOG::LogInfo(const char *file, int line, const char *func,
                    const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Log(LOG_INFO, file, line, func, fmt, args);
  va_end(args);
}
void TWLOG::LogTestPoint(const char *file, int line, const char *func,
                         const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Log(LOG_TEST_POINT, file, line, func, fmt, args);
  va_end(args);
}
void TWLOG::LogTestTime(const char *file, int line, const char *func,
                        const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Log(LOG_TEST_TIME, file, line, func, fmt, args);
  va_end(args);
}

void TWLOG::ThreadFunc() {
  while (!exit_) {
    // std::cerr << "Log thread started" << std::endl;
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return !log_queue_.empty() || exit_; });

    if (exit_ && log_queue_.empty()) {
      break;
    }

    std::string log_message = log_queue_.front();
    log_queue_.pop();

    lock.unlock();

    logfile_ << log_message << std::endl;
    logfile_.flush();
    // std::cerr << "Log message written to file: " << log_message << std::endl;
  }
  std::cerr << "Log thread stopped" << std::endl;
}

std::string TWLOG::GenerateTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  // Convert to local time safely and portably
  std::tm buf;
#if defined(_WIN32)
  localtime_s(&buf, &in_time_t); // Windows-specific
#else
  localtime_r(&in_time_t, &buf); // POSIX-compliant systems
#endif

  std::ostringstream oss;
  oss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S");

  return oss.str();
}
