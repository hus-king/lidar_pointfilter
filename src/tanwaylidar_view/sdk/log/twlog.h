#ifndef TWLOG_H
#define TWLOG_H

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
// #include <os-inl.h>

enum LOGLEVEL {
  LOG_OFF = 0,
  LOG_INFO,
  LOG_TEST_POINT,
  LOG_TEST_TIME,
  LOG_DEBUG
};

class TWLOG {
public:
  static TWLOG &GetInstance();
  void SetFileName(const std::string &filename);
  void SetLogLevel(LOGLEVEL level);
  void Start();
  void Stop();
  void LogDebug(const char *file, int line, const char *func, const char *fmt,
                ...);
  void LogInfo(const char *file, int line, const char *func, const char *fmt,
               ...);
  void LogTestPoint(const char *file, int line, const char *func,
                    const char *fmt, ...);
  void LogTestTime(const char *file, int line, const char *func,
                   const char *fmt, ...);

  LOGLEVEL StringToLogLevel(const std::string &levelStr) {
    if (levelStr == "LOG_OFF")
      return LOG_OFF;
    if (levelStr == "LOG_INFO")
      return LOG_INFO;
    if (levelStr == "LOG_TEST_POINT")
      return LOG_TEST_POINT;
    if (levelStr == "LOG_TEST_TIME")
      return LOG_TEST_TIME;
    if (levelStr == "LOG_DEBUG")
      return LOG_DEBUG;
    return LOG_OFF;
  }

private:
  TWLOG() : log_level_(LOG_OFF) {} // 默认日志等级为 LOG_OFF
  TWLOG(const TWLOG &) = delete;
  TWLOG &operator=(const TWLOG &) = delete;

  void ThreadFunc();
  std::string GenerateTimestamp();
  std::string FormatLogMessage(LOGLEVEL level, const char *file, int line,
                               const char *func, const char *msg);
  // 声明 Log 函数
  void Log(LOGLEVEL level, const char *file, int line, const char *func,
           const char *fmt, va_list args);

  std::string filename_;
  std::ofstream logfile_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool exit_ = false;
  std::queue<std::string> log_queue_;
  bool running_ = false;
  LOGLEVEL log_level_;
};

#endif // TWLOG_H
