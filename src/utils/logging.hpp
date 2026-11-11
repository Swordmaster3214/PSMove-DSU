#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

enum class LogLevel { Debug, Info, Warn, Error };

struct LogEntry {
    uint64_t seq;          // Monotonic sequence number
    double time_s;         // Seconds since program start
    LogLevel level;
    std::string message;
};

class Logger {
public:
    static void initialize(bool verbose);
    static void set_verbose(bool verbose);
    static bool is_verbose();

    static void info(const std::string& message);
    static void debug(const std::string& message);
    static void error(const std::string& message);
    static void warn(const std::string& message);

    // Copies entries with seq >= since_seq into `out`. Returns the next seq to ask for.
    static uint64_t get_entries(uint64_t since_seq, std::vector<LogEntry>& out);
    static void clear_history();
    static const char* level_name(LogLevel level);

private:
    static void write(LogLevel level, const std::string& message);

    static std::atomic<bool> verbose_;
    static std::mutex mutex_;
    static std::deque<LogEntry> history_;
    static uint64_t next_seq_;
};
