#include "utils/logging.hpp"
#include <chrono>

namespace {
constexpr size_t MAX_HISTORY = 4000;
const auto program_start = std::chrono::steady_clock::now();
}

std::atomic<bool> Logger::verbose_{false};
std::mutex Logger::mutex_;
std::deque<LogEntry> Logger::history_;
uint64_t Logger::next_seq_ = 0;

void Logger::initialize(bool verbose) {
    verbose_ = verbose;
}

void Logger::set_verbose(bool verbose) {
    verbose_ = verbose;
}

bool Logger::is_verbose() {
    return verbose_.load();
}

const char* Logger::level_name(LogLevel level) {
    switch (level) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Error: return "ERROR";
    }
    return "?";
}

void Logger::write(LogLevel level, const std::string& message) {
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - program_start).count();
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostream& os = (level == LogLevel::Error) ? std::cerr : std::cout;
    os << "[" << level_name(level) << "] " << message << std::endl;

    history_.push_back({next_seq_++, t, level, message});
    if (history_.size() > MAX_HISTORY) history_.pop_front();
}

void Logger::info(const std::string& message) {
    write(LogLevel::Info, message);
}

void Logger::debug(const std::string& message) {
    if (!verbose_.load(std::memory_order_relaxed)) return;
    write(LogLevel::Debug, message);
}

void Logger::error(const std::string& message) {
    write(LogLevel::Error, message);
}

void Logger::warn(const std::string& message) {
    write(LogLevel::Warn, message);
}

uint64_t Logger::get_entries(uint64_t since_seq, std::vector<LogEntry>& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& e : history_) {
        if (e.seq >= since_seq) out.push_back(e);
    }
    return next_seq_;
}

void Logger::clear_history() {
    std::lock_guard<std::mutex> lock(mutex_);
    history_.clear();
}
