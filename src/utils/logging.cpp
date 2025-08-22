#include "utils/logging.hpp"
#include <chrono>
#include <iomanip>

bool Logger::verbose_ = false;
std::mutex Logger::mutex_;

void Logger::initialize(bool verbose) {
    verbose_ = verbose;
}

void Logger::set_verbose(bool verbose) {
    verbose_ = verbose;
}

void Logger::info(const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "[INFO] " << message << std::endl;
}

void Logger::debug(const std::string& message) {
    if (!verbose_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "[DEBUG] " << message << std::endl;
}

void Logger::error(const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cerr << "[ERROR] " << message << std::endl;
}

void Logger::warn(const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "[WARN] " << message << std::endl;
}
