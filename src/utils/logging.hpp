#pragma once
#include <string>
#include <iostream>
#include <mutex>

class Logger {
public:
    static void initialize(bool verbose);
    static void set_verbose(bool verbose);
    
    static void info(const std::string& message);
    static void debug(const std::string& message);
    static void error(const std::string& message);
    static void warn(const std::string& message);
    
private:
    static bool verbose_;
    static std::mutex mutex_;
};
