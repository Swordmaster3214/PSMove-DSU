#pragma once
#include <atomic>
#include <signal.h>

#ifdef _WIN32
#include <windows.h>
#endif

class SignalHandler {
public:
    static void setup();
    static bool should_exit();
    
private:
    static std::atomic<bool> exit_requested_;
    
#ifdef _WIN32
    static BOOL WINAPI console_ctrl_handler(DWORD dwCtrlType);
#else
    static void handle_sigint(int);
#endif
};
