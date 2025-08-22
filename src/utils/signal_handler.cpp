#include "utils/signal_handler.hpp"

std::atomic<bool> SignalHandler::exit_requested_(false);

void SignalHandler::setup() {
#ifdef _WIN32
    SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
#else
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);
#endif
}

bool SignalHandler::should_exit() {
    return exit_requested_.load();
}

#ifdef _WIN32
BOOL WINAPI SignalHandler::console_ctrl_handler(DWORD dwCtrlType) {
    switch (dwCtrlType) {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            exit_requested_ = true;
            return TRUE;
        default:
            return FALSE;
    }
}
#else
void SignalHandler::handle_sigint(int) {
    exit_requested_ = true;
}
#endif
