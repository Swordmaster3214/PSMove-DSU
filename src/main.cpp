#include "config/constants.hpp"
#include "config/config.hpp"
#include "config/config_file.hpp"
#include "utils/logging.hpp"
#include "utils/signal_handler.hpp"
#include "utils/licenses.hpp"
#include "psmove/psmove_pairing.hpp"
#include "psmove/linux_setup.hpp"
#include "dsu/dsu_server.hpp"
#ifdef DSU_WITH_GUI
#include "gui/gui_app.hpp"
#endif
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {
void print_usage(const char* exe) {
    std::cout <<
        "Usage: " << exe << " [options]\n"
        "\n"
        "  --headless          Run without the GUI (console only)\n"
        "  --pair              Pair a USB-connected controller from the console\n"
        "  --linux-setup       Linux: install udev rules / BlueZ setting (needs root)\n"
        "  --verbose           Detailed logs and latency diagnostics\n"
        "  --config <path>     Settings file (default: " << ConfigFile::default_path() << ")\n"
        "  --map <p> <y> <r>   Gyro source axis (0=X 1=Y 2=Z) for pitch, yaw, roll\n"
        "  --flip <p> <y> <r>  Invert gyro pitch, yaw, roll (0/1)\n"
        "  --simulate <n>      Add n virtual controllers (for testing without hardware)\n"
        "  --licenses          Show the license of this program and the code it includes\n"
        "  --help              Show this help\n";
}
} // namespace

int main(int argc, char** argv) {
    Config config;

    // First pass: options that affect where settings come from.
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config.config_path = argv[++i];
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (std::strcmp(argv[i], "--licenses") == 0) {
            Licenses::print_all(std::cout);
            return 0;
        }
    }
    if (config.config_path.empty()) config.config_path = ConfigFile::default_path();

    // Saved settings, then command-line overrides on top.
    std::string config_error;
    if (!ConfigFile::load(config.config_path, config, config_error)) {
        std::cerr << "[WARN] " << config_error << " - using default settings\n";
    }

    bool linux_setup_only = false;
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--verbose") == 0) {
            config.verbose = true;
            Logger::set_verbose(true);
        } else if (std::strcmp(argv[i], "--pair") == 0) {
            config.pair_mode = true;
        } else if (std::strcmp(argv[i], "--linux-setup") == 0) {
            linux_setup_only = true;
        } else if (std::strcmp(argv[i], "--headless") == 0 || std::strcmp(argv[i], "--no-gui") == 0) {
            config.headless = true;
        } else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            ++i;  // handled above
        } else if (std::strcmp(argv[i], "--simulate") == 0 && i + 1 < argc) {
            config.simulate = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--map") == 0 && i + 3 < argc) {
            config.map_pitch = std::atoi(argv[++i]);
            config.map_yaw = std::atoi(argv[++i]);
            config.map_roll = std::atoi(argv[++i]);
            std::cout << "Mapping set pitch=" << config.map_pitch
                      << " yaw=" << config.map_yaw
                      << " roll=" << config.map_roll << "\n";
        } else if (std::strcmp(argv[i], "--flip") == 0 && i + 3 < argc) {
            config.flip_pitch = std::atoi(argv[++i]) != 0;
            config.flip_yaw = std::atoi(argv[++i]) != 0;
            config.flip_roll = std::atoi(argv[++i]) != 0;
            std::cout << "Flip set pitch=" << config.flip_pitch
                      << " yaw=" << config.flip_yaw
                      << " roll=" << config.flip_roll << "\n";
        }
    }

    // Initialize logging
    Logger::initialize(config.verbose);

    if (linux_setup_only) return run_linux_setup();

    // Handle pairing mode
    if (config.pair_mode) {
        PSMovePairing pairing;
        return pairing.run_pairing_mode();
    }

    // Setup signal handling
    SignalHandler::setup();

#ifdef DSU_WITH_GUI
    if (!config.headless) {
        GuiApp gui;
        if (gui.init()) {
            DSUServer server(config);
            Logger::info("DSU server starting...");
            // Keep running even if the port is taken, so the GUI can show why and retry.
            if (!server.start(false)) {
                Logger::error("Failed to start DSU server");
            }
            gui.run(server, config);
            server.stop();
            gui.shutdown();
            Logger::info("DSU server shutdown complete.");
            return 0;
        }
        Logger::warn("Could not open a window (no display?) - running headless. Use --headless to skip the GUI.");
    }
#endif

    // Start DSU server
    DSUServer server(config);

    Logger::info("DSU server starting...");
    Logger::info("Make sure your controllers are paired using --pair mode first.");

    if (!server.start()) {
        Logger::error("Failed to start DSU server");
        return 1;
    }

    // Main loop
    server.run();

    Logger::info("DSU server shutdown complete.");
    return 0;
}
