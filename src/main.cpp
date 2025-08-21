#include "config/constants.hpp"
#include "config/config.hpp"
#include "utils/logging.hpp"
#include "utils/signal_handler.hpp"
#include "psmove/psmove_pairing.hpp"
#include "dsu/dsu_server.hpp"
#include <iostream>
#include <cstring>

int main(int argc, char** argv) {
    Config config;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--verbose") == 0) {
            config.verbose = true;
            Logger::set_verbose(true);
        } else if (std::strcmp(argv[i], "--pair") == 0) {
            config.pair_mode = true;
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
    
    // Handle pairing mode
    if (config.pair_mode) {
        PSMovePairing pairing;
        return pairing.run_pairing_mode();
    }
    
    // Setup signal handling
    SignalHandler::setup();
    
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
