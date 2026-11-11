#pragma once
#include "config/config.hpp"
#include <string>

// Persists the runtime settings of Config (button and axis mapping) as a
// simple "key = value" text file.
namespace ConfigFile {
    // Platform default location:
    //   Windows: %APPDATA%\PSMove-DSU\config.ini
    //   macOS:   ~/Library/Application Support/PSMove-DSU/config.ini
    //   Linux:   $XDG_CONFIG_HOME/psmove-dsu/config.ini (or ~/.config/...)
    std::string default_path();

    // Loads settings into `config`. Missing file is not an error (returns true).
    // Returns false and fills `error` only if the file exists but can't be read.
    bool load(const std::string& path, Config& config, std::string& error);

    // Writes settings, creating the parent directory if needed.
    bool save(const std::string& path, const Config& config, std::string& error);
}
