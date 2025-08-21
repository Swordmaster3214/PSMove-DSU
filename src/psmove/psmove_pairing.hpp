#pragma once
#include <string>

class PSMovePairing {
public:
    int run_pairing_mode();
    
private:
    void wait_for_keypress();
    std::string format_mac_address(const char* serial);
};
