#include "psmove/psmove_pairing.hpp"
#include "psmove/linux_setup.hpp"
#include "utils/logging.hpp"
#include "utils/platform.hpp"
#include <psmove.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

void PSMovePairing::wait_for_keypress() {
    std::cout << "Press ENTER to continue...";
    std::cout.flush();
    
    while (true) {
        int ch = getchar();
        if (ch == ' ' || ch == '\n' || ch == '\r') {
            break;
        }
    }
    std::cout << std::endl;
}

std::string PSMovePairing::format_mac_address(const char* serial) {
    if (!serial || strlen(serial) < 12) {
        return "UNKNOWN";
    }
    
    std::string mac;
    for (int i = 0; i < 12; i += 2) {
        if (i > 0) mac += ":";
        mac += serial[i];
        mac += serial[i + 1];
    }
    return mac;
}

int PSMovePairing::run_pairing_mode() {
    std::cout << "\n=== PS Move Controller Pairing Mode ===\n";
    Logger::debug("Initializing PSMoveAPI...");
    
    // Test basic functionality
    int initial_count = psmove_count_connected();
    Logger::debug("Initial USB controller count: " + std::to_string(initial_count));
    
    // Check existing Bluetooth controllers
    PSMove* test_bt = psmove_connect();
    if (test_bt) {
        char* bt_serial = psmove_get_serial(test_bt);
        Logger::debug("Found existing Bluetooth controller: " + std::string(bt_serial ? bt_serial : "UNKNOWN"));
        psmove_disconnect(test_bt);
    } else {
        Logger::debug("No existing Bluetooth controllers found");
    }
    
    std::cout << "\n";
    
    // Step 1: Disconnect all controllers
    std::cout << "1. Disconnect all PS Move devices from your computer.\n";
    wait_for_keypress();
    
    // Step 2: Connect via USB
    std::cout << "2. Connect the controller you would like to pair to your computer using a USB cable.\n";
    std::cout << "   Do not power it on. Detecting...\n";
    
    // Wait for USB controller
    PSMove* usb_controller = nullptr;
    std::cout << "Waiting for USB controller";
    
    for (int attempts = 0; attempts < 60; attempts++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "." << std::flush;
        
        int count = psmove_count_connected();
        if (count > 0) {
            if (attempts % 10 == 0) {
                std::cout << "\n   Found " << count << " USB device(s), attempting connection..." << std::flush;
            }
            
            usb_controller = psmove_connect_by_id(0);
            if (usb_controller) {
                PSMove_Connection_Type conn_type = psmove_connection_type(usb_controller);
                if (conn_type == Conn_USB) {
                    char* test_serial = psmove_get_serial(usb_controller);
                    if (test_serial && strlen(test_serial) > 0) {
                        break; // Success!
                    } else {
                        Logger::debug("Controller detected but not responsive, retrying...");
                        psmove_disconnect(usb_controller);
                        usb_controller = nullptr;
                    }
                } else {
                    Logger::debug("Controller detected but not USB connection, retrying...");
                    psmove_disconnect(usb_controller);
                    usb_controller = nullptr;
                }
            }
        }
    }
    
    std::cout << std::endl;
    
    if (!usb_controller) {
        Logger::error("No USB controller detected. Make sure the controller is connected via USB cable.");
        return 1;
    }
    
    // Step 3: Get controller info and pair
    char* serial = psmove_get_serial(usb_controller);
    if (!serial) {
        Logger::error("Could not read controller serial number.");
        psmove_disconnect(usb_controller);
        return 1;
    }
    
    std::string mac_addr = format_mac_address(serial);
    std::cout << "3. Detected! Pairing process initiated with controller " << mac_addr << ".\n";
    
    if (psmove_connection_type(usb_controller) != Conn_USB) {
        Logger::error("Controller is not connected via USB.");
        psmove_disconnect(usb_controller);
        return 1;
    }
    
    std::cout << "   Writing pairing data to controller...\n";
    
    // Try standard pairing
    int pair_result = psmove_pair(usb_controller);
    if (pair_result != 1) {
        Logger::error("Pairing failed. This could indicate USB communication issue, controller malfunction, or Bluetooth adapter not working properly.");
        psmove_disconnect(usb_controller);
        return 1;
    }
    
    std::cout << "   Pairing succeeded!\n";
    std::cout << "4. Computer host address written to controller.\n";
    psmove_disconnect(usb_controller);
    
    // Step 4: Disconnect USB and connect over Bluetooth
    std::cout << "5. Disconnect the controller from your computer.\n";
    std::cout << "6. Press the PS button on the controller. The red LED will blink, then stay lit once it is\n";
    std::cout << "   connected over Bluetooth.\n\n";

    std::cout << "=== Pairing Complete ===\n";
    std::cout << "You only need to pair once, unless you later connect the controller to a PS3 or pair it with\n";
    std::cout << "another PC. Then run the DSU server normally.\n\n";
    std::cout << "If the LED keeps blinking: charge the controller over USB (low battery), turn Bluetooth off and\n";
    std::cout << "on (or restart your computer) and press the PS button again.\n";
#if defined(_WIN32)
    std::cout << "Windows: pairing can take several attempts. Use the Microsoft Bluetooth stack (no MotionInJoy etc.).\n";
#elif defined(__APPLE__)
    std::cout << "macOS: if a PIN dialog appears, close it and run --pair again. Built-in Bluetooth supports at most\n";
    std::cout << "2 controllers at once on macOS Sierra and later.\n";
#else
    std::cout << "Linux: if it pairs but won't connect, BlueZ may be rejecting it (ClassicBondedOnly).\n";
#endif

#if defined(__linux__)
    run_linux_setup();
#endif

    wait_for_keypress();
    
    return 0;
}
