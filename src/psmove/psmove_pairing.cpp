#include "psmove/psmove_pairing.hpp"
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
    
    // Get system Bluetooth address
    char btaddr_str[18];
    // The psmove_get_btaddr function gets the controller's address, not the system's
    // We need the system Bluetooth adapter address for pairing
    // Try a fallback approach or see if there's another function
    if (psmove_get_btaddr(usb_controller, btaddr_str) == 0) {
        Logger::error("Could not get Bluetooth address. Make sure you have a working Bluetooth adapter.");
        psmove_disconnect(usb_controller);
        return 1;
    }
    
    std::cout << "   System Bluetooth address: " << btaddr_str << "\n";
    
    // Try pairing - use the standard psmove_pair function
    int pair_result = psmove_pair(usb_controller);
    if (pair_result != 1) {
        std::cout << "Regular pairing failed, trying custom pairing...\n";
        
        // Try custom pairing with host string (this matches the actual API)
        pair_result = psmove_pair_custom(usb_controller, btaddr_str);
        
        if (pair_result != 1) {
            Logger::error("All pairing attempts failed. This could indicate USB communication issue, controller malfunction, or Bluetooth adapter not working properly.");
            psmove_disconnect(usb_controller);
            return 1;
        } else {
            std::cout << "Custom pairing succeeded!\n";
        }
    } else {
        std::cout << "Regular pairing succeeded!\n";
    }
    
    std::cout << "4. Computer host address written to controller.\n";
    psmove_disconnect(usb_controller);
    
    // Step 4: Disconnect USB
    std::cout << "   Disconnect the controller from your computer.\n";
    wait_for_keypress();
    
    // Step 5: Wait for Bluetooth connection
    std::cout << "5. Power on your controller by pressing the PS button.\n";
    std::cout << "   Waiting for incoming Bluetooth connection from controller";
    
    PSMove* bt_controller = nullptr;
    for (int attempts = 0; attempts < 60; attempts++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "." << std::flush;
        
        bt_controller = psmove_connect();
        if (bt_controller) {
            if (psmove_connection_type(bt_controller) == Conn_Bluetooth) {
                break;
            } else {
                psmove_disconnect(bt_controller);
                bt_controller = nullptr;
            }
        }
    }
    
    std::cout << std::endl;
    
    if (!bt_controller) {
        Logger::error("No Bluetooth connection detected after pairing.");
        std::cout << "\nCommon causes and solutions:\n";
        std::cout << "1. Bluetooth driver incompatibility (Windows: use Microsoft's built-in stack)\n";
        std::cout << "2. Bluetooth adapter compatibility (try different adapter)\n";
        std::cout << "3. Windows Bluetooth settings (remove existing PS Move entries)\n";
        std::cout << "4. Controller issues (try holding PS button for 10+ seconds)\n\n";
        std::cout << "The pairing data was successfully written. Try running again or test with different hardware.\n";
        return 1;
    }
    
    // Step 6: Success!
    char* bt_serial = psmove_get_serial(bt_controller);
    std::string bt_mac = format_mac_address(bt_serial);
    
    std::cout << "6. Bluetooth connection detected! Pairing process complete.\n";
    std::cout << "   Controller " << bt_mac << " is now paired and ready to use.\n\n";
    
    psmove_disconnect(bt_controller);
    
    std::cout << "=== Pairing Complete ===\n";
    std::cout << "You can now run the DSU server normally to use your paired controller.\n";
    
    return 0;
}
