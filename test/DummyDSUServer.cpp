#include <iostream>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <arpa/inet.h>

// DSU protocol constants
constexpr int DSU_PORT = 26760;
constexpr float DEG_TO_RAD = M_PI / 180.0f;

#pragma pack(push, 1)
struct DSUMotionPacket {
    uint8_t header[4];        // "DSUS"
    uint16_t protocol_version;
    uint8_t message_type;     // 0x01 = motion data
    uint8_t slot;             // Controller slot (0)
    uint64_t mac;             // Fake MAC
    uint8_t connected;        // 1 if connected
    uint8_t battery;          // 0xEE = charging, 0xEF = full
    float gyro[3];            // rad/s
    float accel[3];           // m/s²
    uint64_t timestamp;       // microseconds
};
#pragma pack(pop)

int main() {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(DSU_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        return 1;
    }

    std::cout << "DSU server running on port " << DSU_PORT << "\n";

    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);

    // Wait for a handshake packet from Dolphin
    char buf[1024];
    ssize_t len = recvfrom(sock, buf, sizeof(buf), 0,
                           (sockaddr*)&client_addr, &client_len);
    if (len > 0) {
        std::cout << "Handshake received from "
                  << inet_ntoa(client_addr.sin_addr) << ":"
                  << ntohs(client_addr.sin_port) << "\n";
    }

    // Send dummy motion data in a loop
    while (true) {
        DSUMotionPacket packet{};
        memcpy(packet.header, "DSUS", 4);
        packet.protocol_version = htons(1001); // DSU v1.01
        packet.message_type = 0x01; // Motion data
        packet.slot = 0;
        packet.mac = htobe64(0x112233445566); // Fake MAC
        packet.connected = 1;
        packet.battery = 0xEF;

        // Fake sine wave motion
        static float angle = 0.0f;
        packet.gyro[0] = sin(angle) * DEG_TO_RAD;
        packet.gyro[1] = cos(angle) * DEG_TO_RAD;
        packet.gyro[2] = 0.0f;
        packet.accel[0] = 0.0f;
        packet.accel[1] = 9.80665f;
        packet.accel[2] = 0.0f;
        packet.timestamp = htobe64((uint64_t)(angle * 1000000));

        sendto(sock, &packet, sizeof(packet), 0,
               (sockaddr*)&client_addr, client_len);

        angle += 0.05f;
        if (angle > 2 * M_PI) angle -= 2 * M_PI;

        usleep(8000); // ~125 Hz
    }

    close(sock);
    return 0;
}
