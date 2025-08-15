// dsu_server_touch_debug_fixed.cpp
// DSU skeleton: single slot (0), sine-wave gyro, neutral sticks, touch zeroed,
// with correct touch hexdump + decoding for debugging.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <iomanip>

// DSU constants
constexpr uint16_t PROTO_VER = 1001;
constexpr int DSU_PORT = 26760;
constexpr uint32_t MSG_PROTOCOL_VER = 0x100000;
constexpr uint32_t MSG_INFO          = 0x100001;
constexpr uint32_t MSG_DATA          = 0x100002;

// CRC32 helper (standard polynomial 0xEDB88320)
uint32_t crc32_le(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
    return ~crc;
}

// Little-endian writers
inline void put_u16(std::vector<uint8_t>& b, uint16_t v) { b.push_back(v & 0xFF); b.push_back((v >> 8) & 0xFF); }
inline void put_u32(std::vector<uint8_t>& b, uint32_t v) { for (int i=0;i<4;i++) b.push_back((v>>(8*i))&0xFF); }
inline void put_u64(std::vector<uint8_t>& b, uint64_t v) { for (int i=0;i<8;i++) b.push_back((v>>(8*i))&0xFF); }
inline void put_f32(std::vector<uint8_t>& b, float f) { uint32_t v; std::memcpy(&v,&f,4); put_u32(b,v); }

// Packet builder
std::vector<uint8_t> make_packet(uint32_t server_id, uint32_t msg_type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> pkt;
    pkt.push_back('D'); pkt.push_back('S'); pkt.push_back('U'); pkt.push_back('S'); // magic
    put_u16(pkt, PROTO_VER);
    put_u16(pkt, static_cast<uint16_t>(payload.size() + 4)); // length = msg_type + payload
    put_u32(pkt, 0); // CRC placeholder
    put_u32(pkt, server_id);
    put_u32(pkt, msg_type);
    pkt.insert(pkt.end(), payload.begin(), payload.end());

    // Compute CRC (with CRC bytes zeroed)
    pkt[8] = pkt[9] = pkt[10] = pkt[11] = 0;
    uint32_t crc = crc32_le(pkt.data(), pkt.size());
    pkt[8]  = crc & 0xFF;
    pkt[9]  = (crc >> 8) & 0xFF;
    pkt[10] = (crc >> 16) & 0xFF;
    pkt[11] = (crc >> 24) & 0xFF;
    return pkt;
}

// Shared beginning for INFO/DATA payloads
void put_shared_begin(std::vector<uint8_t>& p, uint8_t slot, uint8_t state_connected,
                      uint8_t model_fullgyro, uint8_t conn_bt, uint64_t mac48, uint8_t battery) {
    p.push_back(slot); p.push_back(state_connected); p.push_back(model_fullgyro); p.push_back(conn_bt);
    for (int i=0;i<6;i++) p.push_back(uint8_t((mac48>>(8*i))&0xFF));
    p.push_back(battery);
}

// Subscription structure (keeps client addr)
struct Subscription { sockaddr_in addr{}; };

int main() {
    int sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { perror("socket"); return 1; }

    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = INADDR_ANY; addr.sin_port = htons(DSU_PORT);
    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    uint32_t server_id = 0x12345678;
    std::cout << "DSU server (touch-debug-fixed) listening on UDP " << DSU_PORT << "\n";

    std::vector<Subscription> subscriptions;
    std::mutex sub_mutex;
    uint8_t slot = 0;
    uint64_t mac48 = 0x665544332211ULL;

    std::atomic<bool> running{true};

    // Receive thread: handle PROTOCOL_VER, INFO, DATA (subscribe)
    std::thread recv_thread([&](){
        while (running) {
            uint8_t buf[2048]; sockaddr_in src{}; socklen_t sl = sizeof(src);
            ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, (sockaddr*)&src, &sl);
            if (n >= 20 && buf[0]=='D' && buf[1]=='S' && buf[2]=='U' && buf[3]=='C') {
                uint32_t msg_type = uint32_t(buf[16]) | (uint32_t(buf[17])<<8) | (uint32_t(buf[18])<<16) | (uint32_t(buf[19])<<24);
                if (msg_type == MSG_PROTOCOL_VER) {
                    std::cout << "[recv] MSG_PROTOCOL_VER\n";
                    std::vector<uint8_t> payload; put_u16(payload, PROTO_VER);
                    auto pkt = make_packet(server_id, MSG_PROTOCOL_VER, payload);
                    sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&src, sl);
                } else if (msg_type == MSG_INFO) {
                    std::cout << "[recv] MSG_INFO\n";
                    std::vector<uint8_t> payload;
                    put_shared_begin(payload, slot, 2, 2, 2, mac48, 0x05);
                    payload.push_back(0x00);
                    auto pkt = make_packet(server_id, MSG_INFO, payload);
                    sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&src, sl);
                    std::cout << "[send] Replied INFO\n";
                } else if (msg_type == MSG_DATA) {
                    std::cout << "[recv] MSG_DATA (subscribe)\n";
                    std::lock_guard<std::mutex> lock(sub_mutex);
                    bool found = false;
                    for (auto &s : subscriptions) {
                        if (s.addr.sin_addr.s_addr == src.sin_addr.s_addr && s.addr.sin_port == src.sin_port) { found = true; break; }
                    }
                    if (!found) {
                        Subscription sub; sub.addr = src; subscriptions.push_back(sub);
                        std::cout << "[info] Added subscription. Total: " << subscriptions.size() << "\n";
                    }
                }
            }
        }
    });

    // Send thread: continuous motion data (~125 Hz)
    std::thread send_thread([&](){
        float angle = 0.0f;
        uint32_t packet_no = 0;
        uint64_t t_us = 0;
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(8)); // ~125 Hz

            std::vector<Subscription> subs_copy;
            { std::lock_guard<std::mutex> lock(sub_mutex); subs_copy = subscriptions; }
            if (subs_copy.empty()) continue;

            // Build payload with correct layout:
            // 0..10  shared begin (11)
            // 11     connected (1)
            // 12..15 packet number (4)
            // 16..19 control bytes (4)
            // 20..35 sticks (16)
            // 36..47 touch1+touch2 (12)
            // 48..55 timestamp (8)
            // 56..67 accel (3 floats)
            // 68..79 gyro  (3 floats)

            std::vector<uint8_t> p;
            put_shared_begin(p, slot, 2, 2, 2, mac48, 0x05);

            // connected + packet #
            p.push_back(1);
            put_u32(p, packet_no++);

            // control bytes
            p.push_back(0); // bitmask1
            p.push_back(0); // bitmask2
            p.push_back(0); // HOME
            p.push_back(0); // Touch

            // sticks & analogs: set primary sticks neutral (128), other analogs zeroed
            // indices: 20..35 in payload
            for (int i = 0; i < 16; ++i) {
                if (i==0 || i==1 || i==2 || i==3) p.push_back(128); // LeftX, LeftY, RightX, RightY neutral
                else p.push_back(0);
            }

            // touch1 (6) + touch2 (6) : zeroed (inactive)
            for (int i = 0; i < 12; ++i) p.push_back(0);

            // DEBUG: correct touch hexdump + decode (touch block is at payload index 36..47)
            size_t touch_start = 36;
            if (p.size() >= touch_start + 12) {
                std::cout << "[dbg] raw touch bytes: ";
                for (size_t i = touch_start; i < touch_start + 12; ++i) {
                    printf("%02X ", p[i]);
                }
                std::cout << "\n";

                uint8_t t1_active = p[touch_start + 0];
                uint8_t t1_id     = p[touch_start + 1];
                uint16_t t1_x     = uint16_t(p[touch_start + 2]) | (uint16_t(p[touch_start + 3]) << 8);
                uint16_t t1_y     = uint16_t(p[touch_start + 4]) | (uint16_t(p[touch_start + 5]) << 8);

                uint8_t t2_active = p[touch_start + 6];
                uint8_t t2_id     = p[touch_start + 7];
                uint16_t t2_x     = uint16_t(p[touch_start + 8]) | (uint16_t(p[touch_start + 9]) << 8);
                uint16_t t2_y     = uint16_t(p[touch_start +10]) | (uint16_t(p[touch_start +11]) << 8);

                std::cout << "[dbg] touch1 a=" << (int)t1_active << " id=" << (int)t1_id
                          << " x=" << t1_x << " y=" << t1_y << "\n";
                std::cout << "[dbg] touch2 a=" << (int)t2_active << " id=" << (int)t2_id
                          << " x=" << t2_x << " y=" << t2_y << "\n";
            }

            // timestamp (u64 microseconds)
            t_us += 5000; // approx 5ms per packet
            put_u64(p, t_us);

            // accelerometer (g)
            float ax = 0.0f, ay = 0.0f, az = 1.0f;
            put_f32(p, ax); put_f32(p, ay); put_f32(p, az);

            // gyroscope (deg/s) — pitch, yaw, roll
            angle += 0.05f; if (angle > 6.2831853f) angle -= 6.2831853f;
            float pitch_dps = 50.0f * sinf(angle);
            float yaw_dps   = 50.0f * cosf(angle);
            float roll_dps  = 0.0f;
            put_f32(p, pitch_dps); put_f32(p, yaw_dps); put_f32(p, roll_dps);

            // finalize packet
            auto pkt = make_packet(server_id, MSG_DATA, p);
            for (auto &s : subs_copy) {
                sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&s.addr, sizeof(s.addr));
            }

            std::cout << "[send] DATA #" << packet_no << " -> " << subs_copy.size()
                      << " clients (payload " << p.size() << " bytes, pkt " << pkt.size() << ")\n";
        }
    });

    recv_thread.join();
    send_thread.join();
    close(sock);
    return 0;
}
