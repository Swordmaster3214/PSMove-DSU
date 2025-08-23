#pragma once
#include <vector>
#include <cstdint>

struct MoveSample {
    float accel_x = 0, accel_y = 0, accel_z = 1.0f;
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;
    uint64_t timestamp_us = 0;
    uint64_t poll_timestamp_us = 0;    // NEW: When data was polled from PSMove
    uint64_t send_timestamp_us = 0;    // NEW: When data was sent over network
    unsigned int buttons = 0;
    unsigned char trigger = 0;
    
    // NEW: Get end-to-end latency
    uint64_t get_latency_us() const {
        if (send_timestamp_us > poll_timestamp_us) {
            return send_timestamp_us - poll_timestamp_us;
        }
        return 0;
    }
};

class DSUProtocol {
public:
    static uint32_t crc32_le(const uint8_t* data, size_t len);
    static std::vector<uint8_t> make_packet(uint32_t server_id, uint32_t msg_type, const std::vector<uint8_t>& payload);
    
    static std::vector<uint8_t> make_protocol_response(uint32_t server_id);
    static std::vector<uint8_t> make_info_response(uint32_t server_id, uint8_t slot, bool connected, int battery, uint64_t mac);
    static std::vector<uint8_t> make_data_packet(uint32_t server_id, uint8_t slot, const MoveSample& sample, 
                                                  uint32_t packet_no, int battery, uint64_t mac, const struct Config& config);

private:
    static void put_u16(std::vector<uint8_t>& b, uint16_t v);
    static void put_u32(std::vector<uint8_t>& b, uint32_t v);
    static void put_u64(std::vector<uint8_t>& b, uint64_t v);
    static void put_f32(std::vector<uint8_t>& b, float f);
    static void put_shared_begin(std::vector<uint8_t>& p, uint8_t slot, uint8_t state_connected,
                                uint8_t model_fullgyro, uint8_t conn_bt, uint64_t mac48, uint8_t battery);
};
