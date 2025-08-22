#include "dsu/dsu_protocol.hpp"
#include "config/constants.hpp"
#include "config/config.hpp"
#include <cstring>

uint32_t DSUProtocol::crc32_le(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
        }
    }
    return ~crc;
}

void DSUProtocol::put_u16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(v & 0xFF);
    b.push_back((v >> 8) & 0xFF);
}

void DSUProtocol::put_u32(std::vector<uint8_t>& b, uint32_t v) {
    for (int i = 0; i < 4; i++) {
        b.push_back((v >> (8 * i)) & 0xFF);
    }
}

void DSUProtocol::put_u64(std::vector<uint8_t>& b, uint64_t v) {
    for (int i = 0; i < 8; i++) {
        b.push_back((v >> (8 * i)) & 0xFF);
    }
}

void DSUProtocol::put_f32(std::vector<uint8_t>& b, float f) {
    uint32_t v;
    std::memcpy(&v, &f, 4);
    put_u32(b, v);
}

std::vector<uint8_t> DSUProtocol::make_packet(uint32_t server_id, uint32_t msg_type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> pkt;
    pkt.push_back('D'); pkt.push_back('S'); pkt.push_back('U'); pkt.push_back('S');
    put_u16(pkt, Constants::PROTO_VER);
    put_u16(pkt, static_cast<uint16_t>(payload.size() + 4));
    put_u32(pkt, 0); // CRC placeholder
    put_u32(pkt, server_id);
    put_u32(pkt, msg_type);
    pkt.insert(pkt.end(), payload.begin(), payload.end());
    
    pkt[8] = pkt[9] = pkt[10] = pkt[11] = 0;
    uint32_t crc = crc32_le(pkt.data(), pkt.size());
    pkt[8] = crc & 0xFF;
    pkt[9] = (crc >> 8) & 0xFF;
    pkt[10] = (crc >> 16) & 0xFF;
    pkt[11] = (crc >> 24) & 0xFF;
    return pkt;
}

void DSUProtocol::put_shared_begin(std::vector<uint8_t>& p, uint8_t slot, uint8_t state_connected,
                                   uint8_t model_fullgyro, uint8_t conn_bt, uint64_t mac48, uint8_t battery) {
    p.push_back(slot);
    p.push_back(state_connected);
    p.push_back(model_fullgyro);
    p.push_back(conn_bt);
    for (int i = 0; i < 6; i++) {
        p.push_back(uint8_t((mac48 >> (8 * i)) & 0xFF));
    }
    p.push_back(battery);
}

std::vector<uint8_t> DSUProtocol::make_protocol_response(uint32_t server_id) {
    std::vector<uint8_t> payload;
    put_u16(payload, Constants::PROTO_VER);
    return make_packet(server_id, Constants::MSG_PROTOCOL_VER, payload);
}

std::vector<uint8_t> DSUProtocol::make_info_response(uint32_t server_id, uint8_t slot, bool connected, int battery, uint64_t mac) {
    std::vector<uint8_t> payload;
    uint8_t batt_byte = (battery >= 0 && battery <= 0xFF) ? uint8_t(battery) : 0x05u;
    uint8_t connected_state = connected ? 2 : 0;
    
    put_shared_begin(payload, slot, connected_state, 2, 2, mac, batt_byte);
    payload.push_back(0x00);
    return make_packet(server_id, Constants::MSG_INFO, payload);
}

std::vector<uint8_t> DSUProtocol::make_data_packet(uint32_t server_id, uint8_t slot, const MoveSample& sample,
                                                    uint32_t packet_no, int battery, uint64_t mac, const Config& config) {
    std::vector<uint8_t> p;
    
    uint8_t batt_byte = (battery >= 0 && battery <= 0xFF) ? uint8_t(battery) : 0x05u;
    put_shared_begin(p, slot, 2, 2, 2, mac, batt_byte);
    p.push_back(1);
    put_u32(p, packet_no);
    p.push_back(0); // bitmask1
    p.push_back(0); // bitmask2
    p.push_back(0); // HOME
    p.push_back(0); // Touch
    
    // Analog sticks (16 bytes)
    for (int i = 0; i < 16; ++i) {
        if (i == 0 || i == 1 || i == 2 || i == 3) {
            p.push_back(128);
        } else {
            p.push_back(0);
        }
    }
    
    // Touch block (12 bytes)
    for (int i = 0; i < 12; ++i) {
        p.push_back(0);
    }
    
    put_u64(p, sample.timestamp_us);
    
    // Map accelerometer
    float accel_arr[3] = { sample.accel_x, sample.accel_y, sample.accel_z };
    auto accel_get = [&](int idx, bool flip) -> float {
        if (idx < 0 || idx > 2) return 0.0f;
        float v = accel_arr[idx];
        return flip ? -v : v;
    };
    
    float ax = accel_get(config.map_accel_x, config.flip_accel_x);
    float ay = accel_get(config.map_accel_y, config.flip_accel_y);
    float az = accel_get(config.map_accel_z, config.flip_accel_z);
    put_f32(p, ax); put_f32(p, ay); put_f32(p, az);
    
    // Map gyro (convert to degrees)
    const float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
    float gyro_arr[3] = { sample.gyro_x * RAD_TO_DEG, sample.gyro_y * RAD_TO_DEG, sample.gyro_z * RAD_TO_DEG };
    auto gyro_get = [&](int idx, bool flip) -> float {
        if (idx < 0 || idx > 2) return 0.0f;
        float v = gyro_arr[idx];
        return flip ? -v : v;
    };
    
    float pitch = gyro_get(config.map_pitch, config.flip_pitch);
    float yaw = gyro_get(config.map_yaw, config.flip_yaw);
    float roll = gyro_get(config.map_roll, config.flip_roll);
    put_f32(p, pitch); put_f32(p, yaw); put_f32(p, roll);
    
    // Button mapping
    uint8_t dsubitmask1 = 0;
    uint8_t dsubitmask2 = 0;
    uint8_t dsuhome = 0;
    
    using namespace Constants;
    if (sample.buttons & Btn_CROSS) dsubitmask2 |= 0x40u;
    if (sample.buttons & Btn_CIRCLE) dsubitmask2 |= 0x20u;
    if (sample.buttons & Btn_SQUARE) dsubitmask2 |= 0x80u;
    if (sample.buttons & Btn_TRIANGLE) dsubitmask2 |= 0x10u;
    if (sample.buttons & Btn_MOVE) dsubitmask2 |= 0x08u;
    if (sample.buttons & Btn_START) dsubitmask1 |= 0x08u;
    if (sample.buttons & Btn_SELECT) dsubitmask1 |= 0x01u;
    if (sample.buttons & Btn_PS) dsuhome = 1u;
    
    if (sample.trigger > Constants::TRIGGER_DIGITAL_THRESHOLD) {
        dsubitmask2 |= 0x02u;
    }
    
    // Write control bytes
    size_t control_offset = 14;
    if (p.size() >= control_offset + 4) {
        p[control_offset + 0] = dsubitmask1;
        p[control_offset + 1] = dsubitmask2;
        p[control_offset + 2] = dsuhome;
    }
    
    // Analog face buttons
    size_t analog_base = 18;
    if (p.size() >= analog_base + 16) {
        p[analog_base + 8] = (sample.buttons & Btn_TRIANGLE) ? 255u : 0u;
        p[analog_base + 9] = (sample.buttons & Btn_CIRCLE) ? 255u : 0u;
        p[analog_base + 10] = (sample.buttons & Btn_CROSS) ? 255u : 0u;
        p[analog_base + 11] = (sample.buttons & Btn_SQUARE) ? 255u : 0u;
        p[analog_base + 12] = (sample.buttons & Btn_MOVE) ? 255u : 0u;
    }
    
    // Trigger analog
    if (Constants::ANALOG_TRIGGER_INDEX >= 0 && Constants::ANALOG_TRIGGER_INDEX < 16 && 
        p.size() >= analog_base + 16) {
        p[analog_base + Constants::ANALOG_TRIGGER_INDEX] = sample.trigger;
    }
    
    return make_packet(server_id, Constants::MSG_DATA, p);
}
