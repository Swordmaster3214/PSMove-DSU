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
    uint8_t mac[6];
    mac_bytes(mac48, mac);
    p.insert(p.end(), mac, mac + 6);
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

void DSUProtocol::mac_bytes(uint64_t mac, uint8_t out[6]) {
    for (int i = 0; i < 6; i++) {
        out[i] = uint8_t((mac >> (8 * i)) & 0xFF);
    }
}

namespace {
// Where each DSU button lives in the packet. Dolphin reads face buttons, D-pad,
// L1/R1 and L2/R2 from the analog bytes and the rest from the bitmasks, so we
// always fill in both.
struct DsuButtonSlot {
    int mask_byte;  // 1 = buttons1, 2 = buttons2, 3 = home, 4 = touch, 0 = none
    uint8_t bit;
    int analog;     // index into DsuButtonState::analog, -1 = none
};

DsuButtonSlot slot_for(DsuButton b) {
    switch (b) {
        case DsuButton::Share:     return {1, 0x01, -1};
        case DsuButton::L3:        return {1, 0x02, -1};
        case DsuButton::R3:        return {1, 0x04, -1};
        case DsuButton::Options:   return {1, 0x08, -1};
        case DsuButton::DpadUp:    return {1, 0x10, 3};
        case DsuButton::DpadRight: return {1, 0x20, 2};
        case DsuButton::DpadDown:  return {1, 0x40, 1};
        case DsuButton::DpadLeft:  return {1, 0x80, 0};
        case DsuButton::L2:        return {2, 0x01, 11};
        case DsuButton::R2:        return {2, 0x02, 10};
        case DsuButton::L1:        return {2, 0x04, 9};
        case DsuButton::R1:        return {2, 0x08, 8};
        // The spec lists the face buttons as "Y, B, A, X" in Nintendo positions
        // (Y = left = Square, B = bottom = Cross, A = right = Circle, X = top =
        // Triangle): digital bits from the MSB, analog bytes in that order.
        // Dolphin reads these buttons from the analog bytes only.
        case DsuButton::Triangle:  return {2, 0x10, 7};
        case DsuButton::Circle:    return {2, 0x20, 6};
        case DsuButton::Cross:     return {2, 0x40, 5};
        case DsuButton::Square:    return {2, 0x80, 4};
        case DsuButton::Home:      return {3, 0x01, -1};
        case DsuButton::Touch:     return {4, 0x01, -1};
        default:                   return {0, 0, -1};
    }
}
} // namespace

DsuButtonState DSUProtocol::encode_buttons(const MoveSample& sample, const Config& config) {
    DsuButtonState st;
    for (size_t i = 0; i < MOVE_INPUT_COUNT; i++) {
        auto input = static_cast<MoveInput>(i);
        DsuButton target = config.button_map[i];
        if (target == DsuButton::None) continue;

        bool pressed;
        uint8_t analog_value;
        if (input == MoveInput::Trigger) {
            analog_value = sample.trigger;
            pressed = sample.trigger > config.trigger_threshold;
        } else {
            pressed = (sample.buttons & Mapping::move_input_mask(input)) != 0;
            analog_value = pressed ? 255 : 0;
        }

        DsuButtonSlot s = slot_for(target);
        if (pressed) {
            switch (s.mask_byte) {
                case 1: st.buttons1 |= s.bit; break;
                case 2: st.buttons2 |= s.bit; break;
                case 3: st.home = 1; break;
                case 4: st.touch = 1; break;
                default: break;
            }
        }
        // Several inputs may share a target; keep the strongest value.
        if (s.analog >= 0 && analog_value > st.analog[s.analog]) {
            st.analog[s.analog] = analog_value;
        }
    }
    return st;
}

bool DSUProtocol::is_pressed(const DsuButtonState& st, int dsu_button) {
    DsuButtonSlot s = slot_for(static_cast<DsuButton>(dsu_button));
    switch (s.mask_byte) {
        case 1: return (st.buttons1 & s.bit) != 0;
        case 2: return (st.buttons2 & s.bit) != 0;
        case 3: return st.home != 0;
        case 4: return st.touch != 0;
        default: return false;
    }
}

DsuMotion DSUProtocol::map_motion(const MoveSample& sample, const Config& config) {
    DsuMotion m;
    const float accel_arr[3] = { sample.accel_x, sample.accel_y, sample.accel_z };
    const float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
    const float gyro_arr[3] = { sample.gyro_x * RAD_TO_DEG, sample.gyro_y * RAD_TO_DEG, sample.gyro_z * RAD_TO_DEG };

    auto pick = [](const float* arr, int idx, bool flip) -> float {
        if (idx < 0 || idx > 2) return 0.0f;
        return flip ? -arr[idx] : arr[idx];
    };

    m.accel[0] = pick(accel_arr, config.map_accel_x, config.flip_accel_x);
    m.accel[1] = pick(accel_arr, config.map_accel_y, config.flip_accel_y);
    m.accel[2] = pick(accel_arr, config.map_accel_z, config.flip_accel_z);
    m.gyro[0] = pick(gyro_arr, config.map_pitch, config.flip_pitch);
    m.gyro[1] = pick(gyro_arr, config.map_yaw, config.flip_yaw);
    m.gyro[2] = pick(gyro_arr, config.map_roll, config.flip_roll);
    return m;
}

std::vector<uint8_t> DSUProtocol::make_data_packet(uint32_t server_id, uint8_t slot, const MoveSample& sample,
                                                    uint32_t packet_no, int battery, uint64_t mac, const Config& config) {
    std::vector<uint8_t> p;
    p.reserve(80);

    uint8_t batt_byte = (battery >= 0 && battery <= 0xFF) ? uint8_t(battery) : 0x05u;
    put_shared_begin(p, slot, 2, 2, 2, mac, batt_byte);   // payload 0..10
    p.push_back(1);                                        // 11: connected
    put_u32(p, packet_no);                                 // 12..15

    DsuButtonState btn = encode_buttons(sample, config);
    p.push_back(btn.buttons1);                             // 16
    p.push_back(btn.buttons2);                             // 17
    p.push_back(btn.home);                                 // 18
    p.push_back(btn.touch);                                // 19

    // 20..23: sticks (centered; the PS Move has none)
    p.push_back(128); p.push_back(128); p.push_back(128); p.push_back(128);
    // 24..35: analog buttons
    p.insert(p.end(), btn.analog, btn.analog + 12);
    // 36..47: touch points (unused)
    p.insert(p.end(), 12, 0);

    put_u64(p, sample.timestamp_us);                       // 48..55

    DsuMotion m = map_motion(sample, config);
    put_f32(p, m.accel[0]); put_f32(p, m.accel[1]); put_f32(p, m.accel[2]);  // 56..67
    put_f32(p, m.gyro[0]); put_f32(p, m.gyro[1]); put_f32(p, m.gyro[2]);     // 68..79

    return make_packet(server_id, Constants::MSG_DATA, p);
}
