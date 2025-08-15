// dsu_server_psmove_cemuhook.cpp
// DSU server + PS Move integration (accel, gyro, buttons, trigger).
// Uses cemuhook/DSU bit positions and maps Move trigger to Analog R2 (payload[34]).
// Build (example on Arch):
// g++ -O2 -std=c++17 dsu_server_psmove_cemuhook.cpp -o dsu_server_psmove_cemuhook -pthread $(pkg-config --cflags --libs libpsmoveapi)
// or: g++ -O2 -std=c++17 dsu_server_psmove_cemuhook.cpp -o dsu_server_psmove_cemuhook -pthread -I/usr/include/psmoveapi -lpsmoveapi

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <psmoveapi/psmove.h> // header on your system: /usr/include/psmoveapi/psmove.h

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
#include <optional>
#include <iomanip>

// -------------------- Configuration --------------------
// Map gyro axes (0 = x, 1 = y, 2 = z) into DSU pitch/yaw/roll:
static int MAP_PITCH = 0;
static int MAP_YAW   = 2;
static int MAP_ROLL  = 1;
static bool FLIP_PITCH = false;
static bool FLIP_YAW   = true;   // you found yaw needed flipping
static bool FLIP_ROLL  = false;

// Accelerometer mapping (which raw axis becomes DSU accel X/Y/Z)
static int MAP_ACCEL_X = 0;
static int MAP_ACCEL_Y = 2;
static int MAP_ACCEL_Z = 1;
// You indicated you changed these to true:
static bool FLIP_ACCEL_X = true;
static bool FLIP_ACCEL_Y = true;
static bool FLIP_ACCEL_Z = false;

// Trigger analog index among the 16 analog bytes (0..15)
// cemuhook mapping: payload[34] = Analog R2 (Right trigger analog) -> index = 14 (20 + 14 = 34)
static const int ANALOG_TRIGGER_INDEX = 14; // set to 14 for R2, set to 15 for L2 if preferred

// Threshold to treat analog trigger as "pressed" for digital R2 bit
static const unsigned char TRIGGER_DIGITAL_THRESHOLD = 10; // tune if needed (0..255)

// Poll & send rates
static constexpr int PSMOVE_POLL_MS = 4;    // ~250 Hz
static constexpr int DSU_SEND_MS    = 8;    // ~125 Hz

// --------------------------------------------------------------------

// DSU constants
constexpr uint16_t PROTO_VER = 1001;
constexpr int DSU_PORT = 26760;
constexpr uint32_t MSG_PROTOCOL_VER = 0x100000;
constexpr uint32_t MSG_INFO          = 0x100001;
constexpr uint32_t MSG_DATA          = 0x100002;

// CRC32 helper
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

                      // -------------------- PSMove manager (single-controller) --------------------

                      struct MoveSample {
                          float accel_x=0, accel_y=0, accel_z=1.0f;   // g
                          float gyro_x=0, gyro_y=0, gyro_z=0;         // rad/s (as returned)
                          uint64_t timestamp_us = 0;
                          unsigned int buttons = 0; // raw button bitmask
                          unsigned char trigger = 0; // 0..255
                      };

                      class PSMoveManager {
                      public:
                          PSMoveManager() = default;
                          ~PSMoveManager() { stop(); }

                          bool start() {
                              if (running_.load()) return true;
                              PSMove *mv = psmove_connect();
                              if (!mv) {
                                  std::cerr << "PSMoveManager: no controller found\n";
                                  return false;
                              }
                              move_ = mv;
                              running_ = true;
                              worker_ = std::thread(&PSMoveManager::thread_main, this);
                              std::cout << "PSMoveManager: connected\n";
                              return true;
                          }

                          void stop() {
                              running_ = false;
                              if (worker_.joinable()) worker_.join();
                              if (move_) {
                                  psmove_disconnect(move_);
                                  move_ = nullptr;
                              }
                          }

                          // thread-safe copy of latest sample
                          std::optional<MoveSample> get_latest() {
                              std::lock_guard<std::mutex> lk(mtx_);
                              return latest_;
                          }

                      private:
                          void thread_main() {
                              while (running_) {
                                  if (!move_) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }
                                  // Poll controller
                                  psmove_poll(move_);

                                  // Read accelerometer and gyro frames
                                  float ax=0, ay=0, az=0;
                                  psmove_get_accelerometer_frame(move_, Frame_SecondHalf, &ax, &ay, &az);

                                  float gx=0, gy=0, gz=0;
                                  psmove_get_gyroscope_frame(move_, Frame_SecondHalf, &gx, &gy, &gz);

                                  unsigned int btns = psmove_get_buttons(move_);
                                  unsigned char trig = psmove_get_trigger(move_); // 0..255

                                  uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch()).count();

                                      {
                                          std::lock_guard<std::mutex> lk(mtx_);
                                          latest_.accel_x = ax; latest_.accel_y = ay; latest_.accel_z = az;
                                          latest_.gyro_x  = gx; latest_.gyro_y  = gy; latest_.gyro_z  = gz;
                                          latest_.buttons = btns;
                                          latest_.trigger = trig;
                                          latest_.timestamp_us = ts;
                                      }

                                      std::this_thread::sleep_for(std::chrono::milliseconds(PSMOVE_POLL_MS));
                              }
                          }

                          PSMove *move_ = nullptr;
                          std::thread worker_;
                          std::atomic<bool> running_{false};
                          std::mutex mtx_;
                          MoveSample latest_;
                      };

                      // -------------------- DSU server (main) --------------------

                      struct Subscription { sockaddr_in addr{}; };

                      int main(int argc, char** argv) {
                          // Optional runtime config via CLI
                          for (int i=1;i<argc;i++) {
                              if (std::strcmp(argv[i], "--map")==0 && i+3<argc) {
                                  MAP_PITCH = std::atoi(argv[++i]);
                                  MAP_YAW   = std::atoi(argv[++i]);
                                  MAP_ROLL  = std::atoi(argv[++i]);
                                  std::cout << "Mapping set pitch="<<MAP_PITCH<<" yaw="<<MAP_YAW<<" roll="<<MAP_ROLL<<"\n";
                              } else if (std::strcmp(argv[i],"--flip")==0 && i+3<argc) {
                                  FLIP_PITCH = std::atoi(argv[++i])!=0;
                                  FLIP_YAW   = std::atoi(argv[++i])!=0;
                                  FLIP_ROLL  = std::atoi(argv[++i])!=0;
                                  std::cout << "Flip set pitch="<<FLIP_PITCH<<" yaw="<<FLIP_YAW<<" roll="<<FLIP_ROLL<<"\n";
                              }
                          }

                          int sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                          if (sock < 0) { perror("socket"); return 1; }

                          sockaddr_in bind_addr{}; bind_addr.sin_family = AF_INET; bind_addr.sin_port = htons(DSU_PORT); bind_addr.sin_addr.s_addr = INADDR_ANY;
                          if (bind(sock, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) { perror("bind"); return 1; }

                          uint32_t server_id = 0x12345678u;
                          std::cout << "DSU server (psmove cemuhook) listening on UDP " << DSU_PORT << "\n";

                          std::vector<Subscription> subscriptions;
                          std::mutex sub_mutex;
                          uint8_t slot = 0;
                          uint64_t mac48 = 0x665544332211ULL;

                          // Start PSMove manager
                          PSMoveManager psm;
                          bool psm_started = psm.start();
                          if (!psm_started) {
                              std::cout << "PSMove not connected — falling back to sine-wave.\n";
                          }

                          std::atomic<bool> running{true};

                          // Receive thread
                          std::thread recv_thread([&](){
                              while (running) {
                                  uint8_t buf[2048]; sockaddr_in src{}; socklen_t sl = sizeof(src);
                                  ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, (sockaddr*)&src, &sl);
                                  if (n >= 20 && buf[0]=='D' && buf[1]=='S' && buf[2]=='U' && buf[3]=='C') {
                                      uint32_t msg_type = uint32_t(buf[16]) | (uint32_t(buf[17])<<8) | (uint32_t(buf[18])<<16) | (uint32_t(buf[19])<<24);
                                      if (msg_type == MSG_PROTOCOL_VER) {
                                          std::vector<uint8_t> payload; put_u16(payload, PROTO_VER);
                                          auto pkt = make_packet(server_id, MSG_PROTOCOL_VER, payload);
                                          sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&src, sl);
                                      } else if (msg_type == MSG_INFO) {
                                          std::vector<uint8_t> payload;
                                          put_shared_begin(payload, slot, 2, 2, 2, mac48, 0x05);
                                          payload.push_back(0x00);
                                          auto pkt = make_packet(server_id, MSG_INFO, payload);
                                          sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&src, sl);
                                          std::cout << "[send] Replied INFO\n";
                                      } else if (msg_type == MSG_DATA) {
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

                          // conversion factor
                          const float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

                          std::thread send_thread([&](){
                              float angle = 0.0f;
                              uint32_t packet_no = 0;
                              uint64_t t_us = 0;
                              int diag_counter = 0;

                              // previous control bytes for toggled diagnostics
                              uint8_t prev_bm1 = 0, prev_bm2 = 0, prev_home = 0;

                              while (running) {
                                  std::this_thread::sleep_for(std::chrono::milliseconds(DSU_SEND_MS));

                                  std::vector<Subscription> subs_copy;
                                  { std::lock_guard<std::mutex> lock(sub_mutex); subs_copy = subscriptions; }
                                  if (subs_copy.empty()) continue;

                                  // Build payload skeleton
                                  std::vector<uint8_t> p;
                                  put_shared_begin(p, slot, 2, 2, 2, mac48, 0x05);

                                  p.push_back(1);
                                  put_u32(p, packet_no++);

                                  // placeholders for control bytes (will be overwritten)
                                  p.push_back(0); // bitmask1
                                  p.push_back(0); // bitmask2
                                  p.push_back(0); // HOME
                                  p.push_back(0); // Touch

                                  // sticks & analogs 16 bytes
                                  for (int i = 0; i < 16; ++i) {
                                      if (i==0 || i==1 || i==2 || i==3) p.push_back(128); else p.push_back(0);
                                  }

                                  // touch block 12 bytes
                                  for (int i = 0; i < 12; ++i) p.push_back(0);

                                  // timestamp (we increment approximate microseconds)
                                  t_us += DSU_SEND_MS * 1000ull;

                                  // get latest sample (thread-safe copy)
                                  std::optional<MoveSample> sopt = psm.get_latest();
                                  bool have_move = false;
                                  MoveSample ms;
                                  if (sopt.has_value()) {
                                      have_move = true;
                                      ms = sopt.value();

                                      // convert gyro rad/s -> deg/s
                                      ms.gyro_x *= RAD_TO_DEG;
                                      ms.gyro_y *= RAD_TO_DEG;
                                      ms.gyro_z *= RAD_TO_DEG;
                                  }

                                  if (!have_move) {
                                      // fallback sine-wave sample and neutral buttons
                                      angle += 0.05f; if (angle > 6.2831853f) angle -= 6.2831853f;
                                      ms.accel_x = 0.0f; ms.accel_y = 0.0f; ms.accel_z = 1.0f;
                                      ms.gyro_x = 50.0f * sinf(angle);
                                      ms.gyro_y = 50.0f * cosf(angle);
                                      ms.gyro_z = 0.0f;
                                      ms.timestamp_us = t_us;
                                      ms.buttons = 0;
                                      ms.trigger = 0;
                                  }

                                  // write timestamp
                                  put_u64(p, ms.timestamp_us ? ms.timestamp_us : t_us);

                                  // --- Accelerometer mapping ---
                                  float accel_arr[3] = { ms.accel_x, ms.accel_y, ms.accel_z };
                                  auto accel_get = [&](int idx, bool flip)->float {
                                      if (idx < 0 || idx > 2) return 0.0f;
                                      float v = accel_arr[idx];
                                      return flip ? -v : v;
                                  };
                                  float ax = accel_get(MAP_ACCEL_X, FLIP_ACCEL_X);
                                  float ay = accel_get(MAP_ACCEL_Y, FLIP_ACCEL_Y);
                                  float az = accel_get(MAP_ACCEL_Z, FLIP_ACCEL_Z);
                                  put_f32(p, ax); put_f32(p, ay); put_f32(p, az);

                                  // --- Gyro mapping (pitch,yaw,roll) ---
                                  float gyro_arr[3] = { ms.gyro_x, ms.gyro_y, ms.gyro_z };
                                  auto gyro_get = [&](int idx, bool flip)->float {
                                      if (idx < 0 || idx > 2) return 0.0f;
                                      float v = gyro_arr[idx];
                                      return flip ? -v : v;
                                  };
                                  float pitch = gyro_get(MAP_PITCH, FLIP_PITCH);
                                  float yaw   = gyro_get(MAP_YAW,   FLIP_YAW);
                                  float roll  = gyro_get(MAP_ROLL,  FLIP_ROLL);
                                  put_f32(p, pitch); put_f32(p, yaw); put_f32(p, roll);

                                  // --- BUTTONS & TRIGGER mapping (cemuhook) ---
                                  uint8_t dsubitmask1 = 0;
                                  uint8_t dsubitmask2 = 0;
                                  uint8_t dsuhome = 0;

                                  // bitmask2 mapping (descending): 128=Y,64=B,32=A,16=X,8=R1,4=L1,2=R2,1=L2
                                  if (ms.buttons & Btn_TRIANGLE)  dsubitmask2 |= 0x80; // Triangle -> Y
                                  if (ms.buttons & Btn_CIRCLE)    dsubitmask2 |= 0x40; // Circle   -> B
                                  if (ms.buttons & Btn_CROSS)     dsubitmask2 |= 0x20; // Cross    -> A
                                  if (ms.buttons & Btn_SQUARE)    dsubitmask2 |= 0x10; // Square   -> X

                                  // Map Move -> R1 (8) by default (change if you prefer L1)
                                  if (ms.buttons & Btn_MOVE)      dsubitmask2 |= 0x08; // R1

                                  // If trigger pressed beyond threshold, set digital R2 bit
                                  if (ms.trigger > TRIGGER_DIGITAL_THRESHOLD) {
                                      dsubitmask2 |= 0x02; // digital R2
                                  }

                                  // bitmask1 mapping (descending): 128=DPAD_LEFT,64=DPAD_DOWN,32=DPAD_RIGHT,16=DPAD_UP,8=OPTIONS(Start),4=R3,2=L3,1=SHARE(Select)
                                  if (ms.buttons & Btn_START)  dsubitmask1 |= 0x08; // Options/Start -> bit 3 (value 8)
                          if (ms.buttons & Btn_SELECT) dsubitmask1 |= 0x01; // Share/Select -> bit 0 (value 1)

                          // HOME (payload[18]) is PS
                          if (ms.buttons & Btn_PS) dsuhome = 1;

                          // Write final control bytes into payload (payload offset = 16)
                          size_t control_offset = 16;
                                  if (p.size() >= control_offset + 4) {
                                      p[control_offset + 0] = dsubitmask1;
                                      p[control_offset + 1] = dsubitmask2;
                                      p[control_offset + 2] = dsuhome;
                                  }

                                  // --- Write trigger analog into the R2 analog slot (payload[20 + ANALOG_TRIGGER_INDEX]) ---
                                  size_t analog_base = 20;
                                  if (ANALOG_TRIGGER_INDEX >= 0 && ANALOG_TRIGGER_INDEX < 16 && p.size() >= analog_base + 16) {
                                      p[analog_base + ANALOG_TRIGGER_INDEX] = ms.trigger; // write raw 0..255
                                  }

                                  // --- Diagnostics & toggled detection ---
                                  if (++diag_counter % 50 == 0) {
                                      std::cout << "[diag] gyro_deg/s(x,y,z)=" << gyro_arr[0] << "," << gyro_arr[1] << "," << gyro_arr[2]
                                      << " -> p,y,r=" << pitch << "," << yaw << "," << roll
                                      << "  trigger=" << (int)ms.trigger
                                      << "  bm1=" << (int)dsubitmask1 << " bm2=" << (int)dsubitmask2 << " HOME=" << (int)dsuhome
                                      << "\n";
                                  }

                                  // toggled bits (which bits changed since last packet) - quick click diagnostics
                                  uint8_t cur_bm1 = dsubitmask1;
                                  uint8_t cur_bm2 = dsubitmask2;
                                  uint8_t cur_home = dsuhome;
                                  uint8_t diff1 = cur_bm1 ^ prev_bm1;
                                  uint8_t diff2 = cur_bm2 ^ prev_bm2;
                                  uint8_t diffh = cur_home ^ prev_home;
                                  if (diff1 || diff2 || diffh) {
                                      std::cout << "[toggled] DSU changes ->";
                                      if (diff1) {
                                          std::cout << " bm1:";
                                          for (int b=0;b<8;b++) if (diff1 & (1<<b)) std::cout << " bit" << b;
                                      }
                                      if (diff2) {
                                          std::cout << " bm2:";
                                          for (int b=0;b<8;b++) if (diff2 & (1<<b)) std::cout << " bit" << b;
                                      }
                                      if (diffh) std::cout << " HOME";
                                      std::cout << "  (raw ms.buttons=0x" << std::hex << ms.buttons << std::dec << ")\n";
                                  }
                                  prev_bm1 = cur_bm1; prev_bm2 = cur_bm2; prev_home = cur_home;

                                  // finalize & send
                                  auto pkt = make_packet(server_id, MSG_DATA, p);
                                  for (auto &s : subs_copy) {
                                      sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&s.addr, sizeof(s.addr));
                                  }

                                  // This line can be very noisy and hide the diagnostic output.
                                  // You can uncomment it if you need to see every packet send confirmation.
                                  /*
                                   *            std::cout << "[send] DATA #" << packet_no << " -> " << subs_copy.size()
                                   *            << " clients (payload " << p.size() << " bytes, pkt " << pkt.size()
                                   *            << ") using psmove=" << (have_move ? "yes" : "no") << "\n";
                                   */
                              }
                          });

                          std::cout << "Press Ctrl-C to quit.\n";
                          recv_thread.join();
                          send_thread.join();

                          running = false;
                          psm.stop();
                          close(sock);
                          return 0;
                      }
