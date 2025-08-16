// dsu_server_psmove.cpp
// Cross-platform DSU server + PS Move integration with automatic USB pairing.
// - Tries BT connect first; if none found, attempts pairing via USB using psmove_pair().
// - Reduced runtime logging by default; use --verbose to enable more output.
// - Keeps analog face-button writes, trigger mapping, battery reading.
//
// Build examples:
// Linux/macOS (pkg-config):
//   g++ -O2 -std=c++17 dsu_server_psmove.cpp -o dsu_server_psmove -pthread $(pkg-config --cflags --libs libpsmoveapi)
// Linux fallback:
//   g++ -O2 -std=c++17 dsu_server_psmove.cpp -o dsu_server_psmove -pthread -I/usr/include/psmoveapi -lpsmoveapi
// Windows (MinGW):
//   g++ -O2 -std=c++17 dsu_server_psmove.cpp -o dsu_server_psmove.exe -lpsmoveapi -lws2_32 -pthread
// Windows (MSVC):
//   cl /O2 /std:c++17 dsu_server_psmove.cpp psmoveapi.lib ws2_32.lib

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#define close closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <psmoveapi/psmove.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <optional>
#include <iomanip>
#include <signal.h>

// -------------------- Config --------------------
static int MAP_PITCH = 0;
static int MAP_YAW   = 2;
static int MAP_ROLL  = 1;
static bool FLIP_PITCH = false;
static bool FLIP_YAW   = true;
static bool FLIP_ROLL  = false;

static int MAP_ACCEL_X = 0;
static int MAP_ACCEL_Y = 2;
static int MAP_ACCEL_Z = 1;
static bool FLIP_ACCEL_X = true;
static bool FLIP_ACCEL_Y = true;
static bool FLIP_ACCEL_Z = false;

static const int ANALOG_TRIGGER_INDEX = 14; // payload[20+14] == payload[34] -> R2 analog
static const unsigned char TRIGGER_DIGITAL_THRESHOLD = 10;

static constexpr int PSMOVE_POLL_MS = 4;    // ~250 Hz
static constexpr int DSU_SEND_MS    = 8;    // ~125 Hz

// Default autopair behavior/timeouts
static const int PAIR_TIMEOUT_SECONDS = 12;
static bool AUTO_PAIR_ENABLED = true;

// Logging verbosity
static bool g_verbose = false; // --verbose CLI to enable more logs

// -----------------------------------------------------------------------------
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

                      // Forward declaration so PSMoveManager can call it (fixes compile order)
                      PSMove *psmove_autopair_and_connect(int pair_timeout_seconds = 12, bool verbose = true);

                      // -------------------- PSMove manager (single-controller) --------------------
                      struct MoveSample {
                          float accel_x=0, accel_y=0, accel_z=1.0f;   // g or the value psmoveapi reports
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

                              // Try bluetooth first
                              PSMove *mv = psmove_connect();
                              if (!mv && AUTO_PAIR_ENABLED) {
                                  std::cout << "[psmove] No Bluetooth controller found. Attempting USB auto-pairing (timeout "
                                  << PAIR_TIMEOUT_SECONDS << "s)...\n";
                                  mv = psmove_autopair_and_connect(PAIR_TIMEOUT_SECONDS, /*verbose=*/true);
                              }

                              if (!mv) {
                                  std::cerr << "[psmove] No controller available (BT or paired). Start failed.\n";
                                  return false;
                              }

                              move_ = mv;
                              running_ = true;
                              worker_ = std::thread(&PSMoveManager::thread_main, this);
                              if (g_verbose) std::cout << "[psmove] Manager started and polling thread spawned.\n";
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

                          std::optional<MoveSample> get_latest() {
                              std::lock_guard<std::mutex> lk(mtx_);
                              return latest_;
                          }

                          // returns battery enum/value from psmoveapi if available; -1 if not available
                          int get_battery_level() {
                              std::lock_guard<std::mutex> lk(mtx_);
                              if (!move_) return -1;
                              int b = psmove_get_battery(move_);
                              return b;
                          }

                      private:
                          // autopair helper moved outside as free function; we still keep thread_main private
                          void thread_main() {
                              int dbg_cnt = 0;
                              while (running_) {
                                  if (!move_) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }

                                  psmove_poll(move_);

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

                                      // occasional debug sample if verbose
                                      if (g_verbose && (++dbg_cnt % 250) == 0) {
                                          std::lock_guard<std::mutex> lk(mtx_);
                                          std::cout << "[psmove-sample] ax=" << latest_.accel_x << " ay=" << latest_.accel_y << " az=" << latest_.accel_z
                                          << " gx=" << latest_.gyro_x << " gy=" << latest_.gyro_y << " gz=" << latest_.gyro_z
                                          << " btn=0x" << std::hex << latest_.buttons << std::dec
                                          << " trig=" << (int)latest_.trigger << " ts=" << latest_.timestamp_us << "\n";
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

                      // -------------------- autopair helper (free function) --------------------
                      // Returns a PSMove* connected over Bluetooth, or nullptr. Verbose prints pairing progress.
                      // This is resilient to psmove_pair() not returning a nice status code: we poll for BT connect.
                      PSMove *psmove_autopair_and_connect(int pair_timeout_seconds /*=12*/, bool verbose /*=true*/) {
                          // 1) Try normal Bluetooth connect first (double-check)
                          PSMove *bt_move = psmove_connect();
                          if (bt_move) {
                              if (verbose) std::cout << "[psmove] Controller already connected via Bluetooth.\n";
                              return bt_move;
                          }

                          int count = psmove_count_connected(); // counts devices we can open (USB)
                          if (count <= 0) {
                              if (verbose) std::cout << "[psmove] No locally connected controllers found (psmove_count_connected == 0).\n";
                              return nullptr;
                          }

                          if (verbose) std::cout << "[psmove] Found " << count << " local device(s). Trying pairing via USB...\n";

                          for (int id = 0; id < count; ++id) {
                              if (verbose) std::cout << "[psmove] Attempting device id " << id << "...\n";
                              PSMove *usb = psmove_connect_by_id(id);
                              if (!usb) {
                                  if (verbose) std::cout << "[psmove] Unable to open device id " << id << ".\n";
                                  continue;
                              }

                              if (verbose) {
                                  std::cout << "[psmove] Connected to device id " << id << " via USB. Calling psmove_pair()...\n";
                                  std::cout << "[psmove] NOTE: pairing may require elevated permissions on this OS (sudo/Administrator).\n";
                              }

                              // Attempt pairing (some builds may print/log internally)
                              psmove_pair(usb);

                              // Cleanly disconnect USB handle; controller should now have host BT address written
                              psmove_disconnect(usb);
                              if (verbose) std::cout << "[psmove] psmove_pair() called for id " << id << ". Waiting for BT connection...\n";

                              // Poll for BT connection
                              const int poll_ms = 250;
                              const int maxpoll = (pair_timeout_seconds * 1000) / poll_ms;
                              for (int i = 0; i < maxpoll; ++i) {
                                  std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
                                  PSMove *try_bt = psmove_connect();
                                  if (try_bt) {
                                      if (verbose) std::cout << "[psmove] Successfully connected via Bluetooth after pairing.\n";
                                      return try_bt;
                                  }
                              }
                              if (verbose) std::cout << "[psmove] Timeout waiting for BT connection for device id " << id << ".\n";
                          }

                          if (verbose) std::cout << "[psmove] Auto-pairing attempted but no Bluetooth connection established.\n";
                          return nullptr;
                      }

                      // -------------------- DSU server (main) --------------------
                      struct Subscription { sockaddr_in addr{}; };

                      static std::atomic<bool> g_running(true);

                      #ifdef _WIN32
                      BOOL WINAPI console_ctrl_handler(DWORD dwCtrlType) {
                          if (dwCtrlType == CTRL_C_EVENT || dwCtrlType == CTRL_CLOSE_EVENT ||
                              dwCtrlType == CTRL_SHUTDOWN_EVENT) {
                              g_running = false;
                          return TRUE;
                              }
                              return FALSE;
                      }
                      #else
                      void handle_sigint(int) { g_running = false; }
                      #endif

                      int main(int argc, char** argv) {
                          // Ctrl-C handling
                          #ifdef _WIN32
                          SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
                          #else
                          signal(SIGINT, handle_sigint);
                          #endif

                          // CLI parsing: allow --verbose and --no-autopair
                          for (int i=1;i<argc;i++) {
                              if (std::strcmp(argv[i], "--verbose")==0) {
                                  g_verbose = true;
                              } else if (std::strcmp(argv[i], "--no-autopair")==0) {
                                  AUTO_PAIR_ENABLED = false;
                              } else if (std::strcmp(argv[i], "--map")==0 && i+3<argc) {
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

                          #ifdef _WIN32
                          // Initialize Winsock
                          WSADATA wsaData;
                          if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
                              std::cerr << "WSAStartup failed\n";
                              return 1;
                          }
                          #endif

                          // create UDP socket
                          #ifdef _WIN32
                          SOCKET sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                          if (sock == INVALID_SOCKET) { std::cerr<<"socket() failed: "<< WSAGetLastError() << "\n"; WSACleanup(); return 1; }
                          #else
                          int sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                          if (sock < 0) { perror("socket"); return 1; }
                          #endif

                          sockaddr_in bind_addr{}; bind_addr.sin_family = AF_INET; bind_addr.sin_port = htons(DSU_PORT); bind_addr.sin_addr.s_addr = INADDR_ANY;
                          if (bind(sock, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
                              #ifdef _WIN32
                              std::cerr << "bind() failed: " << WSAGetLastError() << "\n";
                              closesocket(sock); WSACleanup();
                              return 1;
                              #else
                              perror("bind"); close(sock); return 1;
                              #endif
                          }

                          uint32_t server_id = 0x12345678u;
                          std::cout << "DSU server (psmove cemuhook autopair) listening on UDP " << DSU_PORT << "\n";

                          std::vector<Subscription> subscriptions;
                          std::mutex sub_mutex;
                          uint8_t slot = 0;
                          uint64_t mac48 = 0x665544332211ULL;

                          // Start PSMove manager
                          PSMoveManager psm;
                          bool psm_started = psm.start();
                          if (!psm_started) {
                              std::cerr << "Failed to start PSMove manager. Exiting.\n";
                              #ifdef _WIN32
                              closesocket(sock); WSACleanup();
                              #else
                              close(sock);
                              #endif
                              return 2;
                          }

                          std::thread recv_thread([&](){
                              while (g_running) {
                                  uint8_t buf[2048]; sockaddr_in src{}; socklen_t sl = sizeof(src);
                                  #ifdef _WIN32
                                  int n = recvfrom(sock, (char*)buf, (int)sizeof(buf), 0, (sockaddr*)&src, &sl);
                                  #else
                                  ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, (sockaddr*)&src, &sl);
                                  #endif
                                  if (!g_running) break;
                                  if (n < 20) continue;
                                  if (!(buf[0]=='D' && buf[1]=='S' && buf[2]=='U' && buf[3]=='C')) continue;

                                  uint32_t msg_type = uint32_t(buf[16]) | (uint32_t(buf[17])<<8) | (uint32_t(buf[18])<<16) | (uint32_t(buf[19])<<24);
                                  if (msg_type == MSG_PROTOCOL_VER) {
                                      std::vector<uint8_t> payload; put_u16(payload, PROTO_VER);
                                      auto pkt = make_packet(server_id, MSG_PROTOCOL_VER, payload);
                                      #ifdef _WIN32
                                      sendto(sock, (char*)pkt.data(), (int)pkt.size(), 0, (sockaddr*)&src, sl);
                                      #else
                                      sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&src, sl);
                                      #endif
                                  } else if (msg_type == MSG_INFO) {
                                      std::vector<uint8_t> payload;
                                      int batt = psm.get_battery_level();
                                      uint8_t batt_byte = (batt >= 0 && batt <= 0xFF) ? uint8_t(batt) : 0x05u;
                                      put_shared_begin(payload, slot, 2, 2, 2, mac48, batt_byte);
                                      payload.push_back(0x00);
                                      auto pkt = make_packet(server_id, MSG_INFO, payload);
                                      #ifdef _WIN32
                                      sendto(sock, (char*)pkt.data(), (int)pkt.size(), 0, (sockaddr*)&src, sl);
                                      #else
                                      sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&src, sl);
                                      #endif
                                      if (g_verbose) std::cout << "[send] Replied INFO (battery=" << (int)batt_byte << ")\n";
                                  } else if (msg_type == MSG_DATA) {
                                      std::lock_guard<std::mutex> lock(sub_mutex);
                                      bool found = false;
                                      for (auto &s : subscriptions) {
                                          if (s.addr.sin_addr.s_addr == src.sin_addr.s_addr && s.addr.sin_port == src.sin_port) { found = true; break; }
                                      }
                                      if (!found) {
                                          Subscription sub; sub.addr = src; subscriptions.push_back(sub);
                                          std::cout << "[info] Client subscribed to DATA. Total: " << subscriptions.size() << "\n";
                                      }
                                  }
                              }
                          });

                          // conversion factor
                          const float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

                          std::thread send_thread([&](){
                              uint32_t packet_no = 0;
                              uint64_t t_us = 0;
                              int diag_counter = 0;
                              while (g_running) {
                                  std::this_thread::sleep_for(std::chrono::milliseconds(DSU_SEND_MS));

                                  std::vector<Subscription> subs_copy;
                                  { std::lock_guard<std::mutex> lock(sub_mutex); subs_copy = subscriptions; }
                                  if (subs_copy.empty()) {
                                      if (g_verbose) std::cout << "[info] no subscribers — skipping send\n";
                                      continue;
                                  }

                                  std::vector<uint8_t> p;
                                  // Use real battery for DATA header too
                                  int batt = psm.get_battery_level();
                                  uint8_t batt_byte = (batt >= 0 && batt <= 0xFF) ? uint8_t(batt) : 0x05u;
                                  put_shared_begin(p, slot, 2, 2, 2, mac48, batt_byte);

                                  p.push_back(1);
                                  put_u32(p, packet_no++);

                                  p.push_back(0); // bitmask1
                                  p.push_back(0); // bitmask2
                                  p.push_back(0); // HOME
                                  p.push_back(0); // Touch

                                  for (int i = 0; i < 16; ++i) {
                                      if (i==0 || i==1 || i==2 || i==3) p.push_back(128); else p.push_back(0);
                                  }

                                  for (int i = 0; i < 12; ++i) p.push_back(0);

                                  t_us += DSU_SEND_MS * 1000ull;

                                  std::optional<MoveSample> sopt = psm.get_latest();
                                  if (!sopt.has_value()) continue; // nothing to send
                                  MoveSample ms = sopt.value();

                                  // Convert gyro rad/s -> deg/s right away
                                  ms.gyro_x *= RAD_TO_DEG;
                                  ms.gyro_y *= RAD_TO_DEG;
                                  ms.gyro_z *= RAD_TO_DEG;

                                  put_u64(p, ms.timestamp_us ? ms.timestamp_us : t_us);

                                  // Map accelerometer
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

                                  // Map gyro (now in deg/s)
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

                                  // occasional diag if verbose
                                  if (g_verbose && (++diag_counter % 200) == 0) {
                                      std::cout << "[diag] gyro_deg/s (x,y,z)="
                                      << gyro_arr[0] << ", " << gyro_arr[1] << ", " << gyro_arr[2]
                                      << " -> pitch,yaw,roll=" << pitch << "," << yaw << "," << roll
                                      << "  battery=" << (int)batt_byte << "\n";
                                  }

                                  // --- BUTTONS & TRIGGER mapping ---
                                  uint8_t dsubitmask1 = 0;
                                  uint8_t dsubitmask2 = 0;
                                  uint8_t dsuhome = 0;

                                  // *** swapped mapping per your device quirks ***
                                  if (ms.buttons & Btn_CROSS)     dsubitmask2 |= 0x40u; // CROSS -> DSU CIRCLE
                                  if (ms.buttons & Btn_CIRCLE)    dsubitmask2 |= 0x20u; // CIRCLE -> DSU CROSS
                                  if (ms.buttons & Btn_SQUARE)    dsubitmask2 |= 0x80u; // SQUARE -> DSU TRIANGLE
                                  if (ms.buttons & Btn_TRIANGLE)  dsubitmask2 |= 0x10u; // TRIANGLE -> DSU SQUARE

                                  if (ms.buttons & Btn_MOVE)      dsubitmask2 |= 0x08u;

                                  if (ms.buttons & Btn_START)     dsubitmask1 |= 0x08u;
                                  if (ms.buttons & Btn_SELECT)    dsubitmask1 |= 0x01u;

                                  if (ms.buttons & Btn_PS) dsuhome = 1u;

                                  // Write control bytes into payload (payload control offset = 16)
                                  size_t control_offset = 16;
                                  if (p.size() >= control_offset + 4) {
                                      p[control_offset + 0] = dsubitmask1;
                                      p[control_offset + 1] = dsubitmask2;
                                      p[control_offset + 2] = dsuhome;
                                  }

                                  // analog base
                                  size_t analog_base = 20;

                                  // Ensure analog face-button bytes are set to 255 when pressed
                                  if (p.size() >= analog_base + 16) {
                                      // use same swapped mapping so analog matches DSU bits
                                      p[analog_base + 8]  = (ms.buttons & Btn_TRIANGLE) ? 255u : 0u; // Triangle ->Analog Y
                                      p[analog_base + 9]  = (ms.buttons & Btn_CIRCLE)   ? 255u : 0u; // Circle   ->Analog B
                                      p[analog_base + 10] = (ms.buttons & Btn_CROSS)    ? 255u : 0u; // Cross    ->Analog A
                                      p[analog_base + 11] = (ms.buttons & Btn_SQUARE)   ? 255u : 0u; // Square   ->Analog X

                                      // Move -> R1 analog (base+12)
                                      p[analog_base + 12]  = (ms.buttons & Btn_MOVE) ? 255u : 0u;
                                  }

                                  // write trigger analog to R2 slot
                                  if (ANALOG_TRIGGER_INDEX >= 0 && ANALOG_TRIGGER_INDEX < 16 && p.size() >= analog_base + 16) {
                                      p[analog_base + ANALOG_TRIGGER_INDEX] = ms.trigger;
                                  }

                                  // digital R2 if trigger pressed beyond threshold
                                  if (ms.trigger > TRIGGER_DIGITAL_THRESHOLD) {
                                      dsubitmask2 |= 0x02u; // digital R2
                                      if (p.size() >= control_offset + 2) p[control_offset + 1] = dsubitmask2;
                                  }

                                  // toggled detection (only print if verbose)
                                  static uint8_t prev_bm1 = 0, prev_bm2 = 0, prev_home = 0;
                                  uint8_t cur_bm1 = dsubitmask1;
                                  uint8_t cur_bm2 = dsubitmask2;
                                  uint8_t cur_home = dsuhome;
                                  uint8_t diff1 = cur_bm1 ^ prev_bm1;
                                  uint8_t diff2 = cur_bm2 ^ prev_bm2;
                                  uint8_t diffh = cur_home ^ prev_home;

                                  if (g_verbose && (diff1 || diff2 || diffh)) {
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
                                      #ifdef _WIN32
                                      sendto(sock, (char*)pkt.data(), (int)pkt.size(), 0, (sockaddr*)&s.addr, sizeof(s.addr));
                                      #else
                                      sendto(sock, pkt.data(), pkt.size(), 0, (sockaddr*)&s.addr, sizeof(s.addr));
                                      #endif
                                  }

                                  if (g_verbose) {
                                      std::cout << "[send] DATA #" << packet_no << " -> " << subs_copy.size()
                                      << " clients (payload " << p.size() << " bytes, pkt " << pkt.size()
                                      << ") using psmove=yes\n";
                                  }
                              }
                          });

                          if (!g_verbose) {
                              std::cout << "Server running (quiet mode). Use --verbose for detailed logs.\n";
                          }

                          recv_thread.join();
                          send_thread.join();

                          // cleanup
                          g_running = false;
                          psm.stop();

                          #ifdef _WIN32
                          closesocket(sock);
                          WSACleanup();
                          #else
                          close(sock);
                          #endif

                          return 0;
                      }
