#include "network/udp_server.hpp"
#include "utils/logging.hpp"
#include <thread>

#ifndef _WIN32
#include <fcntl.h>
#else
#include <processthreadsapi.h>
#endif

UDPServer::UDPServer(int port) : port_(port), sock_(-1), running_(false) {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        Logger::error("WSAStartup failed");
    }
#endif
}

UDPServer::~UDPServer() {
    stop();
#ifdef _WIN32
    WSACleanup();
#endif
}

bool UDPServer::start() {
    if (running_.load()) return true;
    
#ifdef _WIN32
    sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) {
        Logger::error("socket() failed: " + std::to_string(WSAGetLastError()));
        return false;
    }
#else
    sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
        Logger::error("socket() failed");
        return false;
    }
#endif
    
    // OPTIMIZATION: Configure socket for low latency
    int opt = 1;
#ifdef _WIN32
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
    
    // Increase send buffer size
    int sendbuf = 65536;
    if (setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, (char*)&sendbuf, sizeof(sendbuf)) != 0) {
        Logger::warn("Failed to set send buffer size: " + std::to_string(WSAGetLastError()));
    }
    
#else
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Increase send buffer size
    int sendbuf = 65536;
    if (setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(sendbuf)) != 0) {
        Logger::warn("Failed to set send buffer size");
    }
    
    // Set socket to non-blocking mode for better performance
    int flags = fcntl(sock_, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
    }
    
    // Set high priority for UDP packets (requires root on Linux)
    int priority = 7;
    setsockopt(sock_, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));
#endif
    
    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(port_);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(sock_, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
#ifdef _WIN32
        Logger::error("bind() failed: " + std::to_string(WSAGetLastError()));
        CLOSE_SOCKET(sock_);
#else
        Logger::error("bind() failed");
        CLOSE_SOCKET(sock_);
#endif
        return false;
    }
    
    running_ = true;
    receive_thread_ = std::thread(&UDPServer::receive_loop, this);
    
    Logger::info("UDP server started on port " + std::to_string(port_) + " (optimized for low latency)");
    return true;
}

void UDPServer::stop() {
    running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    if (sock_ >= 0) {
        CLOSE_SOCKET(sock_);
        sock_ = -1;
    }
}

void UDPServer::send_to(const std::vector<uint8_t>& data, const sockaddr_in& addr) {
    if (!running_.load() || sock_ < 0) return;
    
#ifdef _WIN32
    int result = sendto(sock_, (char*)data.data(), (int)data.size(), 0, (sockaddr*)&addr, sizeof(addr));
    if (result == SOCKET_ERROR) {
        int error = WSAGetLastError();
        if (error != WSAEWOULDBLOCK) {
            Logger::debug("sendto failed: " + std::to_string(error));
        }
    }
#else
    ssize_t result = sendto(sock_, data.data(), data.size(), MSG_DONTWAIT, (sockaddr*)&addr, sizeof(addr));
    if (result < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            Logger::debug("sendto failed: " + std::string(strerror(errno)));
        }
    }
#endif
}

void UDPServer::send_to_multiple(const std::vector<uint8_t>& data, const std::vector<sockaddr_in>& addresses) {
    if (!running_.load() || sock_ < 0) return;
    
    // OPTIMIZATION: Batch send operations
    for (const auto& addr : addresses) {
#ifdef _WIN32
        sendto(sock_, (char*)data.data(), (int)data.size(), 0, (sockaddr*)&addr, sizeof(addr));
#else
        sendto(sock_, data.data(), data.size(), MSG_DONTWAIT, (sockaddr*)&addr, sizeof(addr));
#endif
    }
}

void UDPServer::set_message_handler(MessageHandler handler) {
    message_handler_ = handler;
}

void UDPServer::receive_loop() {
    uint8_t buffer[2048];
    
    // OPTIMIZATION: Set thread priority for network thread
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif
    
    while (running_.load()) {
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        
#ifdef _WIN32
        int bytes = recvfrom(sock_, (char*)buffer, sizeof(buffer), 0, (sockaddr*)&from, &from_len);
        if (bytes == SOCKET_ERROR) {
            int error = WSAGetLastError();
            if (error == WSAEWOULDBLOCK || error == WSAEINTR) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
        }
#else
        ssize_t bytes = recvfrom(sock_, buffer, sizeof(buffer), MSG_DONTWAIT, (sockaddr*)&from, &from_len);
        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
        }
#endif
        
        if (!running_.load()) break;
        if (bytes < 0) continue;
        
        if (message_handler_) {
            message_handler_(buffer, bytes, from);
        }
    }
}
