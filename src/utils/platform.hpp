#pragma once

#ifdef _WIN32
// Prevent old winsock.h from being included anywhere
#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

// Configure Windows headers
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

// Include networking headers in correct order
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>  // For advanced TCP/UDP options

// Now windows.h won't include winsock.h
#include <windows.h>

#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
using socket_t = SOCKET;
constexpr socket_t INVALID_SOCKET_HANDLE = INVALID_SOCKET;
#define CLOSE_SOCKET closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>  // For TCP_NODELAY
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>        // For O_NONBLOCK
#include <errno.h>        // For errno
#include <string.h>       // For strerror
using socket_t = int;
constexpr socket_t INVALID_SOCKET_HANDLE = -1;
#define CLOSE_SOCKET close
#endif
