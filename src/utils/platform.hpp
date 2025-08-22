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

// Now windows.h won't include winsock.h
#include <windows.h>

#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#define CLOSE_SOCKET closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#define CLOSE_SOCKET close
#endif
