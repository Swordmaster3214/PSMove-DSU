#pragma once

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
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

// PSMoveAPI compatibility - only define what's not in the main headers
#ifndef PSMOVE_BTADDR_SIZE
#define PSMOVE_BTADDR_SIZE 6
typedef struct {
    unsigned char data[PSMOVE_BTADDR_SIZE];
} PSMove_Data_BTAddr;
#endif
