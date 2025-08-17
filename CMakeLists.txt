cmake_minimum_required(VERSION 3.10)
project(dsu_server_psmove LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Allow user to pass PSMOVE_ROOT (from workflow or local)
if(NOT DEFINED PSMOVE_ROOT)
    set(PSMOVE_ROOT "${CMAKE_SOURCE_DIR}/psmoveapi_install")
endif()

message(STATUS "Using PSMOVE_ROOT: ${PSMOVE_ROOT}")

# Include and lib paths
include_directories(${PSMOVE_ROOT}/include)
link_directories(${PSMOVE_ROOT}/lib)

# Our source
add_executable(dsu_server_psmove dsu_server_psmove.cpp)

# Platform-specific binary naming
if(WIN32)
    set_target_properties(dsu_server_psmove PROPERTIES OUTPUT_NAME "dsu_server_psmove-windows")
elseif(APPLE)
    set_target_properties(dsu_server_psmove PROPERTIES OUTPUT_NAME "dsu_server_psmove-macos")
elseif(UNIX AND NOT APPLE)
    set_target_properties(dsu_server_psmove PROPERTIES OUTPUT_NAME "dsu_server_psmove-linux")
endif()

# Platform-specific libraries
if(WIN32)
    message(STATUS "Configuring for Windows")
    target_link_libraries(dsu_server_psmove PRIVATE psmoveapi)
elseif(APPLE)
    message(STATUS "Configuring for macOS")
    target_link_libraries(dsu_server_psmove PRIVATE psmoveapi)
elseif(UNIX AND NOT APPLE)
    message(STATUS "Configuring for Linux")
    # Link bluetooth explicitly, since psmoveapi depends on it
    target_link_libraries(dsu_server_psmove PRIVATE psmoveapi bluetooth)
endif()

# Install rules (optional, for local installs)
install(TARGETS dsu_server_psmove DESTINATION bin)
