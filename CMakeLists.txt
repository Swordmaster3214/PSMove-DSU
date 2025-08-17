cmake_minimum_required(VERSION 3.10)
project(dsu_server_psmove)

set(CMAKE_CXX_STANDARD 17)

# Allow user to specify where psmoveapi is located
if(NOT DEFINED PSMOVE_ROOT)
    set(PSMOVE_ROOT "${CMAKE_SOURCE_DIR}/psmoveapi_install")
endif()

# Include headers
include_directories(${PSMOVE_ROOT}/include)

# Add library search path
link_directories(${PSMOVE_ROOT}/lib)

# Source files
add_executable(dsu_server_psmove dsu_server_psmove.cpp)

# Platform-specific libraries
if(WIN32)
    target_link_libraries(dsu_server_psmove psmoveapi setupapi ws2_32)
elseif(APPLE)
    target_link_libraries(dsu_server_psmove psmoveapi usb-1.0)
else()
    # Linux
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    pkg_check_modules(BLUETOOTH REQUIRED bluez)

    include_directories(${LIBUSB_INCLUDE_DIRS} ${BLUETOOTH_INCLUDE_DIRS})
    link_directories(${LIBUSB_LIBRARY_DIRS} ${BLUETOOTH_LIBRARY_DIRS})

    target_link_libraries(dsu_server_psmove
        psmoveapi
        ${LIBUSB_LIBRARIES}
        ${BLUETOOTH_LIBRARIES}
        pthread
    )
endif()
