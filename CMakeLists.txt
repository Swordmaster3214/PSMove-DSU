cmake_minimum_required(VERSION 3.10)
project(dsu_server_psmove)

set(CMAKE_CXX_STANDARD 17)

# Allow user to specify where psmoveapi is located
if(NOT DEFINED PSMOVE_ROOT)
    set(PSMOVE_ROOT "${CMAKE_SOURCE_DIR}/psmoveapi_install")
endif()

# Check if psmoveapi exists
if(NOT EXISTS "${PSMOVE_ROOT}/include/psmove.h")
    message(FATAL_ERROR "psmoveapi not found at ${PSMOVE_ROOT}. Please set PSMOVE_ROOT to the correct path.")
endif()

# Include headers
include_directories(${PSMOVE_ROOT}/include)

# Add library search path
link_directories(${PSMOVE_ROOT}/lib)

# Source files
add_executable(dsu_server_psmove dsu_server_psmove.cpp)

# Platform-specific libraries
if(WIN32)
    # Windows libraries - added winmm for multimedia timer functions
    target_link_libraries(dsu_server_psmove 
        psmoveapi 
        setupapi 
        ws2_32 
        winmm
    )
elseif(APPLE)
    # macOS libraries
    find_library(USB_LIBRARY usb-1.0 REQUIRED)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation REQUIRED)
    find_library(IOKIT_LIBRARY IOKit REQUIRED)
    
    target_link_libraries(dsu_server_psmove 
        psmoveapi 
        ${USB_LIBRARY}
        ${COREFOUNDATION_LIBRARY}
        ${IOKIT_LIBRARY}
    )
else()
    # Linux
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    pkg_check_modules(BLUETOOTH REQUIRED bluez)

    # Check for udev (often needed for device enumeration)
    pkg_check_modules(UDEV libudev)

    include_directories(${LIBUSB_INCLUDE_DIRS} ${BLUETOOTH_INCLUDE_DIRS})
    link_directories(${LIBUSB_LIBRARY_DIRS} ${BLUETOOTH_LIBRARY_DIRS})

    set(LINUX_LIBS
        psmoveapi
        ${LIBUSB_LIBRARIES}
        ${BLUETOOTH_LIBRARIES}
        pthread
    )
    
    # Add udev if found
    if(UDEV_FOUND)
        list(APPEND LINUX_LIBS ${UDEV_LIBRARIES})
        include_directories(${UDEV_INCLUDE_DIRS})
        link_directories(${UDEV_LIBRARY_DIRS})
    endif()

    target_link_libraries(dsu_server_psmove ${LINUX_LIBS})
endif()

# Set output directory for consistency across platforms
set_target_properties(dsu_server_psmove PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}
)

# Enable debug info for release builds (useful for debugging)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(WIN32)
        target_compile_options(dsu_server_psmove PRIVATE /Zi)
        target_link_options(dsu_server_psmove PRIVATE /DEBUG)
    else()
        target_compile_options(dsu_server_psmove PRIVATE -g)
    endif()
endif()
