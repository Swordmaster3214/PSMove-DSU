cmake_minimum_required(VERSION 3.10)
project(dsu_server_psmove)
set(CMAKE_CXX_STANDARD 17)

# Determine platform suffix for executable name
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(PLATFORM_SUFFIX "win64")
    else()
        set(PLATFORM_SUFFIX "win32")
    endif()
elseif(APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
        set(PLATFORM_SUFFIX "macos_arm64")
    else()
        set(PLATFORM_SUFFIX "macos_x64")
    endif()
else()
    # Linux and other Unix-like systems
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
            set(PLATFORM_SUFFIX "linux_arm64")
        else()
            set(PLATFORM_SUFFIX "linux_x64")
        endif()
    else()
        set(PLATFORM_SUFFIX "linux_x86")
    endif()
endif()

# Set the executable name with platform suffix
set(EXECUTABLE_NAME "dsu_server_psmove_${PLATFORM_SUFFIX}")

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
include_directories(src)

# Add library search path
link_directories(${PSMOVE_ROOT}/lib)

# Source files
set(SOURCES
    src/main.cpp
    src/dsu/dsu_server.cpp
    src/dsu/dsu_protocol.cpp
    src/psmove/psmove_manager.cpp
    src/psmove/psmove_pairing.cpp
    src/network/udp_server.cpp
    src/utils/logging.cpp
    src/utils/signal_handler.cpp
)

# Create executable with platform-specific name
add_executable(${EXECUTABLE_NAME} ${SOURCES})

# Platform-specific libraries
if(WIN32)
    target_link_libraries(${EXECUTABLE_NAME} 
        psmoveapi setupapi ws2_32 winmm
    )
elseif(APPLE)
    find_library(USB_LIBRARY usb-1.0 REQUIRED)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation REQUIRED)
    find_library(IOKIT_LIBRARY IOKit REQUIRED)
    
    target_link_libraries(${EXECUTABLE_NAME} 
        psmoveapi ${USB_LIBRARY} ${COREFOUNDATION_LIBRARY} ${IOKIT_LIBRARY}
    )
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    pkg_check_modules(BLUETOOTH REQUIRED bluez)
    pkg_check_modules(UDEV libudev)
    
    include_directories(${LIBUSB_INCLUDE_DIRS} ${BLUETOOTH_INCLUDE_DIRS})
    link_directories(${LIBUSB_LIBRARY_DIRS} ${BLUETOOTH_LIBRARY_DIRS})
    
    set(LINUX_LIBS psmoveapi ${LIBUSB_LIBRARIES} ${BLUETOOTH_LIBRARIES} pthread)
    
    if(UDEV_FOUND)
        list(APPEND LINUX_LIBS ${UDEV_LIBRARIES})
        include_directories(${UDEV_INCLUDE_DIRS})
        link_directories(${UDEV_LIBRARY_DIRS})
    endif()
    
    target_link_libraries(${EXECUTABLE_NAME} ${LINUX_LIBS})
endif()

# Set output directory
set_target_properties(${EXECUTABLE_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}
)

# Enable debug info for release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(WIN32)
        target_compile_options(${EXECUTABLE_NAME} PRIVATE /Zi)
        target_link_options(${EXECUTABLE_NAME} PRIVATE /DEBUG)
    else()
        target_compile_options(${EXECUTABLE_NAME} PRIVATE -g)
    endif()
endif()

# Print the final executable name for confirmation
message(STATUS "Building executable: ${EXECUTABLE_NAME}")
