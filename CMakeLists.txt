cmake_minimum_required(VERSION 3.21)

# Link the MSVC C/C++ runtime statically so the .exe does not require the
# Visual C++ Redistributable on the user's machine. Must be set before project().
option(DSU_STATIC_RUNTIME "Statically link the MSVC runtime (Windows/MSVC only)" ON)
if(DSU_STATIC_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Without this, macOS binaries require the same macOS version as the machine
# that built them (CI runners are always recent). 10.15 is the first release
# with std::filesystem; arm64 builds are raised to 11.0 automatically.
# Must be set before project().
if(CMAKE_HOST_APPLE AND NOT CMAKE_OSX_DEPLOYMENT_TARGET)  # APPLE isn't set until project()
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15" CACHE STRING "Minimum macOS version")
endif()

project(dsu_server_psmove LANGUAGES C CXX)
if(APPLE)
    # OBJCXX for PSMoveAPI's .mm files. OBJC is needed too: without it CMake
    # compiles GLFW's .m files as Objective-C++, which rejects GLFW's C code.
    enable_language(OBJC OBJCXX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Link-time optimisation for Release builds (our code and the static third-party
# libraries). Smaller and slightly faster; turn off if a toolchain misbehaves.
option(DSU_ENABLE_LTO "Enable link-time optimisation in Release builds" ON)
if(DSU_ENABLE_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT DSU_LTO_SUPPORTED OUTPUT DSU_LTO_OUTPUT LANGUAGES C CXX)
    if(DSU_LTO_SUPPORTED)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    else()
        message(STATUS "LTO not supported by this toolchain, skipping: ${DSU_LTO_OUTPUT}")
    endif()
endif()

# Tune for the CPU doing the build (-march=native). The binary may crash with
# an illegal instruction on other CPUs, so it is opt-in, never used for releases,
# and the executable gets a "_native" suffix. Applies to third-party code too.
option(DSU_NATIVE "Optimise for this machine's CPU (binary is not portable)" OFF)
if(DSU_NATIVE)
    include(CheckCXXCompilerFlag)
    if(MSVC)
        message(WARNING "DSU_NATIVE: MSVC has no equivalent of -march=native, ignoring")
        set(DSU_NATIVE OFF)
    else()
        check_cxx_compiler_flag(-march=native DSU_HAS_MARCH_NATIVE)
        check_cxx_compiler_flag(-mcpu=native  DSU_HAS_MCPU_NATIVE)  # Apple Silicon, some ARM
        if(DSU_HAS_MARCH_NATIVE)
            set(DSU_NATIVE_FLAG -march=native)
        elseif(DSU_HAS_MCPU_NATIVE)
            set(DSU_NATIVE_FLAG -mcpu=native)
        else()
            message(WARNING "DSU_NATIVE: compiler supports neither -march=native nor -mcpu=native, ignoring")
            set(DSU_NATIVE OFF)
        endif()
    endif()
endif()
if(DSU_NATIVE)
    message(STATUS "Native build: ${DSU_NATIVE_FLAG} (not portable to other CPUs)")
    add_compile_options(${DSU_NATIVE_FLAG})
endif()

# compile_commands.json for clangd and other tooling (Makefile/Ninja generators
# only; clangd finds it in build/ automatically).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Single-config generators (Make, Ninja) otherwise build with no optimisation
# and no flags at all when -DCMAKE_BUILD_TYPE is omitted.
get_property(DSU_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT DSU_IS_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

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
if(DSU_NATIVE)
    set(EXECUTABLE_NAME "${EXECUTABLE_NAME}_native")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW) # use extraction time as archive timestamps
endif()

include(PSMoveAPI)

include(ImGui)

# Version string shown in the GUI and diagnostics report. Regenerated on every
# build (see cmake/GitVersion.cmake) so it follows the checkout.
find_package(Git QUIET)
set(DSU_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_custom_target(dsu_version
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -DOUTPUT=${DSU_GENERATED_DIR}/dsu_version.h
        -DGIT_EXECUTABLE=${GIT_EXECUTABLE}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GitVersion.cmake
    BYPRODUCTS ${DSU_GENERATED_DIR}/dsu_version.h
    COMMENT "Updating version header"
    VERBATIM
)

include(EmbedText)

set(DSU_LICENSE_SOURCE "// Generated by CMakeLists.txt from LICENSE and THIRD_PARTY_LICENSES.md.\n")
dsu_embed_text(DSU_LICENSE_SOURCE dsu_license_gpl "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
dsu_embed_text(DSU_LICENSE_SOURCE dsu_license_third_party "${CMAKE_CURRENT_SOURCE_DIR}/THIRD_PARTY_LICENSES.md")
set(DSU_LICENSE_CPP "${CMAKE_CURRENT_BINARY_DIR}/generated/license_texts.cpp")
file(CONFIGURE OUTPUT "${DSU_LICENSE_CPP}" CONTENT "${DSU_LICENSE_SOURCE}" @ONLY)

# ---------------------------------------------------------------------------
# DSU server
# ---------------------------------------------------------------------------
# Source files
set(SOURCES
    src/main.cpp
    src/config/config_file.cpp
    src/config/mapping.cpp
    src/dsu/dsu_server.cpp
    src/dsu/dsu_protocol.cpp
    src/psmove/psmove_manager.cpp
    src/psmove/psmove_pairing.cpp
    src/psmove/nav_pairing.cpp
    src/psmove/linux_setup.cpp
    src/network/udp_server.cpp
    src/utils/logging.cpp
    src/utils/signal_handler.cpp
    src/utils/licenses.cpp
    ${DSU_LICENSE_CPP}
)

if(DSU_BUILD_GUI)
    list(APPEND SOURCES
        src/gui/gui_app.cpp
        src/gui/gui_panels.cpp
    )
endif()

# Create executable with platform-specific name
add_executable(${EXECUTABLE_NAME} ${SOURCES})

target_link_libraries(${EXECUTABLE_NAME} PRIVATE psmoveapi_static)
target_include_directories(${EXECUTABLE_NAME} PRIVATE src ${DSU_GENERATED_DIR}
    ${PSMOVEAPI_DIR}/src ${HIDAPI_DIR}/hidapi)  # psmove_port.h, hidapi.h
add_dependencies(${EXECUTABLE_NAME} dsu_version)

if(DSU_BUILD_GUI)
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE imgui_static)
    target_compile_definitions(${EXECUTABLE_NAME} PRIVATE DSU_WITH_GUI)
endif()

# Platform-specific configurations
if(WIN32)
    # Windows-specific compile definitions to prevent winsock conflicts
    target_compile_definitions(${EXECUTABLE_NAME} PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        _WINSOCKAPI_
    )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Don't depend on the build machine's libgcc/libstdc++ versions.
    # (System libs such as libudev/libdbus/libbluetooth stay dynamic.)
    target_link_options(${EXECUTABLE_NAME} PRIVATE -static-libgcc -static-libstdc++)
endif()

# Set output directory
set_target_properties(${EXECUTABLE_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}
)

# Debug info for release builds. Generator expressions, because multi-config
# generators (Visual Studio, Xcode) leave CMAKE_BUILD_TYPE empty. /DEBUG turns
# off linker optimisation by default, so switch it back on.
if(MSVC)
    target_compile_options(${EXECUTABLE_NAME} PRIVATE $<$<CONFIG:Release>:/Zi>)
    target_link_options(${EXECUTABLE_NAME} PRIVATE
        $<$<CONFIG:Release>:/DEBUG /OPT:REF /OPT:ICF>)
else()
    target_compile_options(${EXECUTABLE_NAME} PRIVATE $<$<CONFIG:Release>:-g>)
endif()

# Hardening for a network-facing server: stack protector, and fortified libc
# calls (needs optimisation, hence Release only). -U first because some
# distribution compilers already define _FORTIFY_SOURCE.
if(NOT MSVC)
    target_compile_options(${EXECUTABLE_NAME} PRIVATE -fstack-protector-strong)
    target_compile_options(${EXECUTABLE_NAME} PRIVATE
        $<$<CONFIG:Release>:-U_FORTIFY_SOURCE> $<$<CONFIG:Release>:-D_FORTIFY_SOURCE=2>)
endif()

# Warnings for our own code only (third-party targets are built with -w).
option(DSU_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
if(MSVC)
    target_compile_options(${EXECUTABLE_NAME} PRIVATE /W4)
    if(DSU_WARNINGS_AS_ERRORS)
        target_compile_options(${EXECUTABLE_NAME} PRIVATE /WX)
    endif()
else()
    target_compile_options(${EXECUTABLE_NAME} PRIVATE -Wall -Wextra)
    if(DSU_WARNINGS_AS_ERRORS)
        target_compile_options(${EXECUTABLE_NAME} PRIVATE -Werror)
    endif()
endif()

# ---------------------------------------------------------------------------
# Tests (optional): the DSU protocol check against a simulated server.
# Run with: ctest --test-dir build
# ---------------------------------------------------------------------------
option(DSU_BUILD_TESTS "Build the test clients and register CTest tests" OFF)
if(DSU_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# `cmake --install build --prefix <dir>` puts the executable and licences in <dir>.
include(GNUInstallDirs)
install(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES LICENSE THIRD_PARTY_LICENSES.md DESTINATION ${CMAKE_INSTALL_DOCDIR})

# Print the final executable name for confirmation
message(STATUS "Building executable: ${EXECUTABLE_NAME}")
