cmake_minimum_required(VERSION 3.13)
project(dsu_server_psmove LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ======================================================
# User-configurable variables:
# - Either pass -DPSMOVE_ROOT=/path/to/extracted_psmoveapi
# - OR pass -DCMAKE_PREFIX_PATH=/path/to/extracted_psmoveapi
# If none provided, CMake will try pkg-config as a fallback.
# ======================================================
option(USE_PKGCONFIG "Try pkg-config to find psmoveapi first" ON)
set(PSMOVE_ROOT "" CACHE PATH "Root directory of extracted psmoveapi (contains include/ and lib/)")

# Helper: canonicalize prefix list
if (PSMOVE_ROOT)
  list(APPEND CMAKE_PREFIX_PATH "${PSMOVE_ROOT}")
endif()

# Try pkg-config first (optional)
if (USE_PKGCONFIG)
  find_package(PkgConfig QUIET)
  if (PkgConfig_FOUND)
    pkg_check_modules(PSMOVE_PKG QUIET psmoveapi)
    if (PSMOVE_PKG_FOUND)
      message(STATUS "Found psmoveapi via pkg-config.")
      include_directories(${PSMOVE_PKG_INCLUDE_DIRS})
      link_directories(${PSMOVE_PKG_LIBRARY_DIRS})
      add_executable(dsu_server_psmove dsu_server_psmove.cpp)
      target_compile_options(dsu_server_psmove PRIVATE ${PSMOVE_PKG_CFLAGS_OTHER})
      target_link_libraries(dsu_server_psmove PRIVATE ${PSMOVE_PKG_LIBRARIES})
      # set rpath so runtime can find libs from pkg-config prefix
      set_target_properties(dsu_server_psmove PROPERTIES
        BUILD_RPATH "${PSMOVE_PKG_LIBRARY_DIRS}"
        INSTALL_RPATH "${PSMOVE_PKG_LIBRARY_DIRS}"
        INSTALL_RPATH_USE_LINK_PATH TRUE
      )
      return()
    else()
      message(STATUS "pkg-config could not find psmoveapi; falling back to CMAKE_PREFIX_PATH / PSMOVE_ROOT search.")
    endif()
  else()
    message(STATUS "pkg-config not available; falling back to CMAKE search.")
  endif()
endif()

# Fallback: look for headers and library under provided prefixes (CMAKE_PREFIX_PATH is honored)
# Look for psmove.h
find_path(PSMOVE_INCLUDE_DIR
  NAMES psmove.h
  PATHS ${CMAKE_PREFIX_PATH}
        ${CMAKE_PREFIX_PATH}/include
        ${CMAKE_PREFIX_PATH}/include/psmoveapi
        ${CMAKE_PREFIX_PATH}/local/include
  NO_DEFAULT_PATH
)

# Try to find the library (name variants)
find_library(PSMOVE_LIBRARY
  NAMES psmove psmoveapi psmoveapi_shared
  PATHS ${CMAKE_PREFIX_PATH}
        ${CMAKE_PREFIX_PATH}/lib
        ${CMAKE_PREFIX_PATH}/lib64
        ${CMAKE_PREFIX_PATH}/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH
)

if (PSMOVE_INCLUDE_DIR AND PSMOVE_LIBRARY)
  message(STATUS "Found psmoveapi headers in: ${PSMOVE_INCLUDE_DIR}")
  message(STATUS "Found psmoveapi library: ${PSMOVE_LIBRARY}")

  # Create an imported target for nicer consumption
  add_library(psmoveapi::psmoveapi UNKNOWN IMPORTED)
  set_target_properties(psmoveapi::psmoveapi PROPERTIES
    IMPORTED_LOCATION "${PSMOVE_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${PSMOVE_INCLUDE_DIR}"
  )

  add_executable(dsu_server_psmove dsu_server_psmove.cpp)
  target_link_libraries(dsu_server_psmove PRIVATE psmoveapi::psmoveapi)

  # Ensure runtime can find shared libs when running from build directory
  # Use first CMAKE_PREFIX_PATH entry (if any) as rpath; fallback to directory of found lib
  if (CMAKE_PREFIX_PATH)
    list(GET CMAKE_PREFIX_PATH 0 _first_prefix)
    set(_rpath "${_first_prefix}/lib")
  else()
    get_filename_component(_rpath "${PSMOVE_LIBRARY}" DIRECTORY)
  endif()

  # On Linux use $ORIGIN-based relative rpath if possible (more portable for distribution)
  # If rpath is absolute, CMake will set it directly.
  set_target_properties(dsu_server_psmove PROPERTIES
    BUILD_RPATH "${_rpath}"
    INSTALL_RPATH "${_rpath}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )

else()
  message(FATAL_ERROR "Could not locate psmoveapi. Provide it via -DPSMOVE_ROOT=/path/to/extracted_psmoveapi or -DCMAKE_PREFIX_PATH=/path/to/extracted_psmoveapi\n"
                     "Searched include paths: ${CMAKE_PREFIX_PATH}\n"
                     "You can also enable pkg-config (USE_PKGCONFIG=ON) and have psmoveapi available there.")
endif()

# Optional: install target (to package easily)
install(TARGETS dsu_server_psmove
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
