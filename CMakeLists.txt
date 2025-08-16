cmake_minimum_required(VERSION 3.10)
project(dsu_server_psmove)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PSMOVE REQUIRED psmoveapi)

include_directories(${PSMOVE_INCLUDE_DIRS})
link_directories(${PSMOVE_LIBRARY_DIRS})

add_executable(dsu_server_psmove dsu_server_psmove.cpp)
target_link_libraries(dsu_server_psmove ${PSMOVE_LIBRARIES})
