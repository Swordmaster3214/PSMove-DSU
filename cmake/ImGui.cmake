# ---------------------------------------------------------------------------
# GUI: Dear ImGui on GLFW + OpenGL 3 (built from source, linked statically)
#
# Chosen because it is small, has no runtime to ship, and runs the same on
# Windows, macOS and Linux (X11 and Wayland). Turn off with -DDSU_BUILD_GUI=OFF
# for a console-only build; the executable then behaves like --headless.
#
# Offline builds: -DFETCHCONTENT_SOURCE_DIR_IMGUI=... -DFETCHCONTENT_SOURCE_DIR_GLFW=...
# ---------------------------------------------------------------------------
option(DSU_BUILD_GUI "Build the graphical interface" ON)

if(DSU_BUILD_GUI)
    FetchContent_Declare(imgui
        URL      https://github.com/ocornut/imgui/archive/refs/tags/v1.91.9b.tar.gz
        URL_HASH SHA256=8e1bbc76c71d74fef2fb85db7e7ca8eba13d6a86623c54992b60162db554ffdb
        SOURCE_SUBDIR cmake_do_not_add_subdirectory
    )
    FetchContent_Declare(glfw
        URL      https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
        URL_HASH SHA256=c038d34200234d071fae9345bc455e4a8f2f544ab60150765d7704e08f3dac01
    )

    set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS   OFF)  # GLFW as a static library

    if(UNIX AND NOT APPLE)
        # GLFW loads libX11 / libwayland at runtime, so the binary still starts on
        # machines without them (and falls back to headless). Wayland support is
        # compiled in only when its development files are installed.
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(GLFW_WAYLAND_DEPS QUIET wayland-client wayland-cursor wayland-egl xkbcommon)
        find_program(WAYLAND_SCANNER wayland-scanner)
        if(GLFW_WAYLAND_DEPS_FOUND AND WAYLAND_SCANNER)
            set(GLFW_BUILD_WAYLAND ON CACHE BOOL "" FORCE)
            message(STATUS "GUI: building GLFW with X11 and Wayland support")
        else()
            set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
            message(STATUS "GUI: building GLFW with X11 support only (Wayland dev packages not found)")
        endif()
        set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
    endif()

    FetchContent_MakeAvailable(imgui glfw)
    set(IMGUI_DIR "${imgui_SOURCE_DIR}")

    add_library(imgui_static STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui_static PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
    target_link_libraries(imgui_static PUBLIC glfw ${CMAKE_DL_LIBS})
    # Stop glfw3.h from including <GL/gl.h>: the GLFW backend makes no GL calls
    # and the OpenGL backend has its own loader, so no GL dev headers are needed.
    target_compile_definitions(imgui_static PRIVATE GLFW_INCLUDE_NONE)
    # ImGui's OpenGL backend loads GL functions itself; only macOS needs the framework linked.
    if(APPLE)
        target_link_libraries(imgui_static PUBLIC "-framework OpenGL")
    elseif(WIN32)
        target_link_libraries(imgui_static PUBLIC opengl32)
    endif()
    set_target_properties(imgui_static PROPERTIES CXX_STANDARD 17 POSITION_INDEPENDENT_CODE ON)
endif()
