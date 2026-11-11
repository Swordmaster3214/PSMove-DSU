#include "gui/gui_app.hpp"
#include "gui/gui_panels.hpp"
#include "dsu/dsu_server.hpp"
#include "utils/logging.hpp"
#include "utils/signal_handler.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <cstdio>
#include <fstream>
#include <string>

namespace {

// The few GL 1.1 calls we make ourselves, loaded through GLFW so we don't need
// GL headers or to link libGL (ImGui's backend loads the rest itself).
#if defined(_WIN32) && !defined(_WIN64)
#define DSU_GLAPI __stdcall   // GL entry points are stdcall on 32-bit Windows
#else
#define DSU_GLAPI
#endif
typedef void (DSU_GLAPI* PFN_glViewport)(int, int, int, int);
typedef void (DSU_GLAPI* PFN_glClearColor)(float, float, float, float);
typedef void (DSU_GLAPI* PFN_glClear)(unsigned int);
PFN_glViewport p_glViewport = nullptr;
PFN_glClearColor p_glClearColor = nullptr;
PFN_glClear p_glClear = nullptr;
constexpr unsigned int GL_COLOR_BUFFER_BIT_ = 0x00004000;

void glfw_error_callback(int error, const char* description) {
    Logger::debug("GLFW error " + std::to_string(error) + ": " + description);
}

bool file_exists(const char* path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

void apply_style() {
    ImGui::StyleColorsDark();
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 0.0f;
    s.ChildRounding = 6.0f;
    s.FrameRounding = 4.0f;
    s.GrabRounding = 4.0f;
    s.PopupRounding = 4.0f;
    s.TabRounding = 4.0f;
    s.FramePadding = ImVec2(8, 5);
    s.ItemSpacing = ImVec2(8, 6);
    s.WindowPadding = ImVec2(12, 12);
    s.CellPadding = ImVec2(6, 4);

    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg]        = ImVec4(0.09f, 0.10f, 0.12f, 1.00f);
    c[ImGuiCol_ChildBg]         = ImVec4(0.12f, 0.13f, 0.16f, 1.00f);
    c[ImGuiCol_PopupBg]         = ImVec4(0.12f, 0.13f, 0.16f, 0.98f);
    c[ImGuiCol_Border]          = ImVec4(0.24f, 0.26f, 0.31f, 1.00f);
    c[ImGuiCol_FrameBg]         = ImVec4(0.18f, 0.19f, 0.23f, 1.00f);
    c[ImGuiCol_FrameBgHovered]  = ImVec4(0.23f, 0.25f, 0.30f, 1.00f);
    c[ImGuiCol_FrameBgActive]   = ImVec4(0.26f, 0.29f, 0.36f, 1.00f);
    c[ImGuiCol_Tab]             = ImVec4(0.14f, 0.15f, 0.19f, 1.00f);
    c[ImGuiCol_TabHovered]      = ImVec4(0.30f, 0.38f, 0.60f, 1.00f);
    c[ImGuiCol_TabSelected]     = ImVec4(0.24f, 0.32f, 0.52f, 1.00f);
    c[ImGuiCol_Button]          = ImVec4(0.22f, 0.29f, 0.46f, 1.00f);
    c[ImGuiCol_ButtonHovered]   = ImVec4(0.29f, 0.38f, 0.60f, 1.00f);
    c[ImGuiCol_ButtonActive]    = ImVec4(0.34f, 0.44f, 0.70f, 1.00f);
    c[ImGuiCol_Header]          = ImVec4(0.22f, 0.29f, 0.46f, 0.70f);
    c[ImGuiCol_HeaderHovered]   = ImVec4(0.29f, 0.38f, 0.60f, 0.80f);
    c[ImGuiCol_TableHeaderBg]   = ImVec4(0.15f, 0.16f, 0.20f, 1.00f);
    c[ImGuiCol_TableRowBgAlt]   = ImVec4(1.00f, 1.00f, 1.00f, 0.03f);
    c[ImGuiCol_PlotHistogram]   = ImVec4(0.36f, 0.62f, 0.95f, 1.00f);
    c[ImGuiCol_CheckMark]       = ImVec4(0.50f, 0.70f, 1.00f, 1.00f);
}

} // namespace

bool GuiApp::init() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        return false;
    }

#if defined(__APPLE__)
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#else
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

    window_ = glfwCreateWindow(1180, 820, "PSMove-DSU", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // vsync: the GUI never needs more than the display rate

    p_glViewport = reinterpret_cast<PFN_glViewport>(glfwGetProcAddress("glViewport"));
    p_glClearColor = reinterpret_cast<PFN_glClearColor>(glfwGetProcAddress("glClearColor"));
    p_glClear = reinterpret_cast<PFN_glClear>(glfwGetProcAddress("glClear"));
    if (!p_glViewport || !p_glClearColor || !p_glClear) {
        Logger::warn("GUI: could not load basic OpenGL functions");
        glfwDestroyWindow(window_);
        glfwTerminate();
        window_ = nullptr;
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // Don't litter the working directory with imgui.ini
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Work out DPI scaling. On Windows/X11, window coordinates are pixels and
    // we scale the UI ourselves; on macOS/Wayland they're already logical
    // units and only the framebuffer is larger.
    float xscale = 1.0f, yscale = 1.0f;
    glfwGetWindowContentScale(window_, &xscale, &yscale);
    int ww = 1, wh = 1, fw = 1, fh = 1;
    glfwGetWindowSize(window_, &ww, &wh);
    glfwGetFramebufferSize(window_, &fw, &fh);
    float fb_scale = ww > 0 ? float(fw) / float(ww) : 1.0f;
    if (fb_scale < 1.0f) fb_scale = 1.0f;
    float ui_scale = xscale / fb_scale;
    if (ui_scale < 1.0f) ui_scale = 1.0f;

    apply_style();
    ImGui::GetStyle().ScaleAllSizes(ui_scale);
    load_fonts(ui_scale, fb_scale);

    if (!ImGui_ImplGlfw_InitForOpenGL(window_, true) || !ImGui_ImplOpenGL3_Init(glsl_version)) {
        Logger::warn("GUI: failed to initialise the ImGui backends");
        ImGui::DestroyContext();
        glfwDestroyWindow(window_);
        glfwTerminate();
        window_ = nullptr;
        return false;
    }
    imgui_ready_ = true;
    return true;
}

void GuiApp::load_fonts(float ui_scale, float fb_scale) {
    ImGuiIO& io = ImGui::GetIO();
    // Rasterise at physical pixel size so text is crisp on HiDPI screens.
    const float size = 15.0f * ui_scale * fb_scale;

    static const char* candidates[] = {
#if defined(_WIN32)
        "C:\\Windows\\Fonts\\segoeui.ttf",
        "C:\\Windows\\Fonts\\tahoma.ttf",
        "C:\\Windows\\Fonts\\arial.ttf",
#elif defined(__APPLE__)
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/System/Library/Fonts/Geneva.ttf",
#else
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/google-noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/TTF/LiberationSans-Regular.ttf",
#endif
    };

    ImFont* font = nullptr;
    for (const char* path : candidates) {
        if (file_exists(path)) {
            font = io.Fonts->AddFontFromFileTTF(path, size);
            if (font) break;
        }
    }
    if (!font) {
        ImFontConfig cfg;
        cfg.SizePixels = 13.0f * ui_scale * fb_scale;
        io.Fonts->AddFontDefault(&cfg);
    }
    io.FontGlobalScale = 1.0f / fb_scale;
}

void GuiApp::run(DSUServer& server, const Config& launch_config) {
    GuiPanels panels(server, launch_config);

    while (!glfwWindowShouldClose(window_) && !SignalHandler::should_exit()) {
        // Minimised: don't burn CPU/GPU drawing nobody sees.
        if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED)) {
            glfwWaitEventsTimeout(0.25);
            continue;
        }
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        panels.draw();

        ImGui::Render();
        int fw, fh;
        glfwGetFramebufferSize(window_, &fw, &fh);
        p_glViewport(0, 0, fw, fh);
        p_glClearColor(0.09f, 0.10f, 0.12f, 1.0f);
        p_glClear(GL_COLOR_BUFFER_BIT_);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window_);
    }
}

void GuiApp::shutdown() {
    if (imgui_ready_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        imgui_ready_ = false;
    }
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}
