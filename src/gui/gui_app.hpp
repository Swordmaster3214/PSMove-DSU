#pragma once

struct GLFWwindow;
struct Config;
class DSUServer;

// Owns the window and the ImGui context. All drawing lives in GuiPanels.
class GuiApp {
public:
    // Opens the window. Returns false if there is no display to open it on.
    bool init();
    // Runs until the window is closed (or Ctrl+C). The server keeps running
    // on its own threads; the GUI only reads its state and edits settings.
    void run(DSUServer& server, const Config& launch_config);
    void shutdown();

private:
    void load_fonts(float ui_scale, float fb_scale);

    GLFWwindow* window_ = nullptr;
    bool imgui_ready_ = false;
};
