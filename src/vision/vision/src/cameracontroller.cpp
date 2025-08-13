#include "cameracontrol.h"


// Camera Controller
CameraController::CameraController(int w, int h, int f) : width(w), height(h), fps(f) {
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
}
bool CameraController::initialize() {
    try {
        auto profile = pipe.start(cfg);
        auto stream = profile.get_stream(RS2_STREAM_COLOR)
                        .as<rs2::video_stream_profile>();
        intrinsics = stream.get_intrinsics();
        serial = profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        return true;
    } catch (const rs2::error& e) {
        std::cerr << "Camera init failed: " << e.what() << std::endl;
        return false;
    }
}
void CameraController::shutdown() {
    try { pipe.stop(); } catch (...) {}
}
rs2::frameset CameraController::poll_frames(int max_retries) {
    for (int i = 0; i < max_retries; ++i) {
        rs2::frameset frames;
        if (pipe.poll_for_frames(&frames)) {
            return frames;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return {};
}
