#pragma once
#include <atomic>
#include <string>

// How pictures go from the radio to the screen (DISPLAY > Frame Mode, saved as
// "frame_mode"; FULL FRAME by default), listed from the lowest latency to the
// highest. Read by the source, the decoder and the renderer as they go,
// so a change applies at once. The /tmp switches still override it for tests:
// kestrel-stream-off, kestrel-early-off, kestrel-split-off.
enum FrameMode {
    kFrameSplit = 0,        // each slice decoded as it lands; the top half and the
                            // lower part go up on two planes as soon as each is decoded
    kFrameWhole = 1,        // each slice decoded as it lands; only whole pictures go up
    kFrameWholeDecode = 2,  // whole pictures to the decoder, whole pictures up
};
static const int kFrameModeCount = 3;
extern std::atomic<int> g_frame_mode;

// The camera's frame rate as the stream's headers give it, 0 until the first.
// Written by the source; the display follows it on Screen Mode Auto
// (DrmDevice::follow_camera).
extern std::atomic<int> g_camera_fps;
// ... and its lines (the picture's height), written before the rate.
extern std::atomic<int> g_camera_lines;

// The settings key of the camera's own frame period as measured, one per
// video mode ("camera_period_us_1080p100"): Screen Mode Auto trims the
// screen's clock to it (DrmDevice), straight away for a mode seen before.
static inline std::string camera_period_key(int lines, int fps) {
    return "camera_period_us_" + std::to_string(lines) + "p" + std::to_string(fps);
}

// DISPLAY > Refresh Sync for the screen in use ("display_lock_<screen ID>"):
// the display lock (display_lock.hpp) holds the vblank just after the pictures.
extern std::atomic<int> g_display_lock;
// SPIKE (camera steering, Refresh Sync CAMERA = 3): the camera's frames this
// much longer (negative: shorter), ns, asked of a kestrel-air every 250 ms
// while g_camera_steer is on (sky cmd 0x43).
extern std::atomic<int> g_camera_trim_ns;
extern std::atomic<bool> g_camera_steer;
// DISPLAY > Sync To ("display_lock_slice"): which part of a picture the lock
// times the refresh to - 1, its first slice (its top, with each half on its
// own plane), the default; 2, its last slice (the whole picture).
extern std::atomic<int> g_display_lock_slice;
// DISPLAY > Sync Margin ("display_lock_margin_ms"): the refresh this much
// later than where the expected wait is least, us. Fewer pictures miss their
// refresh (each one shows a refresh late, a hitch), for that much latency.
extern std::atomic<int> g_display_lock_margin_us;
// What the display lock sees, as of its last pass, for the menu's help line:
// the screen mode's refresh, the camera's measured rate (0: not known yet),
// and whether it holds the screen in step now - if not, why (its log line).
extern std::atomic<float> g_dl_mode_hz, g_dl_camera_fps;
extern std::atomic<bool> g_dl_holding;
extern std::atomic<const char*> g_dl_why;

static inline const char *frame_mode_label(int m) {
    switch (m) {
        case kFrameWhole: return "FULL FRAME";
        case kFrameWholeDecode: return "FULL DECODE";
        default: return "SPLIT";
    }
}
