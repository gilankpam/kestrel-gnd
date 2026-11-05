#ifndef DRM_DEVICE_H  // Check if DRM_DEVICE_H is not defined
#define DRM_DEVICE_H  // Define DRM_DEVICE_H

#include <atomic>
#include <string>
#include <vector>
#include <deque>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>

#include "screen_tap.hpp"
#include "utils/slice_times.hpp"

extern "C" {
    #include "drm/drm.h"
    #include <gbm.h>
    #include <EGL/egl.h>
    #include <EGL/eglext.h>
    #include <GLES2/gl2.h>
    #include <GLES2/gl2ext.h>

extern bool g_disable_gl;
}

struct FlipStats {
    uint64_t recv_ts;
    uint64_t dec_start_ts;
    uint64_t dec_end_ts;
    uint64_t submission_start_ts;
    uint32_t tx_age_us;
    uint32_t fb_id;
    SliceTimesPtr slices;      // the picture's slices, each timed on its own
    // Shown in halves: rows from split_row down are bottom's (0: not split).
    uint32_t split_row = 0;
    SliceTimesPtr bottom_slices;
};

// A picture with per-slice timing that a flip landing at vblank_us showed
// some rows of (SliceTimes::shown says which slices).
struct SliceFlip {
    uint64_t vblank_us;
    SliceTimesPtr slices;
    unsigned skipped = 0;      // (the renderer's) pictures before it never shown
};

struct CompletedStats {
    uint64_t total_latency_us;
    uint64_t display_latency_us;
    uint64_t submission_latency_us;
    uint64_t completion_ts;
    bool available;
};

// Render cadence: measured from hardware flip-completion timestamps, so it captures
// what the eye sees regardless of whether a hiccup came from the link, decode, or
// display. stutters is cumulative; p99/max are over the rolling gap window (~2s @120fps).
struct RenderCadence {
    float p99_ms;
    float max_ms;
    uint32_t stutters;
};

class DrmDevice {
private:
    struct modeset_output *output_list_original = nullptr;

    int video_zpos = 0;
    pthread_mutex_t video_mutex;
    pthread_cond_t video_cond;

    int osd_zpos = 0;
    pthread_mutex_t osd_mutex;
    // true = stock's blocking atomic commit (no completion event); false = the
    // original non-blocking commit + PAGE_FLIP_EVENT. See page_flip().
    bool blocking_flip = false;
    uint32_t pending_osd_fb_id = 0;   // rendered, waiting for a commit to carry it
    uint32_t current_osd_fb_id = 0;   // in the last commit made
    // The OSD fb the last in-flight video flip carried, and the one the display
    // is showing. A buffer in either (or pending) must not be drawn into again.
    uint32_t inflight_osd_fb_id = 0;
    uint32_t onscreen_osd_fb_id = 0;
    // Which OSD frame each of those holds: a new one per set_osd_fb, since the
    // fbs themselves are reused. For the screen recorder.
    uint64_t osd_gen_ = 0, pending_osd_gen_ = 0, current_osd_gen_ = 0;
    // The OSD fbs' DMA-BUFs and pitches, for the screen recorder (osd.cpp
    // registers each one as it makes it). Under osd_mutex.
    std::map<uint32_t, std::pair<int, uint32_t>> osd_fb_dmabuf_;
    // The picture the next page_flip shows (renderer thread only), and the
    // last one a flip showed - where a recording started with no video
    // flowing begins. The latter, and the plane's rectangle, under shown_m_.
    std::shared_ptr<DecodedUnit> flip_picture_;
    std::mutex shown_m_;
    std::shared_ptr<DecodedUnit> last_shown_;
    std::atomic<uint64_t> last_video_flip_us{0};   // last video flip submitted
    uint64_t last_pageflip_ts = 0;

    // GBM/EGL state
    struct gbm_device *gbm_dev = nullptr;
    EGLDisplay egl_display = EGL_NO_DISPLAY;
    EGLContext egl_context = EGL_NO_CONTEXT;
    
    pthread_t event_thread;
    std::atomic<bool> event_thread_exit{false};
    std::atomic<bool> flip_pending{false};  // Atomic to prevent races with event thread
    uint64_t flip_submit_us = 0;            // watchdog: when the pending flip was submitted

    pthread_mutex_t stats_mutex;
    pthread_mutex_t flip_mutex;
    pthread_cond_t flip_cond;
    FlipStats pending_stats;
    CompletedStats latest_stats = {0};
    std::deque<SliceFlip> slice_flips_;   // under stats_mutex
    std::atomic<bool> split_on_{false};   // the second video plane is showing something
    void note_slice_flip(uint64_t vblank_us);   // stats_mutex held
    uint64_t last_flip_submit_ts = 0;
    bool enable_vrr = true;

    // Display hotplug: the event thread probes the connector every ~2s and, when a
    // display (re)appears, redoes the full modeset. Dedicated atomic request so the
    // hotplug commit never races the OSD thread's reuse of osd_request.
    bool display_connected = false;
    uint64_t last_hotplug_check_us = 0;
    drmModeAtomicReq *hotplug_request = nullptr;
    void check_display_hotplug();

    // Auto following the camera (follow_camera): on the event thread too, so
    // the mode change is never in the middle of a hotplug modeset.
    uint32_t follow_done_fps_ = 0;     // the camera rate the mode was last chosen for
    uint32_t follow_done_lines_ = 0;   // and its lines
    std::atomic<bool> refollow_{false};
    void check_follow_camera();
    bool set_crtc_mode(const drmModeModeInfo& m);
    // ... and its pixel clock trimmed to the camera's measured rate.
    uint32_t trim_refused_clock_ = 0;  // a clock the driver refused, kHz
    uint32_t trim_said_clock_ = 0;     // one said to be too far off, kHz
    void check_clock_trim();

    // Render-cadence tracking (guarded by stats_mutex, written in the flip handler)
    static const int CADENCE_RING = 256;
    uint32_t cadence_gaps_us[CADENCE_RING] = {0};
    int      cadence_idx = 0;
    int      cadence_count = 0;
    uint64_t cadence_last_flip_us = 0;
    double   cadence_gap_ewma_us = 0.0;   // expected frame period (EWMA of normal gaps)
    uint32_t cadence_stutters = 0;        // cumulative gaps > max(2.5x expected, 20ms)
public:
    struct FlipSample { uint64_t t_us; uint32_t gap_us; };
private:
    // Every flip's completion time and the gap since the one before, the last
    // ~4 minutes of them (stats screen). Under stats_mutex.
    static const int FLIP_LOG = 32768;
    std::vector<FlipSample> flip_log_ = std::vector<FlipSample>(FLIP_LOG);
    int flip_log_head_ = 0, flip_log_n_ = 0;
    void push_flip_log(uint64_t t_us, uint32_t gap_us) {
        flip_log_[(flip_log_head_ + flip_log_n_) % FLIP_LOG] = { t_us, gap_us };
        if (flip_log_n_ < FLIP_LOG) flip_log_n_++; else flip_log_head_ = (flip_log_head_ + 1) % FLIP_LOG;
    }

public:
    EGLConfig egl_config;

public:
    void init(uint16_t mode_width, uint16_t mode_height, uint32_t mode_vrefresh, int video_zpos, int osd_zpos, bool enable_vrr = true);
    void set_frame_size(uint32_t width, uint32_t height);
    void set_picture_scale(int pct);   // whole picture, both planes, 60..100 (100 = full screen)
    // The scaled picture rectangle inside a base_w x base_h area, centred and
    // even-aligned. ONE implementation for both consumers: the video plane's
    // destination in page_flip, and the OSD's scissor - so the overlay's clip
    // edge lands on exactly the pixels the hardware-scaled video ends on.
    void picture_rect(int base_w, int base_h, int &x, int &y, int &w, int &h) const;

    // The gallery shrinks the live picture into a tile on its strip: where the
    // video plane goes instead of the picture rectangle, in screen pixels from
    // the top left, and it may run off the screen (the strip scrolls it past
    // the edge) - the plane is cut at the edge and the picture cut with it, so
    // what shows is not squeezed. The OSD stages one per frame, and it takes
    // effect with that frame's OSD buffer (set_osd_fb), so the hole the OSD
    // leaves for the picture and the picture's rectangle change together.
    struct Tile { bool active = false; int x = 0, y = 0, w = 0, h = 0; };
    void stage_tile(const Tile& t);
    // The part of a tile that is on a sw x sh screen, even-aligned as the plane
    // needs it; false if none of it is. The OSD leaves exactly this as the hole
    // the picture shows through, so it has to be the plane's own arithmetic.
    static bool tile_on_screen(const Tile& t, int sw, int sh, int &x0, int &y0, int &x1, int &y1);
    static Tile even_tile(const Tile& t);
    // How many OSD frames a video flip picked up, and how many were replaced
    // before one did (profiling: a replaced frame is a step of an animation the
    // screen never shows).
    std::atomic<uint64_t> osd_consumed_{0}, osd_superseded_{0};
    // KESTREL_PROF: how long OSD frames waited between being rendered (set_osd_fb) and being
    // committed - by a video flip that carried them, or by a commit of their own.
    std::atomic<uint32_t> osd_age_hist_[6] = {};     // <5 <10 <20 <35 <60 more (ms)
    uint64_t pending_osd_set_us_ = 0;
    // Called on the event thread each time a flip has landed, once its bookkeeping is done.
    // The OSD uses it to start an animation frame right after a flip (see OSD::run).
    void (*flip_hook)(void*) = nullptr;
    void* flip_hook_ctx = nullptr;
    void cleanup();
    void cond_signal();
    // video
    void perform_modeset_video(int fb_id);
    // split_row > 0 with bottom_fb: the picture in halves - fb_id's rows above
    // split_row (picture rows) on the video plane, bottom_fb's from there down
    // on the second one (has_split_plane). Otherwise fb_id whole.
    bool page_flip(int fb_id, uint64_t recv_ts = 0, uint64_t dec_start_ts = 0, uint64_t dec_end_ts = 0, uint64_t sub_start_ts = 0, uint32_t tx_age_us = 0, SliceTimesPtr slices = nullptr,
                   uint32_t split_row = 0, int bottom_fb = 0, SliceTimesPtr bottom_slices = nullptr);
    bool has_split_plane() const { return output_list && output_list->video2_plane.id != 0; }
    // A picture can go up in halves: the second plane is there (full-screen
    // or in a gallery tile, each half scaled the same way).
    bool split_possible() { return has_split_plane(); }
    // The screen row a picture row of a pic_h-row picture is shown on now,
    // with the gallery tile or Picture Size the video plane has.
    int screen_row_of(uint32_t row, uint32_t pic_h);
    CompletedStats get_latest_stats();
    // The flips that landed since the last call, of pictures with per-slice
    // timing (page_flip's slices), oldest first.
    void take_slice_flips(std::vector<SliceFlip>& out);
    RenderCadence get_render_cadence();

    // What the screen showed, for the screen recorder (screen_tap.hpp).
    ScreenTap tap;
    // The decoded picture the next page_flip carries - the renderer says so
    // just before it.
    void set_flip_picture(std::shared_ptr<DecodedUnit> du) { flip_picture_ = std::move(du); }
    // FrontBuffer mode: the picture set_flip_picture named was copied into
    // the scan-out buffer just now, and shows from the next refresh on.
    void direct_shown();
    // Start the tap (screen recording), seeded with what is on screen now.
    void start_screen_tap();
    // Let go of every decoded picture held here and in the tap. The decoder,
    // before it replaces its buffers (see ScreenTap::drop_pictures).
    void drop_screen_pictures();
    // The picture the display is showing (null before the first), for drawing a second copy of it.
    std::shared_ptr<DecodedUnit> shown_picture();
    // The flips since `since_us`, oldest first.
    void copy_flip_log(uint64_t since_us, std::vector<FlipSample>& out);
    // Appends the flips newer than after_us to out, oldest first, looking only at those
    // (the log is in time order, so it walks back from the newest).
    void append_flip_log_after(uint64_t after_us, std::vector<FlipSample>& out);
    // Whether a flip is a picture reaching the screen: true with vsync (Atomic
    // mode). Without it the flips are probes, and their timing says nothing
    // about smoothness. Set once by main().
    bool flips_are_frames = false;
    // Block until the next vblank: its sequence number and time (CLOCK_MONOTONIC us).
    bool wait_vblank(uint32_t* seq, uint64_t* ts_us);
    // The refresh rate of the current mode, in Hz (rounded).
    int refresh_hz() const;
    // An OSD fb's DMA-BUF (owned here) and pitch in bytes.
    void register_osd_fb(uint32_t fb_id, int dmabuf_fd, uint32_t stride);
    int  osd_fb_dmabuf(uint32_t fb_id, uint32_t* stride);

    // osd
    bool wait_for_flip_completion(int timeout_ms = 10);
    void set_osd_fb(uint32_t fb_id);
    // Whether an OSD fb may still be read by the display: pending, in flight
    // or on screen. The OSD releases its buffers by this, not by count.
    bool osd_fb_in_use(uint32_t fb_id);
    void set_blocking_flip(bool on) { blocking_flip = on; }

    // Hardware picture controls on the HDMI connector (brightness, contrast,
    // saturation, hue - all 0..100, default 50). These act on the whole display
    // pipeline, so they lift the video and the OSD together.
    bool set_display_property(const char* name, uint64_t value);

    // FrontBuffer mode never waits for a vblank - the frame is memcpy'd straight
    // into the scanout buffer and is live as soon as the copy lands. Record the
    // real dec->visible latency here instead of timing a page flip the frame
    // never went through.
    void record_direct_frame(uint64_t recv_ts, uint64_t dec_start_ts,
                             uint64_t dec_end_ts, uint64_t sub_start_ts,
                             uint32_t tx_age_us);
    static void* drm_event_thread_func(void* arg);
    void handle_events();

    // OpenGL OSD helpers
    void init_gl();
    void* get_egl_display() { return egl_display; }
    void* get_egl_context() { return egl_context; }
    struct gbm_device* get_gbm_device() { return gbm_dev; }
    int get_osd_zpos() { return osd_zpos; }
    bool is_flip_pending() { return flip_pending; }
    // VRR on: allowed by the settings and the sink is vrr_capable (FreeSync).
    // Each frame then goes out when committed, not at a fixed vblank.
    bool vrr_active() const { return enable_vrr && output_list && output_list->vrr_capable; }
    // Kernel timestamp (CLOCK_MONOTONIC us) of the last vblank on our CRTC.
    bool last_vblank(uint64_t *ts_us, uint32_t *seq = nullptr);
    // The current mode, copied under mode_copy_m_: set_crtc_mode assigns it
    // on the DRM event thread while the display lock and the renderer read it.
    // A lock of its own rather than crtc_mode_m, which is held across a
    // blocking modeset: a reader on a vblank-paced thread waits for a 68-byte
    // copy, never for the modeset.
    drmModeModeInfo current_mode() const {
        std::lock_guard<std::mutex> lk(mode_copy_m_);
        return output_list ? output_list->mode : drmModeModeInfo{};
    }
    // How long after a vblank the scan-out reaches a screen row, in
    // microseconds. The vblank's time is taken as the start of the active
    // area, which is when it is earliest.
    static double row_time_of(const drmModeModeInfo& m, int row) {
        return m.clock ? (double)m.htotal * row * 1000.0 / m.clock : 0;
    }
    double row_time_us(int row) const { return row_time_of(current_mode(), row); }
    int screen_rows() const { return current_mode().vdisplay; }
    // One refresh, in microseconds: the frame being scanned out, which the
    // display lock may make longer than the mode's (set_frame_period).
    double frame_period_us() const {
        const double now = frame_period_now_us_.load(std::memory_order_relaxed);
        return now > 0 ? now : mode_period_us();
    }
    // One refresh of a mode, in microseconds.
    static double period_of(const drmModeModeInfo& m) {
        return m.clock ? (double)m.htotal * m.vtotal * 1000.0 / m.clock : 0;
    }
    // ... and of the current one.
    double mode_period_us() const { return period_of(current_mode()); }
    std::atomic<double> frame_period_now_us_{0};

public:
    int drm_fd = -1;
    struct modeset_output *output_list = nullptr;
    // The ID (utils/screen_id.h) of the screen this was set up for, "" if it
    // had none. A different one plugged in later restarts kestrel-gnd.
    std::string display_id;
    // Screen Mode Auto: the refresh follows the camera's frame rate - set
    // before init, with the rate last seen ("camera_fps") for the first mode.
    // A different rate in the stream later (g_camera_fps) changes the mode at
    // the same size, the highest whole multiple of it the screen offers. No
    // phase lock: the two clocks still drift apart by their tolerances.
    bool follow_camera = false;
    uint32_t follow_fps = 0;
    // ... and its lines ("camera_lines").
    uint32_t follow_lines = 0;
    // The display lock sets each frame's length (Refresh Sync): how long the
    // one being scanned out is, us, 0 for the mode's own (frame_period_us).
    void set_frame_period(double us) { frame_period_now_us_.store(us, std::memory_order_relaxed); }
    // Refresh Sync changed (FREE wants the fastest mode): Auto chooses again.
    void refollow() { refollow_.store(true); }
    // Held while the CRTC's mode changes (Auto following the camera, a hotplug
    // modeset), and by the display lock (display_lock.hpp) around its register
    // writes: never one over the other's.
    std::mutex crtc_mode_m;
    // Guards output_list->mode for current_mode(). Taken inside crtc_mode_m by
    // set_crtc_mode, and never held across anything else: a leaf.
    mutable std::mutex mode_copy_m_;
    // Video variables
	uint32_t video_frm_width;
	uint32_t video_frm_height;
	int video_fb_x, video_fb_y, video_fb_width, video_fb_height;
	int video_pfd;
    uint64_t video_ts;
    int picture_scale_pct = 100;  // whole-picture scale; applied to both planes at commit

private:
    // What the video plane is showing and where, as of the last page_flip.
    // When frames stop, the last one stays on the plane in that rectangle,
    // and only a new frame would move it - so a Picture Size change while
    // the link is down is applied from the OSD side (see set_osd_fb).
    int last_video_fb_id_ = 0;
    int vrect_x_ = 0, vrect_y_ = 0, vrect_w_ = 0, vrect_h_ = 0;
    int vsrc_x_ = 0, vsrc_y_ = 0, vsrc_w_ = 0, vsrc_h_ = 0;   // and the part of the picture in it
    void sync_frozen_video_rect();
    // Where the video plane goes, and which part of the picture goes there.
    // Destination in CRTC pixels; source in picture pixels.
    void video_geometry(const Tile& tile, int &dx, int &dy, int &dw, int &dh,
                        int &sx, int &sy, int &sw, int &sh) const;
    // Under osd_mutex: staged by the OSD, the one that goes with the pending
    // OSD buffer, and the one that goes with the buffer on screen.
    Tile staged_tile_, pending_tile_, current_tile_;
};

#endif // DRM_DEVICE_H  // End of the header guard