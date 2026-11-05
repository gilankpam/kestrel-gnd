#include <ostream>
#include <cmath>
#include "renderer.hpp"   // DecodedUnit, for the screen tap
#include <iostream>
#include <algorithm>
#include <cstring>
#include <poll.h>
#include <chrono>
#include <unistd.h>

#include "drm.hpp"
#include "utils/ltrace.hpp"
#include "common.hpp"             // kestrel_request_restart
#include "utils/screen_id.h"
#include "utils/time_util.h"
#include "utils/prof.hpp"
#include "settings.hpp"
#include "utils/scheduling_helper.hpp"
#include "frame_mode.hpp"

std::atomic<int> g_camera_fps{0};
std::atomic<int> g_camera_lines{0};

// Set from main() when --no-osd is given: skips all EGL/GLES bring-up.
bool g_disable_gl = false;


void DrmDevice::init(uint16_t mode_width, uint16_t mode_height, uint32_t mode_vrefresh, int vzpos, int ozpos, bool vrr) {
    enable_vrr = vrr;
    int ret;
    char card[20]; 
    for (int i = 0; i < 10; i++) {
        snprintf(card, sizeof(card), "/dev/dri/card%d", i);
        ret = modeset_open(&drm_fd, card);
        if(ret==0){
            printf("Using drm device %s\n", card);
            break;
        }
    }
    if (ret < 0) {
        printf("modeset_open(/dev/dri/card[0-9]) =  %d\n", ret);
    }
	assert(drm_fd >= 0);
	output_list = (struct modeset_output *)malloc(sizeof(struct modeset_output));
    output_list_original = output_list;
	ret = modeset_prepare(drm_fd, output_list, mode_width, mode_height, mode_vrefresh,
                          follow_camera && g_display_lock.load() != 2 ? follow_fps : 0);
    // No connected display is not fatal: booting headless (or with a flaky HDMI
    // cable) used to fall through the compiled-out assert below onto an
    // UNINITIALIZED output struct. Wait here and pick the display up when it
    // appears — mode selection, framebuffers and GL are all sized from it.
    if (ret != 0) {
        printf("[drm] no connected display — waiting for one to be plugged in...\n");
        fflush(stdout);
        while (ret != 0) {
            sleep(2);
            ret = modeset_prepare(drm_fd, output_list, mode_width, mode_height, mode_vrefresh,
                          follow_camera && g_display_lock.load() != 2 ? follow_fps : 0);
        }
    }
	assert(!ret);
    printf("modeset_prepare successful. Using connector %u, crtc %u\n", output_list->connector.id, output_list->crtc.id);
    fflush(stdout);
    output_list->allow_vrr = enable_vrr;
    follow_done_fps_ = follow_camera ? follow_fps : 0;
    follow_done_lines_ = follow_camera ? follow_lines : 0;

    video_zpos = vzpos;
    video_fb_x = 0;
	video_fb_y = 0;
	video_fb_width = output_list->mode.hdisplay;
	video_fb_height = output_list->mode.vdisplay;	

    osd_zpos = ozpos;

    ret = pthread_mutex_init(&video_mutex, NULL);
	assert(!ret);
	ret = pthread_cond_init(&video_cond, NULL);
	assert(!ret);
	ret = pthread_mutex_init(&osd_mutex, NULL);
	assert(!ret);
    ret = pthread_mutex_init(&stats_mutex, NULL);
	assert(!ret);
    ret = pthread_mutex_init(&flip_mutex, NULL);
	assert(!ret);
    ret = pthread_cond_init(&flip_cond, NULL);
	assert(!ret);

    if (!g_disable_gl) {
        init_gl();
    } else {
        printf("GL/EGL disabled (--no-osd): GPU stays idle\n");
    }

    display_connected = true;                    // init requires a connected display
    hotplug_request = drmModeAtomicAlloc();
    assert(hotplug_request);

    // Start DRM event handler thread to prevent ENOMEM (event queue full)
    event_thread_exit = false;
    ret = pthread_create(&event_thread, NULL, drm_event_thread_func, this);
    assert(!ret);
}

// Look at the connector and, on a disconnected→connected transition, redo the full
// modeset (the CRTC is left off after an unplug; page_flip commits carry no
// ALLOW_MODESET, so without this the screen stays black until an app restart).
// That reuses the mode, framebuffers and GL surface as they are, so it is only
// done for the same screen coming back still offering the mode in use. Any
// other screen - or one that no longer offers the mode - restarts kestrel-gnd,
// which sets the display up from scratch in that screen's own mode: sent the
// old mode, a screen that does not take it just stays black.
// The connector's state as the kernel last knew it (GetConnectorCurrent): its
// HDMI hot-plug line keeps that up to date and re-reads the EDID when a screen
// comes. GetConnector forces a probe - an EDID read over HDMI every 2 s.
void DrmDevice::check_display_hotplug() {
    drmModeConnector *conn = drmModeGetConnectorCurrent(drm_fd, output_list->connector.id);
    if (!conn) return;
    bool now_connected = (conn->connection == DRM_MODE_CONNECTED);
    drmModeFreeConnector(conn);

    // A screen that came, or one swapped for another within a poll (no
    // disconnect ever seen): the EDID is read every poll while connected, so a
    // swap shows as a new ID.
    char id[16] = "";
    if (now_connected) screen_id(id, sizeof(id));
    const bool appeared = now_connected && !display_connected;
    const bool swapped = now_connected && display_connected && id[0] && display_id != id;

    if (appeared || swapped) {
        // Refresh Sync off at once: its one-line timing changes are only for
        // a screen they were tried and kept on, and some screens blank on
        // them. Back below only for the very screen it was kept for.
        const int sync_was = g_display_lock.exchange(0);
        // A forced probe (GetConnector, an EDID read): just after the hot-plug
        // line rises the connector's current state can still hold the last
        // screen's EDID and modes, which made a new screen look like the old
        // one - set up in the old screen's mode, with its Refresh Sync.
        // A screen's DDC can still be refusing reads just after its hot-plug
        // line rises ("ddc read failed"): a probe that got no EDID is tried
        // again, up to ~1 s - only at a hot-plug, while the screen is dark.
        bool mode_offered = false;
        for (int attempt = 0; attempt < 4; attempt++) {
            if (attempt) usleep(300000);
            mode_offered = false;
            if (drmModeConnector *probed = drmModeGetConnector(drm_fd, output_list->connector.id)) {
                for (int i = 0; i < probed->count_modes; i++) {
                    const drmModeModeInfo &m = probed->modes[i];
                    if (m.hdisplay == output_list->mode.hdisplay && m.vdisplay == output_list->mode.vdisplay &&
                        m.vrefresh == output_list->mode.vrefresh)
                        mode_offered = true;
                }
                drmModeFreeConnector(probed);
            }
            screen_id(id, sizeof(id));   // after the probe, sysfs has this screen's EDID
            if (id[0]) break;
        }
        // A different screen, one that does not offer the mode in use, or one
        // with no EDID ID (nothing tells it from another): restarted, which
        // sets the display up in that screen's own mode - Screen Mode Auto
        // picks again - with that screen's own Refresh Sync, off unless kept
        // for it.
        if (display_id != id || !mode_offered || !id[0]) {
            printf("[drm] display hot-plugged: %s screen %s%s - restarting to set it up (refresh sync off)\n",
                   !id[0] ? "a" : display_id != id ? "a different" : "the same", id[0] ? id : "(no EDID ID)",
                   mode_offered ? "" : ", which does not offer the mode in use");
            fflush(stdout);
            display_connected = now_connected;
            kestrel_request_restart();
            return;
        }
        g_display_lock = sync_was;   // the same screen, its own kept value
        printf("[drm] display hot-plugged — performing full modeset\n");
        fflush(stdout);
        output_list->initialized = false;        // next commit re-adds connector/crtc props

        pthread_mutex_lock(&osd_mutex);
        uint32_t osd_fb = current_osd_fb_id;
        pthread_mutex_unlock(&osd_mutex);

        plane_info osd_plane_info = {
            .plane = &output_list->osd_plane,
            .fb_id = (int)osd_fb,
            .width = output_list->mode.hdisplay,
            .height = output_list->mode.vdisplay,
            .zpos = osd_zpos,
        };
        if (osd_fb > 0) {
            set_drm_object_property(hotplug_request, &output_list->osd_plane, "alpha", 65535);
            set_drm_object_property(hotplug_request, &output_list->osd_plane, "pixel blend mode", 1);
        }
        // video_info=NULL → the full modeset lights the CRTC with the placeholder
        // video FB; the next real video/OSD flip then takes over as usual.
        std::lock_guard<std::mutex> lk(crtc_mode_m);
        int ret = modeset_perform_modeset(drm_fd, output_list, hotplug_request, NULL,
                                          (osd_fb > 0) ? &osd_plane_info : NULL);
        if (ret < 0)
            fprintf(stderr, "[drm] hotplug modeset failed: %d\n", ret);
    } else if (!now_connected && display_connected) {
        printf("[drm] display unplugged\n");
        fflush(stdout);
        // A probe now, while nothing is there: only that drops the screen's
        // EDID and modes. The kernel's HDMI driver (Rockchip dw-hdmi) reuses
        // the EDID it holds rather than reading the next screen's, so the
        // next screen - an Alienware after the headset - looked like the
        // last one and was sent its mode and its Refresh Sync: black.
        if (drmModeConnector *probed = drmModeGetConnector(drm_fd, output_list->connector.id))
            drmModeFreeConnector(probed);
    }
    display_connected = now_connected;
}

void* DrmDevice::drm_event_thread_func(void* arg) {
    DrmDevice* self = static_cast<DrmDevice*>(arg);
    SchedulingHelper::configure_thread(SchedulingHelper::ThreadRole::DrmEvent); 
    self->handle_events();
    return nullptr;
}

void DrmDevice::handle_events() {
    drmEventContext evctx = {};
    evctx.version = DRM_EVENT_CONTEXT_VERSION;
    evctx.page_flip_handler = [](int fd, unsigned int frame, unsigned int sec, unsigned int usec, void* data) {
        if (data) {
            DrmDevice* self = reinterpret_cast<DrmDevice*>(data);
            
            static uint64_t last_flip_complete = 0;
            static int complete_count = 0;
            // Use the kernel-provided timestamp for the actual hardware flip event
            uint64_t completion_ts = (uint64_t)sec * 1000000ULL + usec;
            if (ltrace::on()) ltrace::rec(ltrace::kFlipDone, ltrace::now_us(), self->pending_stats.fb_id, completion_ts);
            
            pthread_mutex_lock(&self->osd_mutex);
            if (self->inflight_osd_fb_id) {
                self->onscreen_osd_fb_id = self->inflight_osd_fb_id;
                self->inflight_osd_fb_id = 0;
            }
            pthread_mutex_unlock(&self->osd_mutex);
            // `frame` is the vblank the flip landed on - the counter
            // drmWaitVBlank returns, which the screen recorder samples by.
            self->tap.landed(frame);
            last_flip_complete = completion_ts;
            
            pthread_mutex_lock(&self->stats_mutex);
            if (self->pending_stats.recv_ts > 0) {
                // High-precision hardware-to-hardware latency calculation
                self->latest_stats.total_latency_us = (completion_ts - self->pending_stats.recv_ts) + self->pending_stats.tx_age_us;
                self->latest_stats.display_latency_us = completion_ts - self->pending_stats.dec_end_ts;
                self->latest_stats.submission_latency_us = completion_ts - self->pending_stats.submission_start_ts;
                self->latest_stats.completion_ts = completion_ts;
                self->latest_stats.available = true;
                self->pending_stats.recv_ts = 0;
            }
            self->note_slice_flip(completion_ts);

            // Render cadence: gap between hardware flip completions. The EWMA of
            // normal gaps is the expected frame period (self-adapts to 60/90/120fps);
            // a gap well past it is a stutter the viewer can perceive.
            if (self->cadence_last_flip_us > 0 && completion_ts > self->cadence_last_flip_us) {
                uint32_t gap = (uint32_t)(completion_ts - self->cadence_last_flip_us);
                self->push_flip_log(completion_ts, gap);
                self->cadence_gaps_us[self->cadence_idx] = gap;
                self->cadence_idx = (self->cadence_idx + 1) % CADENCE_RING;
                if (self->cadence_count < CADENCE_RING) self->cadence_count++;

                // Floor 45ms: on a 60Hz panel flips quantize to vsync and the
                // EWMA trains between the 8/16ms mix, so 25-33ms gaps (1-2
                // missed vsyncs) fired constantly at steady state — verified
                // invisible. 45ms = a freeze the eye actually catches.
                double expect = self->cadence_gap_ewma_us;
                double thresh = (expect > 0.0) ? expect * 3.5 : 0.0;
                if (thresh < 45000.0) thresh = 45000.0;
                if (expect > 0.0 && (double)gap > thresh) {
                    self->cadence_stutters++;
                } else {
                    // Only normal gaps train the expectation, so a stutter burst
                    // doesn't inflate the baseline and mask the next one.
                    self->cadence_gap_ewma_us = (expect <= 0.0)
                        ? (double)gap : (0.95 * expect + 0.05 * (double)gap);
                }
            }
            self->cadence_last_flip_us = completion_ts;
            pthread_mutex_unlock(&self->stats_mutex);

            // Free for the next flip only now, with this one's figures taken: a
            // commit in between would put its picture in pending_stats first.
            self->flip_pending = false;
            pthread_mutex_lock(&self->flip_mutex);
            pthread_cond_broadcast(&self->flip_cond);
            pthread_mutex_unlock(&self->flip_mutex);
            if (self->flip_hook) self->flip_hook(self->flip_hook_ctx);
        }
    };


    while (!event_thread_exit) {
        struct pollfd pfd = { .fd = drm_fd, .events = POLLIN };
        // Flip events wake the poll the moment they arrive; the timeout only
        // paces the exit check and the 2 s hotplug probe below. It was 1 ms -
        // a thousand wakeups a second on the OSD's core for nothing.
        int ret = poll(&pfd, 1, 500);
        if (event_thread_exit) break;   // before touching output_list: cleanup may be freeing it
        if (ret > 0 && (pfd.revents & POLLIN)) {
            // Read as many events as available
            int err = drmHandleEvent(drm_fd, &evctx);
            if (err < 0) {
                // perror("drmHandleEvent");
            }
        }

        // Low-frequency display hotplug probe (full connector probe costs ~ms,
        // so keep it well away from the 1ms flip-event cadence).
        uint64_t now = get_time_us();
        if (now - last_hotplug_check_us >= 2000000) {
            last_hotplug_check_us = now;
            check_display_hotplug();
            check_clock_trim();
        }
        check_follow_camera();
    }
}

// Screen Mode Auto follows the camera (follow_camera): a frame rate new in
// the stream puts the screen on the highest refresh that is a whole multiple
// of it, at the size it has now (the framebuffers and the GL surface are
// that size) - or, with none, on the highest refresh at that size, Auto's
// choice without a camera. One modeset, so one blank, as for any mode change.
// The rate is kept as "camera_fps", and the next start comes up in its mode.
void DrmDevice::check_follow_camera() {
    if (refollow_.exchange(false)) follow_done_fps_ = follow_done_lines_ = 0;
    const int fps = g_camera_fps.load(std::memory_order_acquire);
    const int lines = g_camera_lines.load(std::memory_order_relaxed);
    if (!follow_camera || !display_connected || fps <= 0 ||
        ((uint32_t)fps == follow_done_fps_ && (uint32_t)lines == follow_done_lines_)) return;
    follow_done_fps_ = (uint32_t)fps;
    follow_done_lines_ = (uint32_t)lines;
    if (Settings::getInstance().getInt("camera_fps", 0) != fps)
        Settings::getInstance().set("camera_fps", fps);
    if (Settings::getInstance().getInt("camera_lines", 0) != lines)
        Settings::getInstance().set("camera_lines", lines);

    drmModeConnector *conn = drmModeGetConnectorCurrent(drm_fd, output_list->connector.id);
    if (!conn) return;
    const drmModeModeInfo cur = output_list->mode;
    // A screen that takes any frame length (Refresh Sync FREE) keeps its
    // fastest mode: the lock makes its frames as long as the camera's, and
    // its quick scan-out lights the lower rows sooner.
    int pick = g_display_lock.load() == 2 ? -1
                                          : modeset_pick_follow(conn, (uint32_t)fps, cur.hdisplay, cur.vdisplay);
    const bool in_step = pick >= 0;
    for (int i = 0; !in_step && i < conn->count_modes; i++) {
        const drmModeModeInfo &m = conn->modes[i];
        if ((m.flags & DRM_MODE_FLAG_INTERLACE) || m.hdisplay != cur.hdisplay || m.vdisplay != cur.vdisplay)
            continue;
        if (pick < 0 || modeset_mode_hz(&m) > modeset_mode_hz(&conn->modes[pick])) pick = i;
    }
    drmModeModeInfo m = {};
    if (pick >= 0) m = conn->modes[pick];
    drmModeFreeConnector(conn);
    if (pick < 0) return;

    const bool same = m.clock == cur.clock && m.htotal == cur.htotal && m.vtotal == cur.vtotal &&
                      m.hsync_start == cur.hsync_start && m.vsync_start == cur.vsync_start;
    printf("display: camera at %d fps - %s %dx%d@%d (%.3f Hz)%s\n", fps, same ? "staying at" : "changing to",
           m.hdisplay, m.vdisplay, m.vrefresh, modeset_mode_hz(&m),
           in_step ? "" : ", the screen has no refresh in step with it at this size");
    fflush(stdout);
    if (!same) set_crtc_mode(m);
}

// Screen Mode Auto in step with the camera's own rate, not the one its mode
// is named by (a "100 fps" Lite+ runs 100.13): the mode modeset_pick_follow
// picks, its pixel clock trimmed to the camera's frame period as the display
// lock measured it (camera_period_key). The lines stay the mode's, so the
// screen sees one steady timing - the goggle X shows static while Refresh
// Sync changes them at 1080p100. Within 0.5% of the mode's own clock; a
// clock the driver refused is not asked for again.
void DrmDevice::check_clock_trim() {
    if (!follow_camera || !display_connected) return;
    const int fps = g_camera_fps.load(std::memory_order_acquire);
    const int lines = g_camera_lines.load(std::memory_order_relaxed);
    if (fps <= 0) return;
    const double pc = Settings::getInstance().getFloat(camera_period_key(lines, fps), 0.0f);
    if (pc * fps < 0.99e6 || pc * fps > 1.01e6) return;

    drmModeConnector *conn = drmModeGetConnectorCurrent(drm_fd, output_list->connector.id);
    if (!conn) return;
    const drmModeModeInfo cur = current_mode();
    const int pick = modeset_pick_follow(conn, (uint32_t)fps, cur.hdisplay, cur.vdisplay);
    drmModeModeInfo m = {};
    if (pick >= 0) m = conn->modes[pick];
    drmModeFreeConnector(conn);
    // Only the mode Auto is on: another timing is the user's, or a change on its way.
    if (pick < 0 || m.htotal != cur.htotal || m.vtotal != cur.vtotal || m.hsync_start != cur.hsync_start ||
        m.vsync_start != cur.vsync_start)
        return;

    const double k = std::max(1.0, std::round(modeset_mode_hz(&m) / fps));
    const uint32_t clock = (uint32_t)std::lround((double)m.htotal * m.vtotal * k * 1e3 / pc);   // kHz
    if (clock == cur.clock || clock == trim_refused_clock_) return;
    // Trimmed already: each change is a modeset, a blank, so not for the
    // camera's period remeasured a few ppm off (the lock keeps a new one at 20).
    if (cur.clock != m.clock && std::fabs((double)clock / cur.clock - 1.0) < 50e-6) return;
    if (std::fabs((double)clock / m.clock - 1.0) > 0.005) {
        if (clock != trim_said_clock_)
            printf("display: not trimming %dx%d@%d to the camera's %.4f fps - %u kHz is over 0.5%% from %u kHz\n",
                   m.hdisplay, m.vdisplay, m.vrefresh, 1e6 / pc, clock, m.clock);
        trim_said_clock_ = clock;
        return;
    }
    m.clock = clock;
    printf("display: %dx%d@%d clock %u -> %u kHz, refresh %.4f Hz for the camera's %.4f fps\n",
           m.hdisplay, m.vdisplay, m.vrefresh, cur.clock, clock, modeset_mode_hz(&m), 1e6 / pc);
    fflush(stdout);
    if (!set_crtc_mode(m)) trim_refused_clock_ = clock;
}

// The CRTC's mode, and nothing else: the planes keep their buffers and
// rectangles (atomic state is sticky). Blocking, so after any flip in flight;
// a page flip submitted meanwhile is refused (EBUSY) and the next one goes.
bool DrmDevice::set_crtc_mode(const drmModeModeInfo& m) {
    std::lock_guard<std::mutex> lk(crtc_mode_m);
    uint32_t blob = 0;
    if (drmModeCreatePropertyBlob(drm_fd, &m, sizeof(m), &blob) != 0) {
        perror("display: mode blob");
        return false;
    }
    drmModeAtomicReq *req = drmModeAtomicAlloc();
    int ret = req ? set_drm_object_property(req, &output_list->crtc, "MODE_ID", blob) : -1;
    if (ret >= 0) ret = drmModeAtomicCommit(drm_fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
    if (req) drmModeAtomicFree(req);
    if (ret < 0) {
        fprintf(stderr, "display: mode change refused: %s\n", strerror(-ret));
        drmModeDestroyPropertyBlob(drm_fd, blob);
        return false;
    }
    drmModeDestroyPropertyBlob(drm_fd, output_list->mode_blob_id);
    output_list->mode_blob_id = blob;
    {
        std::lock_guard<std::mutex> mk(mode_copy_m_);
        output_list->mode = m;
    }
    return true;
}

void DrmDevice::init_gl() {
    // Create GBM device
    gbm_dev = gbm_create_device(drm_fd);
    if (!gbm_dev) {
        fprintf(stderr, "Failed to create GBM device\n");
        return;
    }

    // Initialize EGL
    // libmali's legacy eglGetDisplay(gbm) returns NULL on RK3568; use the GBM
    // platform display instead (verified working via egltest).
    {
        PFNEGLGETPLATFORMDISPLAYEXTPROC getPlatDisp =
            (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
        egl_display = getPlatDisp ? getPlatDisp(EGL_PLATFORM_GBM_KHR, gbm_dev, nullptr)
                                  : eglGetDisplay((EGLNativeDisplayType)gbm_dev);
    }
    if (egl_display == EGL_NO_DISPLAY) {
        fprintf(stderr, "Failed to get EGL display\n");
        return;
    }

    EGLint major, minor;
    if (!eglInitialize(egl_display, &major, &minor)) {
        fprintf(stderr, "Failed to initialize EGL\n");
        return;
    }

    printf("EGL Initialized: %d.%d\n", major, minor);

    // Bind GLES API
    eglBindAPI(EGL_OPENGL_ES_API);

    // Find EGL Config. No multisampling unless hud_msaa is set.
    //
    // 4x MSAA was the default, and on this Mali-G52 it was most of the HUD's
    // cost: the only multisampled window config also carries a 24-bit depth
    // and 8-bit stencil buffer, and a 1080p frame took 15 ms of GPU time
    // (measured from its native fence) against 6 ms without - which held the
    // GPU at 800 MHz permanently instead of letting it idle at 200. The HUD
    // is flat 2D; the antialiasing it bought was on the thin slanted rails.
    // Mesa swrast/softpipe has no MSAA window config at all, hence the
    // fallback when it is asked for.
    const EGLint config_attribs_msaa[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_SAMPLE_BUFFERS, 1,
        EGL_SAMPLES, 4,
        EGL_NONE
    };
    const EGLint config_attribs_plain[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_NONE
    };

    EGLint num_configs = 0;
    const bool try_msaa = Settings::getInstance().getBool("hud_msaa", false);
    if (!try_msaa || !eglChooseConfig(egl_display, config_attribs_msaa, &egl_config, 1, &num_configs) || num_configs == 0) {
        if (try_msaa) fprintf(stderr, "No 4x MSAA EGL config available; retrying without MSAA\n");
        if (!eglChooseConfig(egl_display, config_attribs_plain, &egl_config, 1, &num_configs) || num_configs == 0) {
            fprintf(stderr, "Failed to choose EGL config (error: 0x%x)\n", eglGetError());
            return;
        }
    }

    {
        EGLint id = 0, samples = 0, sbuf = 0, depth = 0, stencil = 0;
        eglGetConfigAttrib(egl_display, egl_config, EGL_CONFIG_ID, &id);
        eglGetConfigAttrib(egl_display, egl_config, EGL_SAMPLES, &samples);
        eglGetConfigAttrib(egl_display, egl_config, EGL_SAMPLE_BUFFERS, &sbuf);
        eglGetConfigAttrib(egl_display, egl_config, EGL_DEPTH_SIZE, &depth);
        eglGetConfigAttrib(egl_display, egl_config, EGL_STENCIL_SIZE, &stencil);
        printf("EGL config %d: samples %d (buffers %d), depth %d, stencil %d\n",
               id, samples, sbuf, depth, stencil);
    }

    // Create EGL Context
    const EGLint context_attribs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 2,
        EGL_NONE
    };

    egl_context = eglCreateContext(egl_display, egl_config, EGL_NO_CONTEXT, context_attribs);
    if (egl_context == EGL_NO_CONTEXT) {
        fprintf(stderr, "Failed to create EGL context\n");
        return;
    }

    printf("EGL Context Created\n");
}

void DrmDevice::set_osd_fb(uint32_t fb_id) {
    pthread_mutex_lock(&osd_mutex);
    if (pending_osd_fb_id > 0) osd_superseded_++;
    pending_osd_fb_id = fb_id;
    pending_osd_gen_ = ++osd_gen_;
    pending_tile_ = staged_tile_;
    if (prof::enabled()) pending_osd_set_us_ = get_time_us();
    pthread_mutex_unlock(&osd_mutex);

    // While video is flowing, the next video flip carries this fb (page_flip
    // bundles the pending OSD fb into its own commit), so there is nothing to
    // do. This used to commit on its own whenever no flip happened to be in
    // flight - nearly always - which cost the OSD thread a blocking commit
    // per frame (up to a vblank, ~4 ms at 120 Hz) and competed with the video
    // thread's flips for the CRTC.
    //
    // "Flowing" is a flip submitted within three frame periods (40 ms at
    // least). Past that the video is stalled or not started, and the OSD has
    // to commit itself or it would never update.
    uint64_t now = get_time_us();
    uint64_t last = last_video_flip_us;
    pthread_mutex_lock(&stats_mutex);
    double gap = cadence_gap_ewma_us;
    pthread_mutex_unlock(&stats_mutex);
    uint64_t window = (uint64_t)(gap * 3.0);
    if (window < 40000) window = 40000;
    bool flowing = video_frm_width != 0 && last && (flip_pending || now - last < window);
    if (flowing) {
        prof::count(prof::kCountOsdPending);
        return;
    }

    pthread_mutex_lock(&osd_mutex);
    // Video resuming: a flip may have been submitted since the check above.
    // It carries the previous fb; leave this one pending for the next flip
    // rather than racing it with a commit of our own.
    if (flip_pending && get_time_us() - flip_submit_us < 250000) {
        pthread_mutex_unlock(&osd_mutex);
        prof::count(prof::kCountOsdPending);
        return;
    }
    prof::count(prof::kCountOsdCommit);
    if (prof::enabled()) {
        const uint64_t age = get_time_us() - pending_osd_set_us_;
        osd_age_hist_[age < 5000 ? 0 : age < 10000 ? 1 : age < 20000 ? 2 : age < 35000 ? 3 : age < 60000 ? 4 : 5]++;
    }
    current_osd_fb_id = pending_osd_fb_id;
    current_osd_gen_ = pending_osd_gen_;
    current_tile_ = pending_tile_;
    pending_osd_fb_id = 0;
    uint32_t commit_fb_id = current_osd_fb_id;
    uint64_t commit_gen = current_osd_gen_;
    pthread_mutex_unlock(&osd_mutex);

    sync_frozen_video_rect();

    if (commit_fb_id > 0) {
        plane_info osd_plane_info = {
            .plane = &output_list->osd_plane,
            .fb_id = (int)commit_fb_id,
            .width = output_list->mode.hdisplay,
            .height = output_list->mode.vdisplay,
            .zpos = osd_zpos,
        };
        // Blocking: when it returns, this fb is the one on screen.
        if (modeset_perform_modeset(drm_fd, output_list, output_list->osd_request, NULL, &osd_plane_info) == 0) {
            pthread_mutex_lock(&osd_mutex);
            onscreen_osd_fb_id = commit_fb_id;
            pthread_mutex_unlock(&osd_mutex);
            if (tap.recording()) {
                uint64_t vb;
                uint32_t seq;
                if (last_vblank(&vb, &seq)) tap.osd_landed(commit_fb_id, commit_gen, seq);
            }
        }
    }
}

bool DrmDevice::osd_fb_in_use(uint32_t fb_id) {
    if (!fb_id) return false;
    pthread_mutex_lock(&osd_mutex);
    bool used = fb_id == pending_osd_fb_id || fb_id == current_osd_fb_id ||
                fb_id == inflight_osd_fb_id || fb_id == onscreen_osd_fb_id;
    pthread_mutex_unlock(&osd_mutex);
    // The screen recorder reading it (RGA): drawing into it now would tear
    // the recorded frame.
    return used || tap.holds_osd(fb_id);
}

void DrmDevice::register_osd_fb(uint32_t fb_id, int dmabuf_fd, uint32_t stride) {
    pthread_mutex_lock(&osd_mutex);
    auto it = osd_fb_dmabuf_.find(fb_id);
    if (it != osd_fb_dmabuf_.end() && it->second.first >= 0) close(it->second.first);
    osd_fb_dmabuf_[fb_id] = std::make_pair(dmabuf_fd, stride);
    pthread_mutex_unlock(&osd_mutex);
}

int DrmDevice::osd_fb_dmabuf(uint32_t fb_id, uint32_t* stride) {
    int fd = -1;
    pthread_mutex_lock(&osd_mutex);
    auto it = osd_fb_dmabuf_.find(fb_id);
    if (it != osd_fb_dmabuf_.end()) {
        fd = it->second.first;
        if (stride) *stride = it->second.second;
    }
    pthread_mutex_unlock(&osd_mutex);
    return fd;
}

bool DrmDevice::wait_vblank(uint32_t* seq, uint64_t* ts_us) {
    drmVBlank vbl = {};
    vbl.request.type = (drmVBlankSeqType)(DRM_VBLANK_RELATIVE |
        ((output_list->crtc_index << DRM_VBLANK_HIGH_CRTC_SHIFT) & DRM_VBLANK_HIGH_CRTC_MASK));
    vbl.request.sequence = 1;
    if (drmWaitVBlank(drm_fd, &vbl)) return false;
    *seq = vbl.reply.sequence;
    *ts_us = (uint64_t)vbl.reply.tval_sec * 1000000ULL + vbl.reply.tval_usec;
    return true;
}

int DrmDevice::refresh_hz() const {
    double p = frame_period_us();
    if (p > 0) return (int)(1e6 / p + 0.5);
    const drmModeModeInfo m = current_mode();
    return m.vrefresh ? (int)m.vrefresh : 60;
}

// Picture Size moves the video plane's rectangle, but that rectangle is only
// written by page_flip, with each new frame. With the link down there is no
// new frame: the last one stays on the plane exactly where it was, and only
// the overlay around it shrank - until the aircraft came back and the first
// frame snapped the picture into place. So while video is stalled, the OSD
// thread (which keeps committing on its own) checks whether the rectangle
// the plane was last given still matches the one the setting asks for, and
// if not, moves the frozen frame itself. A rectangle-only commit: atomic
// state is sticky, so the FB and the source stay as page_flip left them.
void DrmDevice::sync_frozen_video_rect() {
    if (last_video_fb_id_ <= 0) return;
    pthread_mutex_lock(&osd_mutex);
    const Tile tile = current_tile_;
    pthread_mutex_unlock(&osd_mutex);
    int vx, vy, vw, vh, sx, sy, sw, sh;
    video_geometry(tile, vx, vy, vw, vh, sx, sy, sw, sh);
    if (vx == vrect_x_ && vy == vrect_y_ && vw == vrect_w_ && vh == vrect_h_ &&
        sx == vsrc_x_ && sy == vsrc_y_ && sw == vsrc_w_ && sh == vsrc_h_) return;

    drmModeAtomicReq *req = drmModeAtomicAlloc();
    if (!req) return;
    set_drm_object_property(req, &output_list->video_plane, "SRC_X", (uint64_t)sx << 16);
    set_drm_object_property(req, &output_list->video_plane, "SRC_Y", (uint64_t)sy << 16);
    set_drm_object_property(req, &output_list->video_plane, "SRC_W", (uint64_t)sw << 16);
    set_drm_object_property(req, &output_list->video_plane, "SRC_H", (uint64_t)sh << 16);
    set_drm_object_property(req, &output_list->video_plane, "CRTC_X", vx);
    set_drm_object_property(req, &output_list->video_plane, "CRTC_Y", vy);
    set_drm_object_property(req, &output_list->video_plane, "CRTC_W", vw);
    set_drm_object_property(req, &output_list->video_plane, "CRTC_H", vh);
    // A picture in halves: the frozen one goes whole on the video plane, the
    // second plane off (its part would stay where it was, a stale strip).
    const bool had_split = split_on_.load();
    if (had_split) {
        set_drm_object_property(req, &output_list->video2_plane, "FB_ID", 0);
        set_drm_object_property(req, &output_list->video2_plane, "CRTC_ID", 0);
    }
    int ret = drmModeAtomicCommit(drm_fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
    drmModeAtomicFree(req);
    if (ret == 0 && had_split) split_on_ = false;
    if (ret == 0) {
        {
            std::lock_guard<std::mutex> lk(shown_m_);
            vrect_x_ = vx; vrect_y_ = vy; vrect_w_ = vw; vrect_h_ = vh;
            vsrc_x_ = sx; vsrc_y_ = sy; vsrc_w_ = sw; vsrc_h_ = sh;
        }
        if (tap.recording()) {
            uint64_t vb;
            uint32_t seq;
            if (last_vblank(&vb, &seq)) tap.moved(vx, vy, vw, vh, seq);
        }
    } else {
        static bool warned = false;
        if (!warned) { warned = true; perror("picture size: moving the frozen video frame"); }
    }
}

void DrmDevice::direct_shown() {
    std::shared_ptr<DecodedUnit> pic = std::move(flip_picture_);
    flip_picture_.reset();
    if (!pic) return;
    int vx, vy, vw, vh;
    picture_rect(video_fb_width, video_fb_height, vx, vy, vw, vh);
    vx += video_fb_x; vy += video_fb_y;
    if (tap.recording()) {
        pthread_mutex_lock(&osd_mutex);
        uint32_t osd_fb = current_osd_fb_id;
        uint64_t osd_gen = current_osd_gen_;
        pthread_mutex_unlock(&osd_mutex);
        uint64_t vb;
        uint32_t seq;
        if (last_vblank(&vb, &seq)) {
            // Copied in mid-scan: the next refresh is the first that shows
            // all of it.
            tap.submitted(pic, osd_fb, osd_gen, vx, vy, vw, vh, get_time_us());
            tap.landed(seq + 1);
        }
    }
    std::lock_guard<std::mutex> lk(shown_m_);
    last_shown_ = std::move(pic);
    vrect_x_ = vx; vrect_y_ = vy; vrect_w_ = vw; vrect_h_ = vh;
}

void DrmDevice::start_screen_tap() {
    ScreenPair seed;
    {
        std::lock_guard<std::mutex> lk(shown_m_);
        if (last_shown_) {
            // A copy of its own, holding the picture: last_shown_ itself is
            // not held while nothing is recorded.
            seed.video = std::make_shared<DecodedUnit>(*last_shown_);
            if (!seed.video->frame_ref && seed.video->hold && seed.video->buf)
                seed.video->frame_ref = seed.video->hold(seed.video->buf);
        }
        seed.vx = vrect_x_; seed.vy = vrect_y_; seed.vw = vrect_w_; seed.vh = vrect_h_;
    }
    pthread_mutex_lock(&osd_mutex);
    seed.osd_fb = onscreen_osd_fb_id ? onscreen_osd_fb_id : current_osd_fb_id;
    seed.osd_gen = current_osd_gen_;
    pthread_mutex_unlock(&osd_mutex);
    uint64_t vb;
    if (!last_vblank(&vb, &seed.seq)) seed.seq = 0;
    tap.set_recording(true, &seed);
}

std::shared_ptr<DecodedUnit> DrmDevice::shown_picture() {
    std::lock_guard<std::mutex> lk(shown_m_);
    return last_shown_;
}

void DrmDevice::drop_screen_pictures() {
    std::shared_ptr<DecodedUnit> drop;
    {
        std::lock_guard<std::mutex> lk(shown_m_);
        drop.swap(last_shown_);
    }
    drop.reset();
    tap.drop_pictures();
}

bool DrmDevice::last_vblank(uint64_t *ts_us, uint32_t *seq) {
    // RELATIVE 0 returns at once, with the time of the vblank just past.
    drmVBlank vbl = {};
    vbl.request.type = (drmVBlankSeqType)(DRM_VBLANK_RELATIVE |
        ((output_list->crtc_index << DRM_VBLANK_HIGH_CRTC_SHIFT) & DRM_VBLANK_HIGH_CRTC_MASK));
    vbl.request.sequence = 0;
    if (drmWaitVBlank(drm_fd, &vbl)) return false;
    *ts_us = (uint64_t)vbl.reply.tval_sec * 1000000ULL + vbl.reply.tval_usec;
    if (seq) *seq = vbl.reply.sequence;
    return true;
}

bool DrmDevice::wait_for_flip_completion(int timeout_ms) {
    if (!flip_pending) return true;
    
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += timeout_ms * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&flip_mutex);
    int rc = 0;
    while(flip_pending && rc == 0) {
        rc = pthread_cond_timedwait(&flip_cond, &flip_mutex, &ts);
    }
    pthread_mutex_unlock(&flip_mutex);

    // Watchdog: a DRM flip event can be lost (display hiccup, modeset race).
    // If flip_pending sticks, the renderer spins forever, decoded frames stop
    // being consumed, the mpp buffer pool exhausts and the DECODER jams
    // (decode_put_packet -1012 loop). Force-clear after 250ms — worst case we
    // tear one frame instead of wedging the whole pipeline.
    if (flip_pending && flip_submit_us > 0 &&
        get_time_us() - flip_submit_us > 250000ULL) {
        fprintf(stderr, "[drm] flip event lost (>250ms) — force-clearing flip_pending\n");
        pthread_mutex_lock(&osd_mutex);
        if (inflight_osd_fb_id) { onscreen_osd_fb_id = inflight_osd_fb_id; inflight_osd_fb_id = 0; }
        pthread_mutex_unlock(&osd_mutex);
        flip_pending = false;
    }
    return !flip_pending;
}

void DrmDevice::record_direct_frame(uint64_t recv_ts, uint64_t dec_start_ts,
                                    uint64_t dec_end_ts, uint64_t sub_start_ts,
                                    uint32_t tx_age_us) {
    uint64_t now = get_time_us();
    pthread_mutex_lock(&stats_mutex);
    latest_stats.total_latency_us      = (now - recv_ts) + tx_age_us;
    latest_stats.display_latency_us    = now - dec_end_ts;
    latest_stats.submission_latency_us = now - sub_start_ts;
    latest_stats.completion_ts         = now;
    latest_stats.available             = true;
    pthread_mutex_unlock(&stats_mutex);
    (void)dec_start_ts;
}

bool DrmDevice::page_flip(int fb_id, uint64_t recv_ts, uint64_t dec_start_ts, uint64_t dec_end_ts, uint64_t sub_start_ts, uint32_t tx_age_us, SliceTimesPtr slices,
                          uint32_t split_row, int bottom_fb, SliceTimesPtr bottom_slices) {
    uint64_t page_flip_start = get_time_us();
    
    if (flip_pending) {
        return false;
    }

    // Store stats for this flip
    pthread_mutex_lock(&stats_mutex);
    pending_stats = {recv_ts, dec_start_ts, dec_end_ts, sub_start_ts, tx_age_us, (uint32_t)fb_id, std::move(slices)};
    pthread_mutex_unlock(&stats_mutex);

    // Two commit styles, selectable at runtime so they can be A/B'd against the
    // OSD's "disp" figure.
    //
    // blocking_flip (what stock ar_ldy_gnd does - all four of its
    // drmModeAtomicCommit sites pass flags 0x400, ALLOW_MODESET only, and the
    // binary has no drmHandleEvent at all): the commit does not return until
    // the flip has landed, so there is never a flip in flight when the next
    // frame arrives. Latency is just "time to the next vblank", ~8ms at 60Hz.
    //
    // The non-blocking path asks for a completion event, which sets
    // flip_pending, which then makes page_flip() reject the NEXT frame until
    // the event arrives - costing an extra half-frame on average. That event
    // exists only to timestamp display latency, so the measurement was paying
    // for itself. With a blocking commit the return time IS the completion
    // time, so the stat survives without the event.
    int flags = blocking_flip ? DRM_MODE_ATOMIC_ALLOW_MODESET
                              : (DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT);

    drmModeAtomicSetCursor(output_list->video_request, 0);
    
    // VRR when allowed and the sink is capable
    set_drm_object_property(output_list->video_request, &output_list->crtc, "VRR_ENABLED", vrr_active() ? 1 : 0);

    // Bundle OSD update. First, so that the video plane's rectangle below is
    // the one that goes with the OSD buffer this flip carries (the gallery's
    // tile moves with its OSD frame).
    pthread_mutex_lock(&osd_mutex);
    if (pending_osd_fb_id > 0) {
        current_osd_fb_id = pending_osd_fb_id;
        current_osd_gen_ = pending_osd_gen_;
        current_tile_ = pending_tile_;
        pending_osd_fb_id = 0;
        osd_consumed_++;
        if (prof::enabled()) {
            const uint64_t age = get_time_us() - pending_osd_set_us_;
            osd_age_hist_[age < 5000 ? 0 : age < 10000 ? 1 : age < 20000 ? 2 : age < 35000 ? 3 : age < 60000 ? 4 : 5]++;
        }
    }
    uint32_t osd_fb = current_osd_fb_id;
    uint64_t osd_gen = current_osd_gen_;
    const Tile tile = current_tile_;
    // Mark the fb this flip carries BEFORE committing: the flip's event can be
    // handled on the event thread before this one runs again, and it must
    // find the fb there to move it on screen. Undone below if the commit fails.
    uint32_t prev_inflight = inflight_osd_fb_id;
    if (!blocking_flip) inflight_osd_fb_id = osd_fb;
    pthread_mutex_unlock(&osd_mutex);

    // Destination rectangle for the video plane, shrunk about the screen
    // centre by picture_scale_pct. At 100 these are exactly the full-screen
    // rectangles this code always used, so the default path is untouched.
    // Sizes and offsets are kept even: the video plane is YUV and the VOP
    // wants 2-pixel alignment for it, and the OSD follows for symmetry.
    // Only the VIDEO plane is scaled here. The OSD sits on a Smart window,
    // and on this VOP2 Smart windows have no scaler: a smaller CRTC_W/H is
    // accepted by the atomic check and then silently ignored (verified in
    // debugfs - video shrank, OSD stayed 1920x1080, nothing in dmesg). The
    // OSD scissors itself to the same rectangle instead, so the two stay in
    // step; picture_rect() is the one place that rectangle is computed.
    int vx, vy, vw, vh, sx, sy, sw, sh;
    video_geometry(tile, vx, vy, vw, vh, sx, sy, sw, sh);

    // In halves: the top of this picture on the video plane, the rest of an
    // older, whole one on the second plane below it - each half up as soon as
    // it is decoded. Also in a gallery tile: the part of the picture the tile
    // shows is cut at the same row and each plane scales its own part. Both
    // planes keep at least two rows, so the second one never goes off (that
    // costs the VOP2 a frame).
    // The VOP2 makes a plane under 4x4 invisible (and says so on every commit):
    // each keeps at least 4 rows of source and of screen.
    const bool split = split_row >= 2 && bottom_fb > 0 && has_split_plane() && sh >= 16 && vh >= 8;
    uint32_t srow = split_row & ~1u;
    int top_sh = sh, top_vh = vh;
    if (split) {
        srow = std::max(srow, (uint32_t)(sy + 4));
        srow = std::min(srow, (uint32_t)(sy + sh - 4)) & ~1u;
        top_sh = (int)srow - sy;
        top_vh = (int)((int64_t)top_sh * vh / sh) & ~1;
        top_vh = std::max(4, std::min(top_vh, vh - 4));
    }
    {
        pthread_mutex_lock(&stats_mutex);
        pending_stats.split_row = split ? srow : 0;
        pending_stats.bottom_slices = split ? std::move(bottom_slices) : nullptr;
        pthread_mutex_unlock(&stats_mutex);
    }
    struct drm_object *p2 = &output_list->video2_plane;
    if (split) {
        set_drm_object_property(output_list->video_request, p2, "FB_ID", bottom_fb);
        set_drm_object_property(output_list->video_request, p2, "CRTC_ID", output_list->crtc.id);
        set_drm_object_property(output_list->video_request, p2, "SRC_X", (uint64_t)sx << 16);
        set_drm_object_property(output_list->video_request, p2, "SRC_Y", (uint64_t)srow << 16);
        set_drm_object_property(output_list->video_request, p2, "SRC_W", (uint64_t)sw << 16);
        set_drm_object_property(output_list->video_request, p2, "SRC_H", (uint64_t)(sy + sh - (int)srow) << 16);
        set_drm_object_property(output_list->video_request, p2, "CRTC_X", vx);
        set_drm_object_property(output_list->video_request, p2, "CRTC_Y", vy + top_vh);
        set_drm_object_property(output_list->video_request, p2, "CRTC_W", vw);
        set_drm_object_property(output_list->video_request, p2, "CRTC_H", vh - top_vh);
        set_drm_object_property(output_list->video_request, p2, "zpos", video_zpos);
        set_drm_object_property(output_list->video_request, p2, "COLOR_ENCODING", 0);   // BT.601, as the top
        set_drm_object_property(output_list->video_request, p2, "COLOR_RANGE", 1);
    } else if (split_on_) {
        set_drm_object_property(output_list->video_request, p2, "FB_ID", 0);
        set_drm_object_property(output_list->video_request, p2, "CRTC_ID", 0);
    }

    // Always set video plane properties
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "FB_ID", fb_id);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "CRTC_ID", output_list->crtc.id);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "SRC_X", (uint64_t)sx << 16);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "SRC_Y", (uint64_t)sy << 16);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "SRC_W", (uint64_t)sw << 16);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "SRC_H", (uint64_t)top_sh << 16);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "CRTC_X", vx);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "CRTC_Y", vy);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "CRTC_W", vw);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "CRTC_H", top_vh);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "zpos", video_zpos);
    // The picture as the air unit encodes it (its VUI): BT.601, full range.
    // The plane's defaults were BT.709 and full: greens and reds a little off.
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "COLOR_ENCODING", 0);
    set_drm_object_property(output_list->video_request, &output_list->video_plane, "COLOR_RANGE", 1);

    if (osd_fb > 0) {
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "FB_ID", osd_fb);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "CRTC_ID", output_list->crtc.id);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "CRTC_X", 0);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "CRTC_Y", 0);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "CRTC_W", output_list->mode.hdisplay);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "CRTC_H", output_list->mode.vdisplay);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "SRC_X", 0);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "SRC_Y", 0);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "SRC_W", output_list->mode.hdisplay << 16);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "SRC_H", output_list->mode.vdisplay << 16);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "zpos", osd_zpos);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "alpha", 65535);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "pixel blend mode", 1); // Pre-multiplied
    }


    // Pacing logic completely removed - frames submit immediately

    flip_pending = true;
    flip_submit_us = get_time_us();
    // The screen recorder: what this commit puts on screen. Told before the
    // commit, because its event can arrive before this thread runs again.
    std::shared_ptr<DecodedUnit> pic = std::move(flip_picture_);
    flip_picture_.reset();
    if (tap.recording() && pic)
        tap.submitted(pic, osd_fb, osd_gen, vx, vy, vw, vh, flip_submit_us);
    int ret = drmModeAtomicCommit(drm_fd, output_list->video_request, flags, this);
    if (ret != 0) tap.submit_failed();
    if (ret == 0 && pic) {
        std::lock_guard<std::mutex> lk(shown_m_);
        last_shown_ = std::move(pic);
    }
    if (ret == 0) {
        split_on_ = split;
        last_video_flip_us = flip_submit_us;
        if (blocking_flip) {
            pthread_mutex_lock(&osd_mutex);
            onscreen_osd_fb_id = osd_fb;
            inflight_osd_fb_id = 0;
            pthread_mutex_unlock(&osd_mutex);
        }
    } else {
        pthread_mutex_lock(&osd_mutex);
        inflight_osd_fb_id = prev_inflight;   // no flip, no event
        pthread_mutex_unlock(&osd_mutex);
    }

    if (blocking_flip && ret == 0) {
        // The commit returned, so the flip is done. Close out the stats here
        // exactly as the event handler would, and leave nothing pending.
        uint64_t completion_ts = get_time_us();
        if (tap.recording()) {
            uint64_t vb;
            uint32_t seq;
            if (last_vblank(&vb, &seq)) tap.landed(seq); else tap.submit_failed();
        }
        if (ltrace::on()) ltrace::rec(ltrace::kFlipDone, completion_ts, (uint32_t)fb_id, completion_ts);
        flip_pending = false;
        pthread_mutex_lock(&stats_mutex);
        if (pending_stats.recv_ts > 0) {
            latest_stats.total_latency_us =
                (completion_ts - pending_stats.recv_ts) + pending_stats.tx_age_us;
            latest_stats.display_latency_us = completion_ts - pending_stats.dec_end_ts;
            latest_stats.submission_latency_us = completion_ts - pending_stats.submission_start_ts;
            latest_stats.completion_ts = completion_ts;
            latest_stats.available = true;
            pending_stats.recv_ts = 0;
        }
        if (pending_stats.slices || pending_stats.bottom_slices) {
            // The commit returned after the vblank: take the vblank's own time.
            uint64_t vb = 0;
            if (!last_vblank(&vb) || vb > completion_ts) vb = completion_ts;
            note_slice_flip(vb);
        }
        pthread_mutex_unlock(&stats_mutex);
        // Same cadence bookkeeping the event handler does, so the stutter
        // counter and the expected-period EWMA stay meaningful in this mode.
        if (cadence_last_flip_us > 0 && completion_ts > cadence_last_flip_us) {
            uint32_t gap = (uint32_t)(completion_ts - cadence_last_flip_us);
            push_flip_log(completion_ts, gap);
            cadence_gaps_us[cadence_idx] = gap;
            cadence_idx = (cadence_idx + 1) % CADENCE_RING;
            if (cadence_count < CADENCE_RING) cadence_count++;
            double expect = cadence_gap_ewma_us;
            double thresh = (expect > 0.0) ? expect * 3.5 : 0.0;
            if (thresh < 45000.0) thresh = 45000.0;
            if (expect > 0.0 && (double)gap > thresh) cadence_stutters++;
            else cadence_gap_ewma_us = (expect <= 0.0)
                     ? (double)gap : (0.95 * expect + 0.05 * (double)gap);
        }
        cadence_last_flip_us = completion_ts;
        pthread_mutex_lock(&flip_mutex);
        pthread_cond_broadcast(&flip_cond);
        pthread_mutex_unlock(&flip_mutex);
    }
    if (ret < 0) {
        printf("drmModeAtomicCommit ret <0\n ");
        if (tile.active) {
            // A tile the plane would not take: say which, and fall back to the
            // picture rectangle for the next flips - left as it was, every
            // video flip (60-100 a second) would be refused the same way.
            static int logged = 0;
            if (++logged <= 8)
                printf("gallery tile refused: tile %d,%d %dx%d -> dst %d,%d %dx%d src %d,%d %dx%d (picture %ux%u)\n",
                       tile.x, tile.y, tile.w, tile.h, vx, vy, vw, vh, sx, sy, sw, sh,
                       video_frm_width, video_frm_height);
            pthread_mutex_lock(&osd_mutex);
            current_tile_ = Tile();
            pthread_mutex_unlock(&osd_mutex);
        }
        flip_pending = false; // Commit failed, no event will be sent
        if (errno == EBUSY) prof::count(prof::kCountFlipBusy);
        if (errno != EBUSY) {
            perror("drmModeAtomicCommit (page_flip)");
        }
        return false;
    }
    last_video_fb_id_ = fb_id;
    {
        std::lock_guard<std::mutex> lk(shown_m_);
        vrect_x_ = vx; vrect_y_ = vy; vrect_w_ = vw; vrect_h_ = vh;
        vsrc_x_ = sx; vsrc_y_ = sy; vsrc_w_ = sw; vsrc_h_ = sh;
    }

    // Track flip submission timing (AFTER flip has been submitted)
    static uint64_t last_flip_submit = 0;
    static int flip_count = 0;
    uint64_t now = get_time_us();
    // if (last_flip_submit > 0) {
    //     uint64_t since_last = (now - last_flip_submit) / 1000; // ms
    //     if (flip_count++ % 500 == 0) { // Report every 500 flips (~4s @ 120fps)
    //         time_t now_sec = time(NULL);
    //         struct tm *tm_info = localtime(&now_sec);
    //         char time_buf[64];
    //         strftime(time_buf, sizeof(time_buf), "%H:%M:%S", tm_info);
    //         
    //         printf("[%s] DEBUG: Flip interval: %llu ms, had_to_wait: %s, wait_time: %.2f ms\n", 
    //                time_buf,
    //                (unsigned long long)since_last,
    //                had_to_wait ? "YES" : "NO",
    //                wait_duration_us / 1000.0);
    //     }
    // }
    last_flip_submit = now;
    
    return true;
}

// The flip that landed at vblank_us put up the pending picture's rows - all of
// them, or those above split_row with the bottom picture's below: each of
// those slices is on screen from this vblank (if not already).
void DrmDevice::note_slice_flip(uint64_t vblank_us) {
    const uint32_t split = pending_stats.split_row;
    if (pending_stats.slices) {
        pending_stats.slices->shown(vblank_us, 0, split ? split : UINT32_MAX);
        if (slice_flips_.size() >= 64) slice_flips_.pop_front();
        slice_flips_.push_back({vblank_us, std::move(pending_stats.slices)});
        pending_stats.slices.reset();
    }
    if (pending_stats.bottom_slices) {
        pending_stats.bottom_slices->shown(vblank_us, split, UINT32_MAX);
        pending_stats.bottom_slices.reset();
    }
    pending_stats.split_row = 0;
}

int DrmDevice::screen_row_of(uint32_t row, uint32_t pic_h) {
    pthread_mutex_lock(&osd_mutex);
    const Tile tile = current_tile_;
    pthread_mutex_unlock(&osd_mutex);
    int vx, vy, vw, vh, sx, sy, sw, sh;
    video_geometry(tile, vx, vy, vw, vh, sx, sy, sw, sh);
    (void)vx; (void)vw; (void)sx; (void)sw;
    const int rows = screen_rows();
    if (sh <= 0 || vh <= 0 || rows <= 0) return pic_h ? (int)((uint64_t)rows * row / pic_h) : 0;
    const int r = vy + (int)(((int64_t)row - sy) * vh / sh);
    return std::max(0, std::min(rows - 1, r));
}

void DrmDevice::take_slice_flips(std::vector<SliceFlip>& out) {
    pthread_mutex_lock(&stats_mutex);
    for (auto& f : slice_flips_) out.push_back(std::move(f));
    slice_flips_.clear();
    pthread_mutex_unlock(&stats_mutex);
}

CompletedStats DrmDevice::get_latest_stats() {
    pthread_mutex_lock(&stats_mutex);
    CompletedStats s = latest_stats;
    latest_stats.available = false; // Consume
    pthread_mutex_unlock(&stats_mutex);
    return s;
}

void DrmDevice::append_flip_log_after(uint64_t after_us, std::vector<FlipSample>& out) {
    pthread_mutex_lock(&stats_mutex);
    int i = flip_log_n_;
    while (i > 0 && flip_log_[(flip_log_head_ + i - 1) % FLIP_LOG].t_us > after_us) i--;
    out.reserve(out.size() + (size_t)(flip_log_n_ - i));
    for (int j = i; j < flip_log_n_; j++) out.push_back(flip_log_[(flip_log_head_ + j) % FLIP_LOG]);
    pthread_mutex_unlock(&stats_mutex);
}

void DrmDevice::copy_flip_log(uint64_t since_us, std::vector<FlipSample>& out) {
    out.clear();
    pthread_mutex_lock(&stats_mutex);
    out.reserve((size_t)flip_log_n_);
    for (int i = 0; i < flip_log_n_; i++) {
        const FlipSample& f = flip_log_[(flip_log_head_ + i) % FLIP_LOG];
        if (f.t_us >= since_us) out.push_back(f);
    }
    pthread_mutex_unlock(&stats_mutex);
}

RenderCadence DrmDevice::get_render_cadence() {
    uint32_t gaps[CADENCE_RING];
    int n;
    RenderCadence rc = {0, 0, 0};

    pthread_mutex_lock(&stats_mutex);
    n = cadence_count;
    memcpy(gaps, cadence_gaps_us, sizeof(uint32_t) * n);
    rc.stutters = cadence_stutters;
    pthread_mutex_unlock(&stats_mutex);

    if (n > 0) {
        std::sort(gaps, gaps + n);
        rc.max_ms = gaps[n - 1] / 1000.0f;
        rc.p99_ms = gaps[(n * 99) / 100 < n ? (n * 99) / 100 : n - 1] / 1000.0f;
    }
    return rc;
}

void DrmDevice::set_frame_size(uint32_t width, uint32_t height) {
	video_frm_width = width;
	video_frm_height = height;
}

// Whole-picture scale. The video plane gets a smaller destination rectangle,
// centred, and the display controller scales it at scanout - no renderer
// involvement, no copy; the OSD scissors itself to the same rectangle. What
// is left round the edge is whatever the CRTC shows where no plane covers it,
// which is black. Read at every commit, so it takes effect on the next frame.
void DrmDevice::set_picture_scale(int pct) {
    if (pct < 60)  pct = 60;
    if (pct > 100) pct = 100;
    picture_scale_pct = pct;
}

void DrmDevice::picture_rect(int base_w, int base_h, int &x, int &y, int &w, int &h) const {
    if (picture_scale_pct >= 100) { x = 0; y = 0; w = base_w; h = base_h; return; }
    // Even sizes and offsets: the video plane is YUV and the VOP wants
    // 2-pixel alignment for it; the OSD follows so the two edges coincide.
    w = (base_w * picture_scale_pct / 100) & ~1;
    h = (base_h * picture_scale_pct / 100) & ~1;
    x = ((base_w - w) / 2) & ~1;
    y = ((base_h - h) / 2) & ~1;
}

void DrmDevice::stage_tile(const Tile& t) {
    pthread_mutex_lock(&osd_mutex);
    staged_tile_ = t;
    pthread_mutex_unlock(&osd_mutex);
}

// The tile with its origin and size rounded down to even numbers. Everything
// else (what part is on screen, which part of the picture goes in it) is worked
// out from this one rectangle: measured from the unrounded edge, an odd origin
// put the on-screen part's edge before the tile's own, a negative offset into the
// picture, and the plane refused the whole commit.
DrmDevice::Tile DrmDevice::even_tile(const Tile& t) {
    Tile e = t;
    e.x = t.x & ~1;
    e.y = t.y & ~1;
    e.w = t.w & ~1;
    e.h = t.h & ~1;
    return e;
}

bool DrmDevice::tile_on_screen(const Tile& t, int sw, int sh, int &x0, int &y0, int &x1, int &y1) {
    if (!t.active || t.w < 2 || t.h < 2) return false;
    const Tile e = even_tile(t);
    x0 = std::max(e.x, 0);
    y0 = std::max(e.y, 0);
    x1 = std::min(e.x + e.w, sw & ~1);
    y1 = std::min(e.y + e.h, sh & ~1);
    return x1 - x0 >= 2 && y1 - y0 >= 2;
}

// The picture rectangle, or the gallery's tile. A tile the strip has scrolled
// past the screen edge is cut there, and the part of the picture that falls in
// the cut is dropped in the same proportion; both stay even (the plane is YUV).
// A tile wholly off the screen leaves the plane where the picture would be: the
// OSD covers the whole screen then, so it is not seen.
void DrmDevice::video_geometry(const Tile& tile, int &dx, int &dy, int &dw, int &dh,
                               int &sx, int &sy, int &sw, int &sh) const {
    picture_rect(video_fb_width, video_fb_height, dx, dy, dw, dh);
    dx += video_fb_x; dy += video_fb_y;
    sx = 0; sy = 0; sw = (int)video_frm_width; sh = (int)video_frm_height;
    if (!tile.active || tile.w < 2 || tile.h < 2 || sw < 2 || sh < 2) return;

    int x0, y0, x1, y1;
    if (!tile_on_screen(tile, output_list->mode.hdisplay, output_list->mode.vdisplay, x0, y0, x1, y1)) return;

    const Tile e = even_tile(tile);
    dx = x0; dy = y0; dw = x1 - x0; dh = y1 - y0;
    sx = (int)((int64_t)(x0 - e.x) * video_frm_width / e.w) & ~1;
    sy = (int)((int64_t)(y0 - e.y) * video_frm_height / e.h) & ~1;
    sw = (int)((int64_t)(x1 - x0) * video_frm_width / e.w) & ~1;
    sh = (int)((int64_t)(y1 - y0) * video_frm_height / e.h) & ~1;
    sw = std::max(2, std::min(sw, (int)video_frm_width - sx));
    sh = std::max(2, std::min(sh, (int)video_frm_height - sy));
}

void DrmDevice::perform_modeset_video(int fb_id) {
    if (fb_id <= 0) return; 
    plane_info video_plane_info = {
        .plane = &output_list->video_plane, 
        .fb_id = fb_id, 
        .width = video_frm_width, 
        .height = video_frm_height, 
        .zpos = video_zpos,
    };

    plane_info osd_plane_info = {
        .plane = &output_list->osd_plane,
        .fb_id = (int)current_osd_fb_id,
        .width = output_list->mode.hdisplay,
        .height = output_list->mode.vdisplay,
        .zpos = osd_zpos,
    };

    if (current_osd_fb_id > 0) {
        // Essential: Set blending properties for the OSD plane during modeset!
        // Otherwise it defaults to opaque and covers the video.
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "alpha", 65535);
        set_drm_object_property(output_list->video_request, &output_list->osd_plane, "pixel blend mode", 1); // Pre-multiplied
    }

    int ret = modeset_perform_modeset(drm_fd, output_list, output_list->video_request, &video_plane_info, (current_osd_fb_id > 0) ? &osd_plane_info : NULL);
    if (ret < 0) {
        static int warn_count = 0;
        if (warn_count++ % 60 == 0) fprintf(stderr, "WARNING: perform_modeset_video failed: %d\n", ret);
    }
}

void DrmDevice::cleanup() {
    // Stop the event thread before anything it uses is freed: it probes the
    // connector (output_list) every 2 s, and polls with a 500 ms timeout.
    event_thread_exit = true;
    pthread_join(event_thread, NULL);

    restore_planes_zpos(drm_fd, output_list);
	drmModeSetCrtc(drm_fd,
			       output_list->saved_crtc->crtc_id,
			       output_list->saved_crtc->buffer_id,
			       output_list->saved_crtc->x,
			       output_list->saved_crtc->y,
			       &output_list->connector.id,
			       1,
			       &output_list->saved_crtc->mode);
	drmModeFreeCrtc(output_list->saved_crtc);
	drmModeAtomicFree(output_list->video_request);
	drmModeAtomicFree(output_list->osd_request);
	if (hotplug_request) drmModeAtomicFree(hotplug_request);
	modeset_cleanup(drm_fd, output_list);

	close(drm_fd);
	drm_fd = -1;   // a decoder going after this (at exit) skips its DRM calls
	
	int ret = pthread_cond_destroy(&video_cond);
	assert(!ret);
	ret = pthread_mutex_destroy(&video_mutex);
	assert(!ret);
    ret = pthread_mutex_destroy(&stats_mutex);
	assert(!ret);
    ret = pthread_mutex_destroy(&flip_mutex);
	assert(!ret);
    ret = pthread_cond_destroy(&flip_cond);
	assert(!ret);

    //free(output_list_original);
}

void DrmDevice::cond_signal() {
    int ret = pthread_mutex_lock(&video_mutex);
	assert(!ret);	
	ret = pthread_cond_signal(&video_cond);
	assert(!ret);	
	ret = pthread_mutex_unlock(&video_mutex);
	assert(!ret);	
}
bool DrmDevice::set_display_property(const char* name, uint64_t value) {
    if (!output_list) return false;
    uint32_t conn_id = output_list->connector.id;
    if (!conn_id) return false;

    drmModeObjectProperties *props =
        drmModeObjectGetProperties(drm_fd, conn_id, DRM_MODE_OBJECT_CONNECTOR);
    if (!props) return false;
    uint32_t prop_id = 0;
    for (uint32_t i = 0; i < props->count_props && !prop_id; i++) {
        drmModePropertyRes *p = drmModeGetProperty(drm_fd, props->props[i]);
        if (!p) continue;
        if (!strcmp(p->name, name)) prop_id = p->prop_id;
        drmModeFreeProperty(p);
    }
    drmModeFreeObjectProperties(props);
    if (!prop_id) return false;

    int ret = drmModeObjectSetProperty(drm_fd, conn_id, DRM_MODE_OBJECT_CONNECTOR,
                                       prop_id, value);
    if (ret) printf("drm: set %s=%llu failed (%d)\n", name, (unsigned long long)value, ret);
    return ret == 0;
}
