#include "display_lock.hpp"
#include "lock_step.hpp"
#include "camera_steer.hpp"
#include "drm.hpp"
#include "frame_mode.hpp"
#include "settings.hpp"
#include "utils/scheduling_helper.hpp"
#include "utils/time_util.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

std::atomic<int> g_display_lock{0};
std::atomic<int> g_camera_trim_ns{0};
std::atomic<bool> g_camera_steer{false};
std::atomic<int> g_display_lock_slice{1};
std::atomic<int> g_display_lock_margin_us{0};
std::atomic<float> g_dl_mode_hz{0}, g_dl_camera_fps{0};
std::atomic<bool> g_dl_holding{false};
std::atomic<const char*> g_dl_why{nullptr};

namespace {
const uint32_t kVop2Base = 0xfe040000u;   // RK3568 / RK3566
const uint32_t kVtotalReg = 0x0c50;       // VP0's DSP_VTOTAL_VS_END, VPn's 0x100 further on each
const uint32_t kImd = 1u << 15;           // applies to the frame being scanned out
const uint64_t kWindowUs = 2000000;       // arrivals looked at: the last 2 s
const size_t kMinSamples = 50;
const double kStayUs = 30;                // a better position by less than this is not worth a move

double wrap(double x, double P) {         // into [-P/2, P/2)
    x = std::fmod(x, P);
    if (x >= P / 2) x -= P;
    if (x < -P / 2) x += P;
    return x;
}
double mod(double x, double P) { x = std::fmod(x, P); return x < 0 ? x + P : x; }
}

DisplayLock::DisplayLock(std::shared_ptr<DrmDevice> dev, double cutoff_us) : dev_(dev), cutoff_us_(cutoff_us) {}

DisplayLock::~DisplayLock() {
    stop();
    if (map_) munmap(map_, 0x1000);
}

void DisplayLock::start() {
    if (run_.exchange(true)) return;
    th_ = std::thread(&DisplayLock::run, this);
}

void DisplayLock::stop() {
    if (!run_.exchange(false)) return;
    if (th_.joinable()) th_.join();
}

void DisplayLock::note_unit(uint64_t pts, bool early, uint64_t t_us, uint64_t last_in_us, bool halves, uint64_t cap_us) {
    std::lock_guard<std::mutex> lk(m_);
    if (early) {
        early_pts_ = pts;
        early_t_ = t_us;
        return;
    }
    if (pts == last_pts_) return;
    last_pts_ = pts;
    const int mode = (g_camera_lines.load(std::memory_order_relaxed) << 16) | g_camera_fps.load(std::memory_order_relaxed);
    if (mode != caps_mode_) {
        caps_.clear();
        caps_mode_ = mode;
    }
    if (cap_us && (caps_.empty() || cap_us > caps_.back())) {
        caps_.push_back(cap_us);
        while (caps_.front() + 10000000 < cap_us) caps_.pop_front();
    }
    uint64_t ready = t_us;
    // DISPLAY > Sync To SLICE 2: the whole picture's time, so the refresh
    // follows the last slice (its top waits for it there, or goes up a
    // refresh before).
    const bool by_top = g_display_lock_slice.load(std::memory_order_relaxed) != 2;
    if (pts == early_pts_ && by_top) ready = halves ? early_t_ : last_in_us ? std::min(t_us, std::max(early_t_, last_in_us)) : t_us;
    if (arrivals_in_.size() < 1000) arrivals_in_.push_back(ready);
    if (cap_us && steer_in_.size() < 1000) steer_in_.push_back({ cap_us, ready });   // CAMERA
}

// The span of the stamps over the pictures it holds, those counted from the
// median gap (a picture lost is a gap of two): a stamp's error over a thousand.
double DisplayLock::camera_period_us() {
    std::vector<uint64_t> c;
    {
        std::lock_guard<std::mutex> lk(m_);
        c.assign(caps_.begin(), caps_.end());
    }
    if (c.size() < 500) return 0;
    std::vector<uint64_t> g;
    for (size_t i = 1; i < c.size(); i++) g.push_back(c[i] - c[i - 1]);
    std::nth_element(g.begin(), g.begin() + g.size() / 2, g.end());
    const double med = (double)g[g.size() / 2];
    if (med <= 0) return 0;
    const double span = (double)(c.back() - c.front());
    const double n = std::round(span / med);
    return n > 0 ? span / n : 0;
}

// The RK3568's (and RK3566's) VOP2 only: the address and the immediate bit are
// that display controller's.
bool DisplayLock::map_register() {
    char compat[512] = {};
    if (FILE *f = fopen("/proc/device-tree/compatible", "r")) {
        const size_t n = fread(compat, 1, sizeof(compat) - 1, f);
        fclose(f);
        for (size_t i = 0; i < n; i++) if (!compat[i]) compat[i] = ' ';
    }
    if (!strstr(compat, "rockchip,rk3568") && !strstr(compat, "rockchip,rk3566")) {
        printf("display lock: not an RK3568 - Refresh Sync is not available\n");
        return false;
    }
    const int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("display lock: /dev/mem");
        return false;
    }
    void *m = mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, kVop2Base);
    close(fd);
    if (m == MAP_FAILED) {
        perror("display lock: mmap");
        return false;
    }
    map_ = m;
    reg_ = (volatile uint32_t *)((uint8_t *)map_ + kVtotalReg + 0x100 * dev_->output_list->crtc_index);
    return true;
}

// Whether to hold the phase now: turned on for this screen, no VRR, and the
// screen able to keep in step at all.
// ON: the refresh a whole multiple of the camera's frame rate - otherwise the
// pictures sweep through the refresh far faster than a line a frame follows.
// FREE: the screen's mode no slower than the camera (frames are only made
// longer than the mode's), and no more than twice as fast.
bool DisplayLock::usable(int lock, double Pm, double Pc, const char **why) {
    if (!lock) { *why = "off"; return false; }
    if (dev_->vrr_active()) { *why = "not with VRR"; return false; }
    if (Pc <= 0) { *why = "waiting for the camera's frame rate"; return false; }
    if (lock == 2) {
        if (Pc < Pm * 0.999) { *why = "the camera is faster than the screen's mode"; return false; }
        return true;
    }
    const double k = std::round(Pc / Pm);
    if (k < 1 || std::fabs(Pc - k * Pm) > 0.01 * Pc) {
        *why = "the refresh is not a whole multiple of the camera's frame rate";
        return false;
    }
    return true;
}

// The frame being scanned out d lines longer (d < 0: shorter), from now. Under
// the mode-change lock, so never over a modeset's own value, and only in the
// first half of the frame: a total set while the scan-out is already past it
// runs the frame to the counter's end (a 33 ms frame once, in a test that
// wrote at any time).
bool DisplayLock::write_lines(int d, uint64_t vb_us, double P) {
    std::lock_guard<std::mutex> lk(dev_->crtc_mode_m);
    const drmModeModeInfo &m = dev_->output_list->mode;
    if (m.vtotal != mode_vtotal_ || m.clock != mode_clock_) return false;
    const uint64_t now = get_time_us();
    if (now < vb_us || (double)(now - vb_us) >= P / 2) return false;
    *reg_ = (orig_ & ~(0x1fffu << 16)) | (((mode_vtotal_ + d) & 0x1fff) << 16) | kImd;
    d_ = d;
    released_ = false;
    return true;
}

// The mode's own value without the immediate bit, a frame after it went in with
// it: the driver's own writes then latch at a frame boundary as they always did.
void DisplayLock::release() {
    std::lock_guard<std::mutex> lk(dev_->crtc_mode_m);
    const drmModeModeInfo &m = dev_->output_list->mode;
    if (m.vtotal == mode_vtotal_ && m.clock == mode_clock_) *reg_ = orig_;
    released_ = true;
}

// Where the vblank should be, as a shift on the same reference as the samples:
// each picture shows at the first vblank at least cutoff_us after it arrives,
// and the expected wait to it is least with the deadline just after one of
// them (between those points it only grows), so those are the candidates. Of
// the ones within kStayUs of the best, the nearest to G. Also the expected
// wait and the share of pictures past the deadline where the vblank is now.
double DisplayLock::target_offset(double P, double G, double *wait_now, double *late_now) {
    const double C = P - cutoff_us_;
    auto cost = [&](double g, double *late) {
        double w = 0;
        int n_late = 0;
        for (const Sample &s : win_) {
            const double x = mod(s.u - g, P);
            if (x <= C + 0.01) w += P - x;
            else { w += 2 * P - x; n_late++; }
        }
        if (late) *late = (double)n_late / win_.size();
        return w / win_.size();
    };
    *wait_now = cost(G, late_now);
    std::vector<double> gs, ws;
    double best = 1e18;
    for (const Sample &s : win_) {
        const double g = mod(s.u + cutoff_us_, P);
        const double w = cost(g, nullptr);
        gs.push_back(g);
        ws.push_back(w);
        if (w < best) best = w;
    }
    if (*wait_now <= best + kStayUs) return G;
    double pick = G, dist = 1e18;
    for (size_t i = 0; i < gs.size(); i++) {
        if (ws[i] > best + kStayUs) continue;
        const double dd = std::fabs(wrap(gs[i] - G, P));
        if (dd < dist) { dist = dd; pick = gs[i]; }
    }
    return pick;
}

void DisplayLock::run() {
    SchedulingHelper::configure_thread(SchedulingHelper::ThreadRole::DrmEvent);
    if (!map_register()) return;

    uint64_t vb_prev = 0;
    uint32_t seq_prev = 0;
    double G = 0;                     // vblank shift applied so far, us (mod the grid's period)
    double target = 0;
    bool have_target = false, moving = false, locked_said = false;
    int frames = 0;
    const char *why_said = "";       // nothing said yet
    uint64_t log_t = 0, cam_t = 0;
    double moved_us = 0;              // since the last log line
    double pc_last = 0, pc_known = 0; // the camera's period: last measured, and in use
    double trim = 0;                  // FREE: lines learned on top of the period's (its error)
    double aim_us = 0;                // test: the vblank this much earlier than the best (A/B)
    uint64_t aim_t = 0;
    int pc_mode = -1;                 // (lines << 16) | fps pc_known is of
    int dir_said = 0;                 // ON: the way frames were last said to move (1 longer, -1 shorter)
    uint64_t st_log_t = 0;            // CAMERA: the last 5 s line
    std::vector<uint64_t> in;

    while (run_.load(std::memory_order_relaxed)) {
        uint32_t seq;
        uint64_t vb;
        if (!dev_->wait_vblank(&seq, &vb)) {
            usleep(100000);
            vb_prev = 0;
            continue;
        }
        const uint64_t now = get_time_us();
        // One copy, so the period and line time are of the mode compared below.
        const drmModeModeInfo mode = dev_->current_mode();
        const double Pm = DrmDevice::period_of(mode), line = DrmDevice::row_time_of(mode, 1);
        if (Pm <= 0 || line <= 0) continue;

        // A new mode (Screen Mode Auto following the camera, a hotplug): the
        // driver set the register for it; start over from that.
        if (mode.clock != mode_clock_ || mode.vtotal != mode_vtotal_) {
            mode_clock_ = mode.clock;
            mode_vtotal_ = 0;
            d_ = 0;
            released_ = true;
            G = 0;
            have_target = moving = false;
            win_.clear();
            vb_prev = 0;
            dev_->set_frame_period(0);
        }

        // The camera's own period: the one measured for this video mode before
        // (camera_period_key), then the capture stamps' - two 10 s readings in
        // a row within 20 ppm, kept for the next time. Until then, the rate
        // its mode is named by.
        const int fps = g_camera_fps.load(std::memory_order_acquire);
        const int lines = g_camera_lines.load(std::memory_order_relaxed);
        const int cmode = (lines << 16) | fps;
        if (cmode != pc_mode) {
            pc_mode = cmode;
            trim = 0;
            pc_last = 0;
            pc_known = fps > 0 ? Settings::getInstance().getFloat(camera_period_key(lines, fps), 0.0f) : 0;
            if (fps <= 0 || pc_known * fps < 0.99e6 || pc_known * fps > 1.01e6) pc_known = 0;
        }
        if (now >= cam_t + 10000000) {
            cam_t = now;
            const double pc = camera_period_us();
            // Within 1% of the rate the mode is named by (a "100 fps" Lite+ runs
            // 100.13): anything further is not this mode's camera.
            if (pc > 0 && fps > 0 && pc * fps > 0.99e6 && pc * fps < 1.01e6) {
                printf("display lock: the camera's frame every %.3f us by this clock (%.4f fps), the screen's mode "
                       "%.3f us\n", pc, 1e6 / pc, Pm);
                fflush(stdout);
                // Two readings in a row within 100 ppm (one is good to some
                // 30-90 ppm over its 10 s), more than 20 ppm from the one in
                // use: the new one. (Within 20 ppm in a row was too strict for
                // a 10 s window: a mixed first reading 12 us short was kept for
                // good, and the lock chased it at 1.5 ms a second.)
                const bool steady = pc_last > 0 && std::fabs(pc - pc_last) < 100e-6 * pc;
                if (steady && std::fabs(pc - pc_known) > 20e-6 * pc) {
                    char v[32];
                    snprintf(v, sizeof(v), "%.3f", pc);
                    Settings::getInstance().set(camera_period_key(lines, fps), std::string(v));
                    pc_known = pc;
                    trim = 0;
                }
            }
            pc_last = pc;
        }
        const double Pc = pc_known > 0 ? pc_known : fps > 0 ? 1e6 / fps : 0;

        // DISPLAY > Sync Margin: the refresh that much later than the least
        // expected wait (fewer pictures past the deadline, more wait for the
        // rest).
        if (now >= aim_t + 1000000) {
            aim_t = now;
            const double a = -(double)g_display_lock_margin_us.load(std::memory_order_relaxed);
            if (a != aim_us) {
                printf("display lock: aiming %.0f us earlier than the least expected wait\n", a);
                fflush(stdout);
            }
            aim_us = a;
        }

        const int lock = g_display_lock.load(std::memory_order_relaxed);
        const char *why = nullptr;
        const bool on = usable(lock, Pm, Pc, &why);
        in_step_.store(on && mode_vtotal_, std::memory_order_relaxed);
        g_dl_mode_hz.store(Pm > 0 ? (float)(1e6 / Pm) : 0.0f, std::memory_order_relaxed);
        g_dl_camera_fps.store(Pc > 0 ? (float)(1e6 / Pc) : 0.0f, std::memory_order_relaxed);
        g_dl_why.store(on ? nullptr : why, std::memory_order_relaxed);
        g_dl_holding.store(on, std::memory_order_relaxed);
        if (!on) {
            g_camera_steer.store(false, std::memory_order_relaxed);
            {
                std::lock_guard<std::mutex> lk(m_);
                steer_in_.clear();
            }
            if (d_ != 0) {
                write_lines(0, vb, Pm);
            } else if (!released_) {
                release();
            }
            dev_->set_frame_period(0);
            if (why != why_said) {
                printf("display lock: %s\n", why);
                fflush(stdout);
                why_said = why;
                locked_said = false;
                dir_said = 0;
            }
            std::lock_guard<std::mutex> lk(m_);
            arrivals_in_.clear();
            vb_prev = 0;
            continue;
        }
        const bool free = lock == 2;
        const bool camera = lock == 3;
        if (why_said) {
            why_said = nullptr;
            printf("display lock: on (%s) - holding the vblank just after the pictures arrive\n",
                   camera ? "CAMERA: the camera steered, the screen left alone"
                          : free ? "FREE: each frame as long as the camera's" : "one line at a time");
            fflush(stdout);
        }

        // The register as the mode set it: vtotal and the sync width. Off by a
        // little is ours, left by a run that did not get to put it back.
        if (!mode_vtotal_) {
            const uint32_t raw = *reg_, v = raw & ~kImd;
            const int vt = (int)((v >> 16) & 0x1fff);
            if ((int)(v & 0x1fff) != mode.vsync_end - mode.vsync_start || vt < (int)mode.vtotal - 1 ||
                vt > 2 * (int)mode.vtotal) {
                static bool said = false;
                if (!said) {
                    said = true;
                    printf("display lock: the display controller's total (0x%08x) is not the mode's (%u lines) - off\n",
                           v, mode.vtotal);
                }
                continue;
            }
            orig_ = (v & ~(0x1fffu << 16)) | ((uint32_t)mode.vtotal << 16);
            mode_vtotal_ = mode.vtotal;
            d_ = vt - (int)mode.vtotal;          // brought back below like any other
            released_ = d_ == 0 && !(raw & kImd);
        }

        // The grid the vblank is held to: the mode's refresh (ON), or one
        // refresh per camera frame, or per half, third... of one, never shorter
        // than the mode's (FREE): B lines more than the mode's each frame.
        double Pt = Pm, B = 0;
        if (free) {
            const double k = std::max(1.0, std::floor(Pc / Pm + 1e-6));
            Pt = Pc / k;
            B = std::max(0.0, Pt / line - mode.vtotal + trim);
            Pt = (mode.vtotal + B) * line;
        }

        // The arrivals in the frame that just ended, on the reference: their
        // phase after its vblank, plus the shift as it was then.
        in.clear();
        {
            std::lock_guard<std::mutex> lk(m_);
            while (!arrivals_in_.empty() && arrivals_in_.front() < vb) {
                in.push_back(arrivals_in_.front());
                arrivals_in_.pop_front();
            }
        }
        if (vb_prev)
            for (uint64_t t : in) win_.push_back({ t, mod(mod((double)t - (double)vb_prev, Pt) + G, Pt) });
        while (!win_.empty() && win_.front().t_us + kWindowUs < vb) win_.pop_front();

        // The frames since the last vblank seen had d_ lines more each (a
        // write holds until the next), the grid's B of them.
        if (vb_prev) {
            const uint32_t n = seq - seq_prev;
            if (n >= 1 && n < 100) {
                G = mod(G + (d_ - B) * line * n, Pt);
                moved_us += (d_ - B) * line * n;
            }
        }
        vb_prev = vb;
        seq_prev = seq;

        if (win_.size() >= kMinSamples && (!have_target || ++frames >= 10)) {
            frames = 0;
            double w, l;
            // (the best found from where the vblank would be without the aim,
            // so staying put means staying aimed)
            target = mod(target_offset(Pt, mod(G + aim_us, Pt), &w, &l) - aim_us, Pt);
            have_target = true;
            if (now >= log_t + 5000000) {
                if (aim_us != 0) target_offset(Pt, G, &w, &l);   // the figures where it is
                if (log_t)
                    printf("display lock: expected wait %.2f ms, %.1f%% past the deadline (%zu pictures in 2 s), "
                           "vblank moved %+.0f us in 5 s, %+d lines now (%.1f for the camera's frame, %+.2f learnt)\n",
                           w / 1000, l * 100, win_.size(), moved_us, d_, B, trim);
                fflush(stdout);
                log_t = now;
                moved_us = 0;
            }
        }

        // CAMERA: the screen's timing never changes; the camera's frames are
        // made a little longer so each picture is ready a margin before its
        // vblank's commit deadline (camera_steer.hpp, after mabur's phase lock).
        // The margin is DISPLAY > Sync Margin, at least 1 ms.
        if (camera) {
            if (d_ != 0) write_lines(0, vb, Pm);
            else if (!released_) release();
            const double margin = std::max(1000.0, (double)g_display_lock_margin_us.load(std::memory_order_relaxed));
            if (!steer_ || steer_margin_ != margin) {
                steer_.reset(new display::CameraSteer(margin, cutoff_us_));
                steer_margin_ = margin;
                printf("display lock: camera steer: ready %.1f ms before each commit deadline\n", margin / 1000);
                fflush(stdout);
            }
            const double fpsc = Pc > 0 ? 1e6 / Pc : 60;
            const int k = std::max(1, (int)std::lround(Pc / Pm));
            {
                std::lock_guard<std::mutex> lk(m_);
                for (const auto &f : steer_in_) steer_->on_frame(f.first, f.second, vb, Pm, k, fpsc);
                steer_in_.clear();
            }
            static const char *const kStateName[] = { "waiting", "acquiring", "locked", "out of range" };
            const auto st0 = steer_->state();
            if (steer_->tick(now)) {
                g_camera_trim_ns.store((int)std::lround(steer_->trim_us() * 1000), std::memory_order_relaxed);
                if (steer_->state() != st0) {
                    printf("display lock: camera steer %s - error %+.0f us, trim %+.2f us a frame, rate %+.2f, "
                           "delivery p99 %.1f ms\n", kStateName[(int)steer_->state()], steer_->error_us(),
                           steer_->trim_us(), steer_->centre_us(), steer_->allowance_us() / 1000);
                    fflush(stdout);
                }
            }
            g_camera_steer.store(steer_->steering(), std::memory_order_relaxed);
            if (now >= st_log_t + 5000000) {
                st_log_t = now;
                printf("display lock: camera steer: %s, error %+.0f us, trim %+.2f us a frame (rate %+.2f, last window's "
                       "%+.2f), delivery p1 %.1f p99 %.1f ms, guard %.1f ms\n", kStateName[(int)steer_->state()],
                       steer_->error_us(), steer_->trim_us(), steer_->centre_us(), steer_->neutral_us(),
                       steer_->fast_us() / 1000, steer_->allowance_us() / 1000, steer_->guard_us() / 1000);
                fflush(stdout);
            }
            dev_->set_frame_period(0);
            continue;
        }

        int want = d_;
        if (!have_target) {
            want = free ? (int)std::lround(B) : 0;
        } else if (free) {
            // As long as a camera frame, and the error back by up to 20 lines a
            // frame: the fraction of a line left over comes back the same way.
            const double e = wrap(target - G, Pt);
            const int c = std::fabs(e) < 0.5 * line ? 0 : (int)std::max(-20.0, std::min(20.0, std::round(e / line)));
            want = std::max(0, std::min((int)mode.vtotal, (int)std::lround(B) + c));
            // Frames that keep coming out longer or shorter than the grid's
            // mean the camera's frame is not quite the period measured: that
            // difference (lines used less lines meant, not the correction on
            // top of the rounded lines - which learnt the rounding) learnt
            // slowly into the grid (10 s or so for a line), within 3 lines,
            // once in step and at most 2 lines a frame (a pull-in's 20 would
            // wind it up).
            if (locked_said)
                trim = std::max(-3.0, std::min(3.0, trim + 0.002 * std::max(-2.0, std::min(2.0, want - B))));
            if (!locked_said && std::fabs(e) < 2 * line) {
                locked_said = true;
                printf("display lock: in step\n");
                fflush(stdout);
            }
        } else {
            // A line at a time, one way only (lock_step.hpp): longer where the
            // screen is the faster, shorter where it is the slower (60.000 Hz
            // against the Lite+'s 60.03 fps) - longer only, there, went round a
            // whole refresh every 12 s. Never both ways: the drift brings the
            // vblank back the other way, so a line one way is all it needs.
            // Until the camera's period is measured it is the mode's: longer.
            const double k = std::max(1.0, std::round(Pc / Pm));
            const bool shorter = k * Pm > Pc * (1 + 1e-6);
            if ((shorter ? -1 : 1) != dir_said) {
                dir_said = shorter ? -1 : 1;
                moving = false;
                printf("display lock: frames made only %s (the screen %s than the camera)\n",
                       shorter ? "shorter" : "longer", shorter ? "slower" : "no slower");
                fflush(stdout);
            }
            const display::LineStep s = display::line_step(wrap(target - G, Pt), Pt, line, shorter, moving);
            moving = s.moving;
            if (s.arrived && !locked_said) {
                locked_said = true;
                printf("display lock: in step\n");
                fflush(stdout);
            }
            want = s.lines;
        }
        if (want != d_) write_lines(want, vb, Pm);
        else if (want == 0 && !released_) release();
        dev_->set_frame_period(d_ ? (mode.vtotal + d_) * line : 0);
    }
    in_step_ = false;
    g_camera_steer.store(false, std::memory_order_relaxed);
    dev_->set_frame_period(0);
    if (reg_ && !released_) {
        uint32_t s;
        uint64_t t;
        const double P = dev_->mode_period_us();
        for (int i = 0; i < 5 && d_ != 0 && dev_->wait_vblank(&s, &t); i++) write_lines(0, t, P);
        dev_->wait_vblank(&s, &t);
        release();
    }
}
