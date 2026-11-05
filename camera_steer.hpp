#pragma once
// DISPLAY > Refresh Sync CAMERA's controller (display_lock.cpp), apart from the
// lock so it can be tested without a display or an air unit
// (tools/test/camera_steer_test.cpp). After mabur's phase lock
// (gs/player/src/phase_lock.cpp): the screen is never touched; the camera's
// frames are made a little longer so each picture is ready a margin before
// the commit deadline of the vblank it is meant for.
//
// What it watches is when each picture was captured, not when it arrived: the
// capture stamps (the air unit's, on this clock by the radio's) hold still to
// a few us where arrivals scatter by milliseconds - more with the bitrate, and
// bimodally with intra refresh (a GOP start's bulk is one slice, out of the
// encoder only once the whole picture is: 6 ms later at 20 Mbps). Delivery is
// measured on its own, as an allowance: the 99th percentile of capture ->
// ready over the last 10 s. So the late pictures are planned for, once, and
// not chased each time one comes.
//
// Each window (1.2 s): the error is the median of where captures fall in the
// refresh, plus the allowance and the margin, against the deadline - wrapped
// into +-P/2. A least-squares line through the capture phases gives the drift,
// which names the trim that would match the screen: the camera's frame is its
// own plus the trim, so trim - drift a frame does. `centre` is an average of
// that (the rate), and the trim asked for is centre less a correction for the
// error (the phase), limited: ~0.1% of a frame a frame while acquiring, a
// third of that once locked. The air unit only makes frames longer (shorter
// than its mode's would change the sensor's mode), up to 8 lines: 0..hi_us.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

namespace display {

class CameraSteer {
public:
    enum class State { Waiting, Acquiring, Locked, OutOfRange };

    static constexpr double kHiUs = 60;              // the air unit's reach: 8 OS02K10 lines are 61 us
    static constexpr uint64_t kWindowUs = 1200000;
    static constexpr uint64_t kSkipUs = 400000;      // a trim sent at a window's start is at the sensor by then
    static constexpr int kMinSamples = 10;
    static constexpr double kCoherentUs = 4000;      // capture phases' p90 - p10 within a window
    static constexpr double kResidUs = 1000;         // RMS off the line: a step inside the window
    static constexpr double kCentreAlpha = 0.3;
    static constexpr double kLockInUs = 1000, kLockOutUs = 3000;
    static constexpr int kLockInWindows = 3, kLockOutWindows = 2;
    static constexpr int kRangeWindows = 5;          // windows in a row whose neutral is out of reach
    static constexpr uint64_t kQuietUs = 3000000;
    static constexpr uint64_t kAcquireLimitUs = 60000000, kRetryUs = 60000000;
    static constexpr uint64_t kAllowanceUs = 10000000;

    // margin_us: ready this long before the deadline; cutoff_us: the deadline,
    // that long before the vblank (the renderer's commit margin and its own work).
    CameraSteer(double margin_us, double cutoff_us) : margin_(margin_us), cutoff_(cutoff_us) {}

    // One picture: captured at cap_us, ready to commit at ready_us; vb_us any
    // vblank of the screen's (its grid, of period Pm); k refreshes a camera frame.
    void on_frame(uint64_t cap_us, uint64_t ready_us, uint64_t vb_us, double Pm, int k, double fps) {
        if (Pm <= 0 || fps <= 0) return;
        // The stream started over (the air unit restarted): what was measured is gone.
        if (last_cap_ && (cap_us < last_cap_ || cap_us - last_cap_ > 500000)) {
            samples_.clear();
            lats_.clear();
        }
        last_cap_ = cap_us;
        Pm_ = Pm;
        k_ = std::max(1, k);
        fps_ = fps;
        if (ready_us > cap_us) {
            lats_.push_back({ cap_us, (double)(ready_us - cap_us) });
            while (!lats_.empty() && lats_.front().t + kAllowanceUs < cap_us) lats_.pop_front();
        }
        if (state_ == State::OutOfRange) return;
        double c = std::fmod((double)((int64_t)cap_us - (int64_t)vb_us), Pm);
        if (c < 0) c += Pm;
        // one continuous line within the window, across the refresh's seam
        if (!samples_.empty()) c -= Pm * std::round((c - samples_.front().c) / Pm);
        samples_.push_back({ cap_us, c });
    }

    // Closes the window when due and runs the states. True when the state or
    // the trim changed.
    bool tick(uint64_t now_us) {
        const State s0 = state_;
        const double t0 = trim_;
        if (!open_) {
            open_ = true;
            win_start_ = now_us;
            win_trim_ = trim_;
            last_window_ = now_us;
        }
        if (state_ == State::OutOfRange) {
            samples_.clear();
            if (now_us - oor_since_ >= kRetryUs) {
                range_count_ = 0;
                have_centre_ = false;
                enter(State::Waiting, now_us);
                win_start_ = now_us;
                win_trim_ = trim_;
            }
        } else if (now_us - win_start_ >= kWindowUs) {
            if ((int)samples_.size() >= kMinSamples) close_window(now_us);
            samples_.clear();
            win_start_ = now_us;
            win_trim_ = trim_;
        }
        if ((state_ == State::Acquiring || state_ == State::Locked) && now_us - last_window_ >= kQuietUs)
            enter(State::Waiting, now_us);       // the trim is held: the air unit keeps it
        if (state_ == State::Acquiring && now_us - acq_since_ >= kAcquireLimitUs) enter(State::OutOfRange, now_us);
        return state_ != s0 || trim_ != t0;
    }

    State state() const { return state_; }
    bool steering() const { return state_ == State::Acquiring || state_ == State::Locked || (state_ == State::Waiting && have_centre_); }
    double trim_us() const { return trim_; }
    double error_us() const { return err_; }
    double centre_us() const { return centre_; }
    double allowance_us() const { return allow_; }
    double neutral_us() const { return neutral_; }

private:
    struct Sample { uint64_t t; double c; };
    struct Lat { uint64_t t; double us; };

    void close_window(uint64_t now_us) {
        last_window_ = now_us;
        // The allowance: delivery's 99th percentile (as mabur's), the late GOP starts included.
        {
            std::vector<double> l;
            l.reserve(lats_.size());
            for (const Lat &x : lats_) l.push_back(x.us);
            if (!l.empty()) {
                const size_t i = std::min(l.size() - 1, (size_t)(0.99 * (double)l.size()));
                std::nth_element(l.begin(), l.begin() + i, l.end());
                allow_ = l[i];
            }
        }
        std::vector<double> cs;
        cs.reserve(samples_.size());
        for (const Sample &s : samples_) cs.push_back(s.c);
        std::sort(cs.begin(), cs.end());
        const size_t n = cs.size();
        const double D = Pm_ - cutoff_;                // ready by here after a vblank: up at the next
        double e = cs[n / 2] + allow_ + margin_ - D;
        e -= Pm_ * std::round(e / Pm_);
        err_ = e;

        // Captures that do not hold still inside a window (a camera far off the
        // screen's rate sweeps through it): no median worth acting on.
        if (cs[(9 * n) / 10] - cs[n / 10] > kCoherentUs) {
            if (state_ == State::Waiting) enter(State::Acquiring, now_us);
            else if (state_ == State::Acquiring) in_count_ = 0;
            else if (state_ == State::Locked && ++out_count_ >= kLockOutWindows) enter(State::Acquiring, now_us);
            update_centre(now_us);
            return;
        }
        update_centre(now_us);
        if (state_ == State::OutOfRange) return;
        const double ae = std::fabs(err_);
        switch (state_) {
        case State::Waiting: enter(State::Acquiring, now_us); break;
        case State::Acquiring:
            if (ae < kLockInUs) { if (++in_count_ >= kLockInWindows) enter(State::Locked, now_us); }
            else in_count_ = 0;
            break;
        case State::Locked:
            if (ae > kLockOutUs) { if (++out_count_ >= kLockOutWindows) enter(State::Acquiring, now_us); }
            else out_count_ = 0;
            break;
        case State::OutOfRange: break;
        }
        if (have_centre_ && (state_ == State::Acquiring || state_ == State::Locked)) {
            const double Pc = 1e6 / fps_;
            const double lim = (state_ == State::Locked ? 1.0 / 3 : 1.0) * 0.001 * Pc;
            // the error taken back in about two seconds
            const double corr = std::max(-lim, std::min(lim, 0.5 * err_ / fps_));
            trim_ = std::max(0.0, std::min(kHiUs, centre_ - corr));
        }
    }

    // The drift through the window, from the captures after the trim in force
    // reached the sensor: the trim with none.
    void update_centre(uint64_t now_us) {
        const uint64_t t0 = win_start_ + kSkipUs;
        double n = 0, st = 0, sc = 0, stt = 0, stc = 0;
        for (const Sample &s : samples_) {
            if (s.t < t0) continue;
            const double t = (double)(s.t - t0) / 1e6;
            n += 1; st += t; sc += s.c; stt += t * t; stc += t * s.c;
        }
        if (n < kMinSamples) return;
        const double den = n * stt - st * st;
        if (den <= 0) return;
        const double slope = (n * stc - st * sc) / den;    // us a second
        const double icpt = (sc - slope * st) / n;
        double rss = 0;
        for (const Sample &s : samples_) {
            if (s.t < t0) continue;
            const double t = (double)(s.t - t0) / 1e6;
            const double r = s.c - (icpt + slope * t);
            rss += r * r;
        }
        if (std::sqrt(rss / n) > kResidUs) return;
        neutral_ = win_trim_ - slope / fps_;
        // Out of reach: a camera the air unit cannot slow to the screen's rate
        // (or speed up to it - it only lengthens frames).
        if (neutral_ < -0.5 || neutral_ > kHiUs + 0.5) {
            if (++range_count_ >= kRangeWindows) enter(State::OutOfRange, now_us);
        } else {
            range_count_ = 0;
        }
        const double nc = std::max(0.0, std::min(kHiUs, neutral_));
        centre_ = have_centre_ ? centre_ + kCentreAlpha * (nc - centre_) : nc;
        have_centre_ = true;
    }

    void enter(State s, uint64_t now_us) {
        state_ = s;
        in_count_ = out_count_ = 0;
        if (s == State::Acquiring) acq_since_ = now_us;
        if (s == State::OutOfRange) {
            oor_since_ = now_us;
            trim_ = 0;
        }
    }

    const double margin_, cutoff_;
    State state_ = State::Waiting;
    std::vector<Sample> samples_;
    std::deque<Lat> lats_;
    uint64_t last_cap_ = 0;
    bool open_ = false, have_centre_ = false;
    uint64_t win_start_ = 0, last_window_ = 0, acq_since_ = 0, oor_since_ = 0;
    double win_trim_ = 0, trim_ = 0, centre_ = 0, err_ = 0, allow_ = 0, neutral_ = 0;
    double Pm_ = 16666.667, fps_ = 60;
    int k_ = 1;
    int in_count_ = 0, out_count_ = 0, range_count_ = 0;
};

}  // namespace display
