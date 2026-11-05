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
// Each window (1.2 s): a least-squares line through where the captures fall in
// the refresh gives the drift, which names the trim that would match the
// screen (the camera's frame is its own plus the trim, so trim - drift a frame
// does); `centre` is an average of that (the rate). The error is where that
// line will be when a trim sent now reaches the sensor, plus the allowance and
// the margin, against the deadline - wrapped into +-P/2, or followed on the way
// round chosen while acquiring (the quicker one: frames only get longer). The
// trim asked for is centre less the error spread over 1.5 s (the phase): the
// whole reach while acquiring, ~0.03% of a frame a frame once locked. The air unit only makes frames longer (shorter
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
    static constexpr double kTakeBackS = 1.5;      // the error's correction spread over this long
    static constexpr double kLockInUs = 1000, kLockOutUs = 3000;
    static constexpr int kLockInWindows = 2, kLockOutWindows = 2;
    static constexpr int kRangeWindows = 5;          // windows in a row whose neutral is out of reach
    static constexpr uint64_t kQuietUs = 3000000;
    static constexpr uint64_t kAcquireLimitUs = 60000000, kRetryUs = 60000000;
    static constexpr uint64_t kAllowanceUs = 10000000;
    static constexpr uint64_t kLinkUpUs = 2000000;   // a stream's first deliveries: the link-up's backlog

    // margin_us: ready this long before the deadline; cutoff_us: the deadline,
    // that long before the vblank (the renderer's commit margin and its own work).
    CameraSteer(double margin_us, double cutoff_us) : margin_(margin_us), cutoff_(cutoff_us) {}

    // One picture: captured at cap_us, ready to commit at ready_us; vb_us any
    // vblank of the screen's (its grid, of period Pm); k refreshes a camera frame.
    void on_frame(uint64_t cap_us, uint64_t ready_us, uint64_t vb_us, double Pm, int k, double fps) {
        if (Pm <= 0 || fps <= 0) return;
        // The stream started over (the air unit restarted): what was measured is gone.
        if (!last_cap_ || cap_us < last_cap_ || cap_us - last_cap_ > 500000) {
            samples_.clear();
            lats_.clear();
            stream_start_ = cap_us;
        }
        last_cap_ = cap_us;
        Pm_ = Pm;
        k_ = std::max(1, k);
        fps_ = fps;
        if (ready_us > cap_us && cap_us >= stream_start_ + kLinkUpUs) {
            lats_.push_back({ cap_us, (double)(ready_us - cap_us) });
            while (!lats_.empty() && lats_.front().t + kAllowanceUs < cap_us) lats_.pop_front();
        }
        if (state_ == State::OutOfRange) return;
        double c = std::fmod((double)((int64_t)cap_us - (int64_t)vb_us), Pm);
        if (c < 0) c += Pm;
        // one continuous line within the window, across the refresh's seam:
        // against the capture before (a frame apart they move by tens of us;
        // against the window's first, a window steered at the whole reach -
        // 5.6 ms at 100 fps, over half the refresh - put its later captures a
        // refresh off, and the line through them was thrown out)
        if (!samples_.empty()) c -= Pm * std::round((c - samples_.back().c) / Pm);
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
    double fast_us() const { return fast_; }
    double guard_us() const { return guard_; }
    double neutral_us() const { return neutral_; }

private:
    struct Sample { uint64_t t; double c; };
    struct Lat { uint64_t t; double us; };

    void close_window(uint64_t now_us) {
        last_window_ = now_us;
        // The allowance: delivery's 99th percentile (as mabur's), the late GOP
        // starts included - of the deliveries within two frames of the median.
        // Slower ones are a link-up's backlog (130 ms at first, falling) or a
        // stall: they miss their refresh whatever is planned, and planning for
        // them would delay every other picture as much.
        {
            std::vector<double> l;
            l.reserve(lats_.size());
            for (const Lat &x : lats_) l.push_back(x.us);
            if (!l.empty()) {
                std::nth_element(l.begin(), l.begin() + l.size() / 2, l.end());
                const double cap = l[l.size() / 2] + 2e6 / fps_;
                l.erase(std::remove_if(l.begin(), l.end(), [&](double v) { return v > cap; }), l.end());
                const size_t i = std::min(l.size() - 1, (size_t)(0.99 * (double)l.size()));
                std::nth_element(l.begin(), l.begin() + i, l.end());
                allow_ = l[i];
                const size_t j = (size_t)(0.01 * (double)l.size());
                std::nth_element(l.begin(), l.begin() + j, l.end());
                fast_ = l[j];
            }
        }
        std::vector<double> cs;
        cs.reserve(samples_.size());
        for (const Sample &s : samples_) cs.push_back(s.c);
        std::sort(cs.begin(), cs.end());
        const size_t n = cs.size();
        update_centre(now_us);
        if (state_ == State::OutOfRange) return;
        // No delivery measured yet (a stream's first seconds): nothing to aim at.
        if (lats_.size() < (size_t)kMinSamples) return;
        const double D = Pm_ - cutoff_;                // ready by here after a vblank: up at the next
        // Where the captures will be when a trim sent now is at the sensor:
        // the window's line carried on (its median, without one).
        const double c = fit_ok_ ? fit_icpt_ + fit_slope_ * ((double)(now_us + kSkipUs - fit_t0_) / 1e6) : cs[n / 2];
        // The whole spread of delivery has to fit between two deadlines: a
        // picture ready before the one before its own goes up a refresh early,
        // over the one still waiting there. So the guard before the deadline
        // is the margin where there is room for it, else half of what the
        // spread leaves of the slot (100 fps: a 10 ms slot, ~5 ms of spread,
        // and the 4 ms margin did not fit - the fast pictures overtook).
        const double slot = k_ * Pm_;
        guard_ = std::min(margin_, std::max(0.0, (slot - (allow_ - fast_)) / 2));
        double e = c + allow_ + guard_ - D;
        e -= Pm_ * std::round(e / Pm_);
        if (state_ == State::Acquiring && have_route_) {
            // the way chosen, followed on: continuous from the last window's
            // error carried on by the drift since (by its own, the error
            // moving over half a refresh between windows was taken a refresh
            // the wrong way, every window: -213 ms at 100 fps)
            const double drift = fit_ok_ ? fit_slope_ : (win_trim_ - centre_) * fps_;   // us a second
            const double expect = err_ + drift * (double)(now_us - err_t_) / 1e6;
            e -= Pm_ * std::round((e - expect) / Pm_);
        } else if (have_centre_) {
            // The quicker way round. Frames only get longer, so the pictures
            // come earlier at most as fast as the camera's own lead over the
            // screen (centre: 8.5 us a frame at 60.03 fps on 60.000 Hz), later
            // up to the rest of the range: a long way later can beat a short
            // way earlier.
            const double Pc = 1e6 / fps_, lim = acquire_limit(Pc);
            const double early = std::max(0.5, std::min(lim, centre_));
            const double late = std::max(0.5, std::min(lim, kHiUs - centre_));
            if (e > 0 && (Pm_ - e) / late < e / early) e -= Pm_;
            else if (e < 0 && (Pm_ + e) / early < -e / late) e += Pm_;
            have_route_ = state_ == State::Acquiring || state_ == State::Waiting;
        }
        err_ = e;
        err_t_ = now_us;

        // Captures that do not hold still inside a window (a camera far off the
        // screen's rate sweeps through it): no median worth acting on.
        if (cs[(9 * n) / 10] - cs[n / 10] > kCoherentUs) {
            if (state_ == State::Waiting) enter(State::Acquiring, now_us);
            else if (state_ == State::Acquiring) in_count_ = 0;
            else if (state_ == State::Locked && ++out_count_ >= kLockOutWindows) enter(State::Acquiring, now_us);
            return;
        }
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
            const double lim = state_ == State::Locked ? 0.001 * Pc / 3 : acquire_limit(Pc);
            // the error taken back over kTakeBackS - and never so fast that a
            // window moves over 40% of the slot, whatever the reach
            const double cap = 0.4 * k_ * Pm_ / (fps_ * (double)kWindowUs / 1e6);
            const double corr = std::max(-std::min(lim, cap), std::min(std::min(lim, cap), err_ / (kTakeBackS * fps_)));
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
        fit_ok_ = false;
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
        fit_ok_ = true;
        fit_t0_ = t0;
        fit_icpt_ = icpt;
        fit_slope_ = slope;
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

    // How far a window's correction may go while acquiring, us a frame.
    // The rate is measured to well under a us a frame, so the whole reach.
    static double acquire_limit(double) { return kHiUs; }

    void enter(State s, uint64_t now_us) {
        state_ = s;
        in_count_ = out_count_ = 0;
        if (s != State::Acquiring) have_route_ = false;
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
    uint64_t last_cap_ = 0, stream_start_ = 0;
    bool open_ = false, have_centre_ = false, have_route_ = false;
    uint64_t win_start_ = 0, last_window_ = 0, acq_since_ = 0, oor_since_ = 0, err_t_ = 0;
    double win_trim_ = 0, trim_ = 0, centre_ = 0, err_ = 0, allow_ = 0, neutral_ = 0;
    double fast_ = 0, guard_ = 0;                    // delivery's 1st percentile; the guard before the deadline
    double Pm_ = 16666.667, fps_ = 60;
    bool fit_ok_ = false;                            // the last window's line, through its captures
    uint64_t fit_t0_ = 0;
    double fit_icpt_ = 0, fit_slope_ = 0;
    int k_ = 1;
    int in_count_ = 0, out_count_ = 0, range_count_ = 0;
};

}  // namespace display
