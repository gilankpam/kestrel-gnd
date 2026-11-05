// Host test of camera_steer.hpp: DISPLAY > Refresh Sync CAMERA's controller,
// closed loop against a simulated air unit and screen - the camera's frames
// stamped at capture, arriving after a jittery delivery (every 17th picture a
// GOP start, later still), shown at the first vblank whose commit deadline
// they make, and overtaken when two make the same one.
//   g++ -std=c++17 -O2 -I. -o camera_steer_test tools/test/camera_steer_test.cpp && ./camera_steer_test
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "camera_steer.hpp"

static int fails;
#define CHECK(c) do { if (!(c)) { fails++; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

// Deterministic normal noise.
struct Rng {
    uint64_t s = 88172645463325252ull;
    double u() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (double)(s >> 11) / 9007199254740992.0; }
    double n() { return std::sqrt(-2 * std::log(u() + 1e-12)) * std::cos(6.283185307179586 * u()); }
};

struct Sim {
    double Pm = 1e6 / 60.0;           // the screen's refresh, untouched
    double Pc0 = 16658.2;             // the camera's own frame (a Lite+ "60 fps": 60.03)
    double line = 7.673;              // one sensor line, us
    double lat0 = 20000;              // capture -> ready to commit, us
    double jitter = 1000;             // its spread (sd), us
    double gop_late = 6000;           // a GOP start's extra (its bulk is one late slice)
    double cutoff = 1300;             // commit margin before the vblank
    double air_lag = 350000;          // a trim reaches the sensor this much later
    double start_phase = 7000;        // where the first capture falls in the refresh
    double margin = 1000;             // ready this long before the deadline (DISPLAY > Sync Margin)
    bool trace = false;               // each window's figures
    double backlog = 0;
    double switch_at = -1, Pm2 = 0, Pc02 = 0, line2 = 0;   // a mode change: the screen and the camera, at switch_at s               // link-up: the first second's pictures this much later, us, falling to none
    // results, from `from_us` on
    int shown = 0, frames = 0, overtaken = 0;
    double trim_min = 1e9, trim_max = -1e9;
    uint64_t locked_at = 0;
};

// n seconds; latency steps by `step_us` at `step_at_s` (a bitrate change).
static void run(Sim &s, double seconds, double from_s, double step_at_s = -1, double step_us = 0) {
    Rng rng;
    display::CameraSteer cs(s.margin, s.cutoff);
    double cap = s.start_phase, err_lines = 0;
    double applied = 0;               // trim in force at the sensor, us a frame
    struct Pending { double at; double trim; } pend[64];
    int np = 0;
    long last_vb = -1;                // the vblank index the last picture went up at
    const uint64_t from = (uint64_t)(from_s * 1e6);
    for (int i = 0; cap < seconds * 1e6; i++) {
        if (s.switch_at >= 0 && cap >= s.switch_at * 1e6 && s.Pm != s.Pm2) {
            s.Pm = s.Pm2; s.Pc0 = s.Pc02; s.line = s.line2;
            cap += 300000;            // the camera restarts in its new mode
        }
        // the air unit: the trim asked for arrives air_lag later, dithered into whole lines
        while (np && pend[0].at <= cap) {
            applied = pend[0].trim;
            for (int j = 1; j < np; j++) pend[j - 1] = pend[j];
            np--;
        }
        const double want = std::max(0.0, std::min(8.0, applied / s.line));
        const int lines = (int)std::floor(want + err_lines + 0.5);
        err_lines += want - lines;
        cap += s.Pc0 + lines * s.line;
        double lat = s.lat0 + s.jitter * std::fabs(rng.n()) + (i % 17 == 0 ? s.gop_late : 0);
        if (step_at_s >= 0 && cap >= step_at_s * 1e6) lat += step_us;
        if (cap < 1e6) lat += s.backlog * (1 - cap / 1e6);
        const double ready = cap + lat;
        // the vblank grid: vblank k at k * Pm; the vblank before `ready`
        const long kb = (long)std::floor(ready / s.Pm);
        const uint64_t vb = (uint64_t)(kb * s.Pm);
        cs.on_frame((uint64_t)cap, (uint64_t)ready, vb, s.Pm, 1, 1e6 / s.Pc0);
        const bool ch = cs.tick((uint64_t)ready);
        if (ch && np < 64) pend[np++] = { ready + s.air_lag, cs.trim_us() };
        if (s.trace && ch)
            std::printf("  %5.1f s %d err %+6.0f trim %5.1f centre %5.1f neutral %5.1f allow %5.1f ms\n", cap / 1e6,
                        (int)cs.state(), cs.error_us(), cs.trim_us(), cs.centre_us(), cs.neutral_us(), cs.allowance_us() / 1000);
        // shown at the first vblank it makes the deadline for
        const long kshow = ready <= (kb + 1) * s.Pm - s.cutoff ? kb + 1 : kb + 2;
        if ((uint64_t)cap >= from) {
            s.frames++;
            if (kshow == last_vb) s.overtaken++;   // the one before it is never shown
            else s.shown++;
            s.trim_min = std::min(s.trim_min, cs.trim_us());
            s.trim_max = std::max(s.trim_max, cs.trim_us());
        }
        last_vb = kshow;
        if (!s.locked_at && cs.state() == display::CameraSteer::State::Locked) s.locked_at = (uint64_t)cap;
    }
}

int main() {
    // The bench case: 60.03 fps against 60.000 Hz, GOP starts 6 ms late (20 Mbps).
    // The neutral trim is 16666.7 - 16658.2 = +8.5 us a frame.
    {
        Sim s;
        run(s, 120, 30);
        std::printf("bench: locked at %.1f s, %d of %d overtaken, trim %.1f..%.1f us\n", s.locked_at / 1e6,
                    s.overtaken, s.frames, s.trim_min, s.trim_max);
        CHECK(s.locked_at > 0 && s.locked_at < 20000000);
        // under 0.2% once locked: with 1 ms of margin, the tail past delivery's
        // 99th percentile (late GOP starts at their slowest) still misses now and then
        CHECK(s.overtaken <= s.frames / 500);
        CHECK(s.trim_min > 2 && s.trim_max < 16);     // around +8.5, never wound up
    }
    // Starting half a refresh off.
    {
        Sim s;
        s.start_phase = 15000;
        run(s, 120, 40);
        std::printf("half off: locked at %.1f s, %d of %d overtaken\n", s.locked_at / 1e6, s.overtaken, s.frames);
        CHECK(s.locked_at > 0 && s.locked_at < 30000000);
        CHECK(s.overtaken <= s.frames / 500);
    }
    // The delivery 5 ms slower at 60 s (the bitrate raised): back in step
    // within 20 s, and clean after.
    {
        Sim s;
        run(s, 140, 80, 60, 5000);
        std::printf("latency step: %d of %d overtaken after 80 s, trim %.1f..%.1f us\n", s.overtaken, s.frames,
                    s.trim_min, s.trim_max);
        CHECK(s.overtaken <= s.frames / 500);
        CHECK(s.trim_min > 2 && s.trim_max < 16);
    }
    // The bench's Sync Margin, 4 ms: that tail is covered too.
    {
        Sim s;
        s.margin = 4000;
        run(s, 120, 30);
        std::printf("4 ms margin: locked at %.1f s, %d of %d overtaken\n", s.locked_at / 1e6, s.overtaken, s.frames);
        CHECK(s.locked_at > 0 && s.locked_at < 20000000);
        CHECK(s.overtaken <= 1);
    }
    // Startup as on the bench: the link-up backlog (pictures 130 ms late at
    // first) and the 4 ms margin - locked within 8 s from either side: the
    // pictures needing to come later (fast: frames can be much longer), or
    // earlier (slow that way: only as fast as the camera's own lead, 8.5 us a
    // frame - so the long way round, later, when that is quicker).
    for (double start : { 7000.0, 11000.0, 15000.0, 3000.0 }) {
        Sim s;
        s.margin = 4000;
        s.backlog = 130000;
        s.start_phase = start;
        run(s, 60, 20);
        std::printf("startup from %.0f us: locked at %.1f s, %d of %d overtaken after 20 s\n", start,
                    s.locked_at / 1e6, s.overtaken, s.frames);
        CHECK(s.locked_at > 0 && s.locked_at < 8000000);
        CHECK(s.overtaken <= 1);
    }
    // 1080p100 on the goggle X: a 10 ms refresh, the Lite+ at 100.128 fps
    // (+12.8 us a frame to match), delivery spread ~5 ms, the bench's 4 ms
    // margin. Delivery spread plus margin no longer fit in a refresh: placed
    // by the late side alone, the early pictures land in the slot before and
    // overtake the one waiting there. The spread has to sit inside the slot.
    {
        Sim s;
        s.Pm = 10000;
        s.Pc0 = 9987.2;
        s.air_lag = 120000;           // sent at once, 5 frames of 10 ms: much less than at 60
        s.cutoff = 1300;
        s.jitter = 2000;
        s.gop_late = 0;               // GOP 340: GOP starts rare
        s.margin = 4000;
        s.backlog = 130000;
        run(s, 90, 30);
        std::printf("100 fps: locked at %.1f s, %d of %d overtaken, trim %.1f..%.1f us\n", s.locked_at / 1e6,
                    s.overtaken, s.frames, s.trim_min, s.trim_max);
        CHECK(s.locked_at > 0 && s.locked_at < 10000000);
        CHECK(s.overtaken <= s.frames / 500);
    }
    // 100 fps from any phase, with the link-up backlog: the full reach moves
    // the phase ~5.7 ms a window, over half the 10 ms refresh - the error
    // followed window to window must not jump a refresh the wrong way (it ran
    // away to -213 ms on the bench and gave up as out of range).
    for (double start : { 1000.0, 3000.0, 5000.0, 7000.0, 9000.0 }) {
        Sim s;
        s.Pm = 10000;
        s.Pc0 = 9987.2;
        s.air_lag = 120000;           // sent at once, 5 frames of 10 ms: much less than at 60
        s.jitter = 2000;
        s.gop_late = 0;
        s.margin = 4000;
        s.backlog = 130000;
        s.start_phase = start;
        run(s, 60, 20);
        std::printf("100 fps from %.0f us: locked at %.1f s, %d of %d overtaken after 20 s, trim %.1f..%.1f us\n",
                    start, s.locked_at / 1e6, s.overtaken, s.frames, s.trim_min, s.trim_max);
        CHECK(s.locked_at > 0 && s.locked_at < 10000000);
        CHECK(s.overtaken <= s.frames / 500);
        CHECK(s.trim_max < 30);
    }
    // The bench's sequence: kestrel-gnd comes up at 1080p60 on 60 Hz and
    // locks, then the camera goes to 100 fps and the screen to 100 Hz. What
    // was learnt at 60 must not carry over (it ran away to -213 ms there).
    for (double start : { 2000.0, 7000.0, 12000.0 }) {
        Sim s;
        s.start_phase = start;
        s.margin = 4000;
        s.backlog = 130000;
        s.switch_at = 20; s.Pm2 = 10000; s.Pc02 = 9987.2; s.line2 = 7.673;
        s.air_lag = 120000;
        s.jitter = 1500;
        s.gop_late = 0;
        run(s, 70, 40);
        std::printf("60 -> 100 fps from %.0f us: %d of %d overtaken after 40 s, trim %.1f..%.1f us\n", start,
                    s.overtaken, s.frames, s.trim_min, s.trim_max);
        CHECK(s.overtaken <= s.frames / 500);
        CHECK(s.trim_max < 30);
    }
    // 100 fps, locked, then delivery 7 ms faster (a bitrate dropped): a big
    // correction at the whole reach moves the phase over half the refresh a
    // window - each window's line and its error must still hold together.
    for (double step : { -7000.0, 7000.0 }) {
        Sim s;
        s.Pm = 10000;
        s.Pc0 = 9987.2;
        s.air_lag = 120000;           // sent at once, 5 frames of 10 ms: much less than at 60
        s.jitter = 1500;
        s.gop_late = 0;
        s.margin = 4000;
        run(s, 90, 50, 30, step);
        std::printf("100 fps, delivery %+.0f us at 30 s: %d of %d overtaken after 50 s, trim %.1f..%.1f us\n", step,
                    s.overtaken, s.frames, s.trim_min, s.trim_max);
        CHECK(s.overtaken <= s.frames / 500);
        CHECK(s.trim_max < 30);
    }
    // A camera slower than the screen by more than the air unit can take back
    // (it only makes frames longer): never locks, and asks for nothing faster.
    {
        Sim s;
        s.Pc0 = 16700;
        run(s, 90, 0);
        std::printf("unreachable: locked at %.1f s, trim %.1f..%.1f us\n", s.locked_at / 1e6, s.trim_min, s.trim_max);
        CHECK(s.locked_at == 0);
        CHECK(s.trim_min >= 0);
    }
    std::printf(fails ? "camera_steer_test: %d FAILED\n" : "camera_steer_test: all passed\n", fails);
    return fails != 0;
}
