// Host test of lock_step.hpp: DISPLAY > Refresh Sync ON's step, a
// line at a time, against a screen a little faster and a little slower than
// the camera - the vblank's place in the refresh simulated frame by frame.
//   g++ -std=c++17 -I. -o display_lock_test tools/test/display_lock_test.cpp && ./display_lock_test
#include <cmath>
#include <cstdio>
#include "lock_step.hpp"

static int fails;
#define CHECK(c) do { if (!(c)) { fails++; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static double wrap(double x, double P) {     // into [-P/2, P/2), as display_lock.cpp's
    x = std::fmod(x, P);
    if (x >= P / 2) x -= P;
    if (x < -P / 2) x += P;
    return x;
}
static double mod(double x, double P) { x = std::fmod(x, P); return x < 0 ? x + P : x; }

struct Run {
    int min_lines = 0, max_lines = 0;   // the frame lengths it used, over the mode's
    double worst_us = 0;                // the largest error once in step, us
    int arrived = 0;                    // frames it reached the target on
};

// A 1080p60 mode (1125 lines) against a camera frame of Pc, for n frames,
// starting a third of a refresh off. Each frame the pictures land Pm - Pc
// earlier in the refresh (later, with the screen the faster); the vblank
// moves by the lines the step asked for.
static Run run(double Pc, bool shorter, int n) {
    const double Pm = 16666.667, line = Pm / 1125;
    double target = Pm / 3, G = 0;
    bool moving = false, in_step = false;
    Run r;
    for (int f = 0; f < n; f++) {
        target = mod(target - (Pm - Pc), Pm);
        const display::LineStep s = display::line_step(wrap(target - G, Pm), Pm, line, shorter, moving);
        moving = s.moving;
        if (s.arrived) { r.arrived++; in_step = true; }
        if (s.lines < r.min_lines) r.min_lines = s.lines;
        if (s.lines > r.max_lines) r.max_lines = s.lines;
        G = mod(G + s.lines * line, Pm);
        if (in_step) r.worst_us = std::fmax(r.worst_us, std::fabs(wrap(target - G, Pm)));
    }
    return r;
}

// The screen the faster (a 60.000 Hz mode, a 59.988 fps camera): longer frames
// only, as before, and held within a few lines.
static void test_faster_screen() {
    const Run r = run(16670.0, false, 20000);
    CHECK(r.arrived >= 1);
    CHECK(r.min_lines == 0 && r.max_lines == 1);
    CHECK(r.worst_us < 4 * 16666.667 / 1125);
}

// The screen the slower (60.000 Hz against the Lite+'s 60.03 fps, measured on
// an Ascent VRX Pro): longer frames only never hold it - the vblank goes round
// a whole refresh again and again.
static void test_slower_screen_longer_only() {
    const Run r = run(16658.1, false, 20000);
    CHECK(r.worst_us > 16666.667 / 4);
}

// ... and shorter frames only hold it, within a few lines, a line short on
// about every other frame.
static void test_slower_screen_shorter() {
    const Run r = run(16658.1, true, 20000);
    CHECK(r.arrived >= 1);
    CHECK(r.min_lines == -1 && r.max_lines == 0);
    CHECK(r.worst_us < 4 * 16666.667 / 1125);
}

// A vblank a little on the drift's side waits for the drift; one a little the
// other way goes round, the one way lines can move it.
static void test_step_direction() {
    const double P = 16666.667, line = P / 1125;
    // e is the target less the vblank: > 0, the vblank early. Longer only: a
    // little late waits, early moves it later, far too late goes round.
    CHECK(display::line_step(-10 * line, P, line, false, false).lines == 0);
    CHECK(display::line_step(10 * line, P, line, false, false).lines == 1);
    CHECK(display::line_step(-P / 3, P, line, false, false).lines == 1);
    // Shorter only, the mirror: a little early waits, late moves it earlier,
    // far too early goes round.
    CHECK(display::line_step(10 * line, P, line, true, false).lines == 0);
    CHECK(display::line_step(-10 * line, P, line, true, false).lines == -1);
    CHECK(display::line_step(P / 3, P, line, true, false).lines == -1);
    // Within half a line while moving: there.
    const display::LineStep s = display::line_step(-0.2 * line, P, line, true, true);
    CHECK(s.lines == 0 && !s.moving && s.arrived);
}

int main() {
    test_faster_screen();
    test_slower_screen_longer_only();
    test_slower_screen_shorter();
    test_step_direction();
    std::printf(fails ? "display_lock_test: %d FAILED\n" : "display_lock_test: all passed\n", fails);
    return fails ? 1 : 0;
}
