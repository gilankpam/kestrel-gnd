#pragma once
// DISPLAY > Refresh Sync ON's step (DisplayLock::run), apart from the lock so
// it can be tested without a display (tools/test/display_lock_test.cpp).

namespace display {

struct LineStep {
    int lines;      // this frame's lines over the mode's: -1, 0 or +1
    bool moving;    // carried to the next frame
    bool arrived;   // reached the target this frame
};

// One frame of ON: a line at a time, and one way only - frames made longer
// where the screen is the faster than the camera, shorter where it is the
// slower (`shorter`). Either way the drift between them brings the vblank
// back the other way, so it never has to move against it. e: the target less
// where the vblank is, us, wrapped into [-P/2, P/2) (> 0: the vblank early);
// P the refresh, line one line's time; moving as the last step left it.
// A vblank a little on the drift's side waits for the drift; one far the
// other way goes round, up to a refresh, the one way lines can move it.
inline LineStep line_step(double e, double P, double line, bool shorter, bool moving) {
    const int s = shorter ? -1 : 1;
    double x = s * e;                 // how far to go, the way lines move it
    if (x < -P / 4) x += P;
    LineStep r{0, moving, false};
    if (!r.moving && x > 3 * line) r.moving = true;
    if (r.moving && x < 0.5 * line) { r.moving = false; r.arrived = true; }
    r.lines = r.moving && x > 0 ? s : 0;
    return r;
}

}  // namespace display
