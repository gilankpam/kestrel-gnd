#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

class DrmDevice;

// DISPLAY > Refresh Sync: the screen's refresh held in step with the pictures,
// on the goggle alone. It watches when each picture reaches the renderer, finds
// the vblank position that makes the expected wait least (a picture past the
// commit deadline waits a whole refresh more), and moves the vblank there by
// making frames longer or shorter (RK3568 VOP2: DSP_VTOTAL_VS_END's
// undocumented immediate bit, from /dev/mem; a write holds until the next).
//
// Per screen ("display_lock_<screen ID>"), off unless turned on, as screens
// differ: the AW3225QF blanks on any change.
//  ON (1): one line at a time, with the refresh a whole multiple of the
//      camera's frame rate - the BOE1080 takes +1 held and +-1 alternating,
//      and blanked on +3 held and on long runs of shorter frames. One way
//      only (lock_step.hpp): longer against a faster screen, shorter against
//      a slower one - a "100 fps" camera is not 100.000 (the Lite+ runs
//      100.13, and 60.03 at 60), so a screen at the mode's own rate is the
//      slower, and there about every other frame is a line short.
//  FREE (2): the screen takes any frame length (the CPO1F41 shows 118 Hz, 100
//      Hz... as the total changes, and keeps its picture): each frame is made
//      as long as one camera frame (lines added to the screen's fastest mode,
//      which keeps its quick scan-out), and the phase is moved up to 20 lines
//      a frame either way.
class DisplayLock {
public:
    // cutoff_us: how long before a vblank a picture has to be there to go up
    // at it (the renderer's commit margin, and a little for its own work).
    DisplayLock(std::shared_ptr<DrmDevice> dev, double cutoff_us);
    ~DisplayLock();
    void start();
    void stop();
    // A picture's top (early) or all of it reached the renderer at t_us
    // (CLOCK_MONOTONIC). What counts is when it could go up: the whole
    // picture's time, or with its top here first, the later of the top's and
    // its last slice's going to the decoder - until then the renderer cannot
    // tell the rest will be decoded in time, and holds the top (early_safe).
    // With each half on its own plane (halves) the top goes up by itself.
    // cap_us: its capture stamp on this clock (0: not known), for the camera's
    // own frame period.
    void note_unit(uint64_t pts, bool early, uint64_t t_us, uint64_t last_in_us, bool halves, uint64_t cap_us);
    // Holding the vblank to the pictures now: the renderer may then put each
    // half up on its own with no spare refresh (SPLIT at one refresh a picture).
    bool in_step() const { return in_step_.load(std::memory_order_relaxed); }
    // The camera's frame period on this clock, us, from the last 10 s of
    // capture stamps; 0 until known.
    double camera_period_us();

private:
    void run();
    bool map_register();
    bool usable(int lock, double Pm, double Pc, const char **why);
    // vtotal + d, immediate (0 = the mode's own), if still early in the frame
    // that started at vb_us; false if not written.
    bool write_lines(int d, uint64_t vb_us, double P);
    void release();                   // back to the mode's own total, immediate bit clear
    double target_offset(double P, double G, double *wait_now, double *late_now);

    std::shared_ptr<DrmDevice> dev_;
    const double cutoff_us_;
    std::thread th_;
    std::atomic<bool> run_{false};
    std::atomic<bool> in_step_{false};

    std::mutex m_;                    // arrivals_in_
    std::deque<uint64_t> arrivals_in_;
    std::deque<uint64_t> caps_;       // capture stamps, the last 10 s, of one frame rate
    int caps_mode_ = 0;               // (lines << 16) | fps they are of
    uint64_t last_pts_ = UINT64_MAX, early_pts_ = UINT64_MAX, early_t_ = 0;

    volatile uint32_t *reg_ = nullptr;
    void *map_ = nullptr;
    uint32_t orig_ = 0;               // the register as the mode has it, immediate bit clear
    uint32_t mode_vtotal_ = 0, mode_clock_ = 0;
    int d_ = 0;                       // lines in the register now (0: the mode's own)
    bool released_ = true;

    // Each arrival's phase in the refresh, against a fixed reference: its
    // phase after the vblank before it, plus the shift applied so far (G).
    struct Sample { uint64_t t_us; double u; };
    std::deque<Sample> win_;
};
