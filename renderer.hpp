#ifndef RENDERER_H  // Check if VDEC_H is not defined
#define RENDERER_H  // Define VDEC_H

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/pixfmt.h>
    #include <libavutil/hwcontext_drm.h>
    #include <libavutil/pixdesc.h>
}

#include <memory>
#include <atomic>
#include <mutex>
#include <unistd.h>

#include "dvr.hpp"
#include "osd.hpp"
#include "drm.hpp"
#include "display_lock.hpp"
#include "utils/scheduling_helper.hpp"
#include "utils/time_util.h"
#include "utils/slice_times.hpp"
#include <functional>
#include "queue/Queue.hpp"

#define MAX_FRAMES 24		// min 16 and 20+ recommended (mpp/readme.txt)
#define READ_BUF_SIZE (1024*1024) // SZ_1M https://github.com/rockchip-linux/mpp/blob/ed377c99a733e2cdbcc457a6aa3f0fcd438a9dff/osal/inc/mpp_common.h#L179

enum RenderMode {
    Disable,     // No rendering (benchmarking only)
    Atomic,      // V-Sync enabled (tear-free)
    FrontBuffer  // No V-Sync (lowest latency, tearing possible)
};

// Structure to hold info about displayable DRM framebuffers
struct DisplayBufferInfo {
    uint32_t drm_fb_id;        // Framebuffer ID from drmModeAddFB2

    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;

    // Optional: for HW buffers:
    int prime_fd;              // The original PRIME FD (for tracking/cleanup)
    uint32_t gem_handle;       // GEM handle for this buffer

    // Optional: for SW buffers:
    uint8_t* sw_base = nullptr; // Y base
    uint32_t sw_u_offset = 0;   // Offset to U plane (for YUV420)
    uint32_t sw_v_offset = 0;   // Offset to V plane (for YUV420)
    uint32_t sw_uv_offset = 0;  // Offset to UV plane (for NV12)
    
    // Format helper
    uint32_t format = 0; 

    bool in_use = false;

    DisplayBufferInfo() : drm_fb_id(0), prime_fd(-1), gem_handle(0), sw_base(nullptr), width(0), height(0), stride(0), sw_u_offset(0), sw_v_offset(0), sw_uv_offset(0), format(0) {}
};

struct DrmBuffer {
    uint32_t handle;
    uint32_t fb_id;
    int prime_fd;
    uint32_t size;
    uint32_t stride;   // driver-aligned pitch returned by CREATE_DUMB (amdgpu needs this, not `width`)
};

struct DecodedUnit{
    uint64_t pts;
    uint32_t width;
    uint32_t height;
    uint64_t recv_ts;
    uint64_t dec_start_ts;
    uint64_t dec_end_ts;
    uint32_t tx_encode_delay_us;
    uint32_t tx_processing_delay_us;
    bool is_keyframe = false;

    // AVFrame
    // AVFrame *av_frame_ref;
    std::shared_ptr<void> frame_ref; // Holds ownership of the allocated frame (if any) with custom deleter
    uint32_t drm_pixel_format = 0;
    uint32_t handles[4] = {0};
    uint32_t pitches[4] = {0};
    uint32_t offsets[4] = {0};
    uint64_t modifiers[4] = {0};

    // HW prime data
    bool has_prime_fd;
    int prime_fd;
    // The decoder's buffer behind prime_fd, and how to keep it from being
    // decoded into again: hold(buf) takes a reference, released when the
    // returned pointer goes. Nothing is held by default; the renderer holds
    // the pictures it shows while the screen is being recorded, since the
    // recorder reads them after the display does.
    void* buf = nullptr;
    std::shared_ptr<void> (*hold)(void* buf) = nullptr;
    // Bumped each time the decoder replaces its buffers: prime_fd numbers
    // are only unique within one epoch.
    uint32_t buf_epoch = 0;

    // A streamed picture shown early: only its first slice(s) are decoded yet,
    // the rest is on its way into the same buffer. bottom_eta_us says when the
    // rest should be decoded (CLOCK_MONOTONIC us, 0 = unknown), kept current by
    // the decoder; the renderer shows it only where the scan-out reaches the
    // missing part after that. The whole picture follows as usual, same pts.
    bool early = false;
    std::shared_ptr<std::atomic<uint64_t>> bottom_eta_us;
    // Each slice's own timing (utils/slice_times.hpp), where the source has it.
    SliceTimesPtr slices;

    // SW data
    uint8_t *data[8];
    int linesize[8];
};

struct Stats{
    // Frames sampled per stats window (osd_refresh, 1 s by default). Every
    // array below is this long and update_stats() stops sampling at it. The
    // guard there used to say 500 while the arrays held 200, so any window
    // with more than 200 frames - a decoder draining its backlog after a
    // stall, an air unit dumping its start-up buffer, a broken stream -
    // wrote floats past the end of the Renderer into the next heap block,
    // and kestrel died somewhere else later (a SIGSEGV in the renderer
    // queue's tryGet(), and a jump into the heap with no backtrace).
    static constexpr int kMaxFrames = 512;
    int frame_counter = 0;
    uint64_t decoding_stats_start = 0;
    uint64_t last_frame_ts = 0;

    float proc_latency_avg[kMaxFrames] = {0};
    float proc_latency_min = 1e18f; // Use a large but float-friendly value
    float proc_latency_max = 0;

    float total_latency_avg[kMaxFrames] = {0};
    float total_latency_min = 1e18f;
    float total_latency_max = 0;

    float decoding_latency_avg[kMaxFrames] = {0};
    float decoding_latency_min = 1e18f;
    float decoding_latency_max = 0;

    float display_latency_avg[kMaxFrames] = {0};
    float display_latency_min = 1e18f;
    float display_latency_max = 0;

    float tx_latency_avg[kMaxFrames] = {0};
    float tx_latency_min = 1e18f;
    float tx_latency_max = 0;


    float frame_pace_avg[kMaxFrames] = {0};
    float frame_pace_min = 1e18f;
    float frame_pace_max = 0;
};

class Renderer {
    private:
        std::shared_ptr<DrmDevice> dev;
        std::shared_ptr<DVR> dvr;
        std::shared_ptr<OSD> osd;

        volatile bool *should_stop;
        Queue* decoded_unit_queue;

        RenderMode render_mode = Disable;
        
        bool buffer_initialized= false;
        // A keyframe was decoded but replaced by a newer picture before it was
        // shown: the picture shown next is as clean, and stands in for it.
        bool keyframe_pending_ = false;
        // Early presentation (DecodedUnit::early): the picture shown early, to
        // take its whole one without a second flip, and the counts of the log.
        uint64_t early_shown_pts_ = UINT64_MAX;
        uint64_t early_boundary_us_ = 0;   // when the scan-out reached its missing part
        uint64_t early_held_pts_ = UINT64_MAX;   // counted once per picture held
        // /tmp/kestrel-early.conf's third number: the spare time asked for
        // between the bottom half's estimated decode and the scan-out, us
        int early_margin_us_ = 1000;
        unsigned early_flips_ = 0, early_late_ = 0, early_held_ = 0, early_adopted_ = 0;
        // The commit halves_commit is making: in halves at this row, with this
        // picture's lower part on the second plane (DrmDevice::page_flip).
        std::shared_ptr<DecodedUnit> split_bottom_;
        uint32_t split_next_row_ = 0;
        bool split_enabled_ = true;              // video_halves: 0 / KESTREL_SPLIT=0 / /tmp/kestrel-split-off
        bool split_off_ = false;
        bool split_active() const;

        // Each half on its own (with the second video plane): every picture's
        // top goes up on the video plane at the first vblank after it is
        // decoded, and its lower part on the second plane at the first vblank
        // it will be decoded by - each half of each picture once, in order,
        // none skipped; both changes of a vblank in one commit. See halves_step.
        struct HalfPic {
            uint64_t pts = 0;
            std::shared_ptr<DecodedUnit> top;     // its top is decoded: early, or whole
            std::shared_ptr<DecodedUnit> whole;   // all of it is decoded
            bool top_shown = false, bottom_shown = false;
            bool key = false;
            uint64_t seen_us = 0;
        };
        std::deque<HalfPic> pics_;
        std::shared_ptr<DecodedUnit> on_top_;      // on the video plane (all of the screen, or the top)
        std::shared_ptr<DecodedUnit> on_bottom_;   // on the second plane, null when on_top_ is whole
        std::shared_ptr<DecodedUnit> prev_top_, prev_bottom_;   // replaced by the last flip, held until the next
        uint32_t on_cut_ = 0;                      // the row the two meet at
        bool halves_mode_ = false;
        bool halves_locked_ = false;   // in halves for the display lock, at one refresh a picture
        uint64_t hold_traced_vb_ = 0;
        // When a picture could go up whole but an older picture's lower part
        // is still waiting: 0 its top over that lower part (the lower parts
        // then trail their tops), 1 that lower part first and this picture at
        // the next vblank, 2 this picture whole and that lower part never.
        // /tmp/kestrel-halves.conf "policy N", read every 5 s.
        int halves_policy_ = 0;
        unsigned halves_holds_ = 0, halves_lower_first_ = 0, halves_lower_superseded_ = 0;
        bool deadline_commit_ = true;
        // VRR: halves go up as soon as they are ready, a commit starting the
        // frame (vrr_halves: 0 or KESTREL_VRR_HALVES=0 for whole pictures).
        bool vrr_halves_ = false;
        uint64_t vrr_hold_n_ = 0;          // stands in for the vblank in the hold trace
        double commit_margin_us_ = 1200, commit_min_slack_us_ = 600;
        // DISPLAY > Refresh Sync (display_lock.hpp), with the deadline commit
        // only; made in run(), and told of each picture in queue_frame().
        std::unique_ptr<DisplayLock> lock_;
        std::atomic<DisplayLock*> lock_live_{nullptr};
        std::atomic<bool> halves_live_{false};   // halves_mode_, for queue_frame's thread
        unsigned halves_whole_ = 0, halves_split_ = 0, halves_lower_ = 0, halves_waited_ = 0,
                 halves_dropped_ = 0, halves_lower_skipped_ = 0;
        std::atomic<int> pictures_in_{0};          // whole pictures from the decoder (the FPS figure)
        std::atomic<uint64_t> pictures_total_{0};
        uint64_t rate_t0_ = 0, rate_n0_ = 0;
        double pic_rate_hz_ = 0;                   // decoded pictures a second, over the last second
        std::string halves_why_;
        bool halves_wanted();
        void halves_take(std::shared_ptr<DecodedUnit> du);
        void halves_step();
        void halves_reset();
        bool halves_commit(const std::shared_ptr<DecodedUnit> &top, const std::shared_ptr<DecodedUnit> &bottom,
                           uint32_t cut);
        bool lower_ready(const HalfPic &p, double to_next_us, uint64_t now_us);
        void heartbeat(uint64_t now);
        bool early_safe(const DecodedUnit *du, double to_next_us, uint64_t now_us);
        bool adopt_early(std::shared_ptr<DecodedUnit> &du);
        std::vector<DisplayBufferInfo> display_buffers;
        int current_display_buffer_idx = -1;
        int next_display_buffer_idx = 0;

        // Resolution-change flush handshake (set by decoder thread, cleared by renderer thread)
        std::atomic<bool> flush_requested{false};
        std::atomic<bool> flush_done{false};
        uint64_t last_submitted_pts = 0;
        const int MAX_DISPLAY_BUFFERS = 32;

        // Decoding stats
        Stats stats_;
        bool console_stats = false;

    private:
        bool create_drm_buffer(uint32_t width, uint32_t height, uint32_t format, DrmBuffer& out, bool export_fd = false);
        void init_buffers(DecodedUnit *du);
        void copy_sw_frame(DisplayBufferInfo& buf_info, DecodedUnit* du);
        void convert_yuv420_to_nv12(DisplayBufferInfo& buf_info, DecodedUnit* du);
        void cleanup_display_buffer(DisplayBufferInfo& buffer_info);
        int get_or_create_display_buffer(DecodedUnit* frame, DisplayBufferInfo& out_info);
        void reset_stats(uint64_t now);
    public:
        virtual ~Renderer() {
            stop_display_lock();
            delete decoded_unit_queue;
        }
        // Stops the display lock's thread (idempotent): before the DRM device
        // it reads goes away.
        void stop_display_lock() { if (lock_) lock_->stop(); }
        virtual bool render_frame(DecodedUnit *du);
        uint64_t sensor_offset_us(const DecodedUnit* du);
        virtual void queue_frame(std::shared_ptr<DecodedUnit> du);
        virtual void present_frame(uint32_t fb_id, uint32_t width, uint32_t height, uint64_t pts, uint64_t recv_ts, uint64_t dec_start_ts, uint64_t dec_end_ts, uint32_t tx_age_us);
        void update_stats(DecodedUnit *du, uint64_t display_ts);
        // Flips of pictures timed slice by slice, until all their slices are decoded.
        std::deque<SliceFlip> slice_wait_;
        struct {
            std::vector<SliceLatency> v[2];   // the first slice, the last
            uint32_t slowest[3] = {0, 0, 0};  // first, middle, last
            uint64_t since = 0;
        } slice_log_;
        void add_latency_sample(uint64_t enc, uint64_t rf, uint64_t dec, uint64_t disp,
                                uint64_t total, bool key, unsigned skipped = 0,
                                const float* slice_ms = nullptr, int nslices = 0,
                                const SliceLatency* waits = nullptr,
                                const SliceStages* sl = nullptr, uint64_t shown_us = 0, uint64_t cap_us = 0,
                                bool air = true);
        template <typename RowTime>
        bool whole_picture_sample(const SliceTimes &st, double period, int rows, RowTime row_us, unsigned skipped);
        int64_t last_shown_pts_ = -1;      // the last picture that reached the screen (per-slice path)
        unsigned skipped_pending_ = 0;     // pictures never shown, for the next sample
        unsigned slice_fail_[6] = {0, 0, 0, 0, 0, 0};   // why a shown picture had no latency (5 s)
        unsigned samples_sliced_ = 0, samples_whole_ = 0, samples_again_ = 0, slice_timeouts_ = 0;   // 5 s
        int64_t last_whole_sample_pts_ = -1;

        Renderer(RenderMode rmode,  std::shared_ptr<DrmDevice> dev_, std::shared_ptr<DVR> dvr_, std::shared_ptr<OSD> osd_, volatile bool* stop_signal, bool cstats = false) : 
            render_mode(rmode), dev(dev_), dvr(dvr_), osd(osd_), should_stop(stop_signal), console_stats(cstats) {
                // Increased queue size to allow decoder to run ahead, 
                // enabling the renderer to skip old frames and pick the latest one.
                decoded_unit_queue = new Queue(8);
            }

        //int get_drm_fd() { return dev->drm_fd; }

        // Called by decoder thread before freeing old DRM/MPP resources on resolution change.
        // Signals the renderer to drain its frame queue and clear its display buffer cache,
        // then blocks until the renderer confirms the flush is done.
        void request_flush() {
            flush_done.store(false);
            flush_requested.store(true);
            // Spin-wait up to 200ms for renderer thread to ack
            for (int i = 0; i < 200 && !flush_done.load(); i++)
                usleep(1000);
        }

        void run();

        static void* run_thread(void* arg) {
            static_cast<Renderer*>(arg)->run();
            return nullptr;
        }

        std::function<void(int, int)> cmd_cb;
        void set_command_callback(std::function<void(int, int)> cb) { cmd_cb = cb; }
};


#endif // RENDERER_H  // End of the header guard