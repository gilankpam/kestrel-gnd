#ifndef AR8030_SOURCE_H
#define AR8030_SOURCE_H

#include "../utils/hevc_repair.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "../vdec/vdec.hpp"
#include "../osd.hpp"
#include "../dvr.hpp"
#include "../webstream.hpp"
#include "../common.hpp"
#include "../utils/slice_times.hpp"
#include "../utils/frame_clock.hpp"
#include "ar8030_sky.hpp"

// Direct AR8030 video source.
//
// The Artosyn AR8030 baseband does not hand us an RTP/UDP stream the way the
// ArtLynk link does: video arrives as a byte stream on a "BB socket" served by
// /ar8030soc/daemon, which brokers access to the driver (/dev/ar_mdev0).
// We read that stream, split it into NAL units on Annex-B start codes and feed
// them straight to the decoder - no UDP hop, no RTP reassembly.
//
// Replaces Pipeline when --ar8030 is given.
class Ar8030Source {
    public:
        Ar8030Source(const VideoCodec& codec_, std::shared_ptr<Vdec> vdec_,
                     std::shared_ptr<DVR> dvr_, std::shared_ptr<OSD> osd_,
                     volatile bool* stop_signal,
                     std::string host_ = "127.0.0.1", int host_port_ = 50000,
                     int slot_ = 0, int video_port_ = 3);
        ~Ar8030Source();

        // Thread entry point (matches Pipeline::run_thread usage in main.cpp).
        static void* run_thread(void* arg) {
            static_cast<Ar8030Source*>(arg)->run();
            return nullptr;
        }

        void run();

    private:
        const VideoCodec& codec;
        std::shared_ptr<Vdec> vdec;
        std::shared_ptr<DVR> dvr;
        std::shared_ptr<OSD> osd;
        volatile bool* should_stop;

        std::string host;
        int host_port;
        int slot;
        int video_port;

        void* bb_host = nullptr;   // bb_host_t*
        void* bb_dev = nullptr;    // bb_dev_handle_t*
        int   sockfd = -1;       // video (port 3)
        int   ctrl_sockfd = -1;  // telemetry/control (port 2)
        int   msp_sockfd  = -1;  // flight-controller MSP passthrough (see msp_bb_port)
        int   connect_fails = 0;
        int   read_timeouts = 0;
        bool  link_reported = false;
    public:
        // Overridable from the command line (--ar8030-freq / --ar8030-bw).
        static unsigned freq_khz;
        static int bandwidth;
        static bool do_pair;
        static int ap_index;   // which bb_mac_addr_N to use as the AP
        static bool chan_auto; // channel adaptation (scan for the AP)
        static bool chan_manual_cli;   // --ar8030-chan-manual: pinned to freq_khz from the start
        static int tx_power_dbm;
        static int tx_power_mw;   // stock's encoding: N = hold N mW, N+1 = auto capped at N
        // The power chosen in the menu (saved per air unit); tx_power_mw is it
        // fitted to what the air unit connected now is offered (ar_pwr_fit),
        // so a Lite's 100mW AUTO never replaces a 500mW AUTO chosen for a Lite+.
        static int tx_power_want_mw;
        // The air unit's project number (TLV 0x11: 4 Lite, 6 RC, 7 Lite+), 0 until
        // it reports; picks the power levels it is offered (ar_pwr_offered).
        static std::atomic<int> air_prj;
        // What the air unit says it is and runs (AIR UNIT > INFO): from its version
        // message (cmd 0x04, stock and kestrel-air) and kestrel-air's info message
        // (0x50, "key=value" lines). Emptied when the link goes, so it is always
        // the unit on the other end.
        struct AirInfo {
            bool version = false;       // a version message came
            bool kestrel = false;       // ... saying kestrel-air
            int  stock[3] = {0, 0, 0};  // the stock firmware's APP_VERSION, both apps
            int  hw = -1;               // board-ID version byte (0x10..0x13)
            uint32_t chipid = 0;        // bytes 1..4 from the stock app (it sends 0); the tag from kestrel-air
            int  rf_hw = -1;            // the RF board's version, TLV 0x14 of cmd 0x03
            int  proto = 0;             // kestrel-air's protocol and feature bits
            uint8_t feat = 0;
            // What it has beyond those bits, as it shows it: kAirFeat864p144
            // (its info), kAirSaidFrameClock, kAirSaidSensorTiming.
            uint16_t said = 0;
            std::vector<std::pair<std::string, std::string>> kv;   // from 0x50
            std::string get(const char *k) const {
                for (const auto &e : kv) if (e.first == k) return e.second;
                return std::string();
            }
        };
        static AirInfo air_info();
        static void clear_air_info();
        // kestrel-air's capabilities, named for the INFO tab, by the feature bit
        // (or AirInfo::said bit) that says the air unit has it on.
        struct AirCap { uint16_t bit; const char *key; const char *name; };
        static const AirCap kAirCaps[10];
        // What the air unit has on, by name: its feature bits and what it said.
        static std::vector<std::string> air_cap_names(const AirInfo &a);
        static int air_floor_us;       // fastest capture -> first slice (air_floor_ms), see air_delay_for
        static bool tx_power_auto;
        void apply_tx_power(int mw); // PA output; stock uses 24
        static bool skip_handshake;
        // Air-unit standby / low RF power: -1 leaves it alone, 0/1 is pushed
        // once after every handshake. Re-sending matters because our
        // SET_CONFIG is a byte-copy of a stock capture, so whatever standby
        // state that capture held is replayed at each link-up.
        static int  standby_mode;
        // BB_SET_PRJ_DISPATCH cmd 138. Stock sends this at link bring-up and we
        // never did; see send_prj_rf_config(). -1 on either disables the call.
        static int  prj_rf_bw;      // stock: 4
        static int  prj_rf_pwr_mw;  // stock: 500
        // Air-side bandwidth, sky cmd 0x24. UNVERIFIED units - stock's GUI
        // stores MHz (5/10/20/40, 255=auto) but whether the sky command carries
        // MHz or a bb_bandwidth_e gear is not established. -1 disables.
        static int  air_bw;
        static int  max_kbps;   // ar8030_max_kbps: the video bitrate cap sent at link-up, 0 = none
        static bool air_logs;   // hud_debug: the air unit's flight logs, sent at link-up
        static bool stream_decode;   // KESTREL_STREAM_DECODE / stream_decode: 0 (default on)
        static int  max_bw_mhz; // ar8030_max_bw_mhz: the video link bandwidth cap sent at link-up, 5/10/20/40, 0 AUTO (stock: 20 or 40)
        static int  max_mcs;    // ar8030_max_mcs: Open8030 here, the highest MCS its receive policy may use (-1: none)
        static int  steady;     // ar8030_steady: Open8030 here, a block sent again steps its MCS down (0/1)
        static std::atomic<bool> caps_due;   // Open8030 here: send the caps (at link-up, from the menu)
        static std::atomic<bool> caps_live;  // ... from the menu: AUTO / off sent too, to lift what was set
        static int  radio_fw;   // ar8030_firmware: 0 stock, 1 Open8030 (bb_only.sh loads it with the radio)
        // ar8030_frame_mode (RADIO > Radio Frame): the TDD frame mode asked of
        // the radio at start and after every link-up, 2 short or 0 standard
        // (set_frame_mode()); frame_mode_changed: the menu changed it, send now.
        static int  frame_mode;
        static std::atomic<bool> frame_mode_changed;
        // Replay stock's startup SET sequence verbatim. See the .cpp.
        static int  replay_stock_rf;
        // BB socket port carrying the flight controller's MSP stream, or 0 to
        // leave it alone. Opt-in: bb_socket_open() has no timeout (section 33),
        // so a port the air unit does not serve can wedge bring-up.
        static int  msp_bb_port;
        // --debug-bb-dump: raw copy of the port-2 telemetry stream, for finding out
        // what the air unit actually sends. Capped so a forgotten flag cannot
        // fill the rootfs.
        static std::string dump_path; // isolate: configure + RX only, never TX
        // --debug-replay: video from a file instead of the baseband (run_replay).
        // --debug-video-dump: the video socket's bytes to a file, as they
        // arrive - something to replay. Capped like dump_path.
        static std::string replay_path, video_dump_path;
        static bool decode_enabled; // --ar8030-decode; off = link/OSD only
        // --ar8030-mode WxH@FPS: retune the camera after the handshake.
        // 0 leaves whatever the stock handshake configured.
        static int mode_w, mode_h, mode_fps, mode_chn;
        // Menu -> radio mode change. The OSD menu runs on its own thread and
        // must not call into the BB SDK, so it just parks an index here and the
        // RX thread applies it on its next pass. -1 = nothing pending.
        static std::atomic<int> pending_mode_index;
        static void request_mode_index(int idx) { pending_mode_index.store(idx); }
        static bool probe_modes;    // --debug-probe-modes: sweep and report acks
        static bool probe_rf;       // --debug-probe-rf: which RF ioctls work here

        // Camera settings parked by the OSD menu thread (field id, value).
        // Same reason as pending_mode_index: the menu must not call the SDK.
        enum CamField { CAM_EV = 1, CAM_SAT, CAM_CONTRAST, CAM_SHARPNESS,
                        CAM_SCENE,
                        // This ordinal (cmd_cb id 0x306) was Anti-Flicker
                        // (sky cmd 0x1F) - dropped because stock's own Camera
                        // menu has no such row at all (confirmed against a
                        // photo of it: Scene, EV, Saturation, Sharpness,
                        // Contrast, WB, Rotate, Ratio, 3D DNR only). Reused
                        // here for "Focus Mode", sky cmd 0x0E
                        // (CMD_SET_CHN_FOCUS) - present in stock's Display
                        // tab (also confirmed by photo) and already had its
                        // own wire command and SkyConfig field, just never
                        // wired to the menu.
                        CAM_FOCUS, CAM_AWB,
                        // Air-unit standby / low-RF-power mode. Not a camera
                        // setting, but it travels the same sky socket, so it
                        // rides the same queue rather than growing a second one.
                        CAM_STANDBY,
                        // Air-unit RF bandwidth, sky cmd 0x24.
                        CAM_BW,
                        // Air-unit TX power in mW, sky cmd 0x22.
                        CAM_PWR,
                        // Camera rotation - stock's "Rotate" row, sky cmd 0x06.
                        CAM_ANGLE,
                        // Image aspect ratio - WAS stock's "Ratio" row, sky cmd
                        // 0x07. Menu row removed (verification pass found this
                        // air unit acks the SET but, unlike every other camera
                        // field, never reflects a change in its own status
                        // echo - looks unsupported on this hardware). The
                        // ordinal stays reserved and unused rather than being
                        // deleted: main.cpp computes CamField from a literal
                        // cmd_cb id as (cmd - 0x300), so CAM_3DNR's 0x30D only
                        // lines up with CAM_3DNR's ordinal as long as nothing
                        // shifts into this slot.
                        CAM_RATIO_UNUSED,
                        // 3D DNR, sky cmd 0x1B. body[35] of the config carries it
                        // too, but that is the air's echo in its status report -
                        // writing it there is acked and ignored.
                        CAM_3DNR,
                        // Ours: the cap on the air unit's video bitrate, kbps
                        // (0 = none), sky cmd 0x40 - kestrel-air only. 0x30E.
                        CAM_MAX_KBPS,
                        // Ours: the cap on the video link's bandwidth, MHz (20
                        // or 40), sky cmd 0x41 - kestrel-air only. 0x30F.
                        CAM_MAX_BW,
                        // Ours: the air unit's flight logs on or off (HUD >
                        // Debug Info), sky cmd 0x42 - kestrel-air only. Not a
                        // menu command id (0x310 up are the RF controls).
                        CAM_AIR_LOGS };
        static void request_setting(int field, int value);

        // RF controls. Which of these the baseband actually honours is decided
        // at run time in configure_link() from the ioctl return codes - on this
        // board BB_SET_BANDWIDTH answers -7, so the menu simply does not offer
        // it. Bits are published to the OSD via OSD::set_rf_caps().
        enum RfCap { RF_CAP_POWER = 1, RF_CAP_MCS = 2, RF_CAP_LNA = 4,
                     RF_CAP_BW = 8, RF_CAP_CHAN = 16 };
        enum RfField { RF_TX_POWER = 1, RF_MCS, RF_LNA, RF_BW, RF_CHAN, RF_HOP,
                       RF_TX_POWER_FIT = 100 };   // ours: the chosen power fitted to the air unit now
        static void request_rf(int field, int value);

        // ---- binding (pairing) -------------------------------------------
        // Binding is a baseband operation, so it has to run on the RX thread
        // like every other ioctl here. The menu thread and the front-panel
        // button thread only park a request, exactly as pending_mode_index
        // does, and read the result back through atomics so the OSD can draw
        // the state without taking a lock.
        //
        // It is a state machine stepped once per RX pass rather than a
        // blocking 30-second loop: the RX thread is also what keeps the OSD
        // repainting when no video is arriving, so blocking it would freeze
        // the very indicator that exists to show binding is happening.
        // BIND_LINKED: the window was closed because the radio started
        // hearing an air unit on its own, so there was nothing left to find.
        // Distinct from BIND_FAILED, which means the window ran out in
        // silence - opposite situations that must not share a caption.
        enum BindState { BIND_IDLE = 0, BIND_RUNNING, BIND_OK, BIND_FAILED,
                         BIND_LINKED };
        static std::atomic<int> bind_state;      // BindState
        static std::atomic<int> bind_secs_left;  // countdown while RUNNING
        static std::atomic<unsigned> bind_peer;  // peer MAC, packed big-endian
        static void request_bind() { bind_request.store(1); }
        static bool bind_busy() { return bind_state.load() == BIND_RUNNING; }
        // How long the radio listens for an air unit, and how long a finished
        // result stays on screen before the indicator clears itself.
        //
        // Stock hardcodes 30s. That is too short here, and the reason is the
        // air unit rather than the radio: it only accepts pairing during a
        // window that opens 25-40s AFTER it boots, so a 30s ground window has
        // to be started at exactly the right moment in the air unit's boot to
        // overlap at all. Measured by hand, that is close to unhittable.
        //
        // 120s lets the natural sequence work instead: press bind, then power
        // cycle the air unit and let it come up inside the window. A second
        // press cancels, so a long window costs nothing.
        static const int kBindWindowS      = 120;
        static const int kBindResultHoldMs = 6000;
    private:
        static std::atomic<int> bind_request;
        void bind_step();                       // one non-blocking pass
        bool bind_begin();
        void bind_finish(bool ok, const uint8_t peer[4], bool cancelled = false);
        // Persist a newly bound air unit into /factory/user_cfg.json so the
        // pairing survives a reboot. Returns false if it could not be written;
        // the bind itself still holds for this session.
        static bool save_paired_mac(const uint8_t mac[4]);
        uint64_t bind_started_ms_   = 0;
        uint64_t bind_next_poll_ms_ = 0;
        uint64_t bind_done_ms_      = 0;
        // Whether the air unit was already talking when the window opened.
        // If it was, this is a deliberate bind of a SECOND air unit while the
        // first is connected, and traffic is not a reason to stop.
        bool     bind_had_data_     = false;
        int      bind_role_         = 1;        // BB_ROLE_DEV until GET_STATUS says otherwise
        // Stability filter for the pair result. Stock requires the same peer
        // address to come back unchanged 11 polls running before it accepts
        // it, restarting the count on any change - early in a window the radio
        // reports addresses that then vanish.
        static const int kBindStableReads = 10; // accept on the 11th agreeing read
        uint8_t  bind_seen_[8][4]   = {};
        int      bind_seen_n_[8]    = {};

        // Annex-B accumulator: bb_socket_read() returns arbitrary chunks, so a
        // NAL can straddle reads. We keep a rolling buffer and only emit
        // complete NALs (i.e. up to the *next* start code) - or up to the end
        // of the air unit's packet, when its header says where that is.
        std::vector<uint8_t> accum;
        uint64_t accum_base_ = 0;        // stream offset of accum[0]
        uint64_t pkt_end_pos_ = 0;       // stream offset just past the open packet's trailer, 0 = unknown
        // The slice checksum kestrel-air puts in each packet's extras (06 02
        // <XOR32>): checked once the payload is all here (check_slice_xor).
        bool     xor_pending_ = false;
        uint64_t xor_start_ = 0;         // stream offset of the payload
        uint32_t xor_len_ = 0, xor_want_ = 0;
        uint64_t xor_checked_ = 0, xor_bad_ = 0, xor_unchecked_ = 0;
        void     check_slice_xor();
        int64_t frame_pts = 0;
        uint64_t frames_seen = 0;
        long long bytes_received = 0;
        uint64_t period_start = 0;
        uint64_t stats_frame_ms_ = 0;    // update_stats: the last frame's arrival (a gap starts a new period)
        uint64_t last_tick_ms = 0;
        uint64_t last_data_ms = 0;   // last read that returned video bytes
        // Last byte off the telemetry socket, and when we last re-sent the
        // handshake because of the gap between the two. An air unit that is
        // talking on port 2 but silent on port 3 is present and linked but has
        // lost its streaming state - which is a different fault from an air
        // unit that is simply off, and the only one re-handshaking can fix.
        uint64_t last_ctrl_ms = 0;
        uint64_t last_rehandshake_ms = 0;
        int      rehandshakes = 0;
        // The radio's own view of the link, polled in the run loop. The
        // handshake has to ride a link that is actually up: sent before the
        // radios have associated it goes into the air and nothing answers.
        int      last_link_state = -1;
        uint64_t retune_down_ms_ = 0;          // the link dropped right after a bandwidth change (0: no)
        uint64_t last_scan_ms = 0;
        int      link_freq_mhz = 0;   // real link frequency, from BB_GET_STATUS
        bool     scan_logged = false;
        uint64_t last_stat_ms = 0;
        int dumped_reads = 0;
        unsigned long long total_bytes = 0;
        unsigned long long nal_count = 0;
        unsigned long long ctrl_bytes = 0;  // telemetry drained off port 2
        unsigned long long msp_bytes = 0;   // FC MSP drained off msp_bb_port
        std::atomic<int> last_link_kbps{0};      // written by the stats thread
        std::atomic<int> last_bw_idx{-1};        // written by the stats thread
        int last_bw_rx_idx = -1;  // rx_status's gear, kept only for the log
        bool air_cfg_logged = false;
        uint8_t  last_air_angle = 0;
        uint16_t last_air_w = 0, last_air_h = 0;
        uint8_t  last_air_fps = 0;
        uint8_t  last_air_dnr3d = 0;
        uint8_t  last_air_focus = 0;
        void*              dump_fp = nullptr;
        unsigned long long dump_written = 0;
        void*              video_dump_fp = nullptr;
        unsigned long long video_dump_written = 0;
        // Access-unit assembly for sliced pictures (see emit_nal): the air
        // unit sends each picture as two slice NALs; group them into one packet/PTS.
        std::vector<uint8_t> au_slices;
        int64_t  au_pts = 0;
        uint8_t  au_nal_type = 0;
        bool     au_open = false;
        // Set when the link comes back: drop video until the stream is arriving
        // at the rate it was captured, then restart from the next keyframe.
        //
        // A reconnecting air unit dumps its startup buffer as fast as the link
        // will carry it - pictures captured 16.7ms apart arriving 0-4ms apart -
        // so the first seconds are history played at up to 16x. Waiting for a
        // keyframe alone does not help: that burst is new-stream data and opens
        // with one. Cadence is the signal that actually separates catching up
        // from being live, and both halves of it are already in every picture
        // header.
        bool     resync_          = false;
        int      resync_live_     = 0;   // consecutive pictures at real-time pace
        uint64_t resync_start_ms_ = 0;
        uint64_t resync_prev_us_  = 0;   // arrival of the previous picture
        uint32_t resync_prev_cap_ = 0;   // air-side capture stamp of the same
        // Three in a row, so one straggler in the burst cannot end it early.
        static constexpr int      kResyncLive  = 3;
        // ...and a ceiling, so a stream that never settles still shows a
        // picture rather than leaving the goggle on the lock screen for good.
        static constexpr uint64_t kResyncMaxMs = 5000;
        // Arrival time of the FIRST slice of the access unit currently being
        // assembled. flush_access_unit() used to hand the decoder the arrival
        // time of the NEXT picture's first slice (the read that triggered the
        // flush), so every end-to-end latency measured against it was offset
        // by a frame interval and understated. The AU's own first-byte arrival
        // is the correct zero point for "glass to glass" on the ground side.
        uint64_t au_first_recv_us = 0;
        uint64_t au_cap_us_ = 0;        // the open picture's capture stamp (the air unit's clock), 0 if none

        // The radio queries behind the HUD run on a thread of their own, so
        // the video never waits for a radio reply: on the video thread the
        // 3 s batch held a picture up by 2-3 ms every time. It polls the link
        // state (400 ms), the link figures and distance (3 s) and the channel
        // scan; the video thread acts on what it finds through the atomics.
        std::thread stats_thread;
        std::mutex stats_mu;
        std::condition_variable stats_cv;
        bool stats_stop = false;
        void start_stats();
        void stop_stats();
        void stats_run();
        std::atomic<int> link_state_{-1};          // newest: poll or event
        std::atomic<unsigned> link_events_{0};     // link-state events so far
        std::atomic<uint64_t> bw_change_ms_{0};    // the last bandwidth-change event (12)
        std::atomic<uint64_t> frame_mode_due_ms_{0}; // a link came up: frame mode 2 owed from then (0: none)
        // The video link's bandwidth gear (bb_bandwidth_e), -1 = no link. From
        // the bandwidth-change events: BB_GET_STATUS reports the uplink's 2.5 MHz.
        std::atomic<int> video_bw_idx_{-1};
        std::atomic<int> stat_mcs_{-1}, stat_dist_{-1};
        std::atomic<int> stat_snr_x10_{0};   // BB_GET_1V1_INFO self SNR, dB x 10 (stats thread)
        std::atomic<int> stat_gains_{0};     // BB_GET_1V1_INFO gain_a | gain_b << 8 (per-antenna AGC)
        std::atomic<bool> stats_soon_{false};      // refresh the link figures early
    public:
        // ar_libre's event callbacks (on its reader thread): raw payloads.
        void on_link_event(const uint8_t* p);
        void on_mcs_event(const uint8_t* p);
        void on_logged_event(int ev, const uint8_t* p);
    private:
        // When the data of the last video read came off USB, from the kernel
        // (ar_libre's arlink_socket_rx_ns), or 0 when not known.
        uint64_t read_rx_us = 0;
        // Arrival time of the LAST slice appended to the AU. Reassembly is
        // au_last - au_first; using the flush-triggering read instead measures
        // first-slice-to-next-picture's-first-slice, i.e. the frame interval.
        uint64_t au_last_recv_us = 0;
        // End of picture: flush_access_unit() as the picture's last slice
        // arrives rather than when the next picture's first does.
        int      ctb_addr_bits = 0;          // slice address width, from the SPS
        uint8_t  pps_dep_slices[64] = {0};   // dependent_slice_segments_enabled_flag per PPS
        int      au_max_addr = -1;           // largest slice start in the picture being built
        int      last_slice_addr = -1;       // learned start of a picture's last slice
        int      last_addr_cand = -1, last_addr_streak = 0;
        // Streamed pictures (Vdec::stream_supported): each goes to the
        // decoder with its first slice and takes the rest as they arrive.
        // That needs the number of slices a picture has, learned with where
        // the last one starts.
        int      au_slice_count = 0;         // slices in the picture being built
        int      last_slice_count = 0;       // learned slices per picture
        int      last_count_cand = 0;
        bool     au_streamed = false;        // the picture went to the decoder from its first slice
        bool     au_stream_done = false;     // ...and its last slice followed
        uint64_t stream_pics = 0, stream_ended = 0, stream_refused = 0;
        void     feed_au(const uint8_t *data, size_t len, int stream_slices);
        uint32_t late_slices = 0, late_window_pics = 0;
        uint64_t late_slices_total = 0;
        static constexpr int      kLearnRun = 32;
        static constexpr uint32_t kLateMax  = 4;
        void     learn_last_slice(int max_addr, int slices);
        void     late_slice();

        // Capture -> arrival, per picture. The video header's capture stamp
        // is the air unit's CLOCK_MONOTONIC in microseconds (checked against
        // the encoder's PTS in /proc/umap/venc), so arrival - capture is the
        // true delay plus a constant clock offset. The offset comes out as
        // the smallest such difference seen lately, which is the offset plus
        // the fastest a picture ever makes it; that fastest time is a
        // property of the air pipeline and was measured with both clocks
        // synced through a host (air_floor_us, air_floor_ms). A window, not an
        // all-time minimum: the two crystals drift apart by ~7 ppm.
        static constexpr int kAirWinSlots = 8;          // x 500 ms
        int64_t  air_win_min[kAirWinSlots];
        uint64_t air_win_slot[kAirWinSlots] = {};
        uint64_t cap_us64 = 0;          // last header's stamp, unwrapped
        uint32_t cap_last32 = 0;
        uint32_t cap_wraps = 0;
        bool     cap_valid = false;
        unsigned hdr_fps = 0;           // from the header, for the calibration
        uint32_t au_air_us = 0;         // this picture's capture -> arrival
        float    air_delay_ms = -1.0f;  // EWMA of it, for the stats line
        uint32_t air_delay_for(uint64_t recv_us);

        // Who the air unit says it is. kestrel-air announces itself in its
        // version message (cmd 0x04, bytes 1..4: "KA", protocol, feature bits);
        // the stock air app sends zeros there. Reset at every link-up, since the
        // next air unit may be the other kind. The feature bits are kestrel-air's
        // (sky.h there): 1 = per-slice air-side times in the header bytes 35..41,
        // 2 = intra refresh (keyframes only when asked for), 4 = the camera's IMU
        // (samples as SEI in the video), 32 = the flight controller's IMU instead
        // (MSP_RAW_IMU polled 20 a second, relayed as every MSP response is).
        static constexpr uint8_t kAirFeatLatInfo = 0x01;
        static constexpr uint8_t kAirFeatIntraRefresh = 0x02;
        static constexpr uint8_t kAirFeatCamImu = 0x04;
        bool     air_ver_seen_ = false;
        bool     air_kestrel_ = false;
        uint8_t  air_proto_ = 0;
        uint8_t  air_feat_ = 0;
        unsigned air_untagged_ = 0;      // sane headers in a row without the tag
        unsigned air_stock_hdrs_ = 0;    // headers since link-up, none tagged, nothing decided
        bool     air_hdr_tag_ = false;   // seen in a slice header, which is more reliable than the message
        void note_air_version(const uint8_t *pl, size_t n);
        void note_air_info(const uint8_t *pl, size_t n);
        void note_air_frame(const uint8_t *pl, size_t n);   // KA_MSG_FRAME
        static std::mutex air_info_mtx_;
        static AirInfo air_info_;
        void note_air_prj(int prj, const char *from);   // what the air unit is: its power levels
        void note_air_announce(bool ka, uint8_t proto, uint8_t feat);
        // Air-side times of each slice, us, over 5 s windows, from the header.
        std::vector<uint32_t> lat_enc_, lat_queue_, lat_write_;
        unsigned lat_depth_max_ = 0;
        uint64_t lat_log_ms_ = 0;
        void note_air_latency(const uint8_t *hdr);
        // The radio's AP clock (BB_GET_AP_TIME, ms), which the air unit also
        // stamps into bytes 26..29 of each header: our now_us() minus the AP
        // clock, in us, refreshed once a second. With it each picture's
        // capture is a moment on our clock, so the air delay is measured
        // instead of pinned to air_floor_us.
        static constexpr uint8_t kAirFeatApClock = 0x08;   // byte 2: the stamp's sub-ms part
        static constexpr uint8_t kAirFeatMaxBw = 0x10;     // takes the goggle's bandwidth cap (sky 0x41)
        static constexpr uint8_t kAirFeatFcImu = 0x20;     // the flight controller's IMU (the Lite+: none on the camera)
        static constexpr uint8_t kAirFeat864p120 = 0x40;   // 864p120: its camera's 1536x864 window at 120 fps
        // Said in its info (KA_MSG_INFO), the header's byte being full: 864p144
        // (864p144=1), its OS02K10's 1536x864 window at 144 fps.
        static constexpr uint16_t kAirFeat864p144 = 0x100;
        // No feature bits, only AirInfo::said, for the INFO tab: its frame
        // (KA_MSG_FRAME) came, and its info says the sensor's readout.
        static constexpr uint16_t kAirSaidFrameClock = 0x200;
        static constexpr uint16_t kAirSaidSensorTiming = 0x400;
        static inline uint16_t air_info_feat_ = 0;   // those bits, from its info (reset with the info)
        static inline bool air_info_seen_ = false;   // its info has come since the link came up
        // The mode stored for this air unit, if it lacks it: the default asked
        // for. Bits said in the info only once it has come.
        void check_air_modes();
        // Written by the stats thread (poll_ap_time), read by the video thread.
        std::atomic<int64_t> ap_off_us_{0};
        std::atomic<bool>    ap_off_valid_{false};
        // The radio clock read off the radio's frame (utils/frame_clock.hpp):
        // our chip's frame edges, labelled by kestrel-air's KA_MSG_FRAME. The
        // coarse offset above drifts some 20 us a second between its rare
        // re-syncs, and the latency figures with it (RF 7 -> 10 ms in minutes).
        FrameClock frame_clock_;                // the radio clock to ~10-20 us, from the frame
        int64_t fc_min_d_ = INT64_MAX;          // fastest slice on it lately, us (frame count check)
        unsigned fc_n_ = 0, fc_count_gen_ = 0;
        bool fc_fresh_ = false;                 // the first window after a count: acted on alone
        int64_t fc_last_k_ = 0;                 // the window before's verdict (frames off)
        bool fc_wait_ = false;                  // a shift asked for, not applied yet
        unsigned fc_shifts_seen_ = 0;
        uint64_t fc_win_ms_ = 0;
        uint64_t ap_poll_ms_ = 0;
        uint64_t pic_cap_us_ = 0;        // the picture being received: its capture, our clock (0 = unknown)
        bool     pic_air_unknown_ = false;   // ... and the radio clock to time it by not checked yet
        bool     au_air_unknown_ = false;    // the open access unit: no air figure (kAirUnknown)
        uint64_t air_known_ms_ = 0;          // the last picture with an air figure
        bool     fc_verified_ = false;       // the frame clock's count checked by the slices since it was made
        uint32_t pic_enc_us_ = 0;        // and capture -> its first slice out of the encoder
        uint32_t au_enc_us_ = 0;         // the same for the open access unit; 0 with au_air_us estimated
        bool     au_measured_ = false;
        // Each slice on its own (utils/slice_times.hpp). The slice header's
        // encoder-out time, our clock, for the slice NAL that follows it.
        uint64_t hdr_out_us_ = 0;
        uint32_t hdr_height_ = 0, hdr_fps_ = 0;
        uint32_t ctb_size_ = 0, ctbs_per_row_ = 0;   // from the SPS
        SliceTimesPtr au_times_;                     // the open access unit's slices
        // How long the sensor takes to read a frame out, top row to bottom:
        // where in time a slice's first row was captured. Learned from how far
        // apart the first and last slices leave the encoder (see note_slice).
        std::vector<uint32_t> readout_win_;
        uint32_t readout_us_ = 0;
        // The sensor's readout as the air unit says it (its info, "readout",
        // us; kestrel-air from 2026-10-12): used instead of the learned one,
        // and what splits a slice's Enc into the sensor's part and the rest.
        // 0: not said (or a new mode not said yet).
        std::atomic<uint32_t> air_readout_us_{0};
        std::atomic<uint32_t> air_exposure_us_{0};   // its exposure now ("exposure", us; 0: not said)
        void note_slice(int addr, bool first_slice, uint64_t recv_us);
        void drop_au();   // the open access unit goes, a streamed one ended in the decoder
        bool ps_since_pic_ = false;          // parameter sets came since the last picture (a GOP start)
        // Where a picture's first slice ends, before its second slice says so.
        // Usually at the last slice's learned start; the air unit's intra
        // refresh cuts its sweep-start pictures - one every sweep, by its
        // picture counter - after the refresh rows instead (CTB 120, row 64,
        // every 17th at 1080p). Learned: the period and phase of those cuts.
        int      cut_last_hseq_ = -1;        // the last cut picture's counter
        int      cut_period_ = 0, cut_confirms_ = 0;
        uint32_t cut_row_ = 0;
        uint32_t pics_since_cut_ = 0;
        uint32_t predict_first_end(int hseq) const;
        // 5 s windows, us: per picture capture -> encoder out, -> first slice
        // here, -> last slice here (reassembled); per slice, handed to the
        // air unit's radio -> here.
        std::vector<uint32_t> ap_enc_, ap_first_, ap_all_, ap_link_;
        std::vector<uint32_t> ap_link_pos_[3];   // the same by the slice's place: first, middle, last
        std::vector<uint32_t> ap_enc_last_;      // capture -> the picture's last slice out of the encoder
        uint64_t ap_log_ms_ = 0;
        void poll_ap_time();
        void bw_poll();                  // /tmp/bw.req: the video link's bandwidth, an experiment
        void send_caps();                // Open8030: the menu's caps, from the stats thread
        void note_ap_delay(const uint8_t *hdr, size_t len, bool new_pic);
        void note_ap_picture(uint64_t last_recv_us);
        // An intra refresh stream has no keyframes to join on: ask for one.
        uint64_t idr_asked_ms_ = 0;
        void request_air_idr();
        uint16_t last_hseq = 0;          // header frame counter, for loss
        bool     hseq_valid = false;
        unsigned long long frames_lost = 0;

        // Stale pictures. An air unit whose encoder is driven in a way its
        // own app does not expect (intra refresh switched on underneath it)
        // sends pictures from seconds ago - its encoder ring over again,
        // header and all - between the live ones. Mixed into the live stream
        // they put two pictures' slices in one access unit, and MPP answered
        // with a hardware timeout and reset on nearly every picture.
        // The header's capture stamp says which pictures those are: anything
        // not newer than the newest picture shown (stale_picture()).
        bool     stale_picture(uint32_t cap32);
        uint32_t live_cap_ = 0;          // capture stamp of the newest picture shown
        bool     live_cap_valid_ = false;
        uint32_t stale_prev_cap_ = 0;    // the last picture refused, and how many
        int      stale_run_ = 0;         // refused in a row that run forward
        bool     hdr_stale_ = false;     // the NALs after the last header are stale
        uint16_t hdr_seq_ = 0;           // the last header's counter, stale or not
        bool     hdr_seq_valid_ = false;
        int      slice_hseq_ = -1;       // counter of the header ahead of the next slice
        bool slice_last_ = false;        // kestrel-air: the next slice is its picture's last (header byte 32, 0x40)
        int dbg_hdr32_ = -1;              // DEBUG (56/60): header byte 32 ahead of the next slice
        int      au_hseq_ = -1;          // ...and of the picture being built
        unsigned long long stale_pics = 0, foreign_slices = 0;
        uint16_t sps_ids_ok_ = 0;        // SPS ids a valid SPS has been seen for
        bool pps_ok_ = false;            // a valid PPS has reached the decoder (H.265)
        // A new clock - the air unit rebooted - also reads as not newer, but
        // runs forward from where it restarted, uninterrupted by live
        // pictures. That many in a row and it is the live stream.
        static constexpr int kStaleAdopt = 16;

        bool connect_bb();
        void subscribe_events();    // stock does this before any socket
        void configure_link();      // candidates + freq + bandwidth
        void send_air_handshake();  // start-streaming frames -> air unit
        void apply_video_mode();    // --ar8030-mode -> SET_CHN_RES
        void probe_video_modes();   // ask the camera which modes it accepts
        void probe_rf_ioctls();     // which BB_SET_* the local radio accepts
        void drain_video_socket();  // discard video while not in the RX loop
        int  wait_sky_status(uint8_t want_cmd, int timeout_ms);
        bool read_sky_ack(uint8_t want_cmd, int timeout_ms);
        void drain_control_socket();  // keep the port-2 RX ring from overflowing
        void on_ctrl_bytes(const uint8_t *buf, int n);  // the air's reports, MSP, an awaited ack
        void drain_msp_socket();      // FC MSP passthrough -> OSD DisplayPort
        void apply_bandwidth();       // must run after the link is up (see .cpp)
        void send_prj_rf_config();    // BB_SET_PRJ_DISPATCH cmd 138 (see .cpp)
        void replay_stock_rf_setup(); // stock's startup SETs, byte-for-byte
        void set_frame_mode();        // SET 16 = 2 again at each link-up (see .cpp)
        sky::Proto sky_proto;       // owns the outgoing seq counter

        // Camera settings are re-sent to the air unit in the SET_CONFIG frames
        // of every handshake, so whatever we send there is what the VTX runs
        // after a power cycle. Replaying stock's captured frames therefore threw
        // away any change made from the menu. We now keep a live copy, persist
        // it per air unit (keyed by the AP MAC from /factory/user_cfg.json, so
        // two paired VTXs keep separate settings), and build the SET_CONFIG
        // frames from it.
        sky::SkyConfig sky_cfg;
        std::string    sky_cfg_key;      // "sky_E5DA0008" - empty until known
        void  set_sky_key_from_mac(const uint8_t mac[4]);
        void  identify_peer();          // the air unit actually linked, from BB_GET_STATUS
        void  load_sky_config();
        void  save_sky_config();
        // The TX power this air unit last ran with, kept beside its camera
        // settings. The channel is not kept: the air unit decides it.
        void  load_sky_link();
        void  save_sky_link();
        std::vector<uint8_t> build_config_frame(const uint8_t *tmpl, size_t len);
        void apply_pending_settings();
        void send_frame_trim();           // SPIKE: camera steering (g_camera_trim_ns)
        std::string dbg_au_;              // DEBUG (56/60): this picture's slices, type:CTB:bytes
        uint64_t trim_sent_ms_ = 0;
        int32_t trim_sent_ns_ = INT32_MIN;
        void apply_pending_rf();
        // A camera setting sent and not acknowledged yet. Its ack is picked
        // out of whatever the control socket brings in (on_ctrl_bytes), read
        // by the main loop between pictures: the video goes on meanwhile. The
        // next setting goes once it is in, or after kSkyAckMs.
        struct SkyAckWait {
            int field = -1;              // -1: none awaited
            int value = 0;
            uint8_t cmd = 0;
            uint64_t until_ms = 0;
        };
        static constexpr int kSkyAckMs = 1000;
        SkyAckWait sky_ack_;
        std::deque<std::pair<int,int>> settings_q_;   // parked settings not sent yet, in order
        void sky_ack_done(int status);
        unsigned rf_caps = 0;
        bool ranging_ok = false;
        int  ranging_tries = 0;
        static std::vector<std::pair<int,int>> pending_rf;
        static std::mutex pending_mtx;
        static std::vector<std::pair<int,int>> pending_settings;
        void report_link_status();  // one-shot link state log
        int  current_rx_mcs();      // link MCS right now, -1 if unavailable
        int  current_link_state();  // 0 not linked, 1 connecting, 2 linked, -1 unknown
        void enable_ranging();      // BB_CFG_DISTC - time-of-flight distance
        int  current_distance();    // BB_GET_DISTC_RESULT, -1 = no fix yet
        // Fill the OSD's link panel. This used to come from ArtosynRpcClient,
        // but that is a second SDK client on the same daemon and cannot run
        // beside us (section 22.1) - so we publish the same fields in-process
        // from the ioctls we already own, otherwise the panel reads "IDLE"
        // while video is plainly decoding.
        void publish_link_stats();
        // Spectrum scan for the channel-scan screen. Same story as the link
        // panel: this used to come from ArtosynRpcClient, which cannot run
        // beside us, so we poll BB_GET_CHAN_INFO in-process instead.
        void publish_chan_scan();
        void scan_air_status(const uint8_t *buf, int n);  // inbound cmd 0x03
        // Every distinct validated VPS/SPS/PPS seen, replayed into a new
        // recording so its hvcC is complete.
        std::vector<std::vector<uint8_t>> dvr_param_sets;
        unsigned long long   ps_dropped = 0;   // corrupt param-set NALs rejected
        bool air_standby = false;      // VTX in low-power/standby mode
        bool air_status_seen = false;  // have we decoded one status frame yet

        // Move the RF link to freq_khz, both ends. Mirrors stock's
        // fpv_bb_set_freq() plus the sky 0x21 it pairs with.
        // Returns 0 on success, negative on failure (frequency not in the radio's table, or an ioctl refused).
        int  set_rf_channel(uint32_t freq_khz, bool hop_en);
        // Undo a pin: every channel back on the ground's work list, then
        // channel adaptation on - the ground searches, as at start.
        int  search_all_channels();
        uint32_t link_freq_khz();   // ground's own TX freq, 0 if unknown

        // Index of freq_khz in the radio's channel table, or -1. Also yields
        // the full table, which BB_SET_WORK_CHAN_LIST needs.
        int  chan_index_for_freq(uint32_t freq_khz, uint32_t *freqs, int *n_out);
    public:
        // What the air unit announced; false/0 for the stock air app and until
        // its version message has arrived (every few seconds after link-up).
        bool    air_is_kestrel() const { return air_kestrel_; }
        uint8_t air_features() const { return air_feat_; }
        // Set by the OSD when the scan screen opens, so we poll faster while
        // the user is looking at it (matching the old RPC client's cadence).
        static std::atomic<bool> scan_active;
    private:
        // (was enter_pair_mode: a blocking 30s loop reachable only from the
        //  command line. Replaced by the bind_* state machine above, which the
        //  menu, the front-panel button and --ar8030-pair all drive.)
        void disconnect_bb();
        void run_replay();
        void consume(const uint8_t* data, size_t len);
        void consume_bytes(const uint8_t* data, size_t len);   // consume() without the flush
        // A streamed picture's slices that came in one read go to the decoder as
        // one part, at the end of the read (send_held_slices): two parts back to
        // back sometimes made it end the picture before its last slice.
        std::vector<uint8_t> held_slices_;
        int64_t held_pts_ = -1;
        void send_held_slices(bool last = false);
        void emit_nal(const uint8_t* nal, size_t len);
        void flush_access_unit(uint64_t recv_us);
        void update_stats(size_t frame_size);
        void update_dvr(std::shared_ptr<std::vector<uint8_t>> frame, uint64_t cap_us);
        bool au_has_keyframe(const std::vector<uint8_t>& au) const;
        bool dvr_keyed_ = false;          // the recording running has had its keyframe
        bool dvr_rec_was_ = false;        // a recording was running at the last picture
        HevcRepair dvr_repair_;           // its lost pictures and slices replaced by copies
        uint64_t dvr_idr_asked_ms_ = 0;   // the last keyframe asked for it
};

#endif
