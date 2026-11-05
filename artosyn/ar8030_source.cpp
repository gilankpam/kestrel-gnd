#include <algorithm>
#include "ar8030_source.hpp"
#include "frame_mode.hpp"
#include "../utils/ltrace.hpp"
#include "bb_watchdog.hpp"
#include "../stab/imu_stream.hpp"
#include "ar8030_handshake.h"
#include "../settings.hpp"
#include <cmath>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cerrno>
#include <chrono>
#include <poll.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>

#include <set>
#include "../utils/scheduling_helper.hpp"

extern "C" {
#include "bb_client.h"

// ar_libre's own client-library calls (Gee/ar_libre), which Artosyn's library
// does not have. Weak, so one binary runs on either: on Artosyn's they are
// NULL, and everything below behaves as it always has.
extern "C" const char* arlink_version(void) __attribute__((weak));
extern "C" uint64_t arlink_socket_rx_ns(int sockfd) __attribute__((weak));
extern "C" int arlink_socket_read_now(int sockfd, void *buf, uint32_t len) __attribute__((weak));
extern "C" int arlink_socket_poll_fd(int sockfd) __attribute__((weak));
extern "C" int arlink_request(uint32_t request_code, const void *in, uint32_t inlen, void *out,
                              uint32_t outcap, uint32_t *got, int timeout_ms) __attribute__((weak));
static bool on_ar_libre() { return arlink_version != nullptr; }

// A socket's own file to poll(), where ar_libre has one (-1: none, or
// Artosyn's library).
static int poll_fd(int sfd) {
    return sfd >= 0 && arlink_socket_read_now && arlink_socket_poll_fd ? arlink_socket_poll_fd(sfd) : -1;
}

// What a socket has queued, without waiting where ar_libre allows it; else
// bb_socket_read's shortest wait, 1 ms. <= 0: nothing.
static int read_queued(int sfd, void *buf, uint32_t len) {
    if (poll_fd(sfd) >= 0) return arlink_socket_read_now(sfd, buf, len);
    return bb_socket_read(sfd, buf, len, 1);
}

}

// artosyn/bb_sdk_shim.cpp - overrides the linked SDK's ioctl length table with
// the newer one extracted from stock ar_ldy_gnd.
extern "C" void bb_sdk_shim_report();

// Read chunk size. The baseband hands us H.265 in bursts; 256K keeps the
// syscall rate low without adding meaningful latency (a 1080p60 IDR at
// 40Mbit/s is ~80K, so a big frame still lands in one or two reads).
#define AR_READ_CHUNK (256 * 1024)

// Guard against a runaway accumulator if we never see a second start code
// (corrupt stream / wrong port). 4MB is far above any sane single NAL.
#define AR_MAX_ACCUM (4 * 1024 * 1024)

static inline uint64_t now_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

static inline uint64_t now_ms() { return now_us() / 1000ULL; }

// ---- H.265 parameter-set sanity --------------------------------------------
//
// A two-byte header that looks right is not enough to trust a VPS/SPS/PPS: a
// 31-byte "SPS" with a valid header but garbage inside (it reads as SPS id 47,
// where 0-15 is the whole range) got cached next to the real one, went into
// every hvcC from then on, and the iPhone refused to decode the web stream at
// all. So the id is read and range-checked, and an SPS must also carry a
// chroma format and a picture size that exist.

namespace {
struct Rbsp {
    std::vector<uint8_t> b;
    size_t bit = 0;
    bool bad = false;
    Rbsp(const uint8_t* p, size_t n) {           // drop emulation-prevention bytes
        int zeros = 0;
        for (size_t i = 0; i < n; i++) {
            if (zeros >= 2 && p[i] == 3) { zeros = 0; continue; }
            b.push_back(p[i]);
            zeros = p[i] == 0 ? zeros + 1 : 0;
        }
    }
    uint32_t u(int n) {
        uint32_t v = 0;
        while (n--) {
            if (bit >= b.size() * 8) { bad = true; return 0; }
            v = (v << 1) | ((b[bit >> 3] >> (7 - (bit & 7))) & 1);
            bit++;
        }
        return v;
    }
    uint32_t ue() {
        int z = 0;
        while (!bad && u(1) == 0) if (++z > 31) { bad = true; return 0; }
        return z ? (1u << z) - 1 + u(z) : 0;
    }
    int32_t se() {
        const uint32_t k = ue();
        return (k & 1) ? (int32_t)((k >> 1) + 1) : -(int32_t)(k >> 1);
    }
    // rbsp_trailing_bits: a one, then nothing but zeros to the end.
    bool at_end() {
        if (bad || u(1) != 1) return false;
        while (bit < b.size() * 8) if (u(1)) return false;
        return true;
    }
};
}  // namespace

// The id of an H.265 VPS/SPS/PPS (payload after the 2-byte header), or -1 if
// the set cannot be real.
static int hevc_param_set_id(int type, const uint8_t* p, size_t n) {
    Rbsp r(p, n);
    if (type == 32) {                                   // VPS
        const uint32_t id = r.u(4);
        r.u(2);                                         // base layer internal/available
        const uint32_t layers = r.u(6), sub = r.u(3);
        r.u(1);                                         // temporal_id_nesting
        const uint32_t reserved = r.u(16);              // vps_reserved_0xffff_16bits
        return (r.bad || layers != 0 || sub > 6 || reserved != 0xFFFF) ? -1 : (int)id;
    }
    if (type == 34) {                                   // PPS
        const uint32_t id = r.ue();
        return (r.bad || id > 63) ? -1 : (int)id;
    }
    if (type != 33) return -1;                          // SPS
    r.u(4);                                             // sps_video_parameter_set_id
    const uint32_t max_sub = r.u(3);                    // sps_max_sub_layers_minus1
    r.u(1);                                             // temporal_id_nesting
    if (max_sub > 6) return -1;
    // profile_tier_level(1, max_sub)
    r.u(88); r.u(8);                                    // general profile + level
    uint32_t prof[8] = {0}, lvl[8] = {0};
    for (uint32_t i = 0; i < max_sub; i++) { prof[i] = r.u(1); lvl[i] = r.u(1); }
    if (max_sub > 0) for (uint32_t i = max_sub; i < 8; i++) r.u(2);
    for (uint32_t i = 0; i < max_sub; i++) { if (prof[i]) r.u(88); if (lvl[i]) r.u(8); }
    const uint32_t id = r.ue();
    const uint32_t chroma = r.ue();
    if (chroma == 3) r.u(1);                            // separate_colour_plane_flag
    const uint32_t w = r.ue(), h = r.ue();
    if (r.bad || id > 15 || chroma > 3 || w < 16 || h < 16 || w > 8192 || h > 8192) return -1;
    return (int)id;
}

// How many bits a slice_segment_address takes in pictures of this SPS -
// Ceil(Log2(PicSizeInCtbsY)) - or 0 if the SPS does not parse. The same walk as
// hevc_param_set_id, carried on to the coding block sizes.
static int hevc_sps_addr_bits(const uint8_t* p, size_t n, uint32_t* ctb_out = nullptr,
                              uint32_t* w_out = nullptr) {
    Rbsp r(p, n);
    r.u(4);
    const uint32_t max_sub = r.u(3);
    r.u(1);
    if (max_sub > 6) return 0;
    r.u(88); r.u(8);
    uint32_t prof[8] = {0}, lvl[8] = {0};
    for (uint32_t i = 0; i < max_sub; i++) { prof[i] = r.u(1); lvl[i] = r.u(1); }
    if (max_sub > 0) for (uint32_t i = max_sub; i < 8; i++) r.u(2);
    for (uint32_t i = 0; i < max_sub; i++) { if (prof[i]) r.u(88); if (lvl[i]) r.u(8); }
    r.ue();                                             // sps_seq_parameter_set_id
    if (r.ue() == 3) r.u(1);                            // chroma_format_idc
    const uint32_t w = r.ue(), h = r.ue();
    if (r.u(1)) { r.ue(); r.ue(); r.ue(); r.ue(); }     // conformance window
    r.ue(); r.ue();                                     // bit depths
    r.ue();                                             // log2_max_pic_order_cnt_lsb_minus4
    const uint32_t ordering_all = r.u(1);
    for (uint32_t i = ordering_all ? 0 : max_sub; i <= max_sub; i++) { r.ue(); r.ue(); r.ue(); }
    const uint32_t min_cb = r.ue() + 3, diff = r.ue();
    if (r.bad || w < 16 || h < 16 || w > 8192 || h > 8192 || min_cb + diff < 4 || min_cb + diff > 6) return 0;
    const uint32_t ctb = 1u << (min_cb + diff);
    const uint32_t ctbs = ((w + ctb - 1) / ctb) * ((h + ctb - 1) / ctb);
    if (ctb_out) *ctb_out = ctb;
    if (w_out) *w_out = w;
    int bits = 0;
    while ((1u << bits) < ctbs) bits++;
    return bits;
}

// scaling_list_data(), read and range-checked; false if it cannot be.
static bool hevc_scaling_list_ok(Rbsp& r) {
    for (int size = 0; size < 4; size++)
        for (int m = 0; m < 6; m += (size == 3) ? 3 : 1) {
            if (!r.u(1)) {                              // scaling_list_pred_mode_flag
                if (r.ue() > (uint32_t)(size == 3 ? m / 3 : m)) return false;
                continue;
            }
            const int coefs = size == 0 ? 16 : 64;
            if (size > 1) {
                const int32_t dc = r.se();              // scaling_list_dc_coef_minus8
                if (dc < -7 || dc > 247) return false;
            }
            for (int i = 0; i < coefs; i++) {
                const int32_t d = r.se();               // scaling_list_delta_coef
                if (d < -128 || d > 127 || r.bad) return false;
            }
        }
    return !r.bad;
}

// Whether an H.265 PPS (payload after the 2-byte header) reads right to its
// rbsp_trailing_bits with every value in range; its SPS's id in *sps_id.
// Random bytes behind a good header all but never do - and MPP re-parses a
// PPS over the one in use, so one that fails half way leaves that one broken:
// seen as a SIGSEGV in its set_sps() on the next slice, the half-parsed PPS
// naming an SPS that did not exist. Extensions are refused: they belong to
// profiles beyond Main 10, which this decoder does not do.
static bool hevc_pps_ok(const uint8_t* p, size_t n, int* sps_id) {
    Rbsp r(p, n);
    const uint32_t pps_id = r.ue(), sid = r.ue();
    r.u(1); r.u(1); r.u(3); r.u(1); r.u(1);   // dependent slices .. cabac_init_present_flag
    if (r.ue() > 14 || r.ue() > 14) return false;   // num_ref_idx_l0/l1_default_active_minus1
    const int32_t qp = r.se();                         // init_qp_minus26
    r.u(1); r.u(1);                                    // constrained_intra_pred, transform_skip
    if (r.u(1) && r.ue() > 3) return false;            // diff_cu_qp_delta_depth
    const int32_t cb = r.se(), cr = r.se();
    if (pps_id > 63 || sid > 15 || qp < -38 || qp > 25 || cb < -12 || cb > 12 ||
        cr < -12 || cr > 12)
        return false;
    r.u(1); r.u(1); r.u(1); r.u(1);                    // chroma qp offsets .. transquant_bypass
    const uint32_t tiles = r.u(1);
    r.u(1);                                            // entropy_coding_sync_enabled_flag
    if (tiles) {
        const uint32_t cols = r.ue(), rows = r.ue();
        if (cols > 19 || rows > 21) return false;
        if (!r.u(1)) {                                 // uniform_spacing_flag
            for (uint32_t i = 0; i < cols; i++) r.ue();
            for (uint32_t i = 0; i < rows; i++) r.ue();
        }
        r.u(1);                                        // loop_filter_across_tiles
    }
    r.u(1);                                            // loop_filter_across_slices
    if (r.u(1)) {                                      // deblocking_filter_control_present
        r.u(1);
        if (!r.u(1)) {
            const int32_t beta = r.se(), tc = r.se();
            if (beta < -6 || beta > 6 || tc < -6 || tc > 6) return false;
        }
    }
    if (r.u(1) && !hevc_scaling_list_ok(r)) return false;
    r.u(1);                                            // lists_modification_present
    if (r.ue() > 4) return false;                      // log2_parallel_merge_level_minus2
    r.u(1);                                            // slice_segment_header_extension
    if (r.u(1) && r.u(8) != 0) return false;           // pps_extension_present, and which
    *sps_id = (int)sid;
    return r.at_end();
}

// A PPS's dependent_slice_segments_enabled_flag, with its id in *id; -1 if it
// does not parse.
static int hevc_pps_dependent_slices(const uint8_t* p, size_t n, int* id) {
    Rbsp r(p, n);
    const uint32_t pps_id = r.ue();
    r.ue();                                             // pps_seq_parameter_set_id
    const uint32_t dep = r.u(1);
    if (r.bad || pps_id > 63) return -1;
    *id = (int)pps_id;
    return (int)dep;
}

// Where in the picture a slice starts, in coding tree blocks (0 for a
// picture's first slice), or -1 if it cannot be read. p is the slice NAL's
// payload after its 2-byte header.
static int hevc_slice_address(int type, const uint8_t* p, size_t n, int addr_bits,
                              const uint8_t dep_slices[64]) {
    if (addr_bits <= 0) return -1;
    Rbsp r(p, n < 32 ? n : 32);
    if (r.u(1)) return 0;                               // first_slice_segment_in_pic_flag
    if (type >= 16 && type <= 23) r.u(1);               // no_output_of_prior_pics_flag
    const uint32_t pps = r.ue();
    if (r.bad || pps > 63) return -1;
    if (dep_slices[pps]) r.u(1);                        // dependent_slice_segment_flag
    const uint32_t addr = r.u(addr_bits);
    return r.bad ? -1 : (int)addr;
}

// Every bb_ioctl() in this SDK waits forever. The public entry point hard-codes
// timeout = -1 (ar8030.c:253), and -1 selects the untimed pthread_cond_wait in
// bs_send_usbpack_and_wait() rather than the timed one beside it. So a single
// lost reply from the daemon parks the calling thread for the life of the
// process - and this loop polls status several times a second, which is why
// the failure looks like "it connected and then stopped" rather than a failure
// to connect. Caught in the act as AR8030_RX sitting in futex_wait_queue_me
// with both sockets open and the log frozen.
//
// bb_ioctl_ex() is the same call with a caller-supplied deadline, exported by
// the same vendor library we already link, so this needs no patched SDK. A
// timeout comes back as a plain failure, which every caller here already
// handles - and a call that fails is recoverable where one that never returns
// is not.
static const int kIoctlTimeoutMs = 1000;
// Bring-up requests get longer than an ordinary ioctl: the daemon has real
// work to do behind BB_INIT_REQ, and a deadline that expires while it is
// simply busy would turn a slow start into a failed one.
static const int kBringupReqTimeoutMs = 3000;
// Consecutive failed bring-ups before we stop asking nicely and restart the
// daemon. Two is roughly 15s of trying, which is long enough to ride out a
// daemon that is merely slow to start and short enough that a stuck one is
// cleared before the pilot gives up on it.
static const int kFailsBeforeDaemonRestart = 2;

// One request at a time: the stats thread and this one both talk to the
// baseband, and Artosyn's client library is not known to take concurrent
// requests (ar_libre's does).
static std::mutex g_bb_req_mu;

static int ar_ioctl(bb_dev_handle_t *dev, uint32_t request, const void *in, void *out) {
    int rc;
    {
        std::lock_guard<std::mutex> one_at_a_time(g_bb_req_mu);
        rc = bb_ioctl_ex(dev, request, in, out, kIoctlTimeoutMs);
    }
    if (rc != 0) {
        // Rate-limited: a wedged link would otherwise fill the log with these.
        static uint64_t last_ms = 0;
        static unsigned suppressed = 0;
        uint64_t now = now_ms();
        if (now - last_ms > 5000) {
            // rc -2 and -7 are the baseband refusing a command it is not ready
            // for (BB_CFG_DISTC before the link is up, BB_SET_BANDWIDTH early)
            // and are expected; a timeout is rc from pthread_cond_timedwait and
            // means the daemon never answered. Both are worth seeing, but only
            // the second is the fault this timeout exists to convert.
            printf("ar8030: ioctl 0x%X returned %d%s\n", request, rc,
                   suppressed ? " [+more]" : "");
            last_ms = now;
            suppressed = 0;
        } else {
            suppressed++;
        }
    }
    return rc;
}


Ar8030Source::Ar8030Source(const VideoCodec& codec_, std::shared_ptr<Vdec> vdec_,
                           std::shared_ptr<DVR> dvr_, std::shared_ptr<OSD> osd_,
                           volatile bool* stop_signal,
                           std::string host_, int host_port_,
                           int slot_, int video_port_)
    : codec(codec_), vdec(vdec_), dvr(dvr_), osd(osd_), should_stop(stop_signal),
      host(std::move(host_)), host_port(host_port_), slot(slot_), video_port(video_port_) {
    accum.reserve(AR_READ_CHUNK * 2);
}

Ar8030Source::~Ar8030Source() { disconnect_bb(); }


// Defaults taken from the stock boot log for this board (rf_board 16):
//   gui_fpv_bb_set_freq, freq=5740000 ; gui_fpv_bb_set_bandwidth, bandwidth=0
unsigned Ar8030Source::freq_khz = 5740000;
// -1 = leave the radio on whatever bandwidth it negotiates.
//
// Forcing this from the ground does NOT work and measurably hurts: the air unit
// is the AP and owns the channel, so a unilateral BB_SET_BANDWIDTH(20M) on the
// DEV side is accepted (returns 0) and then costs throughput - measured 3.06 ->
// 2.10 Mbps at the USB layer, MCS 10 -> 8.
int Ar8030Source::bandwidth = -1;
bool Ar8030Source::do_pair = false;
int Ar8030Source::ap_index = 0;
bool Ar8030Source::chan_auto = true;
bool Ar8030Source::chan_manual_cli = false;
int Ar8030Source::tx_power_mw  = kArPwrDefaultMw;
int Ar8030Source::tx_power_want_mw = kArPwrDefaultMw;
std::atomic<int> Ar8030Source::air_prj{0};
std::mutex Ar8030Source::air_info_mtx_;
Ar8030Source::AirInfo Ar8030Source::air_info_;

Ar8030Source::AirInfo Ar8030Source::air_info() {
    std::lock_guard<std::mutex> g(air_info_mtx_);
    return air_info_;
}

const Ar8030Source::AirCap Ar8030Source::kAirCaps[10] = {
    { kAirFeatLatInfo,      "air-timing",    "Air Timing" },
    { kAirFeatApClock,      "radio-clock",   "Radio Clock" },
    { kAirSaidFrameClock,   "frame-clock",   "Frame Clock" },
    { kAirSaidSensorTiming, "sensor-timing", "Sensor Timing" },
    { kAirFeatIntraRefresh, "intra-refresh", "Intra Refresh" },
    { kAirFeatCamImu,       "cam-imu",       "Camera IMU" },
    { kAirFeatFcImu,        "fc-imu",        "FC IMU" },
    { kAirFeatMaxBw,        "bandwidth-cap", "Bandwidth Cap" },
    { kAirFeat864p120,      "864p120",       "864p120" },
    { kAirFeat864p144,      "864p144",       "864p144" },
};

std::vector<std::string> Ar8030Source::air_cap_names(const AirInfo &a) {
    std::vector<std::string> out;
    const uint16_t have = (uint16_t)(a.feat | a.said);
    for (const auto &c : kAirCaps) if (have & c.bit) out.push_back(c.name);
    return out;
}

void Ar8030Source::clear_air_info() {
    std::lock_guard<std::mutex> g(air_info_mtx_);
    air_info_ = AirInfo();
    air_info_feat_ = 0;
    air_info_seen_ = false;
}

void Ar8030Source::check_air_modes() {
    // Not before it has said what it is (its info can come first).
    if (!air_ver_seen_) return;
    const uint16_t have = air_kestrel_ ? (uint16_t)(air_feat_ | air_info_feat_) : 0;
    const uint16_t known = air_info_seen_ ? 0xffff : 0x00ff;
    for (int i = 0; i < sky::kFpvModeCount; i++) {
        const sky::VideoMode &m = sky::kFpvModes[i];
        if (m.w == sky_cfg.ch0_w && m.h == sky_cfg.ch0_h && m.fps == sky_cfg.ch0_fps && (m.need & known & ~have)) {
            const int d = sky::default_fpv_mode_index();
            printf("ar8030: the air unit has no %s: asking for %s\n", m.name, sky::kFpvModes[d].name);
            if (osd) osd->set_video_mode_current(d);
            request_mode_index(d);
            break;
        }
    }
}
int Ar8030Source::tx_power_dbm = kArPwrLevels[ar_pwr_index(kArPwrDefaultMw)].dbm;
bool Ar8030Source::tx_power_auto = false;
bool Ar8030Source::skip_handshake = false;
int Ar8030Source::msp_bb_port = 2;   // the FC stream rides the telemetry socket
int Ar8030Source::standby_mode = -1;
// Default OFF. Replaying stock's cmd138(4, 500) is accepted (rc=0) but on its
// own it COSTS throughput: measured 3.06 -> 2.09 Mbps at the USB layer, MCS 10
// -> 8, which is within noise of the BB_SET_BANDWIDTH(20M) result (2.10). Both
// widen the ground receiver while the air unit keeps transmitting narrow, so
// the extra noise bandwidth just lowers SNR. Enable only together with the
// air-side half below.
int Ar8030Source::air_bw        = -1;
int Ar8030Source::max_kbps      = 0;
bool Ar8030Source::air_logs     = false;
int Ar8030Source::max_bw_mhz    = 40;
int Ar8030Source::max_mcs       = -1;
int Ar8030Source::steady        = 0;
std::atomic<bool> Ar8030Source::caps_due{true};
std::atomic<bool> Ar8030Source::caps_live{false};
int Ar8030Source::radio_fw      = 0;
int Ar8030Source::frame_mode    = 2;
std::atomic<bool> Ar8030Source::frame_mode_changed{false};
int Ar8030Source::replay_stock_rf = 1;
int Ar8030Source::prj_rf_bw     = -1;
int Ar8030Source::prj_rf_pwr_mw = -1;
std::string Ar8030Source::dump_path;
std::string Ar8030Source::replay_path, Ar8030Source::video_dump_path;
int Ar8030Source::mode_w = 0;
int Ar8030Source::mode_h = 0;
int Ar8030Source::mode_fps = 0;
int Ar8030Source::mode_chn = 0;
std::atomic<int> Ar8030Source::pending_mode_index{-1};
bool Ar8030Source::probe_modes = false;
bool Ar8030Source::probe_rf = false;
std::atomic<bool> Ar8030Source::scan_active{false};
std::mutex Ar8030Source::pending_mtx;
std::vector<std::pair<int,int>> Ar8030Source::pending_settings;
std::vector<std::pair<int,int>> Ar8030Source::pending_rf;
std::atomic<int> Ar8030Source::bind_request{0};
std::atomic<int> Ar8030Source::bind_state{Ar8030Source::BIND_IDLE};
std::atomic<int> Ar8030Source::bind_secs_left{0};
std::atomic<unsigned> Ar8030Source::bind_peer{0};

void Ar8030Source::request_rf(int field, int value) {
    std::lock_guard<std::mutex> lk(pending_mtx);
    pending_rf.push_back({field, value});
}

void Ar8030Source::request_setting(int field, int value) {
    std::lock_guard<std::mutex> lk(pending_mtx);
    pending_settings.push_back({field, value});
}
bool Ar8030Source::decode_enabled = false;
// Pictures decoded from their first slice (Vdec::stream_supported): on unless
// KESTREL_STREAM_DECODE=0.
bool Ar8030Source::stream_decode = []() {
    const char *e = getenv("KESTREL_STREAM_DECODE");
    return !(e && atoi(e) == 0);
}();

// Parse the paired air-unit MACs out of /factory/user_cfg.json. The file is a
// flat object holding "bb_mac_addr_N": [ "0xE5", "0xDA", "0x00", "0x08" ], the
// same values the stock firmware logged as candi_mac[N].
static int ar_load_candidate_macs(bb_mac_t *out, int max) {
    FILE *f = fopen("/factory/user_cfg.json", "r");
    if (!f) return 0;
    char buf[8192];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    buf[n] = 0;
    fclose(f);

    int count = 0;
    char *p = buf;
    while (count < max && (p = strstr(p, "bb_mac_addr_")) != NULL) {
        char *lb = strchr(p, '[');
        if (!lb) break;
        char *rb = strchr(lb, ']');
        if (!rb) break;
        *rb = 0;
        unsigned vals[BB_MAC_LEN];
        int got = 0;
        char *q = lb + 1;
        while (got < BB_MAC_LEN) {
            char *hx = strstr(q, "0x");
            if (!hx) break;
            vals[got++] = (unsigned)strtoul(hx, &q, 16);
        }
        *rb = ']';
        if (got == BB_MAC_LEN) {
            for (int i = 0; i < BB_MAC_LEN; i++)
                out[count].addr[i] = (uint8_t)vals[i];
            count++;
        }
        p = rb;
    }
    return count;
}


// Events stock subscribes to before touching a socket (capture: class 03 01,
// args 01, 00, 02, 08, 0C). 0=LINK_STATE 1=MCS_CHANGE 2=CHAN_CHANGE
// 8=PRJ_DISPATCH2 (custom dispatch over USB RPC) 12=MAX.
static void ar_event_cb(void *arg, void *user) {
    (void)arg;
    (void)user;   // we only need the subscription to exist, not to act on it
}

// On ar_libre an event's arg is its payload as the chip sent it, which is
// what these decode; what Artosyn's library passes is not known, so there the
// subscriptions stay as they were.
static void ar_event_link_cb(void *arg, void *user) {
    if (arg && user) static_cast<Ar8030Source*>(user)->on_link_event((const uint8_t*)arg);
}
static void ar_event_mcs_cb(void *arg, void *user) {
    if (arg && user) static_cast<Ar8030Source*>(user)->on_mcs_event((const uint8_t*)arg);
}
template <int EV>
static void ar_event_log_cb(void *arg, void *user) {
    if (arg && user) static_cast<Ar8030Source*>(user)->on_logged_event(EV, (const uint8_t*)arg);
}

// Link state (event 0): slot, new state, old state - 0 not linked, 1
// connecting, 2 linked. Acted on at once; the stats thread's poll stays the
// safety net, and drops an answer that an event overtook.
void Ar8030Source::on_link_event(const uint8_t* p) {
    if (p[0] != (uint8_t)slot || p[1] > 2) return;
    link_events_.fetch_add(1);
    link_state_.store(p[1]);
    frame_clock_.set_linked(p[1] == 2);
    if (p[1] == 2) frame_mode_due_ms_.store(now_ms() + 1000);   // see set_frame_mode()
    // A link starts on the config's 5 MHz (gear 2) and widens from there, each
    // step an event 12. A drop that a bandwidth change caused re-forms the same
    // link on its new width, so it keeps the gear.
    if (p[1] == 2 && video_bw_idx_.load() < 0) video_bw_idx_.store(2);
    else if (p[1] == 0 && now_ms() - bw_change_ms_.load() > 2000) {
        video_bw_idx_.store(-1);
        air_prj.store(0);          // the next air unit says what it is
        clear_air_info();
    }
    printf("ar8030: event: link state %u -> %u\n", p[2], p[1]);
    if (p[1] == 2) frame_clock_.reframe();   // a new link, its own frame
    if (p[1] == 2) caps_due.store(true);     // the menu's caps again on it
    if (osd) {
        osd->update_artosyn_link_state(p[1]);
        osd->signal_render(prof::kWakeLink);
    }
}

// MCS change (event 1): slot, direction (1 receive, 0 transmit), new, old -
// confirmed against the polled MCS and the chip's own log ("Slot 0 RX MCS 8
// -> 10"). The radio moves between rungs about once a second while linked,
// which the 3 s poll never showed. The receive MCS goes to the HUD at once,
// and the stats thread is asked to refresh the rest (the data rate) early.
void Ar8030Source::on_mcs_event(const uint8_t* p) {
    if (p[0] != (uint8_t)slot || p[1] != 1 || p[2] > 15) return;
    stat_mcs_.store(p[2]);
    stats_soon_.store(true);
    if (osd) {
        osd->update_artosyn_rx_mcs(p[2]);
        osd->signal_render(prof::kWakeLink);
    }
}

// Channel change (2) and bandwidth change (12, {0, 1, new gear, old gear}):
// logged. A bandwidth change is also timed - the radio re-forms the link to
// change to 40 MHz, and that brief drop is not a new link (see the video loop).
void Ar8030Source::on_logged_event(int ev, const uint8_t* p) {
    printf("ar8030: event %d: %02X %02X %02X %02X\n", ev, p[0], p[1], p[2], p[3]);
    if (ev == 12) {
        bw_change_ms_.store(now_ms());
        if (p[2] <= 5) video_bw_idx_.store(p[2]);
        // Only a step to or from 40 MHz (gear 5) changes the radio's frame: 5,
        // 10 and 20 MHz share theirs, and a search across a step only fails.
        if (p[2] == 5 || p[3] == 5) frame_clock_.reframe();
    }
}

void Ar8030Source::subscribe_events() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    static const int evts[] = { 1, 0, 2, 8, 12 };   // stock's order
    for (size_t i = 0; i < sizeof(evts) / sizeof(evts[0]); i++) {
        bb_set_event_callback_t ev;
        memset(&ev, 0, sizeof(ev));
        ev.event = (bb_event_e)evts[i];
        ev.callback = ar_event_cb;
        ev.user = NULL;
        if (on_ar_libre()) {
            ev.user = this;
            switch (evts[i]) {
            case 0:  ev.callback = ar_event_link_cb; break;
            case 1:  ev.callback = ar_event_mcs_cb; break;
            case 2:  ev.callback = ar_event_log_cb<2>; break;
            case 12: ev.callback = ar_event_log_cb<12>; break;
            default: ev.user = NULL; break;
            }
        }
        printf("ar8030: subscribe event %d -> %d\n", evts[i],
               ar_ioctl(dev, BB_SET_EVENT_SUBSCRIBE, &ev, NULL));
    }
}

void Ar8030Source::configure_link() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;

    bb_mac_t macs[BB_CONFIG_MAX_SLOT_CANDIDATE];
    int n = ar_load_candidate_macs(macs, BB_CONFIG_MAX_SLOT_CANDIDATE);
    if (n > 0) {
        for (int i = 0; i < n; i++)
            printf("ar8030: candidate[%d] = %02X:%02X:%02X:%02X\n", i,
                   macs[i].addr[0], macs[i].addr[1], macs[i].addr[2], macs[i].addr[3]);
        bb_set_candidate_t cand;
        memset(&cand, 0, sizeof(cand));
        cand.slot = BB_SLOT_0;
        cand.mac_num = (uint8_t)n;
        memcpy(cand.mac_tab, macs, sizeof(bb_mac_t) * n);
        printf("ar8030: BB_SET_CANDIDATES -> %d\n", ar_ioctl(dev, BB_SET_CANDIDATES, &cand, NULL));
    } else {
        // Empty on a goggle never paired under stock (fpvOS merges stock's
        // list off the NAND at boot) and never bound here: not a fault, just
        // nothing to link with yet.
        printf("ar8030: no paired air units in /factory/user_cfg.json - bind one from the menu\n");
    }

    // Which side are we? The VRX runs as BB_ROLE_DEV (stock logs "rold=DEV"),
    // and a DEV associates by being told the AP's MAC - BB_SET_CANDIDATES is
    // the AP-side command and does nothing for us on its own.
    bb_get_status_in_t sin;
    memset(&sin, 0, sizeof(sin));
    sin.user_bmp = 0x03FF;
    bb_get_status_out_t sout;
    memset(&sout, 0, sizeof(sout));
    if (ar_ioctl(dev, BB_GET_STATUS, &sin, &sout) == 0) {
        printf("ar8030: role=%u (%s) mode=%u cfg_sbmp=0x%02x rt_sbmp=0x%02x\n",
               sout.role, sout.role == BB_ROLE_AP ? "AP" : "DEV", sout.mode,
               sout.cfg_sbmp, sout.rt_sbmp);
        if (sout.role == BB_ROLE_DEV && n > 0) {
            // Which of the paired units to name before the link: the one last
            // linked (identify_peer() saves it), not simply the oldest entry
            // of stock's pairing ring - that named an air unit that was not
            // there and kept every per-air setting under its key. The link
            // itself goes by the whole candidate list; identify_peer() fixes
            // the choice once the radio says who answered.
            int idx = (ap_index >= 0 && ap_index < n) ? ap_index : 0;
            if (ap_index == 0) {
                const unsigned last = (unsigned)Settings::getInstance().getInt("ar8030_last_peer", 0);
                for (int i = 0; i < n; i++) {
                    const unsigned m = ((unsigned)macs[i].addr[0] << 24) | ((unsigned)macs[i].addr[1] << 16) |
                                       ((unsigned)macs[i].addr[2] << 8) | macs[i].addr[3];
                    if (last && m == last) { idx = i; break; }
                }
            }
            set_sky_key_from_mac(macs[idx].addr);   // per-VTX settings namespace
            load_sky_link();                        // its TX power
            bb_set_ap_mac_t ap;
            memset(&ap, 0, sizeof(ap));
            ap.mac = macs[idx];
            printf("ar8030: BB_SET_AP_MAC(%02X:%02X:%02X:%02X) -> %d\n",
                   ap.mac.addr[0], ap.mac.addr[1], ap.mac.addr[2], ap.mac.addr[3],
                   ar_ioctl(dev, BB_SET_AP_MAC, &ap, NULL));
        }
    }

    // Pin the transmit power before anything keys the PA: left alone it sits
    // at the config default of up to 28 dBm, which browns the board out as
    // soon as we transmit. Honours the saved level including its AUTO flag.
    apply_tx_power(tx_power_mw);

    if (bandwidth < 0) {
        printf("ar8030: bandwidth left to the link (ar8030_bw unset)\n");
    } else {
        bb_set_bandwidth_t bw;
        memset(&bw, 0, sizeof(bw));
        bw.slot = BB_SLOT_0;
        bw.dir = BB_DIR_TX;
        bw.bandwidth = (uint8_t)bandwidth;
        int rc_bw = ar_ioctl(dev, BB_SET_BANDWIDTH, &bw, NULL);
        printf("ar8030: BB_SET_BANDWIDTH(%d) -> %d\n", bandwidth, rc_bw);
    }
    // Bandwidth answers -7 *here* because the link is not up yet, but once the
    // handshake has run every value 0-5 is accepted (verified twice with
    // --debug-probe-rf). So the -7 is a timing artifact, not a missing
    // capability, and the control is offered regardless.
    rf_caps |= RF_CAP_BW;

    // MCS mode and LNA are set to the values we want anyway (auto/auto), so
    // this doubles as a harmless capability check.
    bb_set_mcs_mode_t mm;
    memset(&mm, 0, sizeof(mm));
    mm.auto_mode = 1;
    int rc_mcs = ar_ioctl(dev, BB_SET_MCS_MODE, &mm, NULL);
    printf("ar8030: BB_SET_MCS_MODE(auto=1) -> %d\n", rc_mcs);
    if (rc_mcs == 0) rf_caps |= RF_CAP_MCS;

    bb_set_lna_mode_t ln;
    memset(&ln, 0, sizeof(ln));
    ln.mode = -1;                       // -1 = automatic gain
    int rc_lna = ar_ioctl(dev, BB_SET_LNA_MODE, &ln, NULL);
    printf("ar8030: BB_SET_LNA_MODE(auto) -> %d\n", rc_lna);
    if (rc_lna == 0) rf_caps |= RF_CAP_LNA;

    // Channel adaptation: with auto_mode=1 the baseband scans/hops to find its
    // AP, which is what we want when we do not know the air unit's channel.
    // Only in manual mode does an explicit BB_SET_FREQ mean anything.
    bb_set_chan_mode_t cm;
    memset(&cm, 0, sizeof(cm));
    cm.auto_mode = chan_auto ? 1 : 0;
    int rc_cm = ar_ioctl(dev, BB_SET_CHAN_MODE, &cm, NULL);
    printf("ar8030: BB_SET_CHAN_MODE(auto=%d) -> %d\n", cm.auto_mode, rc_cm);
    if (rc_cm == 0) rf_caps |= RF_CAP_CHAN;

    enable_ranging();

    printf("ar8030: RF capabilities: power=%d mcs=%d lna=%d bandwidth=%d chan=%d\n",
           !!(rf_caps & RF_CAP_POWER), !!(rf_caps & RF_CAP_MCS),
           !!(rf_caps & RF_CAP_LNA),   !!(rf_caps & RF_CAP_BW),
           !!(rf_caps & RF_CAP_CHAN));
    if (osd) osd->set_rf_caps(rf_caps);

    if (chan_auto) {
        printf("ar8030: channel auto mode - skipping fixed BB_SET_FREQ\n");
        return;
    }

    bb_set_freq_t fr;
    memset(&fr, 0, sizeof(fr));
    fr.user = 0;
    fr.dir_bmp = (1 << BB_DIR_TX) | (1 << BB_DIR_RX);
    fr.freq_khz = freq_khz;
    printf("ar8030: BB_SET_FREQ(%u kHz) -> %d\n", freq_khz,
           ar_ioctl(dev, BB_SET_FREQ, &fr, NULL));
}


// ---- binding ---------------------------------------------------------------
//
// Ask the radio to listen for an air unit announcing itself. bind_begin()
// opens the window, bind_step() polls it, bind_finish() closes it and - on
// success - writes the peer into /factory/user_cfg.json so the pairing is
// still there after a reboot.

bool Ar8030Source::bind_begin() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) {
        printf("ar8030: bind requested but the baseband is not open yet\n");
        return false;
    }
    // Stock reads the role first and uses it to decide what to write once a
    // peer is found: a DEV takes BB_SET_AP_MAC, an AP takes BB_SET_CANDIDATES.
    // Read it here rather than at the end, so a status failure aborts before
    // the radio is put into pairing mode.
    {
        bb_get_status_in_t sin;
        memset(&sin, 0, sizeof(sin));
        sin.user_bmp = 0;                       // stock passes 0 here
        bb_get_status_out_t sout;
        memset(&sout, 0, sizeof(sout));
        if (ar_ioctl(dev, BB_GET_STATUS, &sin, &sout) != 0) {
            printf("ar8030: bind aborted - BB_GET_STATUS failed\n");
            return false;
        }
        bind_role_ = sout.role;
        printf("ar8030: prepare pair start slot_bmp=0x%02x role=%s\n",
               1 << BB_SLOT_0, bind_role_ == BB_ROLE_AP ? "AP" : "DEV");
    }

    // Do NOT clear the candidate list or the pinned AP here.
    //
    // An earlier version did, on the theory that our short 22-byte
    // BB_SET_CANDIDATES struct had left garbage in the chip. That reasoning
    // was sound but the cure was worse: a bare-RPC pair window that skips
    // both clears pairs within seconds, every time, while kestrel with the
    // clears in place never caught anything across five attempts on real
    // hardware. Stock does not clear either. The likeliest reading is that
    // mac_num = 0 means "accept nobody" on this firmware rather than "accept
    // anybody", which turns the pair window into a window onto nothing.
    //
    // Set KESTREL_BIND_CLEAR=1 to put the clears back for comparison.
    const char *doclear = getenv("KESTREL_BIND_CLEAR");
    if (doclear && doclear[0] == '1') {
        bb_conf_candidates_t none;
        memset(&none, 0, sizeof(none));
        none.slot    = BB_SLOT_0;
        none.mac_num = 0;
        printf("ar8030: clear candidates for bind -> %d\n",
               ar_ioctl(dev, BB_SET_CANDIDATES, &none, NULL));
        bb_set_ap_mac_t clear;
        memset(&clear, 0, sizeof(clear));
        printf("ar8030: clear AP MAC for bind -> %d\n",
               ar_ioctl(dev, BB_SET_AP_MAC, &clear, NULL));
    }

    memset(bind_seen_, 0, sizeof(bind_seen_));
    memset(bind_seen_n_, 0, sizeof(bind_seen_n_));
    {
        const uint64_t t = now_ms();
        bind_had_data_ = (last_ctrl_ms && t - last_ctrl_ms < 3000) ||
                         (last_data_ms && t - last_data_ms < 3000);
        if (bind_had_data_)
            printf("ar8030: an air unit is already talking - binding anyway "
                   "(a second unit); traffic will not stop this window\n");
    }
    // Start the idle-read counter fresh. An unbound goggle has usually been
    // sitting at "no video" for a long time already, so without this the
    // teardown timer could be one read away from firing when the window ends.
    read_timeouts = 0;

    bb_set_pair_mode_t pm;
    memset(&pm, 0, sizeof(pm));
    pm.start = 1;
    pm.slot_bmp = (1 << BB_SLOT_0);
    int r = ar_ioctl(dev, BB_SET_PAIR_MODE, &pm, NULL);
    printf("ar8030: BB_SET_PAIR_MODE(start) -> %d\n", r);
    if (r != 0) return false;

    printf("ar8030: BINDING - hold the air unit's bind button until its LED turns\n"
           "        RED, which is its pairing mode (%ds to do it)\n",
           kBindWindowS);
    bind_started_ms_   = now_ms();
    bind_next_poll_ms_ = bind_started_ms_ + 500;
    bind_done_ms_      = 0;
    bind_secs_left.store(kBindWindowS);
    bind_state.store(BIND_RUNNING);
    if (osd) osd->signal_render(prof::kWakeLink);
    return true;
}

void Ar8030Source::bind_finish(bool ok, const uint8_t peer[4], bool cancelled) {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (dev) {
        bb_set_pair_mode_t pm;
        memset(&pm, 0, sizeof(pm));
        pm.start = 0;
        pm.slot_bmp = (1 << BB_SLOT_0);
        printf("ar8030: BB_SET_PAIR_MODE(stop) -> %d\n",
               ar_ioctl(dev, BB_SET_PAIR_MODE, &pm, NULL));
    }
    bind_done_ms_ = now_ms();
    bind_secs_left.store(0);

    if (cancelled) {
        // The user's own doing, not an outcome: nothing to report and nothing
        // to hold on screen. Straight back to idle, window closed.
        bind_peer.store(0);
        bind_state.store(BIND_IDLE);
        bind_done_ms_ = 0;
        if (osd) osd->signal_render(prof::kWakeLink);
        report_link_status();
        return;
    }

    if (ok && peer) {
        bind_peer.store(((unsigned)peer[0] << 24) | ((unsigned)peer[1] << 16) |
                        ((unsigned)peer[2] << 8)  |  (unsigned)peer[3]);
        bind_state.store(BIND_OK);
        if (osd) osd->signal_render(prof::kWakeLink);

        // Program the peer the way stock does, which depends on which side we
        // are. A DEV is told the one AP to associate with; an AP is given a
        // candidate list of DEVs allowed to join it. The goggle is always the
        // DEV, but the branch costs nothing and documents the asymmetry.
        if (dev) {
            if (bind_role_ == BB_ROLE_AP) {
                bb_conf_candidates_t cand;
                memset(&cand, 0, sizeof(cand));
                cand.slot    = BB_SLOT_0;
                cand.mac_num = 1;
                memcpy(cand.mac_tab[0].addr, peer, BB_MAC_LEN);
                printf("ar8030: BB_SET_CANDIDATES(peer) -> %d\n",
                       ar_ioctl(dev, BB_SET_CANDIDATES, &cand, NULL));
            } else {
                bb_set_ap_mac_t ap;
                memset(&ap, 0, sizeof(ap));
                memcpy(ap.mac.addr, peer, BB_MAC_LEN);
                printf("ar8030: BB_SET_AP_MAC(%02X:%02X:%02X:%02X) -> %d\n",
                       peer[0], peer[1], peer[2], peer[3],
                       ar_ioctl(dev, BB_SET_AP_MAC, &ap, NULL));
            }
        }

        if (save_paired_mac(peer))
            printf("ar8030: bound peer saved to /factory/user_cfg.json\n");
        else
            printf("ar8030: WARNING could not save the bind - it will be lost on reboot\n");
    } else {
        bind_peer.store(0);
        bind_state.store(BIND_FAILED);
        if (osd) osd->signal_render(prof::kWakeLink);
        printf("ar8030: binding timed out after %ds - no air unit answered.\n"
               "        The air unit only announces itself in ITS pairing mode:\n"
               "        hold its bind button until the LED turns RED, then retry.\n",
               kBindWindowS);
    }
    report_link_status();
}

void Ar8030Source::bind_step() {
    const uint64_t now = now_ms();
    int st = bind_state.load();

    // Let a finished result sit on screen a few seconds, then clear it.
    if ((st == BIND_OK || st == BIND_FAILED || st == BIND_LINKED) && bind_done_ms_ &&
        now - bind_done_ms_ > (uint64_t)kBindResultHoldMs) {
        bind_state.store(BIND_IDLE);
        bind_done_ms_ = 0;
        st = BIND_IDLE;
        if (osd) osd->signal_render(prof::kWakeLink);
    }

    // A request parked by the menu or the front-panel button.
    if (bind_request.exchange(0) == 1) {
        if (st == BIND_RUNNING) {
            // Pressing again while a window is open cancels it. With a
            // two-minute window there has to be a way out other than waiting
            // it out or pulling the power.
            printf("ar8030: bind cancelled by a second press\n");
            bind_finish(false, NULL, true);
            return;
        }
        if (!bind_begin()) {
            bind_state.store(BIND_FAILED);
            bind_done_ms_ = now;
            if (osd) osd->signal_render(prof::kWakeLink);
        }
        return;
    }

    if (bind_state.load() != BIND_RUNNING) return;

    // NOT "close the window when data arrives". That was tried and it cannot
    // work: BB_SET_PAIR_MODE suspends normal association at the radio, so a
    // known air unit does not link and no data flows for as long as the
    // window is open. Confirmed on hardware - the link only comes up once
    // binding is cancelled. Any rule keyed to traffic is dead code here.
    //
    // What the window costs is therefore a paused link, which is why the
    // caption says so and why a second press cancels.

    int elapsed = (int)((now - bind_started_ms_) / 1000);
    int left = kBindWindowS - elapsed;
    if (left < 0) left = 0;
    // Repaint when the displayed second changes. The OSD renders on frame
    // arrival, and during a bind there is by definition no video yet, so
    // without this the countdown would freeze at 30 on screen.
    if (left != bind_secs_left.exchange(left) && osd) osd->signal_render(prof::kWakeLink);

    // Stock polls every 20ms. We poll as often as the RX loop comes round,
    // capped at that rate; the stability rule below is what matters, not the
    // exact cadence.
    if (now < bind_next_poll_ms_) return;
    bind_next_poll_ms_ = now + 20;

    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (dev) {
        // in = NULL: the daemon's length table gives BB_GET_PAIR_RESULT an
        // input length of 0, and stock passes a null input pointer. out is
        // 164 bytes; the struct is now exactly that (see bb_client.h), but
        // the oversized buffer stays as belt and braces against a daemon that
        // writes more than its table claims.
        alignas(8) uint8_t pr_buf[256];
        memset(pr_buf, 0, sizeof(pr_buf));
        bb_get_pair_out_t& pr = *reinterpret_cast<bb_get_pair_out_t*>(pr_buf);
        if (ar_ioctl(dev, BB_GET_PAIR_RESULT, NULL, pr_buf) == 0 && pr.slot_bmp) {
            // Stock does not accept the first MAC it sees. It requires the
            // same address to come back unchanged 11 polls running, and
            // restarts the count whenever it changes. Early in a window the
            // radio reports addresses that then vanish, so taking the first
            // one binds to noise.
            for (int i = 0; i < BB_SLOT_MAX; i++) {
                if (!(pr.slot_bmp & (1 << i))) continue;
                if (memcmp(bind_seen_[i], pr.peer_mac[i].addr, BB_MAC_LEN) != 0) {
                    memcpy(bind_seen_[i], pr.peer_mac[i].addr, BB_MAC_LEN);
                    bind_seen_n_[i] = 1;
                    // Log the first sighting of any address. When a bind
                    // fails this separates "the radio heard nothing" from
                    // "the radio heard something and we rejected it", which
                    // are entirely different bugs.
                    printf("ar8030: bind sees slot%d %02X:%02X:%02X:%02X\n", i,
                           bind_seen_[i][0], bind_seen_[i][1],
                           bind_seen_[i][2], bind_seen_[i][3]);
                } else if (bind_seen_n_[i] < 1000) {
                    bind_seen_n_[i]++;
                }
            }
            const int slot = BB_SLOT_0;
            if (bind_seen_n_[slot] > kBindStableReads) {
                printf("ar8030: BOUND! slot_bmp=0x%02x peer=%02X:%02X:%02X:%02X"
                       " (stable over %d reads)\n",
                       pr.slot_bmp, bind_seen_[slot][0], bind_seen_[slot][1],
                       bind_seen_[slot][2], bind_seen_[slot][3],
                       bind_seen_n_[slot]);
                bind_finish(true, bind_seen_[slot]);
                return;
            }
        }
    }
    if (left <= 0) bind_finish(false, NULL);
}

// Write the bound air unit into /factory/user_cfg.json, the way stock does.
//
// Stock's save_ap_candidate() keeps a 100-entry RING, not a single peer:
// "bb_mac_addr_0".."bb_mac_addr_99", each a JSON array of four "0xNN" strings,
// with "save_candidate_position" holding the next slot to write and advancing
// (pos + 1) % 100. A MAC already present anywhere in the ring is left alone
// rather than rewritten - that is why re-binding the same air unit does not
// churn the file. Matching this matters because the whole ring is what gets
// pushed back to the radio as the candidate list.
//
// There is no JSON library here, so the file is parsed with string scans and
// rewritten whole. Unknown keys are not preserved; the stock file has only
// these three, and inventing others would be worse than dropping them.
bool Ar8030Source::save_paired_mac(const uint8_t mac[4]) {
    static const char *kPath = "/factory/user_cfg.json";
    static const char *kTmp  = "/factory/.user_cfg.json.new";
    static const int   kRing = 100;

    int     pwr_cal = 0, save_pos = 0;
    uint8_t ring[kRing][BB_MAC_LEN];
    bool    have[kRing];
    memset(ring, 0, sizeof(ring));
    memset(have, 0, sizeof(have));

    if (FILE *f = fopen(kPath, "r")) {
        std::vector<char> buf(64 * 1024);
        size_t n = fread(buf.data(), 1, buf.size() - 1, f);
        buf[n] = 0;
        fclose(f);
        const char *b = buf.data();
        if (const char *p = strstr(b, "\"gnd_power_cal_dbm\""))
            if (const char *c = strchr(p, ':')) pwr_cal = atoi(c + 1);
        if (const char *p = strstr(b, "\"save_candidate_position\""))
            if (const char *c = strchr(p, ':')) save_pos = atoi(c + 1);
        for (int i = 0; i < kRing; i++) {
            char key[32];
            snprintf(key, sizeof(key), "\"bb_mac_addr_%d\"", i);
            const char *p = strstr(b, key);
            if (!p) continue;
            const char *q = strchr(p, '[');
            if (!q) continue;
            // Stop at the closing bracket so a short array cannot run on into
            // the next entry and silently borrow its bytes.
            const char *endq = strchr(q, ']');
            int got = 0;
            while (q && got < BB_MAC_LEN) {
                const char *hx = strstr(q, "0x");
                if (!hx || (endq && hx > endq)) break;
                char *end = NULL;
                ring[i][got++] = (uint8_t)strtoul(hx, &end, 16);
                q = end;
            }
            have[i] = (got == BB_MAC_LEN);
        }
    }

    // Already known? Stock says "mac same, not need save." and leaves the file
    // untouched, so re-binding the same air unit is a no-op on disk.
    for (int i = 0; i < kRing; i++)
        if (have[i] && memcmp(ring[i], mac, BB_MAC_LEN) == 0) {
            printf("ar8030: %02X:%02X:%02X:%02X already saved (slot %d)\n",
                   mac[0], mac[1], mac[2], mac[3], i);
            return true;
        }

    if (save_pos < 0 || save_pos >= kRing) save_pos = 0;
    memcpy(ring[save_pos], mac, BB_MAC_LEN);
    have[save_pos] = true;
    int next_pos = (save_pos + 1) % kRing;

    mkdir("/factory", 0755);          // harmless if it is already there
    FILE *f = fopen(kTmp, "w");
    if (!f) {
        printf("ar8030: cannot write %s: %s\n", kTmp, strerror(errno));
        return false;
    }
    fprintf(f, "{\n    \"gnd_power_cal_dbm\": %d,\n"
               "    \"save_candidate_position\": %d", pwr_cal, next_pos);
    for (int i = 0; i < kRing; i++) {
        if (!have[i]) continue;
        fprintf(f, ",\n    \"bb_mac_addr_%d\": [\n"
                   "        \"0x%02X\",\n        \"0x%02X\",\n"
                   "        \"0x%02X\",\n        \"0x%02X\"\n    ]",
                i, ring[i][0], ring[i][1], ring[i][2], ring[i][3]);
    }
    fprintf(f, "\n}\n");
    // The identity file is the one thing that must never end up truncated, so
    // it is written beside the original and moved over it only once complete.
    bool ok = (fflush(f) == 0) && (fsync(fileno(f)) == 0);
    fclose(f);
    if (!ok || rename(kTmp, kPath) != 0) {
        printf("ar8030: cannot replace %s: %s\n", kPath, strerror(errno));
        unlink(kTmp);
        return false;
    }
    chmod(kPath, 0600);               // stock ships it owner-only
    sync();                           // stock sync()s after saving; so do we
    printf("ar8030: saved %02X:%02X:%02X:%02X at ring slot %d (next %d)\n",
           mac[0], mac[1], mac[2], mac[3], save_pos, next_pos);
    return true;
}



// ---- per-air-unit camera settings -----------------------------------------
//
// Keyed by the AP MAC we associate with, so a second paired VTX keeps its own
// settings. The MAC is known from /factory/user_cfg.json before the link is up,
// which is what we need: the SET_CONFIG frames go out during the handshake.
void Ar8030Source::set_sky_key_from_mac(const uint8_t mac[4]) {
    char buf[32];
    snprintf(buf, sizeof(buf), "sky_%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3]);
    sky_cfg_key = buf;
}

// The per-air settings key and the AP MAC follow the unit that answered, read
// from the link status at the handshake (before the camera config goes out).
// The key chosen at start is a guess from the pairing list.
void Ar8030Source::identify_peer() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    bb_get_status_in_t in;
    memset(&in, 0, sizeof(in));
    in.user_bmp = 0x03FF;
    bb_get_status_out_t st;
    memset(&st, 0, sizeof(st));
    if (ar_ioctl(dev, BB_GET_STATUS, &in, &st) != 0 || st.role != BB_ROLE_DEV) return;
    const uint8_t *p = st.link_status[0].peer_mac.addr;
    const bool blank = (!p[0] && !p[1] && !p[2] && !p[3]) ||
                       (p[0] == 0xFF && p[1] == 0xFF && p[2] == 0xFF && p[3] == 0xFF);
    printf("ar8030: linked peer %02X:%02X:%02X:%02X (state %u)%s\n", p[0], p[1], p[2], p[3],
           st.link_status[0].state, blank ? " - not reported, settings key kept" : "");
    if (blank || st.link_status[0].state != 2) return;
    // Only a unit this goggle is paired with: Open8030 reports a placeholder
    // (66:00:00:00) there, and adopting it moved every setting to a new key.
    bb_mac_t macs[BB_CONFIG_MAX_SLOT_CANDIDATE];
    const int n = ar_load_candidate_macs(macs, BB_CONFIG_MAX_SLOT_CANDIDATE);
    bool paired = false;
    for (int i = 0; i < n && !paired; i++) paired = memcmp(macs[i].addr, p, BB_MAC_LEN) == 0;
    if (!paired) {
        printf("ar8030: linked peer is not a paired unit - settings key kept\n");
        return;
    }
    char key[32];
    snprintf(key, sizeof(key), "sky_%02X%02X%02X%02X", p[0], p[1], p[2], p[3]);
    Settings::getInstance().set("ar8030_last_peer",
        (int)(((unsigned)p[0] << 24) | ((unsigned)p[1] << 16) | ((unsigned)p[2] << 8) | p[3]));
    if (sky_cfg_key == key) return;
    printf("ar8030: settings key %s -> %s\n", sky_cfg_key.c_str(), key);
    sky_cfg_key = key;
    load_sky_link();
    apply_tx_power(tx_power_mw);
    bb_set_ap_mac_t ap;
    memset(&ap, 0, sizeof(ap));
    memcpy(ap.mac.addr, p, BB_MAC_LEN);
    printf("ar8030: BB_SET_AP_MAC(%02X:%02X:%02X:%02X) -> %d\n", p[0], p[1], p[2], p[3],
           ar_ioctl(dev, BB_SET_AP_MAC, &ap, NULL));
}

void Ar8030Source::load_sky_config() {
    // Seed from the stock capture so every field we do not persist keeps the
    // value stock boots with, then let stored settings override.
    const uint8_t *f = ar_hs_5;
    unsigned lf = f[4] | (f[5] << 8);
    sky::read_config_payload(f + 7, (lf >> 4) - 1, &sky_cfg);
    if (sky_cfg_key.empty()) return;

    Settings &st = Settings::getInstance();
    const std::string k = sky_cfg_key + "_";
    sky_cfg.ch0_w       = (uint16_t)st.getInt(k + "ch0_w",  sky_cfg.ch0_w);
    sky_cfg.ch0_h       = (uint16_t)st.getInt(k + "ch0_h",  sky_cfg.ch0_h);
    sky_cfg.ch0_fps     = (uint8_t) st.getInt(k + "ch0_fps", sky_cfg.ch0_fps);
    sky_cfg.ev_x10      = (int8_t)  st.getInt(k + "ev",      sky_cfg.ev_x10);
    sky_cfg.scenes      = (uint8_t) st.getInt(k + "scene",   sky_cfg.scenes);
    sky_cfg.saturation  = (uint8_t) st.getInt(k + "sat",     sky_cfg.saturation);
    sky_cfg.sharpness   = (uint8_t) st.getInt(k + "sharp",   sky_cfg.sharpness);
    sky_cfg.contrast    = (uint8_t) st.getInt(k + "contrast", sky_cfg.contrast);
    sky_cfg.awb_cct     = (uint16_t)st.getInt(k + "awb",     sky_cfg.awb_cct);
    sky_cfg.anti_flicker= (uint8_t) st.getInt(k + "flicker", sky_cfg.anti_flicker);
    sky_cfg.angle       = (uint8_t) st.getInt(k + "flip",    sky_cfg.angle);
    sky_cfg.dnr_3d      = (uint8_t) st.getInt(k + "dnr3d",   sky_cfg.dnr_3d);
    sky_cfg.focus_en    = (uint8_t) st.getInt(k + "focus",   sky_cfg.focus_en);
    printf("ar8030: [%s] restored camera cfg: ch0=%ux%u@%u ev=%d sat=%u sharp=%u "
           "contrast=%u scene=%u awb=%u flicker=%u angle=%u focus=%u\n",
           sky_cfg_key.c_str(), sky_cfg.ch0_w, sky_cfg.ch0_h, sky_cfg.ch0_fps,
           sky_cfg.ev_x10, sky_cfg.saturation, sky_cfg.sharpness, sky_cfg.contrast,
           sky_cfg.scenes, sky_cfg.awb_cct, sky_cfg.anti_flicker, sky_cfg.angle,
           sky_cfg.focus_en);
    // The video mode is this air unit's too: the one the handshake asks for,
    // so nothing asks for another after it (a mode picked for the last unit
    // switched this one's encoder twice at every link-up).
    if (mode_chn == 0 && sky_cfg.ch0_w && sky_cfg.ch0_h && sky_cfg.ch0_fps) {
        mode_w = sky_cfg.ch0_w; mode_h = sky_cfg.ch0_h; mode_fps = sky_cfg.ch0_fps;
        for (int i = 0; osd && i < sky::kFpvModeCount; i++)
            if (sky::kFpvModes[i].w == mode_w && sky::kFpvModes[i].h == mode_h && sky::kFpvModes[i].fps == mode_fps)
                osd->set_video_mode_current(i);
    }

    if (osd) {
        osd->set_camera_config(sky_cfg.ev_x10, sky_cfg.saturation, sky_cfg.contrast,
                               sky_cfg.sharpness, sky_cfg.scenes,
                               sky_cfg.awb_cct, sky_cfg.angle,
                               sky_cfg.dnr_3d, sky_cfg.focus_en);
    }

    // Point the menu row at the mode we are actually about to push. main()
    // publishes the list before the link exists, so without this the row shows
    // the built-in default no matter what the air unit is running.
    if (osd) {
        for (int i = 0; i < sky::kFpvModeCount; i++) {
            if (sky::kFpvModes[i].w == sky_cfg.ch0_w &&
                sky::kFpvModes[i].h == sky_cfg.ch0_h &&
                sky::kFpvModes[i].fps == sky_cfg.ch0_fps) {
                osd->set_video_mode_current(i);
                break;
            }
        }
    }
}

void Ar8030Source::save_sky_config() {
    if (sky_cfg_key.empty()) return;
    Settings &st = Settings::getInstance();
    const std::string k = sky_cfg_key + "_";
    st.set(k + "ch0_w",   (int)sky_cfg.ch0_w);
    st.set(k + "ch0_h",   (int)sky_cfg.ch0_h);
    st.set(k + "ch0_fps", (int)sky_cfg.ch0_fps);
    st.set(k + "ev",      (int)sky_cfg.ev_x10);
    st.set(k + "scene",   (int)sky_cfg.scenes);
    st.set(k + "sat",     (int)sky_cfg.saturation);
    st.set(k + "sharp",   (int)sky_cfg.sharpness);
    st.set(k + "contrast",(int)sky_cfg.contrast);
    st.set(k + "awb",     (int)sky_cfg.awb_cct);
    st.set(k + "flicker", (int)sky_cfg.anti_flicker);
    st.set(k + "flip",    (int)sky_cfg.angle);     // read back as "flip" (load_sky_config)
    st.set(k + "dnr3d",   (int)sky_cfg.dnr_3d);
    st.set(k + "focus",   (int)sky_cfg.focus_en);
}

// The radio side of the same: the TX power this air unit last ran with. Until
// it has its own, the goggle-wide tx_power_mw main() loaded stands in. The
// channel is not kept - the air unit decides it, and the goggle searches.
void Ar8030Source::load_sky_link() {
    if (sky_cfg_key.empty()) return;
    tx_power_want_mw = Settings::getInstance().getInt(sky_cfg_key + "_tx_power_mw", tx_power_want_mw);
    tx_power_mw  = ar_pwr_fit(air_prj.load(), tx_power_want_mw);
    tx_power_dbm = kArPwrLevels[ar_pwr_index(tx_power_mw)].dbm;
    printf("ar8030: [%s] restored radio cfg: power=%s (chosen %s)\n", sky_cfg_key.c_str(),
           kArPwrLevels[ar_pwr_index(tx_power_mw)].label, kArPwrLevels[ar_pwr_index(tx_power_want_mw)].label);
}

// The power chosen, not the one a smaller air unit is held to: a Lite's fit
// saved here would cap the next Lite+ (the units can share one key - one
// pairing - and did, 2026-10-12).
void Ar8030Source::save_sky_link() {
    if (sky_cfg_key.empty()) return;
    Settings::getInstance().set(sky_cfg_key + "_tx_power_mw", tx_power_want_mw);
}

// Rebuild one captured SET_CONFIG frame around our live settings. The stock
// body is the template, so the bytes we have not identified keep stock values;
// only the known fields and the timestamp change.
// Note for anyone re-sending a config frame outside the handshake: give it a
// fresh sequence number. Replaying the captured seq makes the air treat the
// frame as a duplicate and drop it silently - it is not acked and has no
// effect. Proved while chasing 3D DNR (references/sky-commands.md).
std::vector<uint8_t> Ar8030Source::build_config_frame(const uint8_t *tmpl, size_t len) {
    unsigned lf = tmpl[4] | (tmpl[5] << 8);
    size_t bodylen = (lf >> 4) - 1;
    std::vector<uint8_t> body(tmpl + 7, tmpl + 7 + bodylen);
    sky::patch_config_payload(body.data(), body.size(), sky_cfg);
    // Body offset 45 (u16 LE) is the TX power, mW, which the air unit applies
    // after the power frame. Left as captured it was stock's 500mW Auto (501):
    // every handshake - each link-up, each 2.5 s of quiet video - undid the
    // chosen power, and asked an Ascent Lite (100 mW) for 26 dBm.
    const int mw = kArPwrLevels[ar_pwr_index(tx_power_mw)].mw;
    if (body.size() >= 47) { body[45] = (uint8_t)(mw & 0xFF); body[46] = (uint8_t)((mw >> 8) & 0xFF); }
    sky_proto.set_seq(tmpl[3]);                       // replaying stock's handshake
    return sky_proto.build(tmpl[6], body.data(), body.size());
}

// The air unit stays silent until the ground unit sends these frames on the
// port-2 (telemetry) socket - captured from a clean boot of the stock app.
// Opening the video socket alone yields at most a short burst of whatever was
// already buffered.
void Ar8030Source::send_air_handshake() {
    // The acquiring screen distinguishes "link up, nothing asked for yet"
    // from "asked, nothing came back", which is the difference between
    // waiting and being ignored.
    if (osd) osd->notify_handshake_sent();
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;

    if (ctrl_sockfd < 0) {
        bb_sock_opt_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.tx_buf_size = 0x0800;
        opt.rx_buf_size = 0x0800;          // stock uses 2 KB here, 56 KB for video
        ctrl_sockfd = bb_socket_open(dev, (bb_slot_e)slot, 2,
                                     BB_SOCK_FLAG_RX | BB_SOCK_FLAG_TX, &opt);
        printf("ar8030: control socket (port 2) fd=%d\n", ctrl_sockfd);
        if (ctrl_sockfd < 0) return;
    }

    identify_peer();
    load_sky_config();
    for (size_t i = 0; i < ar_handshake_n; i++) {
        const uint8_t *data = ar_handshake[i].data;
        size_t         len  = ar_handshake[i].len;
        // The two SET_CONFIG frames carry the camera settings the VTX will run
        // with, so build them from our stored config instead of replaying the
        // capture - otherwise a menu change is undone at the next handshake.
        std::vector<uint8_t> rebuilt;
        const char *from = "";
        if (data[6] == sky::CMD_SET_CONFIG) {
            rebuilt = build_config_frame(data, len);
            data = rebuilt.data(); len = rebuilt.size();
            from = "   (video config, from stored settings)";
        } else if (data[6] == sky::CMD_SET_BB_PWR) {
            // Stock's two power frames carry its own 500mW Auto. This air
            // unit's power instead, so a link-up never sets it to anything
            // else first. Same u16 LE payload as request_setting(CAM_PWR).
            const int mw = kArPwrLevels[ar_pwr_index(tx_power_mw)].mw;
            const uint8_t v[2] = { (uint8_t)(mw & 0xFF), (uint8_t)((mw >> 8) & 0xFF) };
            sky_proto.set_seq(data[3]);
            rebuilt = sky_proto.build(data[6], v, sizeof(v));
            data = rebuilt.data(); len = rebuilt.size();
            from = "   (air power, from stored settings)";
        }
        int w = bb_socket_write(ctrl_sockfd, data, (uint32_t)len, 500);
        printf("ar8030: handshake[%zu] seq=0x%02X len=%zu -> %d%s\n",
               i, data[3], len, w, from);
        usleep(250000);  // spread the PA burst: back-to-back TX appears to brown the board out
        if (*should_stop) { printf("ar8030: handshake aborted (stopping)\n"); return; }
    }

    // The replayed frames end at seq 0x09; anything we synthesise from here on
    // has to carry on from there or the air unit drops it as out of order.
    sky_proto.set_seq(ar_handshake[ar_handshake_n - 1].data[3] + 1);
    if (probe_rf) probe_rf_ioctls();
    if (probe_modes) probe_video_modes();
    // The resolution is the handshake's (this air unit's stored mode): no
    // mode switch after it - that is for a different mode picked in the menu.
}

// Read frames off the port-2 socket until one acknowledges want_cmd. The air
// unit interleaves unsolicited telemetry here, so we skip anything else.
bool Ar8030Source::read_sky_ack(uint8_t want_cmd, int timeout_ms) {
    if (ctrl_sockfd < 0) return false;
    uint8_t buf[512];
    for (int waited = 0; waited < timeout_ms; waited += 50) {
        if (*should_stop) return false;
        int n = bb_socket_read(ctrl_sockfd, buf, sizeof(buf), 50);
        if (n <= 0) continue;   // -1 is a plain timeout, not an error
        on_ctrl_bytes(buf, n);     // the air's reports, MSP, an awaited ack: not lost here
        // Frames can be batched in one read; walk every FE A5 boundary.
        for (int i = 0; i + 12 <= n; i++) {
            if (buf[i] != 0xFE || buf[i + 1] != 0xA5) continue;
            unsigned len = ((buf[i + 4] | (buf[i + 5] << 8)) >> 4) + 11;
            if (i + (int)len > n) continue;
            uint8_t cmd = 0, status = 0; bool is_ack = false;
            if (!sky::Proto::parse(buf + i, len, &cmd, &is_ack, &status)) continue;
            if (is_ack && cmd == want_cmd) {
                printf("ar8030: sky ack cmd=0x%02X status=%u (%s)\n",
                       cmd, status, status == 0 ? "accepted" : "REJECTED");
                return status == 0;
            }
        }
    }
    printf("ar8030: no ack for cmd 0x%02X within %d ms\n", want_cmd, timeout_ms);
    return false;
}

// The air unit pushes telemetry onto the port-2 socket continuously. Nothing
// consumed it after the startup handshake, so the SDK's per-socket RX ring
// (SOCK_LEN_APP_TO_DAEMON, 256 KB) filled up and every later packet was dropped
// by so_rpc_cb() with "recv data leak!! get = N , push= 0". Drain it every pass.
//
// NB: bb_socket_read() blocks forever on timeout <= 0 (it falls through to a
// bare pthread_cond_wait), so the poll below must pass a positive timeout.
void Ar8030Source::drain_control_socket() {
    if (ctrl_sockfd < 0) return;
    uint8_t buf[1024];
    for (int i = 0; i < 64; i++) {          // bounded: never starve the video read
        int n = read_queued(ctrl_sockfd, buf, sizeof(buf));
        if (n <= 0) break;                  // -1 here is just "ring empty"
        on_ctrl_bytes(buf, n);

        // Raw capture of whatever the air unit is sending, for offline
        // analysis. 4 MB is ~18 minutes at the measured 3.8 KB/s.
        if (!dump_path.empty() && dump_written < 4u * 1024 * 1024) {
            if (!dump_fp) {
                dump_fp = (void *)fopen(dump_path.c_str(), "wb");
                if (dump_fp) printf("ar8030: dumping port-2 to %s\n", dump_path.c_str());
                else dump_path.clear();
            }
            if (dump_fp) {
                fwrite(buf, 1, (size_t)n, (FILE *)dump_fp);
                fflush((FILE *)dump_fp);
                dump_written += (unsigned long long)n;
            }
        }
    }
}

// What the control socket brought in, wherever it was read (the main loop's
// drain, or a wait for a mode change's ack): the air's reports, the flight
// controller's MSP riding it, and the ack a camera setting is waiting for.
void Ar8030Source::on_ctrl_bytes(const uint8_t *buf, int n) {
    ctrl_bytes += (unsigned long long)n;
    last_ctrl_ms = now_ms();            // the air unit is talking to us
    scan_air_status(buf, n);
    if (sky_ack_.field >= 0) {
        // Frames can be batched in one read; walk every FE A5 boundary.
        for (int i = 0; i + 12 <= n; i++) {
            if (buf[i] != 0xFE || buf[i + 1] != 0xA5) continue;
            unsigned len = ((buf[i + 4] | (buf[i + 5] << 8)) >> 4) + 11;
            if (i + (int)len > n) continue;
            uint8_t cmd = 0, status = 0; bool is_ack = false;
            if (!sky::Proto::parse(buf + i, len, &cmd, &is_ack, &status)) continue;
            if (is_ack && cmd == sky_ack_.cmd) {
                sky_ack_done(status);
                break;
            }
        }
    }
    // --debug-msp-bb-port 2 means "the FC stream is inside the telemetry we
    // already drain" - tee it rather than opening a second socket on a
    // port this client already owns.
    if (msp_bb_port == 2 && osd) {
        msp_bytes += (unsigned long long)n;
        osd->update_msp_data(buf, (size_t)n);
    }
}

// The awaited camera setting acknowledged: accepted, it goes into the stored
// config, which every handshake sends the air unit again.
void Ar8030Source::sky_ack_done(int status) {
    printf("ar8030: sky ack cmd=0x%02X status=%d (%s)\n",
           sky_ack_.cmd, status, status == 0 ? "accepted" : "REJECTED");
    if (status == 0) {
        const int v = sky_ack_.value;
        switch (sky_ack_.field) {
            case CAM_3DNR:      sky_cfg.dnr_3d       = (uint8_t)v;  break;
            case CAM_EV:        sky_cfg.ev_x10       = (int8_t)v;   break;
            case CAM_SAT:       sky_cfg.saturation   = (uint8_t)v;  break;
            case CAM_CONTRAST:  sky_cfg.contrast     = (uint8_t)v;  break;
            case CAM_SHARPNESS: sky_cfg.sharpness    = (uint8_t)v;  break;
            case CAM_SCENE:     sky_cfg.scenes       = (uint8_t)v;  break;
            case CAM_AWB:       sky_cfg.awb_cct      = (uint16_t)v; break;
            case CAM_ANGLE:     sky_cfg.angle        = (uint8_t)v;  break;
            case CAM_FOCUS:     sky_cfg.focus_en     = (uint8_t)v;  break;
            // CAM_STANDBY deliberately absent: its offset inside the
            // SET_CONFIG body is not known, so writing it there would
            // corrupt a field we do not understand.
            default: break;
        }
        save_sky_config();
    }
    sky_ack_ = SkyAckWait();
}

void Ar8030Source::scan_air_status(const uint8_t *buf, int n) {
    for (int i = 0; i + 8 <= n; i++) {
        if (buf[i] != 0xFE || buf[i + 1] != 0xA5) continue;
        unsigned len = ((buf[i + 4] | (buf[i + 5] << 8)) >> 4) + 11;
        if (len < 12 || i + (int)len > n) continue;

        // First sighting only for each inbound sky cmd. The per-change dump
        // that lived here through the section 40 investigation identified
        // cmd 0x05 as the VTX telemetry frame (see the decoder below), so
        // the change spam is no longer needed and the log stays quiet.
        {
            static bool seen_cmd[256] = { false };
            uint8_t cmd_byte = buf[i + 6];
            if (!seen_cmd[cmd_byte]) {
                seen_cmd[cmd_byte] = true;
                const uint8_t *pl = buf + i + 6;
                size_t pl_len = len - 6 - 5;
                printf("ar8030: inbound cmd 0x%02x payload=%zu bytes:", cmd_byte, pl_len);
                for (size_t k = 0; k < pl_len && k < 96; k++) printf(" %02X", pl[k]);
                if (pl_len > 96) printf(" ...");
                printf("  (first sighting - sec 40)\n");
            }
        }

        // cmd 0x05 - VTX periodic telemetry. Identified by:
        //   (a) the air unit's own binary has fpv_cmd_get_rf_temp() which
        //       reads a global at [ctx+0x3fc] and packs it as a °C byte;
        //       fpv_bb_update_rf_board_temp() computes that value from a
        //       thermistor ADC via an interpolated LUT (see section 40b);
        //   (b) empirically, byte 30 fluctuates by ~1-2°C every frame -
        //       characteristic ADC noise on a live sensor - and byte 33
        //       fell in lockstep with TX power dropping from 501 to 200 mW
        //       (bytes 9-10 in the same frame), which is exactly RF PA
        //       cooling behaviour.
        // So byte 30 is the SoC/CPU temperature and byte 33 is the RF
        // board temperature, both single-byte °C. Publish the max as the
        // VTX temperature - whichever is closer to a thermal limit is the
        // number that matters. Log periodically for post-mortems.
        if (buf[i + 6] == 0x04) note_air_version(buf + i + 6, len - 6 - 5);
        if (buf[i + 6] == 0x50) note_air_info(buf + i + 6, len - 6 - 5);
        if (buf[i + 6] == 0x51) note_air_frame(buf + i + 6, len - 6 - 5);

        if (buf[i + 6] == 0x05 && (len - 6 - 5) >= 34) {
            const uint8_t *pl = buf + i + 6;
            // VTX CPU temp = byte 1, VTX RF board temp = byte 33, both °C.
            // Ground-truth confirmed against the stock app's debug OSD, which
            // shows "S TEMP: 50" for the VTX SoC while byte 1 read 0x30=48;
            // the 2°C offset is normal sensor noise, and byte 1 has drifted
            // 42-52°C across many samples in exactly the range stock displays.
            //
            // Two earlier hypotheses got this wrong: (1) byte 30 alone gave
            // a bimodal 59/63 alias and jumped to 148 out of standby; (2)
            // bytes 29-30 as BE u16 / 10 read 83°C in standby but stock
            // clearly shows 50°C for the same state, so that field is not
            // temperature - see below for what it turned out to be instead.
            uint8_t vtx_cpu = pl[1];
            if (osd) osd->set_vtx_temp((float)vtx_cpu);

            static uint64_t vtx_log_ms = 0;
            uint64_t vtx_now = now_ms();
            if (vtx_now - vtx_log_ms >= 5000) {
                vtx_log_ms = vtx_now;
                printf("ar8030: VTX cpu=%uC (cmd 0x05 b1)\n", vtx_cpu);
            }

            // Bytes 29-30 track whether the VTX is actually radiating at
            // normal power, not standby/low-power - a real, live signal
            // rather than the (confirmed unreliable, see
            // ar8030-power-and-standby-verified.md) TLV_STANDBY_MODE the air
            // separately reports. Confirmed on the bench: TLV 0x12 stayed
            // latched at standby=ON through an entire arm/disarm cycle,
            // while this field cleanly stepped from ~840 (idle) to ~714
            // (armed) in lockstep with VTX temperature climbing 47C->64C -
            // a ~130-count gap with no overlap across ~45 samples each side.
            // kVtxStandbyThresh sits in the middle of that gap; it is a
            // two-sample calibration and may need retuning against more
            // hardware/distance/thermal conditions.
            static constexpr uint16_t kVtxStandbyThresh = 780;
            uint16_t b2930 = ((uint16_t)pl[29] << 8) | pl[30];
            bool low_power = b2930 >= kVtxStandbyThresh;
            if (osd) osd->set_vtx_low_power(low_power);

            static uint16_t last_b2930 = 0xFFFF;
            static bool     b2930_seen = false;
            if (!b2930_seen || b2930 != last_b2930) {
                b2930_seen  = true;
                last_b2930  = b2930;
                printf("ar8030: VTX bytes[29:30]=%u (0x%04X) cpu=%uC "
                       "-> %s\n",
                       b2930, b2930, vtx_cpu, low_power ? "STANDBY" : "active");
            }
        }

        if (buf[i + 6] != 0x03) { i += len - 1; continue; }


        // payload starts at the cmd byte; drop the 4-byte CRC and trailing BB.
        const uint8_t *pl = buf + i + 6;
        size_t pl_len = len - 6 - 5;
        // The air unit's own config rides in the same frame - "49-byte body +
        // TLV tail", the same shape as the outbound SET_CONFIG (section 32).
        // Logged once, not applied: the field offsets were recovered from the
        // *outbound* serialiser, and feeding a mis-parsed body back into
        // sky_cfg would push corruption to the VTX on the next handshake.
        // Compare it against the "restored camera cfg" line above before
        // trusting it.
        if (pl_len > 1) {
            sky::SkyConfig air;
            if (sky::read_config_payload(pl + 1, pl_len - 1, &air)) {
                // Re-report whenever the camera's own view changes, not just
                // once: this is the only read-back that says whether a setting
                // actually reached it. The ack channel cannot be relied on -
                // see KNOWN_ISSUES.
                if (!air_cfg_logged || air.angle != last_air_angle ||
                    air.ch0_w != last_air_w || air.ch0_h != last_air_h ||
                    air.ch0_fps != last_air_fps || air.dnr_3d != last_air_dnr3d ||
                    air.focus_en != last_air_focus) {
                    air_cfg_logged = true;
                    last_air_angle = air.angle;
                    stab::ImuStream::get().set_camera_angle(air.angle ? 180 : 0);   // the picture turns the gyro's axes too
                    last_air_w = air.ch0_w; last_air_h = air.ch0_h;
                    last_air_fps = air.ch0_fps;
                    last_air_dnr3d = air.dnr_3d;
                    last_air_focus = air.focus_en;
                    // The level it runs, which nothing on this side stores: the
                    // menu showed the stock capture's byte instead (Auto, or Off
                    // before) while the air unit ran its own (High by default).
                    // Kept, as the menu's settings are: a handshake restores
                    // the stored config, and the stock capture's byte would win.
                    if (sky_cfg.dnr_3d != air.dnr_3d) {
                        sky_cfg.dnr_3d = air.dnr_3d;
                        save_sky_config();
                    }
                    if (osd) osd->set_camera_dnr(air.dnr_3d);
                    // Its rotation likewise: the menu shows what it runs.
                    if (osd) osd->set_camera_angle(air.angle);
                    printf("ar8030: air unit reports ch0=%ux%u@%u ev=%d sat=%u "
                           "sharp=%u contrast=%u scene=%u awb=%u flicker=%u "
                           "angle=%u dnr3d=%u focus=%u\n",
                           air.ch0_w, air.ch0_h, air.ch0_fps, air.ev_x10,
                           air.saturation, air.sharpness, air.contrast,
                           air.scenes, air.awb_cct, air.anti_flicker, air.angle,
                           air.dnr_3d, air.focus_en);
                }
            }
        }

        // What the air unit is decides which power levels it is offered, as on
        // stock. The power chosen is fitted to them - brought down to the
        // nearest one a Lite is offered, or back up to the chosen one for a
        // Lite+ after a Lite - and sent to it; the choice itself is kept.
        int rf_hw = sky::Proto::find_tlv(pl, pl_len, sky::Proto::TLV_SKY_RF_HWVER);
        if (rf_hw >= 0) {
            std::lock_guard<std::mutex> g(air_info_mtx_);
            air_info_.rf_hw = rf_hw;
        }
        note_air_prj(sky::Proto::find_tlv(pl, pl_len, sky::Proto::TLV_SKY_PRJ_NAME), "settings");

        int sb = sky::Proto::find_tlv(pl, pl_len, sky::Proto::TLV_STANDBY_MODE);
        if (sb >= 0) {
            bool now = (sb != 0);
            if (!air_status_seen || now != air_standby) {
                printf("ar8030: air unit standby/low-power = %s\n", now ? "ON" : "OFF");
                air_standby = now;
                air_status_seen = true;
                if (osd) osd->set_air_standby(now);
            }
        }

        i += len - 1;
    }
}

// Flight-controller MSP passthrough. Stock never reads this off a BB socket -
// its ar_ldy_gnd takes the FC OSD as UDP datagrams on :55068 and nothing in the
// stock rootfs sends to that port, so the bytes reach it as IP over a baseband
// network device we do not bring up. Reading the passthrough port directly is
// the equivalent path for a client that already owns the BB session.
//
// Opt-in via --debug-msp-bb-port: if the air unit does not serve the port,
// bb_socket_open() blocks with no timeout (section 33) and wedges bring-up.
// BB_SET_PRJ_DISPATCH (SET 200), project command 138.
//
// Recovered from ar_ldy_gnd, not guessed. The builder is at 0x9f5dc:
//
//     buf = 256 zero bytes
//     buf[0] = 0x8A                 // cmd 138
//     p = buf + 4
//     p[0] = (uint8_t)arg1          // -> buf[4]
//     *(uint32_t *)(p + 4) = arg2   // -> buf[8]
//     ar_ioctl(dev, 0x020000C8, buf, NULL, 1000)
//
// and its single caller at 0x9bca0 passes (4, 500) - guarded on the product
// subtype being 2, which is this board ("vrx pro2"):
//
//     bl  0x94d44          ; getter for the --subtype global
//     cmp w0, #2
//     b.ne skip
//     mov w1, #0x1f4       ; 500
//     mov w0, #0x4         ; 4
//     bl  0x9f5dc
//
// Confirmed on the wire: a usbmon capture of stock's link bring-up shows
// SET 200 with payload 8A 00 00 00 | 04 00 00 00 | F4 01 00 00, exactly this.
//
// The 4 and 500 are almost certainly a bandwidth gear (BB_BW_20M) and a
// transmit power in mW - stock's menu reads 20 MHz and 500 mW - but that
// reading is inferred. What is certain is the byte sequence, so we replay it
// rather than trying to be clever about the semantics.
// Stock's startup SET sequence, replayed byte-for-byte.
//
// Taken from tools/bbproxy-cleanboot.log (payload = frame[18 .. 18+len]). Every
// one of these is in the extracted v2 ioctl table with exactly the length seen
// on the wire, which is a good consistency check on the decode:
//
//   SET 5 =1B  SET 8 =2B  SET 9 =3B  SET 12=2B
//   SET 13=2B  SET 16=1B  SET 32=2B  SET 36=16B
//
// The SET 36 entries are the MCS adaptation policy - ar_ldy_gnd logs them as
//   "[mcs item],idx=%d,ldpc_dw_conti_num=%d,snr=[%d,%d],ldpc=[%d,%d],keep=[%d,%d]"
// and the builder at 0xa0754 lays the 16 bytes out as
//   [0..3] rsv  [4] mcs  [5] ldpc_dw_conti_num  [6..7] snr_up  [8..9] snr_dw
//   [10] ldpc_up  [11] ldpc_dw  [12..13] keep_up  [14..15] keep_dw
// Bytes 1..3 read as 9D D9 85 in the capture only because that builder never
// writes them - it is uninitialised stack, so we send zeroes.
//
// The seven entries cover MCS {1,2,5,7,8,10,12}, exactly the levels section 24
// found this radio accepts, with strictly increasing SNR thresholds. We send
// none of this today, which is the likeliest reason our link sits at 2.5 MHz /
// MCS 8 with a 3.23 Mbps ceiling while stock reaches 20 MHz and ~18.7.
struct StockSet { int order; int len; uint8_t data[16]; };
static const StockSet kStockRfSetup[] = {
    {  9, 3, { 0x00, 0x1B, 0x18 } },
    {  9, 3, { 0x02, 0x18, 0x18 } },
    {  8, 2, { 0x00, 0x18 } },
    { 12, 2, { 0x08, 0x00 } },
    { 13, 2, { 0x08, 0x02 } },
    { 32, 2, { 0x00, 0x01 } },
    { 12, 2, { 0x08, 0x01 } },
    { 16, 1, { 0x02 } },
    {  5, 1, { 0x01 } },
    { 36, 16, { 0,0,0,0, 0x01, 0x02, 0x24,0x00, 0x1D,0x00, 0x00, 0x04, 0xE8,0x03, 0x0F,0x00 } },
    { 36, 16, { 0,0,0,0, 0x02, 0x02, 0x5C,0x00, 0x41,0x00, 0x00, 0x04, 0xDC,0x05, 0x0F,0x00 } },
    { 36, 16, { 0,0,0,0, 0x05, 0x02, 0xA8,0x00, 0x77,0x00, 0x00, 0x04, 0xDC,0x05, 0x0F,0x00 } },
    { 36, 16, { 0,0,0,0, 0x07, 0x02, 0x2F,0x01, 0xF1,0x00, 0x00, 0x04, 0x20,0x03, 0x0F,0x00 } },
    { 36, 16, { 0,0,0,0, 0x08, 0x02, 0x56,0x02, 0xDB,0x01, 0x00, 0x03, 0x20,0x03, 0x0F,0x00 } },
    { 36, 16, { 0,0,0,0, 0x0A, 0x02, 0xA8,0x04, 0xB3,0x03, 0x00, 0x04, 0xE8,0x03, 0x0C,0x00 } },
    { 36, 16, { 0,0,0,0, 0x0C, 0x02, 0x36,0x07, 0xBA,0x05, 0x00, 0x02, 0xE8,0x03, 0x01,0x00 } },
};

void Ar8030Source::replay_stock_rf_setup() {
    if (!replay_stock_rf) { printf("ar8030: stock RF setup replay disabled\n"); return; }
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;

    int ok = 0, fail = 0;
    for (size_t i = 0; i < sizeof(kStockRfSetup) / sizeof(kStockRfSetup[0]); i++) {
        const StockSet &s = kStockRfSetup[i];
        uint8_t buf[16];
        memset(buf, 0, sizeof(buf));
        memcpy(buf, s.data, (size_t)s.len);
        if (s.order == 16) buf[0] = (uint8_t)frame_mode;   // RADIO > Radio Frame (set_frame_mode())
        int req = (2 << 24) | s.order;          // BB_REQUEST(BB_REQ_SET, order)
        int rc = ar_ioctl(dev, req, buf, NULL);
        if (rc == 0) ok++; else fail++;
        printf("ar8030: stock SET %-3d ->%3d  [", s.order, rc);
        for (int j = 0; j < s.len; j++) printf(" %02X", buf[j]);
        printf(" ]\n");
    }
    printf("ar8030: stock RF setup replay: %d accepted, %d refused\n", ok, fail);
}

// The radio's TDD frame. SET order 16 = 2 (in the stock setup above) is the
// firmware's "frame-change mode 2": the uplink cut to 6 symbols at MCS 0 and
// the slot's 400 us guard gone - 4650.88 us at 20 MHz against mode 0's
// 5633.92, and 3250.56 at 40 MHz against 4233.60. Every link that forms starts on mode 0 - a new
// one, or the same one re-formed by a step to or from 40 MHz - so it is sent
// again after each link-up: from the link event, as a re-form is often over
// before the video loop looks (seen 2026-10-14: the step to 40 MHz stayed on
// mode 0), 1 s on and 1 s clear of the radio's own bandwidth steps, which
// come right at link-up (one sent with them re-formed the link once).
// Mode 2 was the fastest at both widths and carries
// more (2026-10-14, 864p120, MCS 10: capture->lit 23.5 vs 23.8 ms and 31.4 vs
// 25.9 Mbps at 20 MHz, 21.8 vs 22.1-22.9 ms and 44.9 vs 34.5 Mbps at 40).
// Its uplink is shorter, though (6 symbols at MCS 0 for 12 at MCS -1, ~3 dB
// less margin by the firmware's numbers, untested at range): RADIO > Radio
// Frame STANDARD asks for mode 0 instead, here and in the startup setup,
// for testing it in the field.
void Ar8030Source::set_frame_mode() {
    if (!replay_stock_rf) return;
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    uint8_t buf[16] = { (uint8_t)frame_mode };
    printf("ar8030: radio frame mode %d (%s) -> %d\n", frame_mode, frame_mode == 2 ? "short" : "standard",
           ar_ioctl(dev, (2 << 24) | 16, buf, NULL));
    frame_clock_.reframe();                   // another frame, if it was not on it
}

void Ar8030Source::send_prj_rf_config() {
    if (prj_rf_bw < 0 || prj_rf_pwr_mw < 0) {
        printf("ar8030: PRJ_DISPATCH rf config disabled\n");
        return;
    }
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;

    uint8_t buf[256];
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x8A;                                  // cmd 138
    buf[4] = (uint8_t)prj_rf_bw;
    uint32_t pwr = (uint32_t)prj_rf_pwr_mw;
    memcpy(buf + 8, &pwr, sizeof(pwr));             // little-endian, as captured

    int rc = ar_ioctl(dev, BB_SET_PRJ_DISPATCH, buf, NULL);
    printf("ar8030: BB_SET_PRJ_DISPATCH cmd138(%d, %d mW) -> %d\n",
           prj_rf_bw, prj_rf_pwr_mw, rc);
}

// Both power ioctls, in one place. Startup and the menu row have to agree:
// hardcoding auto=off at startup meant a saved "500mW AUTO" came back as a
// fixed level after a reboot.
//
// Auto goes first. The other order lets the adaptation loop move off the level
// we just asked for before it has even been read back.
// What to tell the air unit. Stock's "+1 means auto" encoding really is what
// goes on the wire: a bbproxy capture of stock caught cmd 0x22 carrying 101 for
// its "100mW Auto" menu entry. We used to strip the +1, which asked the air for
// a fixed level and silently disabled its adaptation. See
// references/sky-commands.md.
static int sky_pwr_mw(const ar_pwr_level &lv) {
    return lv.mw;                 // kArPwrLevels already stores AUTO as N+1
}

void Ar8030Source::apply_tx_power(int mw) {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    const ar_pwr_level &lv = kArPwrLevels[ar_pwr_index(mw)];

    // 3 bytes, not 1. Our bb_get_pwr_auto_out_t is a single uint8_t, but the
    // v2 table (extracted from stock) says SET 9 takes 3, so bb_ioctl copied
    // two bytes of stack past the struct. Stock's own payloads read as
    // {enable, max_dBm, min_dBm} - 00 1B 18 = off/27/24, 01 1B 0E = on/27/14 -
    // which matches its "en power auto [%d %d]" log.
    uint8_t pa[3] = { (uint8_t)(lv.automode ? 1 : 0),
                      (uint8_t)lv.dbm,
                      (uint8_t)(lv.automode ? kArPwrLevels[0].dbm : lv.dbm) };
    int rc_auto = ar_ioctl(dev, BB_SET_POWER_AUTO, pa, NULL);

    bb_set_pwr_in_t p;
    memset(&p, 0, sizeof(p));
    p.usr = 0;
    p.pwr = (uint8_t)lv.dbm;
    int rc_pwr = ar_ioctl(dev, BB_SET_POWER, &p, NULL);

    printf("ar8030: TX power %s (%d dBm, auto=%d) -> auto:%d set:%d\n",
           lv.label, lv.dbm, lv.automode ? 1 : 0, rc_auto, rc_pwr);
    tx_power_dbm  = lv.dbm;
    tx_power_mw   = lv.mw;
    tx_power_auto = lv.automode;
    if (rc_pwr == 0) rf_caps |= RF_CAP_POWER;

    // Mirror it to the air unit. Guarded because configure_link() calls this
    // before the control socket exists; the post-handshake call covers startup.
    if (ctrl_sockfd >= 0) request_setting(CAM_PWR, sky_pwr_mw(lv));
}

void Ar8030Source::apply_bandwidth() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    bb_set_bandwidth_t p;
    memset(&p, 0, sizeof(p));
    p.slot = BB_SLOT_0;
    p.dir = BB_DIR_TX;
    p.bandwidth = (uint8_t)bandwidth;
    int rc = ar_ioctl(dev, BB_SET_BANDWIDTH, &p, NULL);
    static const char *bw_name[] = { "1.25M", "2.5M", "5M", "10M", "20M", "40M" };
    printf("ar8030: BB_SET_BANDWIDTH(%d = %s) post-link -> %d\n", bandwidth,
           (bandwidth >= 0 && bandwidth < 6) ? bw_name[bandwidth] : "?", rc);
}

void Ar8030Source::drain_msp_socket() {
    if (msp_bb_port <= 0) return;
    if (msp_bb_port == 2) return;        // teed off the control socket instead

    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;

    if (msp_sockfd < 0) {
        bb_sock_opt_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.tx_buf_size = 0x0800;
        opt.rx_buf_size = 0x0800;
        msp_sockfd = bb_socket_open(dev, (bb_slot_e)slot, (uint32_t)msp_bb_port,
                                    BB_SOCK_FLAG_RX | BB_SOCK_FLAG_TX, &opt);
        printf("ar8030: MSP socket (port %d) fd=%d\n", msp_bb_port, msp_sockfd);
        if (msp_sockfd < 0) {
            msp_bb_port = 0;             // do not retry every pass
            return;
        }
    }

    uint8_t buf[1024];
    for (int i = 0; i < 64; i++) {       // bounded, like the telemetry drain
        int n = read_queued(msp_sockfd, buf, sizeof(buf));
        if (n <= 0) break;               // -1 is just "ring empty"

        // First bytes go to the log verbatim. If this port carries MSP at all
        // the '$' framing is visible immediately, and if it carries something
        // else that is worth seeing rather than silently feeding the parser.
        static int dumped = 0;
        if (dumped < 4) {
            dumped++;
            printf("ar8030: msp rx %d bytes:", n);
            for (int j = 0; j < n && j < 32; j++) printf(" %02X", buf[j]);
            printf("%s\n", n > 32 ? " ..." : "");
        }

        msp_bytes += (unsigned long long)n;
        if (osd) osd->update_msp_data(buf, (size_t)n);
    }
}

void Ar8030Source::drain_video_socket() {
    if (sockfd < 0) return;
    std::vector<uint8_t> tmp(16384);
    for (int i = 0; i < 64; i++) {
        int n = bb_socket_read(sockfd, tmp.data(), (uint32_t)tmp.size(), 1);
        if (n <= 0) break;
    }
}

// Wait for an ack of want_cmd, draining video meanwhile.
// Returns the ack status (0 = accepted, >0 = refused), or -1 on timeout.
int Ar8030Source::wait_sky_status(uint8_t want_cmd, int timeout_ms) {
    uint8_t buf[512];
    for (int waited = 0; waited < timeout_ms; waited += 20) {
        if (*should_stop) return -1;
        drain_video_socket();
        int n = bb_socket_read(ctrl_sockfd, buf, sizeof(buf), 20);
        if (n <= 0) continue;
        on_ctrl_bytes(buf, n);     // the air's reports, MSP, an awaited ack: not lost here
        for (int i = 0; i + 12 <= n; i++) {
            if (buf[i] != 0xFE || buf[i + 1] != 0xA5) continue;
            unsigned len = ((buf[i + 4] | (buf[i + 5] << 8)) >> 4) + 11;
            if (i + (int)len > n) continue;
            uint8_t cmd = 0, status = 0; bool is_ack = false;
            if (!sky::Proto::parse(buf + i, len, &cmd, &is_ack, &status)) continue;
            if (is_ack && cmd == want_cmd) return (int)status;
        }
    }
    return -1;
}

// Drain whatever the menu parked and push it to the camera. Each of these is a
// 4-byte little-endian value; the ids come from the GUI dispatcher in
// ar_ldy_gnd.
// One at a time, as when each waited here for its ack - the video thread
// then stopped reading for up to a second per setting, the radio's ring
// overflowed and the picture broke up until the next keyframe. Now the ack is
// awaited between pictures (on_ctrl_bytes) and the next goes when it is in.
void Ar8030Source::apply_pending_settings() {
    {
        std::lock_guard<std::mutex> lk(pending_mtx);
        settings_q_.insert(settings_q_.end(), pending_settings.begin(), pending_settings.end());
        pending_settings.clear();
    }
    if (ctrl_sockfd < 0) {               // no link: dropped, as they always were
        settings_q_.clear();
        sky_ack_ = SkyAckWait();
        return;
    }
    if (sky_ack_.field >= 0) {
        if (now_ms() < sky_ack_.until_ms) return;
        printf("ar8030: no ack for cmd 0x%02X within %d ms\n", sky_ack_.cmd, kSkyAckMs);
        sky_ack_ = SkyAckWait();
    }
    while (!settings_q_.empty()) {
        const std::pair<int,int> item = settings_q_.front();
        settings_q_.pop_front();
        uint8_t cmd; const char* name;
        switch (item.first) {
            case CAM_EV:        cmd = sky::CMD_SET_EV;           name = "EV";           break;
            case CAM_SAT:       cmd = sky::CMD_SET_SAT;          name = "saturation";   break;
            case CAM_CONTRAST:  cmd = sky::CMD_SET_CONTRAST;     name = "contrast";     break;
            case CAM_SHARPNESS: cmd = sky::CMD_SET_SHARPNESS;    name = "sharpness";    break;
            case CAM_SCENE:     cmd = sky::CMD_SET_SCENES;       name = "scene";        break;
            case CAM_AWB:       cmd = sky::CMD_SET_AWB;          name = "white balance";break;
            case CAM_ANGLE:     cmd = sky::CMD_SET_ANGLE;       name = "rotate";       break;
            case CAM_3DNR:      cmd = sky::CMD_SET_3DNR;       name = "3D DNR";       break;
            case CAM_STANDBY:   cmd = sky::CMD_SET_STANDBY;      name = "standby";      break;
            case CAM_BW:        cmd = sky::CMD_SET_BB_BANDWIDTH; name = "air bandwidth";break;
            case CAM_MAX_KBPS:  cmd = sky::CMD_KA_MAX_BITRATE; name = "video bitrate cap"; break;
            case CAM_MAX_BW:    cmd = sky::CMD_KA_MAX_BW;      name = "video link bandwidth cap MHz"; break;
            case CAM_AIR_LOGS:  cmd = sky::CMD_KA_LOGS;        name = "air flight logs"; break;
            case CAM_PWR:       cmd = sky::CMD_SET_BB_PWR;      name = "air power mW"; break;
            case CAM_FOCUS:     cmd = sky::CMD_SET_CHN_FOCUS;   name = "focus mode";   break;
            default: continue;
        }
        // kestrel-air's own commands: kept for when a kestrel-air announces
        // itself (note_air_announce sends them then), never to the stock app.
        if ((item.first == CAM_MAX_KBPS || item.first == CAM_MAX_BW || item.first == CAM_AIR_LOGS) && !air_kestrel_) {
            printf("ar8030: %s = %d kept for a kestrel-air (this air unit %s)\n", name, item.second,
                   air_ver_seen_ ? "is the stock app" : "has not said what it is yet");
            continue;
        }
        // ar_ldy_gnd 0x89060 builds this one as {cmd 0x23, len 1}: a single
        // byte, unlike most camera settings, which are a u32.
        // Payload width differs per command: ar_ldy_gnd 0x88fd8 writes cmd 0x22
        // as two little-endian bytes (len 2), 0x89060 writes standby as one,
        // and most camera settings are u32.
        std::vector<uint8_t> f;
        if (item.first == CAM_PWR) {
            uint8_t v[2] = { (uint8_t)(item.second & 0xFF),
                             (uint8_t)((item.second >> 8) & 0xFF) };
            f = sky_proto.build(cmd, v, sizeof(v));
        } else if (item.first == CAM_FOCUS) {
            // {u8 chn, u8 enable} - ar_ldy_gnd's own debug line for this
            // command is "GUI_CMD_SET_CHN_FOCUS, chn=%d, en=%d". chn 0 is
            // the FPV stream, the only channel this menu ever controls.
            uint8_t v[2] = { 0, (uint8_t)item.second };
            f = sky_proto.build(cmd, v, sizeof(v));
        } else if (item.first == CAM_STANDBY || item.first == CAM_BW ||
                   item.first == CAM_MAX_BW || item.first == CAM_AIR_LOGS) {
            f = sky_proto.build_u8(cmd, (uint8_t)item.second);
        } else {
            f = sky_proto.build_u32(cmd, (uint32_t)item.second);
        }
        int w = bb_socket_write(ctrl_sockfd, f.data(), (uint32_t)f.size(), 500);
        printf("ar8030: set %s = %d (cmd 0x%02X) -> %d\n", name, item.second, cmd, w);
        if (w > 0) {
            sky_ack_.field = item.first;
            sky_ack_.value = item.second;
            sky_ack_.cmd = cmd;
            sky_ack_.until_ms = now_ms() + kSkyAckMs;
            return;                      // the next once this one's ack is in
        }
    }
}

// Apply RF changes parked by the menu. These are ioctls on our own radio, so
// they take effect immediately; the return code is logged so an unsupported
// value is visible rather than silently ignored.
void Ar8030Source::apply_pending_rf() {
    std::vector<std::pair<int,int>> todo;
    {
        std::lock_guard<std::mutex> lk(pending_mtx);
        if (pending_rf.empty()) return;
        todo.swap(pending_rf);
    }
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    for (size_t i = 0; i < todo.size(); i++) {
        const int field = todo[i].first, val = todo[i].second;
        switch (field) {
            case RF_TX_POWER: {
                // val is mW in stock's encoding (N, or N+1 for auto capped
                // at N); apply_tx_power() owns the mapping and both ioctls.
                // The menu's choice: kept, and run fitted to this air unit.
                tx_power_want_mw = val;
                apply_tx_power(ar_pwr_fit(air_prj.load(), val));
                save_sky_link();
                break;
            }
            case RF_TX_POWER_FIT:
                // The choice fitted to the air unit that just said what it is;
                // the choice stays as it was.
                apply_tx_power(ar_pwr_fit(air_prj.load(), tx_power_want_mw));
                break;
            case RF_MCS: {
                // val < 0 means AUTO: flip the mode rather than pin an index.
                bb_set_mcs_mode_t m; memset(&m, 0, sizeof(m));
                m.auto_mode = (val < 0) ? 1 : 0;
                printf("ar8030: RF MCS mode auto=%d -> %d\n", m.auto_mode,
                       ar_ioctl(dev, BB_SET_MCS_MODE, &m, NULL));
                if (val >= 0) {
                    bb_set_mcs_t p; memset(&p, 0, sizeof(p));
                    p.slot = (uint8_t)slot; p.mcs = (uint8_t)val;
                    printf("ar8030: RF MCS %d -> %d\n", val,
                           ar_ioctl(dev, BB_SET_MCS, &p, NULL));
                }
                break;
            }
            case RF_LNA: {
                bb_set_lna_mode_t p; memset(&p, 0, sizeof(p));
                p.mode = (int8_t)val;
                printf("ar8030: RF LNA mode %d -> %d\n", val,
                       ar_ioctl(dev, BB_SET_LNA_MODE, &p, NULL));
                break;
            }
            case RF_BW: {
                bb_set_bandwidth_t p; memset(&p, 0, sizeof(p));
                p.slot = (uint8_t)slot; p.dir = BB_DIR_TX; p.bandwidth = (uint8_t)val;
                printf("ar8030: RF bandwidth %d -> %d\n", val,
                       ar_ioctl(dev, BB_SET_BANDWIDTH, &p, NULL));
                break;
            }
            case RF_HOP: {
                if (val) {
                    printf("ar8030: RF channel hop ON\n");
                    search_all_channels();          // and undo any pin's work list
                    break;
                }
                bb_set_chan_mode_t m; memset(&m, 0, sizeof(m));
                m.auto_mode = 0;
                printf("ar8030: RF channel hop OFF -> %d\n",
                       ar_ioctl(dev, BB_SET_CHAN_MODE, &m, NULL));
                chan_auto = false;
                break;
            }
            case RF_CHAN: {
                // val < 0 = hand the channel back to AUTO/ACS; otherwise val is
                // the target frequency in kHz and both ends have to move.
                // BB_SET_FREQ is deliberately not used here - stock never calls
                // it, and on its own it moves nothing (section 31).
                if (val < 0) {
                    printf("ar8030: RF chan mode AUTO\n");
                    search_all_channels();
                } else {
                    int rc = set_rf_channel((uint32_t)val, false);
                    printf("ar8030: RF channel -> %u kHz rc=%d\n",
                           (unsigned)val, rc);
                    // set_rf_channel() owns chan_auto - it is the only thing
                    // that knows whether the retune landed.
                }
                break;
            }
            default: break;
        }
    }
    report_link_status();
}

// --debug-probe-rf: find out which RF controls this board actually honours.
// Unlike the mode probe these are all local ioctls on our own radio - they do
// not touch the air unit - so this is safe to run on a live link (it does
// briefly perturb TX power / MCS, and restores stock's settings at the end).
void Ar8030Source::probe_rf_ioctls() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    printf("ar8030: ---- RF ioctl probe ----\n");

    // Capped at stock's 24 dBm on purpose: this board browns out under load,
    // and a sweep to 33 dBm/2W is a good way to make it reset mid-probe.
    for (int dbm = 8; dbm <= 24; dbm += 1) {
        bb_set_pwr_in_t p; memset(&p, 0, sizeof(p));
        p.usr = 0; p.pwr = (uint8_t)dbm;
        printf("ar8030: rfprobe POWER %2d dBm -> %d\n", dbm,
               ar_ioctl(dev, BB_SET_POWER, &p, NULL));
    }
    for (int a = 0; a <= 1; a++) {
        bb_set_pwr_auto_in_t p; memset(&p, 0, sizeof(p));
        p.pwr_auto = (uint8_t)a;
        printf("ar8030: rfprobe POWER_AUTO %d -> %d\n", a,
               ar_ioctl(dev, BB_SET_POWER_AUTO, &p, NULL));
    }
    for (int a = 0; a <= 1; a++) {
        bb_set_mcs_mode_t p; memset(&p, 0, sizeof(p));
        p.auto_mode = (uint8_t)a;
        printf("ar8030: rfprobe MCS_MODE auto=%d -> %d\n", a,
               ar_ioctl(dev, BB_SET_MCS_MODE, &p, NULL));
    }
    for (int m = 0; m <= 13; m++) {
        bb_set_mcs_t p; memset(&p, 0, sizeof(p));
        p.slot = 0; p.mcs = (uint8_t)m;
        printf("ar8030: rfprobe MCS %2d -> %d\n", m,
               ar_ioctl(dev, BB_SET_MCS, &p, NULL));
    }
    for (int m = -1; m <= 3; m++) {
        bb_set_lna_mode_t p; memset(&p, 0, sizeof(p));
        p.mode = (int8_t)m;
        printf("ar8030: rfprobe LNA %2d -> %d\n", m,
               ar_ioctl(dev, BB_SET_LNA_MODE, &p, NULL));
    }
    for (int b = 0; b <= 5; b++) {
        bb_set_bandwidth_t p; memset(&p, 0, sizeof(p));
        p.slot = 0; p.dir = 1; p.bandwidth = (uint8_t)b;
        printf("ar8030: rfprobe BANDWIDTH %d -> %d\n", b,
               ar_ioctl(dev, BB_SET_BANDWIDTH, &p, NULL));
    }
    for (int a = 0; a <= 1; a++) {
        bb_set_chan_mode_t p; memset(&p, 0, sizeof(p));
        p.auto_mode = (uint8_t)a;
        printf("ar8030: rfprobe CHAN_MODE auto=%d -> %d\n", a,
               ar_ioctl(dev, BB_SET_CHAN_MODE, &p, NULL));
    }

    // Restore what configure_link() set up, so the probe leaves no trace.
    printf("ar8030: ---- RF probe done, restoring ----\n");
    { bb_set_pwr_auto_in_t p; memset(&p, 0, sizeof(p)); p.pwr_auto = 0;
      ar_ioctl(dev, BB_SET_POWER_AUTO, &p, NULL); }
    { bb_set_pwr_in_t p; memset(&p, 0, sizeof(p));
      p.usr = 0; p.pwr = (uint8_t)tx_power_dbm;
      ar_ioctl(dev, BB_SET_POWER, &p, NULL); }
    { bb_set_mcs_mode_t p; memset(&p, 0, sizeof(p)); p.auto_mode = 1;
      ar_ioctl(dev, BB_SET_MCS_MODE, &p, NULL); }
    { bb_set_chan_mode_t p; memset(&p, 0, sizeof(p)); p.auto_mode = chan_auto ? 1 : 0;
      ar_ioctl(dev, BB_SET_CHAN_MODE, &p, NULL); }
}

// --debug-probe-modes: walk a candidate list and report which the camera
// acknowledges. The stock GlassesUI only offers four FPV modes, but that is its
// menu, not necessarily the hardware's limit - this asks the hardware directly.
//
// Caveat: a status-0 ack means the command was accepted, not that the camera
// really delivers that mode. Confirm anything interesting by watching the
// decoder's reported geometry and frame rate afterwards.
void Ar8030Source::probe_video_modes() {
    static const struct { uint16_t w, h; uint8_t fps; } cand[] = {
        { 1280,  720,  30 }, { 1280,  720,  50 }, { 1280,  720,  60 },
        { 1280,  720,  90 }, { 1280,  720, 100 }, { 1280,  720, 120 },
        { 1920, 1080,  30 }, { 1920, 1080,  50 }, { 1920, 1080,  60 },
        { 1920, 1080,  90 }, { 1920, 1080, 100 }, { 1920, 1080, 120 },
        { 2560, 1440,  30 }, { 2560, 1440,  60 },
        { 2720, 1528,  30 }, { 2720, 1528,  60 },
        { 3840, 2160,  30 }, { 3840, 2160,  60 },
    };
    printf("ar8030: ---- mode probe (ch%d) ----\n", mode_chn);
    for (size_t i = 0; i < sizeof(cand) / sizeof(cand[0]); i++) {
        std::vector<uint8_t> f = sky_proto.set_chn_res((uint8_t)mode_chn,
                                                       cand[i].w, cand[i].h, cand[i].fps);
        int w = bb_socket_write(ctrl_sockfd, f.data(), (uint32_t)f.size(), 500);
        int st = (w > 0) ? wait_sky_status(sky::CMD_SET_CHN_RES, 4000) : -2;
        const char *verdict = st == 0  ? "ACCEPTED"
                            : st >  0  ? "NAK"
                            : st == -1 ? "no ack (inconclusive)" : "write failed";
        if (st > 0) printf("ar8030: probe %4ux%-4u@%-3u -> %s status=%d\n",
                           cand[i].w, cand[i].h, cand[i].fps, verdict, st);
        else        printf("ar8030: probe %4ux%-4u@%-3u -> %s\n",
                           cand[i].w, cand[i].h, cand[i].fps, verdict);
        // Let the camera finish restarting its encoder, draining video the
        // whole time so the video ring cannot back the USB path up.
        for (int t = 0; t < 3000 && !*should_stop; t += 50) { drain_video_socket(); usleep(50000); }
        if (*should_stop) { printf("ar8030: mode probe aborted (stopping)\n"); return; }
    }
    printf("ar8030: ---- probe done, restoring 1920x1080@120 ----\n");
    std::vector<uint8_t> f = sky_proto.set_chn_res((uint8_t)mode_chn, 1920, 1080, 120);
    bb_socket_write(ctrl_sockfd, f.data(), (uint32_t)f.size(), 500);
    read_sky_ack(sky::CMD_SET_CHN_RES, 1200);
}

// --ar8030-mode: ask the air unit to switch the FPV channel to a new
// resolution/frame rate. This is GUI_CMD_SET_CHN_RES in the stock app - the
// same 6-byte SET_CHN_RES command its menu sends when you pick a video mode.
void Ar8030Source::apply_video_mode() {
    if (!mode_w || !mode_h || !mode_fps) return;
    if (ctrl_sockfd < 0) return;

    std::vector<uint8_t> f = sky_proto.set_chn_res((uint8_t)mode_chn,
                                                   (uint16_t)mode_w,
                                                   (uint16_t)mode_h,
                                                   (uint8_t)mode_fps);
    int w = bb_socket_write(ctrl_sockfd, f.data(), (uint32_t)f.size(), 500);
    printf("ar8030: SET_CHN_RES ch%d %dx%d@%d seq=0x%02X -> %d\n",
           mode_chn, mode_w, mode_h, mode_fps, f[3], w);
    if (w > 0 && read_sky_ack(sky::CMD_SET_CHN_RES, 1000) && mode_chn == 0) {
        // Accepted: remember it so the next handshake asks for the same mode.
        sky_cfg.ch0_w   = (uint16_t)mode_w;
        sky_cfg.ch0_h   = (uint16_t)mode_h;
        sky_cfg.ch0_fps = (uint8_t)mode_fps;
        save_sky_config();
    }
}

// The video bitrate and the link MCS have to be read together to tell a
// link-limited stream from an encoder that is simply targeting a low rate.
// report_link_status() only fires when the link is idle, which is exactly when
// there is no bitrate to compare against - hence this.
// The baseband can measure air<->ground distance by time of flight. It is off
// by default; BB_CFG_DISTC turns it on. `offset` is a close-range calibration
// (the raw td_out reading with the units side by side) subtracted from every
// result - left at 0 here because we have not calibrated this pair, so treat
// the absolute value as uncalibrated until someone measures it at a known range.
void Ar8030Source::enable_ranging() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    bb_conf_distc_t d;
    memset(&d, 0, sizeof(d));
    d.enable  = 1;
    d.window  = 3;    // 2^3 = 8-sample average; the raw reading is noisy
    d.timeout = 9;    // 2.6*(9+1) = 26 ms
    d.offset  = 0;    // uncalibrated
    int rc = ar_ioctl(dev, BB_CFG_DISTC, &d, NULL);
    // Note: this answers -2 on this board, but ranging is NOT unavailable -
    // bb_config_gnd_pro.json already carries "dist_calc": {enable:true,
    // window:3, timeout:63, offset:20}, applied when the firmware is uploaded.
    // BB_CFG_DISTC is a runtime re-configuration and the firmware refuses it;
    // the results are readable regardless, so never gate the read on this.
    printf("ar8030: BB_CFG_DISTC -> %d%s\n", rc,
           rc ? "  (already set from bb_config dist_calc; reading anyway)" : "");
}

// -1 while the baseband has no fix.
int Ar8030Source::current_distance() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return -1;
    bb_get_distc_result_in_t in;
    memset(&in, 0, sizeof(in));
    in.slot_bmp = (uint8_t)(1 << slot);
    bb_get_distc_result_out_t out;
    memset(&out, 0, sizeof(out));
    if (ar_ioctl(dev, BB_GET_DISTC_RESULT, &in, &out) != 0) return -1;
    return out.distance[slot];
}



uint32_t Ar8030Source::link_freq_khz() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return 0;
    bb_get_status_in_t sin;
    memset(&sin, 0, sizeof(sin));
    sin.user_bmp = 0x03FF;
    bb_get_status_out_t sout;
    memset(&sout, 0, sizeof(sout));
    if (ar_ioctl(dev, BB_GET_STATUS, &sin, &sout) != 0) return 0;
    for (int pass = 0; pass < 2; pass++)
        for (int i = 0; i < BB_DATA_USER_MAX; i++) {
            const bb_user_status_t &us = sout.user_status[i];
            if (us.tx_status.freq_khz == 0 && us.rx_status.freq_khz == 0) continue;
            if (pass == 0 && !(sout.rt_sbmp & (1 << i))) continue;
            return us.tx_status.freq_khz;
        }
    return 0;
}


int Ar8030Source::chan_index_for_freq(uint32_t freq_khz, uint32_t *freqs, int *n_out) {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return -1;
    // Oversized for the same reason as publish_chan_scan(): the daemon writes
    // 1028 bytes, not the 260 our header's struct declares.
    uint8_t raw[4096];
    memset(raw, 0, sizeof(raw));
    if (ar_ioctl(dev, BB_GET_CHAN_INFO, NULL, raw) != 0) return -1;
    struct chan_info_real {
        uint8_t  chan_num, auto_mode, acs_chan, work_chan;
        uint32_t freq[128];
        int32_t  power[128];
    };
    const chan_info_real &ci = *(const chan_info_real *)raw;
    int n = ci.chan_num;
    if (n <= 0) return -1;
    if (n > 128) n = 128;
    int idx = -1;
    for (int i = 0; i < n; i++) {
        if (freqs) freqs[i] = ci.freq[i];
        if (ci.freq[i] == freq_khz) idx = i;
    }
    if (n_out) *n_out = n;
    return idx;
}

int Ar8030Source::set_rf_channel(uint32_t freq_khz, bool hop_en) {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return -1;
    if (ctrl_sockfd < 0) {
        printf("ar8030: set_rf_channel: no control socket\n");
        return -4;
    }

    uint32_t freqs[128];
    int n = 0;
    int idx = chan_index_for_freq(freq_khz, freqs, &n);
    if (idx < 0) {
        printf("ar8030: set_rf_channel(%u kHz): not in the radio's channel table\n",
               freq_khz);
        return -1;
    }
    printf("ar8030: set_rf_channel freq=%u kHz idx=%d hop_en=%d (of %d channels)\n",
           freq_khz, idx, (int)hop_en, n);

    // --- air unit -----------------------------------------------------------
    // Both of these are required. 0x29 declares which channels the air unit is
    // allowed to use; without it the air ACKs 0x21 with status 0 and stays put,
    // because the target is not in its permitted set. Verified on hardware.
    std::vector<uint8_t> wf = sky_proto.set_work_chan_list(&freq_khz, 1);
    int ww = bb_socket_write(ctrl_sockfd, wf.data(), (uint32_t)wf.size(), 500);
    int wst = wait_sky_status(sky::CMD_SET_WORK_CHAN_LIST, 1500);
    printf("ar8030:   sky 0x29 SET_WORK_CHAN_LIST -> %d bytes, ack=%d\n", ww, wst);
    if (ww != (int)wf.size()) return -4;

    std::vector<uint8_t> f = sky_proto.set_bb_freq(hop_en, (uint8_t)slot, freq_khz);
    int w = bb_socket_write(ctrl_sockfd, f.data(), (uint32_t)f.size(), 500);
    int st = wait_sky_status(sky::CMD_SET_BB_FREQ, 1500);
    printf("ar8030:   sky 0x21 SET_BB_FREQ %u kHz -> %d bytes, ack=%d\n",
           freq_khz, w, st);
    if (w != (int)f.size()) return -4;

    // --- ground -------------------------------------------------------------
    // The ground will follow on its own while it is in ACS/auto, but that is a
    // re-acquire, not a command. Drive it explicitly so both ends are
    // deterministic, then fall back to auto if the explicit path does not take.
    if (!hop_en) {
        bb_set_chan_mode_t cm;
        memset(&cm, 0, sizeof(cm));
        cm.auto_mode = 0;
        int rc = ar_ioctl(dev, BB_SET_CHAN_MODE, &cm, NULL);
        printf("ar8030:   BB_SET_CHAN_MODE(auto=0) -> %d\n", rc);
        chan_auto = false;

        usleep(150000);

        bb_work_chan_list_t wl;
        memset(&wl, 0, sizeof(wl));
        wl.chan_num    = 1;
        wl.chan_idx[0] = (uint8_t)idx;
        rc = ar_ioctl(dev, BB_SET_WORK_CHAN_LIST, &wl, NULL);
        printf("ar8030:   BB_SET_WORK_CHAN_LIST(idx=%d) -> %d\n", idx, rc);

        usleep(150000);

        // chan_index is an index into the radio's full channel table (the one
        // BB_GET_CHAN_INFO returns), not into the work list installed above -
        // confirmed on hardware.
        bb_set_chan_t sc;
        memset(&sc, 0, sizeof(sc));
        sc.chan_dir   = BB_DIR_RX;
        sc.chan_index = (uint8_t)idx;
        rc = ar_ioctl(dev, BB_SET_CHAN, &sc, NULL);
        printf("ar8030:   BB_SET_CHAN(dir=RX idx=%d) -> %d\n", idx, rc);

    } else {
        bb_set_chan_mode_t cm;
        memset(&cm, 0, sizeof(cm));
        cm.auto_mode = 1;
        printf("ar8030:   BB_SET_CHAN_MODE(auto=1) -> %d\n",
               ar_ioctl(dev, BB_SET_CHAN_MODE, &cm, NULL));
        chan_auto = true;
    }

    // --- verify -------------------------------------------------------------
    // Poll rather than assume: the air moves first and the ground needs a
    // moment either way.
    for (int i = 0; i < 20; i++) {
        usleep(200000);
        uint32_t cur = link_freq_khz();
        if (cur == freq_khz) {
            printf("ar8030:   ground confirmed on %u kHz after %d ms\n",
                   cur, (i + 1) * 200);
            return 0;
        }
    }
    // Explicit control did not land. ACS-follow is known to work, so restore
    // auto rather than leaving the ground pinned to a channel it is not on -
    // that combination is what wedged the baseband during bring-up testing.
    printf("ar8030:   WARNING ground did not reach %u kHz (now %u kHz); "
           "restoring auto/ACS\n", freq_khz, link_freq_khz());
    search_all_channels();
    return -5;
}

// A pin narrows the ground's work channel list to the one channel
// (set_rf_channel), and channel adaptation only searches what is on that list:
// turning auto back on alone left the ground looking at a single channel, and
// an air unit that had moved - after its own restart, say - was not found
// until the idle reconnect reopened the radio, about 100 s later.
int Ar8030Source::search_all_channels() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return -1;
    uint32_t freqs[128];
    int n = 0;
    chan_index_for_freq(0, freqs, &n);          // for the channel count only
    int rc_wl = -1;
    if (n > 0) {
        bb_work_chan_list_t wl;
        memset(&wl, 0, sizeof(wl));
        wl.chan_num = (uint8_t)n;
        for (int i = 0; i < n; i++) wl.chan_idx[i] = (uint8_t)i;
        rc_wl = ar_ioctl(dev, BB_SET_WORK_CHAN_LIST, &wl, NULL);
    }
    bb_set_chan_mode_t cm;
    memset(&cm, 0, sizeof(cm));
    cm.auto_mode = 1;
    int rc = ar_ioctl(dev, BB_SET_CHAN_MODE, &cm, NULL);
    printf("ar8030:   searching all %d channels: BB_SET_WORK_CHAN_LIST -> %d, "
           "BB_SET_CHAN_MODE(auto=1) -> %d\n", n, rc_wl, rc);
    if (rc == 0) chan_auto = true;
    return rc;
}

void Ar8030Source::publish_chan_scan() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev || !osd) return;
    // ~14 Hz while the scan screen is open, otherwise every 2 s.
    const uint64_t period = scan_active.load() ? 70 : 2000;
    uint64_t now = now_ms();
    if (last_scan_ms && now - last_scan_ms < period) return;
    last_scan_ms = now;

    // The daemon writes more than sizeof(bb_get_chan_info_out_t) into the output
    // buffer, so hand it a generously oversized one and view the struct through
    // it. Passing the bare struct smashes the stack (the RPC client got away
    // with the same request because it received into a sized vector).
    uint8_t raw[4096];
    memset(raw, 0, sizeof(raw));
    if (ar_ioctl(dev, BB_GET_CHAN_INFO, NULL, raw) != 0) return;
    // The SDK's bb_get_chan_info_out_t declares freq[]/power[] as
    // BB_CONFIG_MAX_CHAN_NUM (32) entries, which is wrong for this response:
    // ar_ldy_gnd's parser (0x9bfc4) indexes power as buf[(j + 128) * 4], i.e.
    // the arrays are 128 entries each and power[] starts at offset 4 + 512.
    // Reading power from offset 132 is what made every channel look maximally
    // busy, and the 260-byte struct is why the reply overflowed the stack.
    struct chan_info_real {
        uint8_t  chan_num, auto_mode, acs_chan, work_chan;
        uint32_t freq[128];
        int32_t  power[128];
    };
    const chan_info_real &ci = *(const chan_info_real *)raw;
    int n = ci.chan_num;
    if (n <= 0) return;
    if (n > 128) n = 128;

    if (n > 64) n = 64;   // chan_scan_info carries 64 slots
    chan_scan_info sc;
    memset(&sc, 0, sizeof(sc));
    sc.chan_num  = n;
    sc.auto_mode = ci.auto_mode;
    sc.acs_chan  = ci.acs_chan;
    sc.work_chan = ci.work_chan;
    for (int i = 0; i < n; i++) {
        sc.freq_mhz[i]  = (int)(ci.freq[i] / 1000);
        sc.power_dbm[i] = (int)ci.power[i];
    }
    // BB_GET_CHAN_INFO's work_chan is NOT an index into this freq[] table: with
    // the link demonstrably on 5839 MHz (BB_GET_STATUS tx_freq) it pointed at
    // 5955/6220 MHz on consecutive reads. Resolve the current channel by
    // matching the real link frequency instead, so the screen can mark it.
    if (link_freq_mhz > 0) {
        for (int i = 0; i < n; i++) {
            if (sc.freq_mhz[i] == link_freq_mhz) { sc.work_chan = i; break; }
        }
    }
    sc.last_update_ms = now;
    // Tests: while /tmp/kestrel-scandump exists, each scan read is printed
    // (MHz:dBm per channel), to compare radio firmwares.
    if (access("/tmp/kestrel-scandump", F_OK) == 0) {
        std::string line = "[scandump]";
        char b[24];
        snprintf(b, sizeof b, " auto=%d acs=%d", ci.auto_mode, ci.acs_chan);
        line += b;
        for (int i = 0; i < n; i++) {
            snprintf(b, sizeof b, " %d:%d", sc.freq_mhz[i], sc.power_dbm[i]);
            line += b;
        }
        printf("%s\n", line.c_str());
    }
    // Only log once the table looks real: right after the socket opens the
    // radio has not populated it yet, which is where "link=0MHz" and absurd
    // dBm values come from (the old RPC client logged the same garbage).
    // Also wait for the real link frequency, otherwise the one-shot log prints
    // the unresolved work_chan and looks wrong even when the screen is right.
    bool plausible = link_freq_mhz > 0 && sc.work_chan < n && sc.freq_mhz[sc.work_chan] > 0;
    if (!scan_logged && plausible) {
        scan_logged = true;
        int busiest = 0;
        for (int i = 1; i < n; i++) if (sc.power_dbm[i] > sc.power_dbm[busiest]) busiest = i;
        printf("[scan] %d channels, link=%dMHz, busiest=%dMHz@%ddBm (%s)\n",
               n, (sc.work_chan < n ? sc.freq_mhz[sc.work_chan] : 0),
               sc.freq_mhz[busiest], sc.power_dbm[busiest],
               sc.auto_mode ? "AUTO/ACS" : "MANUAL");
    }
    osd->set_chan_scan(sc);
}

void Ar8030Source::publish_link_stats() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev || !osd) return;

    artosyn_stats st;
    memset(&st, 0, sizeof(st));
    st.rf_bw_idx = -1;          // 0 is a real gear (1.25 MHz), so -1 = unknown
    auto to_db = [](uint16_t s) { return s ? 10.0f * log10f((float)s / 36.0f) : 0.0f; };

    bb_get_1v1_info_in_t info_in;
    memset(&info_in, 0, sizeof(info_in));
    info_in.frame_num = 64;
    // Oversized like the CHAN_INFO buffers: the chip's reply is 216 bytes, 8
    // more than our header's struct declares, and the client library copies
    // all of it.
    union {
        bb_get_1v1_info_out_t info;
        uint8_t raw[256];
    } info_buf;
    memset(&info_buf, 0, sizeof(info_buf));
    bb_get_1v1_info_out_t &info = info_buf.info;
    if (ar_ioctl(dev, BB_GET_1V1_INFO, &info_in, &info_buf) == 0) {
        st.snr        = to_db(info.self.snr);
        stat_snr_x10_.store((int)(st.snr * 10.0f + 0.5f));
        stat_gains_.store(info.self.gain_a | info.self.gain_b << 8);
        st.snr_raw    = info.self.snr;
        st.mcs        = info.self.tx_mcs;
        st.power      = info.self.tx_power;
        // The air VTX power we commanded (cmd 0x22), for the OSD to show instead
        // of the ground uplink readback above. tx_power_mw carries the
        // kArPwrLevels encoding (N, or N+1 for auto-capped-at-N).
        st.air_pwr_mw = tx_power_mw;
        st.gains_cur  = info.self.gain_a;
        st.gains_max  = info.self.gain_b;
        st.ldpc_error = info.self.ldpc_num_err_ratio / 10000.0f;
        st.ldpc_total = info.self.ldpc_tlv_err_ratio / 10000.0f;
        st.active     = true;
    }

    bb_get_status_in_t sin;
    memset(&sin, 0, sizeof(sin));
    sin.user_bmp = 0x03FF;
    bb_get_status_out_t sout;
    memset(&sout, 0, sizeof(sout));
    if (ar_ioctl(dev, BB_GET_STATUS, &sin, &sout) == 0) {
        st.state = sout.link_status[0].state;
        st.role  = sout.role;
        snprintf(st.mac, sizeof(st.mac), "%02X:%02X:%02X:%02X",
                 sout.mac.addr[0], sout.mac.addr[1], sout.mac.addr[2], sout.mac.addr[3]);
        snprintf(st.peer_mac, sizeof(st.peer_mac), "%02X:%02X:%02X:%02X",
                 sout.link_status[0].peer_mac.addr[0], sout.link_status[0].peer_mac.addr[1],
                 sout.link_status[0].peer_mac.addr[2], sout.link_status[0].peer_mac.addr[3]);
        static const float bw_mhz_f[] = { 1.25f, 2.5f, 5.f, 10.f, 20.f, 40.f };
        for (int pass = 0; pass < 2 && !st.tx_freq && !st.rx_freq; pass++) {
            for (int i = 0; i < BB_DATA_USER_MAX; i++) {
                const bb_user_status_t &us = sout.user_status[i];
                if (us.tx_status.freq_khz == 0 && us.rx_status.freq_khz == 0) continue;
                if (pass == 0 && !(sout.rt_sbmp & (1 << i))) continue;
                st.tx_freq   = us.tx_status.freq_khz / 1000;
                st.rx_freq   = us.rx_status.freq_khz / 1000;
                // TX side: bb_phy_status_t's header warns the RX copy is not
                // meaningful ("如果是RX端此字段无意义"), and this radio bears
                // that out for frequency - rx_status.freq_khz reads 2100 MHz on
                // a link plainly on 5839. Both copies report the same gear, so
                // it makes no difference here, but tx_status is the one whose
                // other fields match reality.
                int bw_tx = us.tx_status.bandwidth;
                int bw_rx = us.rx_status.bandwidth;
                st.rf_bw_idx = (bw_tx >= 0 && bw_tx < 6) ? bw_tx : -1;
                last_bw_rx_idx = (bw_rx >= 0 && bw_rx < 6) ? bw_rx : -1;
                st.active    = true;
                break;
            }
        }
    }

    bb_get_mcs_in_t mcs_in;
    memset(&mcs_in, 0, sizeof(mcs_in));
    mcs_in.dir = BB_DIR_RX; mcs_in.slot = (uint8_t)slot;
    bb_get_mcs_out_t mcs_out;
    memset(&mcs_out, 0, sizeof(mcs_out));
    if (ar_ioctl(dev, BB_GET_MCS, &mcs_in, &mcs_out) == 0) {
        st.rx_mcs_val        = mcs_out.mcs;
        st.rx_data_rate_kbps = mcs_out.throughput;
    }

    st.pwr_auto = tx_power_auto;

    // The video link's own width, where the events have said (ar_libre).
    if (video_bw_idx_.load() >= 0) st.rf_bw_idx = video_bw_idx_.load();
    st.radio_frame_us = (float)frame_clock_.local_period();
    st.radio_frame_mode_want = air_ver_seen_ && !air_kestrel_ ? 0 : frame_mode;   // stock air: standard

    last_link_kbps = st.rx_data_rate_kbps;   // for the periodic stat line
    last_bw_idx    = st.rf_bw_idx;           // live RF bandwidth gear

    if (st.tx_freq > 0) link_freq_mhz = st.tx_freq;

    // Sky cmd 0x26 asks the air unit for its own measured video delay - the
    // number stock shows, computed where both timestamps share a clock domain
    // so the offset that defeats a ground-side calculation never arises.
    //
    // Currently a DEAD END, kept for the next attempt: the air ACKs 0x26 with
    // a bare "26" and no payload, because its handler is gated on bit 2 of
    // vsend_ctx+0x28 (a flow-control bitmap) and nothing we send sets that bit.
    // Whatever enables it has not been identified.
    //
    // Off by default: it is an unknown command to the air unit and there is no
    // reason to send it every second in normal operation. Set
    // KESTREL_SKY_DELAY_PROBE=1 to resume experimenting.
    {
        static int probe = -1;
        if (probe < 0) {
            const char *e = getenv("KESTREL_SKY_DELAY_PROBE");
            probe = (e && *e == '1') ? 1 : 0;
        }
        if (probe && ctrl_sockfd >= 0) {
            std::vector<uint8_t> f = sky_proto.build_u8(sky::CMD_GET_VIDEO_DELAY, 0);
            int w = bb_socket_write(ctrl_sockfd, f.data(), (uint32_t)f.size(), 200);
            static bool req_logged = false;
            if (!req_logged) {
                req_logged = true;
                printf("ar8030: sent CMD_GET_VIDEO_DELAY (0x26) -> %d bytes\n", w);
            }
        }
    }

    st.last_update_ms = now_ms();
    static bool freq_logged = false;
    if (!freq_logged && st.rx_freq > 0) {
        freq_logged = true;
        printf("[link] BB_GET_STATUS rx_freq=%d MHz tx_freq=%d MHz bw=%s MHz (rx_status gear reads %s)\n",
               st.rx_freq, st.tx_freq, ar_bw_label(st.rf_bw_idx),
               ar_bw_label(last_bw_rx_idx));
    }
    osd->update_artosyn_stats(st);
}

int Ar8030Source::current_rx_mcs() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return -1;
    bb_get_status_in_t in;
    memset(&in, 0, sizeof(in));
    in.user_bmp = 0x03FF;
    bb_get_status_out_t st;
    memset(&st, 0, sizeof(st));
    if (ar_ioctl(dev, BB_GET_STATUS, &in, &st) != 0) return -1;
    if (st.link_status[0].state != 2) return -1;   // only meaningful when linked
    return (int)st.link_status[0].rx_mcs;
}

int Ar8030Source::current_link_state() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return -1;
    bb_get_status_in_t in;
    memset(&in, 0, sizeof(in));
    in.user_bmp = 0x03FF;
    bb_get_status_out_t st;
    memset(&st, 0, sizeof(st));
    if (ar_ioctl(dev, BB_GET_STATUS, &in, &st) != 0) return -1;
    return (int)st.link_status[0].state;
}

void Ar8030Source::report_link_status() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    if (!dev) return;
    bb_get_status_in_t in;
    memset(&in, 0, sizeof(in));
    bb_get_status_out_t st;
    memset(&st, 0, sizeof(st));
    if (ar_ioctl(dev, BB_GET_STATUS, &in, &st) != 0) return;
    const bb_link_status_t *ls = &st.link_status[0];
    const char *state = ls->state == 2 ? "CONNECTED"
                      : ls->state == 1 ? "connecting" : "not linked";
    printf("ar8030: role=%u link slot0 state=%u (%s) rx_mcs=%u peer=%02X:%02X:%02X:%02X"
           " local=%02X:%02X:%02X:%02X\n",
           st.role, ls->state, state, ls->rx_mcs,
           ls->peer_mac.addr[0], ls->peer_mac.addr[1], ls->peer_mac.addr[2], ls->peer_mac.addr[3],
           st.mac.addr[0], st.mac.addr[1], st.mac.addr[2], st.mac.addr[3]);
}

bool Ar8030Source::connect_bb() {
    // The whole bring-up is bracketed, not just bb_socket_open. Every one of
    // these SDK calls waits on the daemon, and a run caught in the act stopped
    // inside bb_dev_open - before a socket was ever asked for. Timing one call
    // only catches the parks that happen in that call; timing the sequence
    // catches all of them, and "we started bringing the link up and never
    // finished" is the condition worth acting on anyway.
    BringupGuard bringup_guard;
    bb_host_t* phost = nullptr;
    if (bb_host_connect(&phost, host.c_str(), host_port)) {
        if (on_ar_libre())
            printf("ar8030: bb_host_connect failed - is arlink.ko loaded and the radio up?\n");
        else
            printf("ar8030: bb_host_connect(%s:%d) failed - is /ar8030soc/daemon running?\n",
                   host.c_str(), host_port);
        return false;
    }
    bb_host = phost;
    static bool said_library = false;
    if (!said_library) {
        said_library = true;
        printf("ar8030: client library: %s\n",
               on_ar_libre() ? arlink_version() : "Artosyn's, through the daemon");
    }

    bb_dev_list_t* devs = nullptr;
    int dev_nr = bb_dev_getlist(phost, &devs);
    if (dev_nr <= 0 || !devs) {
        printf("ar8030: bb_dev_getlist found no baseband (ret=%d)\n", dev_nr);
        return false;
    }

    bb_dev_handle_t* hdev = bb_dev_open(devs[0]);
    bb_dev_freelist(devs);
    if (!hdev) {
        printf("ar8030: bb_dev_open failed\n");
        return false;
    }
    bb_dev = hdev;
    bb_sdk_shim_report();

    // bb_init/bb_start are idempotent from the app's point of view: the daemon
    // may already have brought the baseband up for another client.
    //
    // These are where bring-up parks, and it is worth being precise about why,
    // because the comment that used to sit here said we could not bound them
    // and that was simply wrong.
    //
    // bb_init() is bb_ioctl(BB_INIT_REQ) and bb_start() is
    // bb_ioctl(BB_START_REQ). bb_ioctl() hard-codes its timeout to -1, which
    // reaches bs_send_usbpack_and_wait() as the untimed branch of a condition
    // wait. When the daemon does not answer BB_INIT_REQ - which it sometimes
    // does not - the thread waits on that condvar for the life of the process.
    // Traced on hardware: "[bbio] req=0xB000002 tmo=-1 held" with no matching
    // return, and the wait still sitting there 52 seconds later.
    //
    // But the wrapper is the only thing that is untimed. bb_ioctl_ex() takes
    // the same requests with a caller-supplied deadline, so issuing them
    // directly bounds exactly the call that hangs, with no change to the vendor
    // library. A caller that can time out can also retry, which one inside an
    // untimed wait cannot do at all.
    auto bounded_req = [&](uint32_t req, const char* name) -> bool {
        for (int attempt = 1; attempt <= 2; attempt++) {
            int rc = bb_ioctl_ex(hdev, req, nullptr, nullptr, kBringupReqTimeoutMs);
            if (rc == 0) return true;
            printf("ar8030: %s returned %d (attempt %d/2)\n", name, rc, attempt);
            if (*should_stop) return false;
        }
        printf("ar8030: %s never answered - the %s is not talking\n", name,
               on_ar_libre() ? "baseband" : "daemon");
        return false;
    };
    if (!bounded_req(BB_INIT_REQ, "bb_init")) return false;
    if (*should_stop) return false;
    if (!bounded_req(BB_START_REQ, "bb_start")) return false;
    if (*should_stop) return false;

    // Match what the stock ar_ldy_gnd does on a clean boot (captured with the
    // daemon MITM proxy): the video socket is opened RX|TX with explicit buffer
    // sizes, not RX-only with default buffers. Opening it RX-only yields a
    // socket that never receives anything.
    //   video (port 3): flags=3, tx_buf=0x0800, rx_buf=0xDC00
    //   telemetry (port 2): flags=3, tx_buf=0x0800, rx_buf=0x0800
    bb_sock_opt_t sopt;
    memset(&sopt, 0, sizeof(sopt));
    sopt.tx_buf_size = 0x0800;
    sopt.rx_buf_size = (video_port == 3) ? 0xDC00 : 0x0800;
    subscribe_events();

    sockfd = bb_socket_open(hdev, (bb_slot_e)slot, (uint32_t)video_port,
                            BB_SOCK_FLAG_RX | BB_SOCK_FLAG_TX, &sopt);
    if (sockfd < 0) {
        printf("ar8030: bb_socket_open failed, slot=%d video_port=%d ret=%d\n",
               slot, video_port, sockfd);
        return false;
    }

    printf("ar8030: video socket open (slot=%d port=%d fd=%d)\n", slot, video_port, sockfd);
    // Arm the resync on every socket we open, not only on an RF state change.
    //
    // The transition to link state 2 is one way a stream begins and not the
    // only one: reopening after an idle timeout, or reconnecting while the
    // radio never visibly dropped, both start a fresh stream with no state
    // transition to notice. Those paths let the air unit's startup burst
    // through unfiltered - which is why coming back from IDLE_2 could still
    // race the picture and cut the acquiring screen short.
    //
    // Arming here covers every path, because there is no stream at all without
    // this socket. On a first connect it costs nothing: the same cadence test
    // decides when to open the gate either way.
    drop_au();
    resync_          = true;
    resync_live_     = 0;
    resync_prev_us_  = 0;
    resync_prev_cap_ = 0;
    resync_start_ms_ = now_ms();
    bb_watchdog_socket_open();
    if (*should_stop) return false;
    configure_link();
    if (*should_stop) return false;
    // --ar8030-pair binds once at startup. It parks a request like every other
    // caller rather than running its own loop, so there is a single bind
    // implementation and the on-screen indicator works for it too. Cleared so
    // a reconnect does not silently re-open the pairing window.
    if (do_pair) { do_pair = false; request_bind(); }
    // Stock's link-setup SETs, then its PRJ_DISPATCH. Order follows the
    // bbproxy capture: the MCS table lands after the link is up.
    replay_stock_rf_setup();

    // Re-apply the user's TX power. The replay is stock's startup sequence
    // verbatim, and stock's own power lives in it: SET 8 is BB_SET_POWER with
    // {usr 0, 0x18} = 24 dBm, and the two SET 9s set the auto range to 27/24.
    // So the replay silently stomps whatever configure_link() just set - pick
    // 100mW in the menu and the radio ends up at 500mW, with the OSD correctly
    // reporting 500mW while the menu says 100mW.
    apply_tx_power(tx_power_mw);

    // Stock sends this immediately after its socket setup and before anything
    // else - see send_prj_rf_config(). Ordering copied from ar_ldy_gnd 0x9bca0.
    send_prj_rf_config();

    // The air-side half. Section 31 established the same shape for the RF
    // channel: both ends have to be told, or the link just desynchronises.
    if (air_bw >= 0) request_setting(CAM_BW, air_bw);
    // kestrel-air's caps (Max Bitrate, Max Bandwidth) go when it announces
    // itself (note_air_announce): the stock air app does not know them.

    if (skip_handshake) printf("ar8030: handshake SKIPPED (--debug-no-handshake)\n");
    else send_air_handshake();

    // Bandwidth, again. The call in configure_link() answers -7 because the
    // link is not up yet; post-handshake every gear 0-5 is accepted (verified
    // with --debug-probe-rf). Skipping this left the radio on its 5 MHz
    // power-on default while stock runs 20 MHz - a 4x narrower channel, and
    // the air unit sizes its encoder to the link, which is where the ~5x
    // bitrate gap against stock came from.
    if (bandwidth >= 0) apply_bandwidth();

    // After the handshake, not before: the handshake's SET_CONFIG replay would
    // otherwise overwrite whatever we just asked for.
    if (standby_mode >= 0) request_setting(CAM_STANDBY, standby_mode);

    // Power was applied to our own radio in configure_link(), before the
    // control socket existed - push it to the air unit now that it does.
    request_setting(CAM_PWR, sky_pwr_mw(kArPwrLevels[ar_pwr_index(tx_power_mw)]));
    link_reported = false;
    start_stats();
    return true;
}

void Ar8030Source::start_stats() {
    stop_stats();
    {
        std::lock_guard<std::mutex> g(stats_mu);
        stats_stop = false;
    }
    link_state_.store(-1);
    frame_clock_.set_linked(false);
    stats_thread = std::thread(&Ar8030Source::stats_run, this);
    // Open8030 on our radio: its frame info (the air radio's clock from the
    // frame stamps). A radio that does not answer is asked again only every
    // 30 s, so the frame clock's steps keep their pace.
    if (radio_fw == 1 && arlink_request)
        frame_clock_.set_info([](FrameStampInfo *si) {
            static int fails = 0;
            static uint64_t quiet_until = 0;
            const uint64_t t = now_ms();
            if (t < quiet_until) return -1;
            uint32_t got = 0;
            // A firmware without the later fields (retries, steady mode: Open8030
            // main has the frame stamps without them) answers shorter: they read 0.
            memset(si, 0, sizeof(*si));
            const int rc = arlink_request(0x0c5a0035, nullptr, 0, si, sizeof(*si), &got, 300);
            if (rc == 0 && got >= offsetof(FrameStampInfo, retries) && si->version >= 1) { fails = 0; return 0; }
            if (++fails >= 3) { fails = 0; quiet_until = t + 30000; }
            return -1;
        });
    frame_clock_.start(
        [this](uint8_t *b) {
            bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
            uint8_t in[4] = {0x6e, 0x0e, 1, 0}, out[256] = {0};
            const int rc = dev ? ar_ioctl(dev, 0x01000064, in, out) : -1;
            if (rc == 0) *b = out[0];
            return rc;
        },
        [this](uint64_t t, double *ap) {
            if (!ap_off_valid_.load(std::memory_order_acquire)) return false;
            *ap = (double)t - (double)ap_off_us_.load(std::memory_order_relaxed);
            return true;
        });
}

void Ar8030Source::stop_stats() {
    frame_clock_.stop();
    if (!stats_thread.joinable()) return;
    {
        std::lock_guard<std::mutex> g(stats_mu);
        stats_stop = true;
    }
    stats_cv.notify_all();
    stats_thread.join();
}

void Ar8030Source::stats_run() {
    pthread_setname_np(pthread_self(), "AR8030_STAT");
    // On ar_libre the link state comes from the chip's event, so the poll is
    // only a safety net against a missed one; on Artosyn's library it is the
    // only source.
    const uint64_t link_period = on_ar_libre() ? 5000 : 400;
    uint64_t next_link = 0, next_stats = now_ms() + 3000, last_stats = 0;
    std::unique_lock<std::mutex> lk(stats_mu);
    while (!stats_stop) {
        lk.unlock();
        uint64_t t = now_ms();
        if (t >= next_link) {
            next_link = t + link_period;
            // Published on this timer, not only when a frame arrives: the
            // HUD's status snapshot used to be refreshed solely from
            // update_stats(), which runs on frame arrival - so the moment the
            // air unit went away and the frames stopped, the snapshot froze at
            // "linked" and nothing downstream could ever see the drop.
            //
            // Only the state is refreshed here. Every other figure - SNR,
            // MCS, distance - keeps its last value, which is exactly what
            // IDLE_2 is meant to show: the last reading, held.
            const unsigned events_before = link_events_.load();
            int ls = current_link_state();
            if (ls >= 0 && link_events_.load() == events_before) {   // not overtaken
                link_state_.store(ls);
                frame_clock_.set_linked(ls == 2);
                if (osd) osd->update_artosyn_link_state(ls);
            }
        }
        // Every 3 s, or sooner - at most once a second - after an MCS change.
        if (t >= next_stats || (stats_soon_.load() && t - last_stats >= 1000)) {
            stats_soon_.store(false);
            last_stats = t;
            next_stats = t + 3000;
            stat_mcs_.store(current_rx_mcs());
            // BB_CFG_DISTC is refused before the link is up (-2), exactly as
            // BB_SET_BANDWIDTH is (-7). Retry a few times once we are linked.
            int dist_m = current_distance();
            stat_dist_.store(dist_m);
            if (osd) osd->set_link_distance(dist_m);
            publish_link_stats();
        }
        publish_chan_scan();             // its own pace: 2 s, 70 ms while the scan screen is up
        send_caps();                     // when due: after a link-up or a menu change
        // Once a second. It was on the video thread, which then waited for the
        // radio's answer - a lost one held the next picture for a second.
        poll_ap_time();
        lk.lock();
        stats_cv.wait_for(lk, std::chrono::milliseconds(35), [this] { return stats_stop; });
    }
}

void Ar8030Source::disconnect_bb() {
    stop_stats();                        // before the handles it uses go
    drop_au();
    // --debug-bb-dump's file was opened lazily and never closed; on a forced exit the
    // tail of the capture was whatever had not been flushed.
    if (dump_fp) { fclose((FILE *)dump_fp); dump_fp = nullptr; }
    if (video_dump_fp) { fclose((FILE *)video_dump_fp); video_dump_fp = nullptr; }
    // Hand the channel back to AUTO/ACS before letting go of the device.
    //
    // Defensive, not a known fix: leaving the radio pinned is measurably
    // survivable (a clean exit from manual mode restarts fine, and
    // configure_link() re-asserts auto at startup anyway). But a forced exit
    // skips this path entirely, and that IS the state that wedges the next
    // bring-up - it hangs in bb_socket_open() for the video port. Restoring
    // here costs one ioctl and narrows the window.
    if (bb_dev && !chan_auto) {
        bb_set_chan_mode_t m;
        memset(&m, 0, sizeof(m));
        m.auto_mode = 1;
        printf("ar8030: restoring channel AUTO/ACS before shutdown -> %d\n",
               ar_ioctl((bb_dev_handle_t*)bb_dev, BB_SET_CHAN_MODE, &m, NULL));
        chan_auto = true;
    }
    // Instrumented: the shutdown hangs somewhere in here and the watchdog then
    // force-exits, which leaves the daemon's slot owned by a dead process and
    // wedges the next bring-up. Print around each step so the last line printed
    // names the culprit. so_deinit() in the SDK waits on exitcv with NO timeout
    // until every in-flight bb_socket_read has deregistered, so a reader that
    // never returns parks the whole teardown.
    #define SHUT_STEP(msg) do { printf("ar8030: shutdown: " msg "\n"); fflush(stdout); } while (0)
    SHUT_STEP("closing video socket");
    if (sockfd >= 0) { bb_socket_close(sockfd); sockfd = -1; }
    SHUT_STEP("closing control socket");
    if (ctrl_sockfd >= 0) { bb_socket_close(ctrl_sockfd); ctrl_sockfd = -1; }
    sky_ack_ = SkyAckWait();             // its ack will not come on this socket
    settings_q_.clear();
    // Hand the device back before letting go of the handle.
    //
    // This is the other half of bb_init/bb_start, and its absence is what
    // poisons the daemon. Closing our handle does not tell it the device is
    // free: it stays marked initialised and started, owned by a client that is
    // no longer there, and the next BB_INIT_REQ then goes unanswered for good.
    // That is why a plain reconnect - "link idle too long, reopening socket" -
    // could park the next bring-up, with nothing having crashed at all.
    //
    // Bounded like the bring-up requests, and errors are ignored: a teardown
    // that hangs or fails is not a reason to keep the handle, and past this
    // point there is nothing left to salvage anyway.
    SHUT_STEP("stopping and deinitialising");
    if (bb_dev) {
        bb_dev_handle_t *d = (bb_dev_handle_t *)bb_dev;
        bb_ioctl_ex(d, BB_STOP_REQ,   nullptr, nullptr, kBringupReqTimeoutMs);
        bb_ioctl_ex(d, BB_DEINIT_REQ, nullptr, nullptr, kBringupReqTimeoutMs);
    }
    SHUT_STEP("closing device");
    if (bb_dev)  { bb_dev_close((bb_dev_handle_t*)bb_dev); bb_dev = nullptr; }
    SHUT_STEP("disconnecting host");
    if (bb_host) { bb_host_disconnect((bb_host_t*)bb_host); bb_host = nullptr; }
    SHUT_STEP("done");
    #undef SHUT_STEP
}

void Ar8030Source::update_stats(size_t frame_size) {
    uint64_t t = now_ms();
    // After a gap (the link went, the video stopped) a new period from this
    // frame. The bytes from before it used to be counted as the first second
    // after it: the first line after a link-up said 8-9 Mbit/s "on" the 5 MHz
    // link just come up, which carries 1.56 - the rate from before the drop
    // (2026-10-16 flight log, read as the air unit flooding the new link).
    if (period_start && t - stats_frame_ms_ > 500) {
        period_start = t;
        bytes_received = 0;
    }
    stats_frame_ms_ = t;
    bytes_received += (long long)frame_size;
    if (period_start == 0) { period_start = t; return; }
    if (t - period_start >= 1000) {
        // Over the time the period really took: a second, or a little more.
        double video_bw = (double)bytes_received * 1000.0 / (double)(t - period_start);   // bytes a second
        period_start = t;
        if (osd) {
            osd->update_video_bandwidth(video_bw / 125000.0);  // -> Mbit/s
            osd->log_csv_row();
            osd->signal_render(prof::kWakeLink);
        }
        if (t - last_stat_ms >= 3000) {
            last_stat_ms = t;
            // The radio's figures, as the stats thread last read them.
            int mcs = stat_mcs_.load();
            int dist_m = stat_dist_.load();
            // link_kbps is the SDK's own "theoretical throughput for this
            // slot" (BB_GET_MCS.throughput) - i.e. what the radio believes it
            // can carry. Printing it next to the measured video rate is the
            // only way to tell a link limit from an encoder limit, and every
            // bandwidth theory so far has been argued without it.
            // Cached by publish_link_stats(), on the stats thread.
            printf("ar8030: rx=%llu bytes nals=%llu frames=%llu lost=%llu telemetry_drained=%llu"
                   " msp=%llu mcs=%d bw=%s MHz link=%.2f Mbps video=%.2f Mbps dist=%d"
                   " air=%.1fms stale=%llu foreign=%llu snr=%.1fdB freq=%d gains=%d/%d"
                   " damaged=%llu/%llu\n",
                   total_bytes, nal_count, (unsigned long long)frames_seen, frames_lost,
                   ctrl_bytes,
                   msp_bytes, mcs >= 0 ? ar_mcs_label(mcs) : -99, ar_bw_label(last_bw_idx.load()), last_link_kbps.load() / 1000.0,
                   video_bw / 125000.0, dist_m,
                   air_delay_ms < 0.0f ? 0.0f : air_delay_ms, stale_pics, foreign_slices,
                   stat_snr_x10_.load() / 10.0, link_freq_mhz,
                   stat_gains_.load() & 255, stat_gains_.load() >> 8,
                   (unsigned long long)xor_bad_, (unsigned long long)xor_checked_);
        }
        bytes_received = 0;
    }
}

void Ar8030Source::update_dvr(std::shared_ptr<std::vector<uint8_t>> frame, uint64_t cap_us) {
    if (osd && osd->is_dvr_screen_enabled()) return;
    // A recording of the camera's stream starts on a keyframe: the MP4 writer
    // takes no picture before one. With intra refresh the air unit sends
    // none unless asked, so a recording got none at all ("0 pictures
    // written"). While a recording has had no keyframe, one is asked for
    // every second, as the resync does.
    DvrRecorder &rec = DvrRecorder::instance();
    if (!rec.is_recording()) {
        if (dvr_rec_was_ && (dvr_repair_.pictures_filled() || dvr_repair_.slices_filled()))
            printf("DVR: the radio lost %u pictures and %u slices of that recording - copies put in their place\n",
                   dvr_repair_.pictures_filled(), dvr_repair_.slices_filled());
        dvr_rec_was_ = false;
        dvr_keyed_ = false;
    } else if (!dvr_rec_was_) {
        dvr_rec_was_ = true;
        dvr_repair_.reset();
    }
    if (rec.is_recording() && !dvr_keyed_) {
        if (au_has_keyframe(*frame)) {
            dvr_keyed_ = true;
        } else if (now_ms() - dvr_idr_asked_ms_ >= 1000) {
            dvr_idr_asked_ms_ = now_ms();
            printf("ar8030: recording without a keyframe yet - asking for one\n");
            request_air_idr();
        }
    }
    // The picture's capture stamp for the sample's duration; arrival times
    // only from an air unit that sends no stamps (then every one is).
    const uint64_t t = cap_us ? cap_us : au_first_recv_us;
    // A picture or slice the radio lost: a copy in its place, so the pictures
    // after it find what they refer to - Apple's decoder stopped at the first
    // loss and waited for a keyframe that never came (see utils/hevc_repair).
    if (rec.is_recording() && codec == VideoCodec::H265 && dvr_repair_.active()) {
        for (auto &u : dvr_repair_.process(*frame, t))
            rec.feed(std::make_shared<std::vector<uint8_t>>(std::move(u.au)), u.us);
        return;
    }
    // Unconditional: DvrRecorder drops the frame when it is not recording, so
    // the rec button can start and stop us without any plumbing here.
    rec.feed(frame, t);
}

// An access unit (Annex-B) holding a keyframe slice: an IRAP picture in
// H.265 (types 16-21), an IDR in H.264.
bool Ar8030Source::au_has_keyframe(const std::vector<uint8_t>& au) const {
    const size_t n = au.size();
    for (size_t i = 0; i + 3 < n; i++) {
        if (au[i] != 0 || au[i + 1] != 0 || au[i + 2] != 1) continue;
        const uint8_t h = au[i + 3];
        if (codec == VideoCodec::H265) {
            const int t = (h >> 1) & 0x3F;
            if (t >= 16 && t <= 21) return true;
        } else if ((h & 0x1F) == 5) {
            return true;
        }
        i += 3;
    }
    return false;
}

// The fastest a picture's first slice gets from its capture stamp to our
// read: the one calibration the air delay below needs, since our clock and
// the air unit's are not synced. Set by air_floor_ms. Measured with both
// clocks synced to a host over USB, on the stock air firmware: 17.4-17.9 ms
// on 2026-09-29 (1080p100 and 720p100), 20.1-20.7 ms on 2026-09-30 (1080p100,
// three thousand pictures a run) - it moves with the link, so the figure is
// only as good as this number for the setup at hand.
int Ar8030Source::air_floor_us = 20500;

// Capture -> arrival of the picture whose header came last, in us, or 0 until
// there is one. See air_win_min in the header.
uint32_t Ar8030Source::air_delay_for(uint64_t recv_us) {
    if (!cap_valid) return 0;
    const int64_t d = (int64_t)recv_us - (int64_t)cap_us64;
    const uint64_t slot = recv_us / 500000;
    int64_t floor = INT64_MAX;
    for (int i = 0; i < kAirWinSlots; i++)
        if (air_win_slot[i] + kAirWinSlots > slot && air_win_min[i] < floor)
            floor = air_win_min[i];
    // A jump of a second either way is a new clock, not a slow picture: an
    // air unit that rebooted starts its stamps again from zero.
    if (floor != INT64_MAX && (d - floor > 1000000 || d - floor < -1000000)) {
        memset(air_win_slot, 0, sizeof(air_win_slot));
        floor = INT64_MAX;
    }
    int64_t& m = air_win_min[slot % kAirWinSlots];
    if (air_win_slot[slot % kAirWinSlots] != slot) {
        air_win_slot[slot % kAirWinSlots] = slot;
        m = d;
    } else if (d < m) {
        m = d;
    }
    if (d < floor) floor = d;
    const int64_t us = d - floor + air_floor_us;
    if (us <= 0 || us > 1000000) return 0;
    const float ms = (float)us / 1000.0f;
    air_delay_ms = (air_delay_ms < 0.0f) ? ms : 0.95f * air_delay_ms + 0.05f * ms;
    if (osd) osd->set_air_delay(air_delay_ms);
    return (uint32_t)us;
}

// cmd 0x04, the air unit's version message. pl[0] is the cmd byte. It is only sent
// in some states of the air app, so a slice header with the tag (below) counts first.
void Ar8030Source::note_air_version(const uint8_t *pl, size_t n) {
    if (n < 8) return;
    const bool ka = pl[1] == 'K' && pl[2] == 'A';
    if (n >= 14) {
        // Bytes 9, 11, 13: the stock firmware's APP_VERSION; 5: the board-ID
        // version (kestrel-air sends them as stock does).
        std::lock_guard<std::mutex> g(air_info_mtx_);
        air_info_.version = true;
        air_info_.kestrel = ka;
        air_info_.stock[0] = pl[9] | (pl[10] << 8);
        air_info_.stock[1] = pl[11] | (pl[12] << 8);
        air_info_.stock[2] = pl[13] | (pl[14] << 8);
        air_info_.hw = pl[5];
        air_info_.chipid = ka ? 0 : (uint32_t)(pl[1] | (pl[2] << 8) | (pl[3] << 16) | ((uint32_t)pl[4] << 24));
    }
    if (!ka && air_hdr_tag_) return;
    note_air_announce(ka, ka ? pl[3] : 0, ka ? pl[4] : 0);
}

// 0x51, kestrel-air's frame (protocol/kestrel_air.h KA_MSG_FRAME): u8
// generation, u32 the frame (us x 256), u32 where its video part ends on the
// radio clock (us x 256), u8 radio frames a picture under the frame lock: the
// frame clock counts our radio's edges from it.
void Ar8030Source::note_air_frame(const uint8_t *pl, size_t n) {
    if (n < 10) return;
    uint32_t tq, pq;
    memcpy(&tq, pl + 2, 4);
    memcpy(&pq, pl + 6, 4);
    if (tq < 1000 * 256 || tq > 20000 * 256 || pq >= tq) return;
    frame_clock_.set_air_frame(tq / 256.0, pq / 256.0, pl[1], n >= 11 ? pl[10] : 0);
    std::lock_guard<std::mutex> g(air_info_mtx_);   // once a second
    air_info_.said |= kAirSaidFrameClock;
}

// 0x50, kestrel-air's info message: "key=value" lines (protocol/kestrel_air.h
// there, KA_MSG_INFO). pl[0] is the cmd byte. Logged when it changes.
void Ar8030Source::note_air_info(const uint8_t *pl, size_t n) {
    std::vector<std::pair<std::string, std::string>> kv;
    std::string text((const char *)pl + 1, n > 0 ? n - 1 : 0), line;
    for (size_t a = 0; a < text.size();) {
        size_t e = text.find('\n', a);
        if (e == std::string::npos) e = text.size();
        line = text.substr(a, e - a);
        a = e + 1;
        const size_t eq = line.find('=');
        if (eq == std::string::npos || eq == 0) continue;
        std::string v = line.substr(eq + 1);
        for (char &c : v) if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7e) c = '?';
        kv.emplace_back(line.substr(0, eq), v);
    }
    // Its board type (stock's: 482 Lite, 472 Lite+) says what it is too.
    for (const auto &e : kv)
        if (e.first == "board") {
            const long b = strtol(e.second.c_str(), nullptr, 10);
            note_air_prj(b == 472 ? AR_AIR_LITE_PLUS : b == 482 ? AR_AIR_LITE : 0, "info");
        }
    // The modes it says it has beyond its header's bits (864p144): the Video
    // Mode row offers them, and a stored one it lacks goes back to the default.
    {
        uint16_t xf = 0;
        for (const auto &e : kv)
            if (e.first == "864p144" && e.second == "1") xf |= kAirFeat864p144;
        const bool first = !air_info_seen_;
        air_info_seen_ = true;
        if (first || xf != air_info_feat_) {
            air_info_feat_ = xf;
            if (osd) osd->set_air_features(air_kestrel_ ? (uint16_t)(air_feat_ | xf) : 0);
            check_air_modes();
        }
    }
    // The sensor's readout and exposure: ours, they change as it runs, so not
    // part of what is logged or shown.
    for (auto it = kv.begin(); it != kv.end();) {
        const long r = strtol(it->second.c_str(), nullptr, 10);
        if (it->first == "readout") air_readout_us_ = r >= 1000 && r <= 40000 ? (uint32_t)r : 0;
        else if (it->first == "exposure") air_exposure_us_ = r > 0 && r <= 200000 ? (uint32_t)r : 0;
        else { ++it; continue; }
        it = kv.erase(it);
    }
    std::lock_guard<std::mutex> g(air_info_mtx_);
    // For the INFO tab's capabilities: 864p144 as the info says it now; a
    // readout said once in this link stays said (a new mode's comes late).
    air_info_.said = (uint16_t)((air_info_.said & ~kAirFeat864p144) | air_info_feat_ |
                                (air_readout_us_ ? kAirSaidSensorTiming : 0));
    if (kv == air_info_.kv) return;
    air_info_.kv = kv;
    printf("ar8030: air unit info:");
    for (const auto &e : kv) printf(" %s=%s", e.first.c_str(), e.second.c_str());
    printf("\n");
}

// What the air unit is (its project number: 4 Lite, 7 Lite+), from its camera
// settings (TLV 0x11, stock's way) or kestrel-air's info (its board type):
// the power levels it is offered, and the chosen power fitted to them.
// kestrel-air sends its settings only after the link-up handshake reaches it,
// which a kestrel-air started after that never had - the goggle then took a
// Lite+ for a Lite and capped it at 100mW AUTO (2026-10-12): the info, every
// 2 s, says it anyway.
void Ar8030Source::note_air_prj(int prj, const char *from) {
    if (prj <= 0 || prj == air_prj.load()) return;
    air_prj.store(prj);
    ar_pwr_offer o = ar_pwr_offered(prj);
    printf("ar8030: air unit is an %s (prj %d, from its %s): power up to %s\n", ar_air_name(prj), prj, from,
           kArPwrLevels[ar_pwr_index(o.mw[o.n - 1])].label);
    int fit = ar_pwr_fit(prj, tx_power_want_mw);
    if (fit != tx_power_mw) {
        printf("ar8030: power for it: %s (chosen %s, was %s)\n", kArPwrLevels[ar_pwr_index(fit)].label,
               kArPwrLevels[ar_pwr_index(tx_power_want_mw)].label, kArPwrLevels[ar_pwr_index(tx_power_mw)].label);
        request_rf(RF_TX_POWER_FIT, fit);
    }
    if (osd) osd->set_air_power_levels();
}

void Ar8030Source::note_air_announce(bool ka, uint8_t proto, uint8_t feat) {
    {
        std::lock_guard<std::mutex> g(air_info_mtx_);
        air_info_.kestrel = ka;
        air_info_.proto = ka ? proto : 0;
        air_info_.feat = ka ? feat : 0;
    }
    if (air_ver_seen_ && ka == air_kestrel_ && proto == air_proto_ && feat == air_feat_) return;
    air_ver_seen_ = true;
    air_kestrel_ = ka;
    air_proto_ = proto;
    air_feat_ = feat;
    if (osd) osd->set_air_max_bw(ka && (feat & kAirFeatMaxBw));
    if (osd) osd->set_air_features(ka ? (uint16_t)(feat | air_info_feat_) : 0);
    if (osd) osd->set_air_stock(!ka);
    // The mode pushed at link-up, if this air unit does not have it (864p120
    // kept from before a downgrade, or a stock air app): back to the default.
    check_air_modes();
    lat_enc_.clear(); lat_queue_.clear(); lat_write_.clear();
    lat_depth_max_ = 0;
    if (ka) {
        printf("ar8030: the air unit is kestrel-air, protocol %u, features:%s%s%s%s%s%s%s\n", proto,
               (feat & kAirFeatLatInfo) ? " air-side-times" : "",
               (feat & kAirFeatIntraRefresh) ? " intra-refresh" : "",
               (feat & kAirFeatCamImu) ? " cam-imu" : "",
               (feat & kAirFeatFcImu) ? " fc-imu" : "",
               (feat & kAirFeatApClock) ? " us-radio-clock" : "",
               (feat & kAirFeatMaxBw) ? " max-bandwidth" : "",
               (feat & kAirFeat864p120) ? " 864p120" : "");
        // Its own settings, which it keeps in RAM only: sent once it has said
        // it is kestrel-air, at every link-up (the announcement comes again),
        // whichever of the two units was powered first. The bandwidth cap
        // too when 40: an air unit that kept a 20 must hear it is lifted -
        // but only to one that widens the link at all (the feature).
        if (max_kbps > 0) request_setting(CAM_MAX_KBPS, max_kbps);
        if (feat & kAirFeatMaxBw) request_setting(CAM_MAX_BW, max_bw_mhz == 0 ? 40 : max_bw_mhz);
        // Off too: an air unit that kept running since an on is still logging.
        request_setting(CAM_AIR_LOGS, air_logs ? 1 : 0);
    } else {
        printf("ar8030: the air unit is the stock air app (no kestrel-air announcement in its "
               "version message): latency timed from its packet stamps (whole ms), the "
               "standard radio frame, no Max Bandwidth / Max Bitrate (kestrel-air only)\n");
    }
}

static float pct_ms(std::vector<uint32_t> &v, int pct) {
    if (v.empty()) return 0.0f;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, v.size() * pct / 100)] / 1000.0f;
}

// One slice's header (hdr[0] = the 0x80 byte), from a kestrel-air that sent
// feature bit 1: bytes 35..41 are the air side's own times, 10 us units, LE.
void Ar8030Source::note_air_latency(const uint8_t *hdr) {
    auto u16 = [&](int o) { return (uint32_t)(hdr[o] | (hdr[o + 1] << 8)) * 10u; };
    lat_enc_.push_back(u16(35));
    lat_queue_.push_back(u16(37));
    lat_write_.push_back(u16(39));
    if (hdr[41] > lat_depth_max_) lat_depth_max_ = hdr[41];
    const uint64_t t = now_ms();
    if (!lat_log_ms_) lat_log_ms_ = t;
    if (t - lat_log_ms_ < 5000) return;
    printf("ar8030: air side, %zu slices in %.1fs: capture->encoder out p50 %.2f p99 %.2f ms | "
           "waiting in the air's ring p50 %.2f p99 %.2f ms | radio write p50 %.2f p99 %.2f ms | "
           "ring depth max %u\n",
           lat_enc_.size(), (t - lat_log_ms_) / 1000.0, pct_ms(lat_enc_, 50), pct_ms(lat_enc_, 99),
           pct_ms(lat_queue_, 50), pct_ms(lat_queue_, 99), pct_ms(lat_write_, 50),
           pct_ms(lat_write_, 99), lat_depth_max_);
    lat_enc_.clear(); lat_queue_.clear(); lat_write_.clear();
    lat_depth_max_ = 0;
    lat_log_ms_ = t;
}

// The radio's AP clock against ours. BB_GET_AP_TIME answers in whole ms, so a
// sample is the middle of the request minus the middle of that ms; a slow
// answer (over 2 ms) says little and is skipped. Averaged (1/8 per sample),
// and started again on a jump of over 20 ms: a new link starts a new clock.
// On the stats thread; the video thread reads the offset.
void Ar8030Source::poll_ap_time() {
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    const uint64_t t = now_ms();
    if (!dev || t - ap_poll_ms_ < 1000) return;
    ap_poll_ms_ = t;
    uint32_t ap_ms = 0;
    const uint64_t t0 = now_us();
    if (ar_ioctl(dev, BB_GET_AP_TIME, nullptr, &ap_ms) != 0) return;
    const uint64_t t1 = now_us();
    if (t1 - t0 > 2000) {
        ap_poll_ms_ = t - 800;           // again in 200 ms: this thread is not real-time
        return;
    }
    const int64_t off = (int64_t)((t0 + t1) / 2) - ((int64_t)ap_ms * 1000 + 500);
    const int64_t cur = ap_off_us_.load(std::memory_order_relaxed);
    const bool valid = ap_off_valid_.load(std::memory_order_relaxed);
    if (!valid || off - cur > 20000 || off - cur < -20000) {
        if (valid)
            printf("ar8030: the radio's AP clock moved by %.1f ms against ours\n",
                   (off - cur) / 1000.0);
        ap_off_us_.store(off, std::memory_order_relaxed);
        ap_off_valid_.store(true, std::memory_order_release);
    } else {
        ap_off_us_.store(cur + (off - cur) / 8, std::memory_order_relaxed);
    }
}

// Open8030 here: the menu's caps on the link. On the stats thread: each try
// is up to a second of SET 22 and two half-second requests, retried every 2 s
// for as long as the radio does not take them - on the video thread (it was in
// bw_poll) a radio that stopped answering held the pictures that long.
void Ar8030Source::send_caps() {
    // The menu's caps (RADIO > Max Bandwidth, Max MCS) on the
    // link - the widest it may go (SET 22, what our radio asks the air's for;
    // live, no restart) and the highest MCS our receive policy may use
    // (request 0x0c5a0036). Our radio keeps both until changed; sent again at
    // every link-up all the same. Every 2 s until all are taken (SET 22 waits
    // for the link); -95 (a radio without the request) counts as taken.
    // AUTO / off: nothing sent at link-up, the radio runs on its own config;
    // chosen from the menu, sent once (40 MHz, MCS none, steady off) to lift
    // what was set before.
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    const uint64_t t = now_ms();
    if (dev && radio_fw == 1 && caps_due.load() && link_state_.load() == 2 && arlink_request) {
        static uint64_t tried = 0;
        if (t - tried >= 2000) {
            tried = t;
            const bool live = caps_live.load();
            const int mhz = max_bw_mhz;
            int rc_bw = 1, rc_mcs = 1, rc_st = 1;   // 1: not sent
            if (mhz > 0 || live) {
                const uint8_t gear = mhz == 0 || mhz >= 40 ? 5 : mhz >= 20 ? 4 : mhz >= 10 ? 3 : 2;
                bb_set_bandwidth_t p;
                memset(&p, 0, sizeof(p));
                p.slot = BB_SLOT_AP;
                p.dir = BB_DIR_RX;
                p.bandwidth = gear;
                rc_bw = ar_ioctl(dev, BB_SET_BANDWIDTH, &p, NULL);
            }
            uint32_t got = 0;
            if (max_mcs >= 0 || live) {
                const uint8_t cap = max_mcs < 0 ? 0xff : (uint8_t)(int8_t)max_mcs;
                rc_mcs = arlink_request(0x0c5a0036, &cap, 1, nullptr, 0, &got, 500);
            }
            if (steady || live) {
                const uint8_t st = steady ? 1 : 0;
                rc_st = arlink_request(0x0c5a0037, &st, 1, nullptr, 0, &got, 500);
            }
            auto rc = [](int r) { return r == 1 ? std::string("not sent") : std::to_string(r); };
            if (rc_bw != 1 || rc_mcs != 1 || rc_st != 1)
            printf("ar8030: caps: %s (SET 22 -> %s), MCS %s (-> %s), steady latency %s (-> %s)\n",
                   mhz ? ("up to " + std::to_string(mhz) + " MHz").c_str() : "AUTO", rc(rc_bw).c_str(),
                   max_mcs < 0 ? "AUTO" : std::to_string(max_mcs).c_str(), rc(rc_mcs).c_str(),
                   steady ? "on" : "off", rc(rc_st).c_str());
            auto taken = [](int r) { return r == 0 || r == 1 || r == -95; };
            if (taken(rc_bw) && taken(rc_mcs) && taken(rc_st)) { caps_due.store(false); caps_live.store(false); }
        }
    }
}

// Experiment: `echo G > /tmp/bw.req` puts the video link on bandwidth gear G
// (bb_bandwidth_e, 0..5 = 1.25, 2.5, 5, 10, 20, 40 MHz; bb_config_*.json starts
// it, the "br" link, at 5 MHz = 2). The air unit is the AP and transmits the
// video, so it has to change first: sky cmd 0x24 with gear + 1 to kestrel-air
// (stock ignores it; its own handshake sends 0 at every link-up), then our
// receive side. The radio also widens the link by itself after link-up. No new picture 3 s later and both go back to the
// gear before - kestrel-air does that by itself when the link is gone, since we
// could not reach it then. Not saved.
void Ar8030Source::bw_poll() {
    static int cur = 2, prev = -1;
    static uint64_t last = 0, at = 0, frames_at = 0;
    bb_dev_handle_t *dev = (bb_dev_handle_t *)bb_dev;
    const uint64_t t = now_ms();
    if (!dev || t - last < 500) return;
    last = t;
    // The widest the link may go with a stock air radio (it drops and re-forms
    // the link at every step to or from 40 MHz) is our Open8030 radio's own
    // call: it keeps to 20 MHz until the air radio's frame stamps say Open8030.
    // Tests: /tmp/kestrel-regwrite holding "offset mask value" (hex) changes
    // those bits of one baseband register (0x0a110000 + offset) through
    // GET/SET 100, for A/B tests of radio settings. Logs before and after.
    if (FILE *f = fopen("/tmp/kestrel-regwrite", "r")) {
        unsigned off = 0, mask = 0, val = 0;
        const int n = fscanf(f, "%x %x %x", &off, &mask, &val);
        fclose(f);
        unlink("/tmp/kestrel-regwrite");
        if (n == 3 && off < 0x1000 && !(off & 3)) {
            uint8_t in[4] = {(uint8_t)off, (uint8_t)(off >> 8), 4, 0}, out[256] = {0};
            if (ar_ioctl(dev, 0x01000064, in, out) == 0) {
                uint32_t before = out[0] | out[1] << 8 | out[2] << 16 | (uint32_t)out[3] << 24;
                uint32_t after = (before & ~mask) | (val & mask);
                uint8_t w[260] = {(uint8_t)off, (uint8_t)(off >> 8), 4, 0,
                                  (uint8_t)after, (uint8_t)(after >> 8), (uint8_t)(after >> 16), (uint8_t)(after >> 24)};
                const int rc = ar_ioctl(dev, 0x02000064, w, NULL);
                printf("ar8030: /tmp/kestrel-regwrite 0x%03x: 0x%08x -> 0x%08x (rc %d)\n", off, before, after, rc);
            }
        }
    }
    // Tests: touching /tmp/kestrel-regdump writes the whole baseband register
    // block (0x0a110000..0x0a110fff, GET 100 in 256-byte reads) to
    // /tmp/kestrel-regs.txt, one "offset value" line per word, to diff radio
    // firmwares on a live link.
    if (access("/tmp/kestrel-regdump", F_OK) == 0) {
        unlink("/tmp/kestrel-regdump");
        if (FILE *o = fopen("/tmp/kestrel-regs.txt", "w")) {
            for (unsigned off = 0; off < 0x1000; off += 256) {
                uint8_t in[4] = {(uint8_t)off, (uint8_t)(off >> 8), 0, 1}, out[256] = {0};
                if (ar_ioctl(dev, 0x01000064, in, out) != 0) { fprintf(o, "0x%03x FAIL\n", off); continue; }
                for (unsigned w = 0; w < 256; w += 4)
                    fprintf(o, "0x%03x %08x\n", off + w,
                            out[w] | out[w + 1] << 8 | out[w + 2] << 16 | (uint32_t)out[w + 3] << 24);
            }
            fclose(o);
            printf("ar8030: /tmp/kestrel-regdump -> /tmp/kestrel-regs.txt\n");
        }
    }
    // Tests: /tmp/kestrel-chan holding a frequency in MHz moves the link there
    // exactly as OK on the channel page does; -1 hands it back to AUTO.
    if (FILE *f = fopen("/tmp/kestrel-chan", "r")) {
        int mhz = 0;
        if (fscanf(f, "%d", &mhz) != 1) mhz = 0;
        fclose(f);
        unlink("/tmp/kestrel-chan");
        if (mhz == -1 || (mhz >= 2000 && mhz <= 7000)) {
            printf("ar8030: /tmp/kestrel-chan: %d\n", mhz);
            request_rf(RF_CHAN, mhz < 0 ? -1 : mhz * 1000);
        }
    }
    auto set_ours = [&](int g) {
        bb_set_bandwidth_t p;
        memset(&p, 0, sizeof(p));
        p.slot = BB_SLOT_AP;
        p.dir = BB_DIR_RX;
        p.bandwidth = (uint8_t)g;
        return ar_ioctl(dev, BB_SET_BANDWIDTH, &p, NULL);
    };
    auto tell_air = [&](int g) {
        if (ctrl_sockfd < 0) return -1;
        // gear + 1: 0 is what stock's own handshake sends, and means "leave it".
        std::vector<uint8_t> f = sky_proto.build_u8(sky::CMD_SET_BB_BANDWIDTH, (uint8_t)(g + 1));
        return bb_socket_write(ctrl_sockfd, f.data(), (uint32_t)f.size(), 200);
    };
    if (prev >= 0 && t - at >= 3000) {
        if (frames_seen == frames_at) {
            printf("ar8030: bandwidth gear %d: no picture in 3 s - back to gear %d\n", cur, prev);
            const int w = tell_air(prev);
            usleep(50000);
            printf("ar8030: bandwidth gear %d: air <- %d, ours -> %d\n", prev, w, set_ours(prev));
            cur = prev;
        } else {
            printf("ar8030: bandwidth gear %d kept: %llu pictures in 3 s\n", cur,
                   (unsigned long long)(frames_seen - frames_at));
        }
        prev = -1;
    }
    FILE *rq = fopen("/tmp/bw.req", "r");
    if (!rq) return;
    int g = -1;
    if (fscanf(rq, "%d", &g) != 1) g = -1;
    fclose(rq);
    unlink("/tmp/bw.req");
    if (g < 0 || g > 5 || prev >= 0 || g == cur) return;
    const int w = tell_air(g);
    usleep(50000);
    printf("ar8030: bandwidth gear %d -> %d: air <- %d, ours -> %d\n", cur, g, w, set_ours(g));
    prev = cur;
    cur = g;
    at = t;
    frames_at = frames_seen;
}

// Bytes 26..29 of a header: the AP clock (ms) the air unit stamped the slice
// with - when stock built the packet; when kestrel-air's encoder handed the
// slice out, with the sub-ms part in byte 2 (kAirFeatApClock). Our AP clock
// now, minus that, is the stamp -> here. Capture -> stamp comes from the same
// header: kestrel-air's encoder time (bytes 35..36, 10 us), or stock's byte
// 30 (whole ms). Without byte 2 both ends read the clock in whole ms, so a
// picture is good to about 1 ms; with it, to the two offsets' noise.
void Ar8030Source::note_ap_delay(const uint8_t *hdr, size_t len, bool new_pic) {
    if (new_pic) { pic_cap_us_ = 0; pic_air_unknown_ = false; }
    hdr_out_us_ = 0;
    if (len >= 18) {
        const uint32_t h = (uint32_t)(hdr[16] | (hdr[17] << 8)), fps = hdr[9];
        // A new video mode (100 <-> 60 fps is 2 <-> 4 slices at the same size):
        // where a picture's slices start has to be learned again.
        if (hdr_fps_ && hdr_height_ && (fps != hdr_fps_ || h != hdr_height_)) {
            printf("ar8030: video mode %ux%u fps -> %u lines %u fps: learning its slices again\n",
                   hdr_height_, hdr_fps_, h, fps);
            last_slice_addr = -1;
            last_addr_cand = -1; last_addr_streak = 0; last_count_cand = 0; last_slice_count = 0;
            cut_last_hseq_ = -1; cut_period_ = 0; cut_confirms_ = 0; pics_since_cut_ = 0;
            readout_win_.clear(); readout_us_ = 0;
            air_readout_us_ = 0;   // the air unit says the new mode's within 2 s
        }
        hdr_height_ = h;
        hdr_fps_ = fps;
        if (fps >= 10 && fps <= 500) {
            g_camera_lines.store((int)h, std::memory_order_relaxed);
            g_camera_fps.store((int)fps, std::memory_order_release);
        }
    }
    if (!ap_off_valid_.load(std::memory_order_acquire) || len < 31) return;
    auto u16 = [&](int o) { return (int64_t)(hdr[o] | (hdr[o + 1] << 8)) * 10; };
    const bool ka = air_kestrel_ && (air_feat_ & kAirFeatLatInfo) && len >= 42;
    const bool fine = ka && (air_feat_ & kAirFeatApClock);
    const uint32_t rt = (uint32_t)hdr[26] | ((uint32_t)hdr[27] << 8) |
                        ((uint32_t)hdr[28] << 16) | ((uint32_t)hdr[29] << 24);
    const uint64_t here = now_us();
    // The radio clock now: on the frame's edges when known (both ends read
    // it there, utils/frame_clock.hpp), else our coarse offset to it.
    uint64_t ap_us;
    const bool on_frame = frame_clock_.ap_us(here, &ap_us);
    if (!on_frame) ap_us = here - (uint64_t)ap_off_us_.load(std::memory_order_relaxed);
    const int64_t frac_us = fine ? ((int64_t)hdr[2] * 1000 + 128) / 256 : 500;
    const int64_t d = (int64_t)(int32_t)((uint32_t)(ap_us / 1000) - rt) * 1000 +
                      (int64_t)(ap_us % 1000) - frac_us;
    if (d < -1000000 || d > 1000000) return;
    // Right after the frame clock counts its edges from our coarse offset,
    // which can be a frame or more off: the fastest of a few hundred slices
    // gets here 2.4 ms to 2.4 ms and a frame after leaving the encoder, or our
    // count is whole frames off. No sooner: the write into the air's radio
    // (~1.3 ms) and our radio's hand-up (~1.7) alone take 3; seen 3.6-4.1 ms on
    // the 3.25 ms frame (40 MHz, mode 2), 5.1-5.9 at 20 MHz (a block written
    // just in time, its airtime, the radio's lead and hand-over). The window
    // was half a frame to one and a half, which on 3.25 ms ends at 4.9.
    // With Open8030 at both ends the fastest comes sooner: 1.93 ms seen on the
    // 3.25 ms frame (2026-10-18), and a floor of 2.4 moved a right count a frame
    // off. 1.5 ms holds both: one frame from it covers stock's 3.6-4.1 there,
    // 4.65-4.92 on 4650.88 and 5.1-5.9 on 5633.92.
    static constexpr double kFastestSliceUs = 1500;
    // 2 s windows, all the time: the first after a count is acted on alone,
    // later ones when two in a row say the same (one window's fastest can be
    // a stray). After a shift, nothing until the frame clock has taken it:
    // a window across the shift held slices from both sides, and the old
    // side asked for the same shift again (2026-10-18: +217 frames twice,
    // then a window on the right side for a moment said all was well - the
    // clock was left 705 ms off).
    if (frame_clock_.count_gen() != fc_count_gen_) {
        fc_count_gen_ = frame_clock_.count_gen();
        fc_verified_ = false;
        fc_fresh_ = true;
        fc_last_k_ = 0;
        fc_min_d_ = INT64_MAX;
        fc_n_ = 0;
        fc_win_ms_ = 0;
    }
    if (fc_wait_ && frame_clock_.shifts_taken() != fc_shifts_seen_) {
        fc_wait_ = false;
        fc_min_d_ = INT64_MAX;
        fc_n_ = 0;
        fc_win_ms_ = 0;
    }
    // Only slices that waited under 1 ms in the air unit's queue: on a slow
    // link they queue there, and the fastest is late for real.
    const int64_t queued = ka ? u16(37) : 0;
    if (on_frame && fine && !fc_wait_ && queued < 1000) {
        fc_min_d_ = std::min(fc_min_d_, d);
        fc_n_++;
        const uint64_t tm = here / 1000;
        if (!fc_win_ms_) fc_win_ms_ = tm;
        if (tm - fc_win_ms_ >= 2000) {
            const double T = frame_clock_.frame_period();
            if (fc_n_ >= 100 && T > 0) {
                const int64_t k = (int64_t)std::floor(((double)fc_min_d_ - kFastestSliceUs) / T);
                // On the frame stamps nothing is counted: a slice out of the
                // window is said, never acted on.
                const bool act = k && (fc_fresh_ || k == fc_last_k_) && !frame_clock_.exact();
                if (k || fc_fresh_)
                    printf("ar8030: frame clock: fastest slice %.2f ms%s\n", fc_min_d_ / 1000.0,
                           act ? ", so our count of edges is off"
                           : k && frame_clock_.exact() ? " (on the frame stamps: left alone)"
                           : k ? " (once: checked again)" : "");
                if (act) {
                    fc_shifts_seen_ = frame_clock_.shifts_taken();
                    fc_wait_ = true;
                    frame_clock_.shift(-k);
                    fc_last_k_ = 0;
                    fc_verified_ = false;
                } else {
                    fc_last_k_ = k;
                    if (!k) fc_verified_ = true;
                }
                fc_fresh_ = false;
            }
            fc_min_d_ = INT64_MAX;
            fc_n_ = 0;
            fc_win_ms_ = tm;
        }
    }
    const int64_t enc = ka ? u16(35) : (int64_t)hdr[30] * 1000;
    // With kestrel-air's frame (the frame clock in play), its figures only
    // once the count is checked: before that our count can be frames off (a
    // first count from our radio's copy of the radio clock, 43-750 ms off
    // under Open8030), and a wrong figure is worse than none.
    if (frame_clock_.air_known() && !(on_frame && (fc_verified_ || frame_clock_.exact()))) {
        pic_air_unknown_ = true;
        return;
    }
    const int64_t queue = ka ? u16(37) : 0;
    if (ka && (hdr[32] & 0x40)) ap_enc_last_.push_back((uint32_t)enc);
    // Not while a link-up's resync drops the pictures: those slices sat in
    // the air radio while the link formed and are never shown, and the
    // per-picture figures leave them out too. Counted, they made the first
    // window after a link-up read p99 765-795 ms with its shown pictures at
    // p99 25-30 (2026-10-20, Open8030 frame stamps: exact from link-up).
    if (!resync_ && d - queue >= 0) {
        ap_link_.push_back((uint32_t)(d - queue));
        if (len >= 33) {   // byte 32: slice index | 0x20 more follow | 0x40 last
            const int pos = (hdr[32] & 0x40) ? 2 : (hdr[32] & 0x1f) == 0 ? 0 : 1;
            ap_link_pos_[pos].push_back((uint32_t)(d - queue));
        }
    }
    // kestrel-air's stamp is when the encoder handed this slice out; stock's,
    // when it built the packet - just after, to the whole ms.
    if (d >= 0 && (uint64_t)d < here) hdr_out_us_ = here - (uint64_t)d;
    if (new_pic && d + enc > 0 && (uint64_t)(d + enc) < here) {
        pic_cap_us_ = here - (uint64_t)(d + enc);
        pic_enc_us_ = (uint32_t)enc;
    }
}

// A picture handed to the decoder whose capture was measured: its first and
// last slice's arrival, against the capture, and every 5 s the spread.
void Ar8030Source::note_ap_picture(uint64_t last_recv_us) {
    if (!au_measured_) return;
    const uint64_t cap = au_first_recv_us - au_air_us;
    ap_enc_.push_back(au_enc_us_);
    ap_first_.push_back(au_air_us);
    if (last_recv_us >= cap) ap_all_.push_back((uint32_t)(last_recv_us - cap));
    const uint64_t t = now_ms();
    if (!ap_log_ms_) ap_log_ms_ = t;
    if (t - ap_log_ms_ < 5000) return;
    printf("ar8030: latency by the radio clock (%s), %zu pictures: capture->encoder out p50 %.2f "
           "p99 %.2f | capture->first slice here p50 %.2f p99 %.2f | capture->all slices here "
           "p50 %.2f p99 %.2f ms | per slice, air unit's radio->here p50 %.2f p99 %.2f ms\n",
           (air_feat_ & kAirFeatApClock) ? "us stamps" : "ms stamps", ap_first_.size(),
           pct_ms(ap_enc_, 50), pct_ms(ap_enc_, 99), pct_ms(ap_first_, 50), pct_ms(ap_first_, 99),
           pct_ms(ap_all_, 50), pct_ms(ap_all_, 99), pct_ms(ap_link_, 50), pct_ms(ap_link_, 99));
    printf("ar8030: per slice, air unit's radio->here by place: first %zu p50 %.2f p99 %.2f | "
           "middle %zu p50 %.2f p99 %.2f | last %zu p50 %.2f p99 %.2f ms | capture->last slice "
           "out of the encoder p50 %.2f p99 %.2f ms | sensor readout, learned, %.2f ms, the air unit's %.2f ms\n",
           ap_link_pos_[0].size(), pct_ms(ap_link_pos_[0], 50), pct_ms(ap_link_pos_[0], 99),
           ap_link_pos_[1].size(), pct_ms(ap_link_pos_[1], 50), pct_ms(ap_link_pos_[1], 99),
           ap_link_pos_[2].size(), pct_ms(ap_link_pos_[2], 50), pct_ms(ap_link_pos_[2], 99),
           pct_ms(ap_enc_last_, 50), pct_ms(ap_enc_last_, 99), readout_us_ / 1000.0, air_readout_us_ / 1000.0);
    ap_enc_.clear(); ap_first_.clear(); ap_all_.clear(); ap_link_.clear(); ap_enc_last_.clear();
    for (auto &v : ap_link_pos_) v.clear();
    ap_log_ms_ = t;
}

// The stock "enable I-frames" message, which the handshake also carries.
void Ar8030Source::request_air_idr() {
    if (ctrl_sockfd < 0) return;
    std::vector<uint8_t> f = sky_proto.build_u8(sky::CMD_ENABLE_IDR, 1);
    int w = bb_socket_write(ctrl_sockfd, f.data(), (uint32_t)f.size(), 200);
    printf("ar8030: asked the air unit for a keyframe (seq=0x%02X) -> %d\n", f[3], w);
}

// Emit one NAL (payload without the leading start code) to the decoder.
void Ar8030Source::emit_nal(const uint8_t* nal, size_t len) {
    if (len == 0) return;

    // The AR8030 multiplexes side-channel packets (RTP-like, first byte 0x80)
    // into the video stream, and an RF bit-error can corrupt a real NAL header.
    // Either way forbidden_zero_bit==1 means this is NOT a valid H.26x NAL.
    // Folding such bytes into a picture's access unit makes the rkvdec2 hardware
    // hang (task timeout -> reset storm -> blocky). Drop them so only clean
    // slices reach the decoder (ffmpeg and the stock firmware skip them too).
    if (nal[0] & 0x80) {
        nal_count++;
        // Not junk: the air unit's per-picture metadata header, repeated
        // ahead of each slice. Layout, from captures against known settings:
        //   [9]      fps        (0x64=100, 0x3C=60)
        //   [12..13] picture counter LE
        //   [14..15] width  LE  (0x0780=1920, 0x0500=1280)
        //   [16..17] height LE  (0x0438=1080, 0x02D0=720)
        //   [18..21] capture stamp LE u32: the air unit's CLOCK_MONOTONIC in
        //            microseconds, the same clock as its encoder PTS
        // Validate before trusting: the Annex-B splitter can hit a false start
        // code inside a header and hand us a shifted fragment. A good header
        // has a plausible resolution and a non-zero fps.
        if (len >= 22) {
            unsigned w   = nal[14] | (nal[15] << 8);
            unsigned h   = nal[16] | (nal[17] << 8);
            unsigned fps = nal[9];
            bool sane = (w >= 160 && w <= 4096) && (h >= 120 && h <= 2160) &&
                        (fps > 0 && fps <= 240);
            if (sane) {
                // Frame counter - detect pictures the link dropped.
                uint16_t hseq = (uint16_t)(nal[12] | (nal[13] << 8));
                const uint32_t cap32 = (uint32_t)nal[18] | ((uint32_t)nal[19] << 8) |
                                       ((uint32_t)nal[20] << 16) | ((uint32_t)nal[21] << 24);
                if (ltrace::on())
                    ltrace::rec(ltrace::kHdr, now_us(), hseq, cap32, nal, len);
                // The header repeats for each slice of a picture, so anything
                // measured per picture has to be gated on the counter changing.
                const bool new_pic = !hdr_seq_valid_ || hseq != hdr_seq_;
                hdr_seq_ = hseq;
                hdr_seq_valid_ = true;
                slice_hseq_ = hseq;
                // kestrel-air marks each picture's last slice: the picture is
                // whole when it is in, whatever its address. A GOP start with
                // intra refresh is cut differently (2 slices, the second from
                // CTB 120 at 1080p), so the learned last address never came
                // and it waited for the next picture - a refresh late, then
                // overtaken: 1 picture in 17 never shown (2026-11-05).
                slice_last_ = air_hdr_tag_ && len >= 33 && (nal[32] & 0x40);
                // A picture from the past, and everything up to the next
                // header with it: nothing it says about the link is true.
                if (new_pic) hdr_stale_ = stale_picture(cap32);
                if (hdr_stale_) return;
                // kestrel-air tags every header (byte 33 'K', 10 protocol, 11 features).
                if (len >= 42 && nal[33] == 0x4B) {
                    air_hdr_tag_ = true;
                    air_untagged_ = 0;
                    note_air_announce(true, nal[10], nal[11]);
                } else if (len >= 42 && air_hdr_tag_ && ++air_untagged_ >= 200) {
                    // The tag stopped (another air unit, or KA_LATINFO=0): the version
                    // message decides again. 200 headers, so one misread cannot clear it.
                    air_hdr_tag_ = false;
                    air_untagged_ = 0;
                    air_ver_seen_ = false;
                } else if (!air_hdr_tag_ && !air_ver_seen_ && ++air_stock_hdrs_ >= 200) {
                    // Video and never the tag: the stock air app, which sends its
                    // version message only in some states (none at all after a
                    // link-up on 2026-10-20). A kestrel-air without the tag says
                    // otherwise in its version message, which then decides.
                    air_stock_hdrs_ = 0;
                    note_air_announce(false, 0, 0);
                }
                if (air_kestrel_ && (air_feat_ & kAirFeatLatInfo) && len >= 42)
                    note_air_latency(nal);
                note_ap_delay(nal, len, new_pic);
                // The capture stamp, unwrapped: 32 bits of microseconds last
                // 71 minutes. An air unit reboot sends it backwards by less
                // than half the range, which is not a wrap.
                if (cap_valid && cap32 < cap_last32 && cap_last32 - cap32 > 0x80000000u)
                    cap_wraps++;
                cap_last32 = cap32;
                cap_us64 = ((uint64_t)cap_wraps << 32) | cap32;
                cap_valid = true;
                hdr_fps = fps;
                if (new_pic) {
                    if (hseq_valid) {
                        uint16_t gap = (uint16_t)(hseq - last_hseq);
                        if (gap > 1 && gap < 1000) {
                            frames_lost += (gap - 1);
                        }
                    }
                    last_hseq = hseq;
                    hseq_valid = true;
                }

                // Are we live yet? Compare how far apart two pictures were
                // captured with how far apart they arrived. Draining a backlog
                // those disagree wildly - 16.7ms of capture in under 4ms of
                // wall time; live they converge.
                if (resync_ && new_pic) {
                    uint32_t cap_hdr = (uint32_t)nal[18] | ((uint32_t)nal[19] << 8) |
                                       ((uint32_t)nal[20] << 16) | ((uint32_t)nal[21] << 24);
                    uint64_t arr_us = now_us();
                    if (resync_prev_us_) {
                        uint32_t d_cap = cap_hdr - resync_prev_cap_;
                        uint64_t d_arr = arr_us - resync_prev_us_;
                        // Skip the outage gap itself and any header we misread:
                        // only ordinary frame spacing says anything about pace.
                        if (d_cap > 0 && d_cap < 1000000) {
                            if (d_arr * 10 >= (uint64_t)d_cap * 6) resync_live_++;
                            else                                   resync_live_ = 0;
                        }
                    }
                    resync_prev_us_  = arr_us;
                    resync_prev_cap_ = cap_hdr;
                }
            }
        }
        return;
    }

    uint8_t nal_type = (codec == VideoCodec::H265) ? ((nal[0] >> 1) & 0x3F)
                                                   : (nal[0] & 0x1F);

    // Not every "NAL" the splitter finds is video. The air unit interleaves
    // small blocks of its own (26-49 bytes, e.g. "01 80 07 38 04 .." carrying
    // 1920x1080, or "0N 00 00 00 64 00" counting up) that follow a start-code
    // pattern, and whose first byte reads as a slice type - roughly one every
    // 36 pictures at 1080p120. Added to a picture, they were shrugged off by
    // MPP and FFmpeg but made the iPhone's decoder fail the web stream within
    // half a second. A header the spec forbids is not a NAL: forbidden bit
    // set, a layer other than the base layer, or temporal id 0.
    if (codec == VideoCodec::H265 && len >= 2 &&
        ((nal[0] & 0x80) || (((nal[0] & 1) << 5) | (nal[1] >> 3)) != 0 || (nal[1] & 7) == 0)) {
        static uint64_t foreign = 0;
        if (++foreign <= 3 || (foreign % 1000) == 0)
            printf("ar8030: skipped a non-video block in the video stream (%zu bytes, "
                   "hdr %02X %02X; %llu so far)\n", len, nal[0], nal[1],
                   (unsigned long long)foreign);
        return;
    }

    // The air unit's IMU samples (kestrel-air --imu) ride in SEI after each picture's last
    // slice. Taken whatever the state of the picture; the decoder drops the SEI itself below.
    if (codec == VideoCodec::H265 && (nal_type == 39 || nal_type == 40))
        stab::ImuStream::get().on_sei_nal(nal, len);

    // Everything up to the next header belongs to a stale picture's.
    if (hdr_stale_) return;

    // feed_packet_to_decoder expects the Annex-B start code present, so rebuild it.
    static thread_local std::vector<uint8_t> unit;
    unit.clear();
    unit.reserve(len + 4);
    unit.insert(unit.end(), {0x00, 0x00, 0x00, 0x01});
    unit.insert(unit.end(), nal, nal + len);

    bool is_vcl = (codec == VideoCodec::H265) ? (nal_type <= 31) : (nal_type >= 1 && nal_type <= 5);

    // Resync gate. After a reacquire, everything up to the first keyframe of the
    // new link is dropped - it is either the tail of the old stream or inter
    // frames with no reference to decode against.
    //
    // Parameter sets are the exception and pass through: the decoder caches them
    // and prepends them to the next IDR, so dropping them would leave the
    // keyframe we are waiting for undecodable.
    if (resync_) {
        const bool ps = (codec == VideoCodec::H265)
                            ? (nal_type == 32 || nal_type == 33 || nal_type == 34)
                            : (nal_type == 7 || nal_type == 8);
        // The same test the decoder uses for is_key, so what opens the stream
        // here is exactly what it will treat as a keyframe.
        const bool key = (nal_type >= 19 && nal_type <= 21) || nal_type == 5;
        if (!ps) {
            const uint64_t held = now_ms() - resync_start_ms_;
            const bool timed_out = held > kResyncMaxMs;
            const bool caught_up = resync_live_ >= kResyncLive || timed_out;
            if (is_vcl && key && caught_up) {
                resync_ = false;
                printf("ar8030: resync - live after %llums (%s), video resumes\n",
                       (unsigned long long)held,
                       timed_out ? "timed out, showing it anyway" : "arriving at capture rate");
            } else {
                // Intra refresh has no keyframes to join on, and the ones the
                // air sent at link-up came before the video did: once the
                // stream is live, ask for one, again every 1.5 s - a keyframe
                // on a new link (5 MHz, low MCS) takes longer than half a
                // second to arrive, and each ask meant another one.
                if (is_vcl && !key && resync_live_ >= kResyncLive && air_kestrel_ &&
                    (air_feat_ & kAirFeatIntraRefresh) && now_ms() - idr_asked_ms_ >= 1500) {
                    idr_asked_ms_ = now_ms();
                    request_air_idr();
                }
                return;                       // still catching up
            }
        }
    }

    // No slice before the decoder has a picture parameter set: it refuses
    // every one ("PPS id out of range"), and with intra refresh the sets only
    // come with a keyframe asked for. The resync opens on a keyframe, but its
    // parameter sets can be lost while the keyframe gets through (a new link
    // at 5 MHz, then re-formed at the step to 20) - and once past the resync
    // nothing asked again: video arrived and nothing was ever shown. Until a
    // valid PPS has come, slices are held back and a keyframe is asked for
    // every 1.5 s, as the resync does.
    if (codec == VideoCodec::H265 && is_vcl && !pps_ok_) {
        if (now_ms() - idr_asked_ms_ >= 1500) {
            idr_asked_ms_ = now_ms();
            printf("ar8030: slices but no picture parameter set yet - asking for a keyframe\n");
            request_air_idr();
        }
        return;
    }
    // The sets in hand are not the decoder's: it takes them in front of a
    // keyframe, and only that. The resync can open on a keyframe's second
    // slice (its first came while the stream was still catching up), which
    // goes nowhere, and then no keyframe ever reached the decoder - every
    // slice after refused "PPS id out of range", the link up and no picture
    // until kestrel-gnd restarted (2026-10-22, 864p120). Until the decoder
    // has one, only keyframes pass, and one is asked for every 1.5 s.
    if (codec == VideoCodec::H265 && is_vcl && vdec && decode_enabled && !vdec->keyframe_in() &&
        !(nal_type >= 19 && nal_type <= 21)) {
        if (now_ms() - idr_asked_ms_ >= 1500) {
            idr_asked_ms_ = now_ms();
            printf("ar8030: the decoder has no keyframe to decode against - asking for one\n");
            request_air_idr();
        }
        return;
    }

    const bool is_param_set = (codec == VideoCodec::H265)
                                  ? (nal_type == 32 || nal_type == 33 || nal_type == 34)
                                  : (nal_type == 7 || nal_type == 8);

    // Validate a parameter set before anything takes it - the decoder too.
    // An RF bit-error can flip a slice NAL into something that *looks* like a
    // parameter set: we saw "41 1F ..." and "44 1F ..." land in hvcC, which
    // decode to nuh_layer_id 35 (the base layer must be 0) and made players
    // report "PPS id out of range" and refuse the file. MPP re-parses a PPS
    // over the one in use, so a bad one handed to it breaks every picture
    // after. Type alone is not enough - the rest of the header has to be sane.
    bool ps_valid = false;
    int ps_id = 0;
    if (is_param_set && len >= 6) {
        if (codec == VideoCodec::H265) {
            // 2-byte header: nuh_layer_id must be 0 and nuh_temporal_id_plus1 1,
            // which for a base-layer parameter set means byte 1 is exactly 0x01
            // and the low bit of byte 0 (the top layer_id bit) is clear. Then
            // the contents: an id in range, and for an SPS a real picture.
            ps_valid = ((nal[0] & 0x01) == 0) && (nal[1] == 0x01);
            if (ps_valid) {
                ps_id = hevc_param_set_id(nal_type, nal + 2, len - 2);
                ps_valid = ps_id >= 0;
            }
            // An SPS has to read on to its coding block sizes, and a PPS to
            // its end, naming an SPS that did.
            if (ps_valid && nal_type == 33) {
                ps_valid = hevc_sps_addr_bits(nal + 2, len - 2) > 0;
                if (ps_valid) sps_ids_ok_ |= (uint16_t)(1u << ps_id);
            } else if (ps_valid && nal_type == 34) {
                int sid = -1;
                ps_valid = hevc_pps_ok(nal + 2, len - 2, &sid) && (sps_ids_ok_ >> sid & 1);
                if (ps_valid) pps_ok_ = true;
            }
        } else {
            ps_valid = ((nal[0] & 0x60) != 0);   // nal_ref_idc != 0
        }
    }

    // When the bytes that completed this NAL came off USB: on ar_libre the
    // kernel's stamp for the read that brought them, free of how long this
    // thread took to be scheduled; otherwise now.
    uint64_t recv_us = read_rx_us ? read_rx_us : now_us();

    // The air unit sends each picture as two slice NALs, its two halves. Feeding each slice to
    // MPP as its own packet/PTS corrupts frame + reference assembly (partial
    // "top-only" pictures and an rkvdec timeout/reset storm). Accumulate all the
    // slices of one picture and feed them as a single access unit with one PTS.
    if (vdec && decode_enabled) {
        if (is_vcl) {
            // Which picture the air unit says this slice is (-1: no header).
            const int hseq = slice_hseq_;
            slice_hseq_ = -1;
            const bool marked_last = slice_last_;
            slice_last_ = false;
            // first_slice_segment_in_pic_flag is the first bit after the HEVC
            // 2-byte NAL header; a 1 marks the start of a new picture.
            uint8_t first_slice = (codec == VideoCodec::H265)
                ? ((len >= 3) ? ((nal[2] >> 7) & 1) : 1)
                : 1;  // H.264 (unused for AR8030): one slice == one picture
            const int addr = (codec == VideoCodec::H265 && len >= 3)
                ? hevc_slice_address(nal_type, nal + 2, len - 2, ctb_addr_bits, pps_dep_slices)
                : -1;
            if (ltrace::on())
                ltrace::rec(ltrace::kSlice, recv_us, (uint32_t)addr,
                            (uint64_t)len | ((uint64_t)first_slice << 32) |
                            ((uint64_t)nal_type << 40));
            if (!first_slice && au_open &&
                (nal_type != au_nal_type || (addr >= 0 && addr <= au_max_addr) ||
                 (hseq >= 0 && au_hseq_ >= 0 && hseq != au_hseq_))) {
                // Not this picture's: another picture's slice (one whose own
                // first slice was lost, or a stale one without a readable
                // header), or one of this picture's again. Appended, MPP
                // decodes two pictures as one ("POC change between slices")
                // and the hardware times out on the result.
                if (++foreign_slices <= 3 || (foreign_slices % 1000) == 0)
                    printf("ar8030: dropped a slice that is not the picture's being built "
                           "(type %d/%d, CTB %d after %d, picture %d/%d; %llu so far)\n",
                           nal_type, au_nal_type, addr, au_max_addr, hseq, au_hseq_,
                           foreign_slices);
                return;
            }
            if (first_slice && au_open) {
                // Completed by the next picture's arrival: the fallback, and
                // how the last slice's start address gets learned.
                learn_last_slice(au_max_addr, au_slice_count);
                flush_access_unit(recv_us);
            }
            if (!first_slice && !au_open) {
                // A slice for a picture already handed to the decoder early.
                // Mostly the air unit's stray fragments (28 bytes, reserved
                // types - they used to be appended to the next picture); a
                // run of them means the learned end was wrong, so relearn.
                late_slice();
                return;
            }
            if (first_slice) {
                // Measured by the radio's clock when there is a capture for
                // this picture, the floor estimate otherwise.
                au_measured_ = pic_cap_us_ && recv_us > pic_cap_us_ &&
                               recv_us - pic_cap_us_ < 1000000;
                au_air_unknown_ = false;
                if (au_measured_) {
                    au_air_us = (uint32_t)(recv_us - pic_cap_us_);
                    au_enc_us_ = std::min(std::min(pic_enc_us_, au_air_us), 0xffffu);
                    const float ms = (float)au_air_us / 1000.0f;
                    air_delay_ms = (air_delay_ms < 0.0f) ? ms : 0.95f * air_delay_ms + 0.05f * ms;
                    if (osd) osd->set_air_delay(air_delay_ms);
                    air_known_ms_ = now_ms();
                } else if (pic_air_unknown_) {
                    // The radio clock not checked yet: no air figure at all (the
                    // HUD shows the ground's part only after a second of these).
                    au_air_us = 0;
                    au_enc_us_ = 0;
                    au_air_unknown_ = true;
                    if (osd && now_ms() - air_known_ms_ > 1000) osd->clear_air_delay();
                } else {
                    au_air_us = air_delay_for(recv_us);
                    au_enc_us_ = 0;
                }
                // Its slices, each timed on its own - only with the capture
                // measured by the radio's clock and the slices' rows known.
                au_times_.reset();
                // A --debug-replay capture from before the radio clock: its
                // arrival stands in for the capture, so the slices still have
                // rows (what a picture in halves is cut at) on the bench.
                const bool bench = !au_measured_ && !replay_path.empty();
                if (((au_measured_ && pic_cap_us_) || bench) && ctb_size_ && ctbs_per_row_ && hdr_height_) {
                    au_times_ = std::make_shared<SliceTimes>();
                    au_times_->height = hdr_height_;
                    au_times_->frame_cap_us = bench ? recv_us : pic_cap_us_;
                    au_times_->readout_us = air_readout_us_.load();
                    au_times_->exposure_us = air_exposure_us_.load();
                    au_times_->key = nal_type >= 16 && nal_type <= 21;
                    au_times_->pts = (int64_t)frame_pts;
                }
                // The air unit's capture stamp of this picture (its header
                // comes before its slices): its clock, not ours - only the
                // differences are used (the recording's sample durations).
                au_cap_us_ = cap_valid ? cap_us64 : 0;
                pic_cap_us_ = 0;   // one picture, one capture
                pic_air_unknown_ = false;
                au_pts = frame_pts;
                au_nal_type = nal_type;   // picture key-ness (IDR vs trailing)
                au_first_recv_us = recv_us;  // zero point for this picture
                au_max_addr = 0;
                au_hseq_ = hseq;
                au_slice_count = 0;
                au_streamed = false;
                au_stream_done = false;
                frames_seen++;            // count real pictures, not slices
                if (++late_window_pics >= 64) { late_window_pics = 0; late_slices = 0; }
            }
            au_slices.insert(au_slices.end(), unit.begin(), unit.end());
            note_slice(addr, first_slice, recv_us);
            au_last_recv_us = recv_us;   // newest slice of this picture
            au_open = true;
            if (addr > au_max_addr) au_max_addr = addr;
            au_slice_count++;
            const bool last = marked_last || (last_slice_addr > 0 && addr == last_slice_addr);
            // Streamed: the decoder starts on the first slice while the rest is
            // still on the air - the air unit sends the second half about half
            // a frame after the first - and finishes about a millisecond after
            // the last lands rather than a whole decode later.
            // touch /tmp/kestrel-stream-off: whole pictures again, without a
            // restart (an air unit often stops sending video when kestrel restarts).
            static bool stream_paused = false;
            static unsigned pause_check = 0;
            if (first_slice && (pause_check++ % 50) == 0) {
                const bool off = access("/tmp/kestrel-stream-off", F_OK) == 0 ||
                                 g_frame_mode.load(std::memory_order_relaxed) == kFrameWholeDecode;
                if (off != stream_paused)
                    printf("ar8030: stream decode %s (frame mode %s, /tmp/kestrel-stream-off)\n",
                           off ? "paused" : "resumed", frame_mode_label(g_frame_mode.load()));
                stream_paused = off;
            }
            if (first_slice) {
                // Keyframes go whole: they carry the parameter sets the decoder
                // needs to start on (see keyframe_in), and with intra refresh
                // come only when asked for - not worth a half-built start.
                if (stream_decode && !stream_paused && Vdec::stream_holds().load() == 0 &&
                    !last && last_slice_addr > 0 && !(nal_type >= 19 && nal_type <= 21) &&
                    last_slice_count > 1 && vdec->stream_supported()) {
                    if (!stream_pics++)
                        printf("ar8030: decoding each picture from its first slice "
                               "(%d slices a picture, stream mode)\n", last_slice_count);
                    feed_au(unit.data(), unit.size(), last_slice_count);
                    au_streamed = true;
                }
            } else if (au_streamed && !au_stream_done) {
                // Held to the end of this read (send_held_slices); the last
                // slice goes at once, with any held before it.
                if (held_pts_ != au_pts) { held_slices_.clear(); held_pts_ = au_pts; }
                held_slices_.insert(held_slices_.end(), unit.begin(), unit.end());
                if (last) send_held_slices(true);
            }
            // The picture's last slice: decode it now, not when the next
            // picture's first slice turns up - a frame interval later (10 ms
            // at 100 fps), which is how long every complete picture used to
            // wait.
            if (last)
                flush_access_unit(recv_us);
        } else if (!is_param_set || ps_valid) {
            // VPS/SPS/PPS are cached and SEI dropped inside the decoder; the
            // cached param sets get prepended to the next IDR automatically.
            vdec->feed_packet_to_decoder(unit.data(), (int)unit.size(), frame_pts,
                                         recv_us, nal_type, 0, 0);
        }
    }

    // Keep our own copy of the parameter sets for the DVR. The decoder's cache
    // is private to it, and a recording started mid-stream needs VPS/SPS/PPS
    // ahead of its first IDR or the muxer emits an empty hvcC and the file
    // plays black. H.265: VPS 32, SPS 33, PPS 34. H.264: SPS 7, PPS 8.
    // Only valid sets (ps_valid, above), and exactly one of each type is
    // kept, so hvcC can never accumulate copies.
    if (is_param_set) {
        if (ps_valid) ps_since_pic_ = true;
        if (ps_valid) {
            // One set per (type, id). This stream uses more than one PPS, so a
            // single set per type left slices unable to resolve theirs
            // ("PPS changed between slices"); keyed by id, every PPS is kept,
            // and a newer set with an id already held - a camera mode change
            // sends a new SPS under the same id - replaces the old one rather
            // than sitting next to it in hvcC.
            auto key_of = [&](const std::vector<uint8_t>& q) {
                // q is Annex-B: 4-byte start code, 2-byte header, payload.
                const int t = (codec == VideoCodec::H265) ? ((q[4] >> 1) & 0x3F) : (q[4] & 0x1F);
                const int id = (codec == VideoCodec::H265)
                                   ? hevc_param_set_id(t, q.data() + 6, q.size() - 6) : 0;
                return std::make_pair(t, id);
            };
            const auto key = std::make_pair((int)nal_type, ps_id);
            bool known = false, changed = false;
            for (auto& q : dvr_param_sets) {
                const bool same = q.size() == unit.size() &&
                                  memcmp(q.data(), unit.data(), unit.size()) == 0;
                if (same) { known = true; break; }
                if (codec == VideoCodec::H265 && key_of(q) == key) {
                    q = unit; known = changed = true; break;
                }
            }
            if (!known) {
                if (dvr_param_sets.size() >= 16) dvr_param_sets.erase(dvr_param_sets.begin());
                dvr_param_sets.push_back(unit);
                changed = true;
            }
            // What reading a slice's start address takes (hevc_slice_address).
            if (codec == VideoCodec::H265 && nal_type == 33) {
                uint32_t ctb = 0, w = 0;
                const int bits = hevc_sps_addr_bits(nal + 2, len - 2, &ctb, &w);
                if (bits > 0 && ctb) {
                    ctb_size_ = ctb;
                    ctbs_per_row_ = (w + ctb - 1) / ctb;
                }
                if (bits != ctb_addr_bits) {
                    printf("ar8030: slice addresses are %d bits in this picture size\n", bits);
                    ctb_addr_bits = bits;
                    last_slice_addr = -1;              // a new picture size: learn again
                    last_addr_cand = -1; last_addr_streak = 0; last_count_cand = 0; last_slice_count = 0;
                }
            } else if (codec == VideoCodec::H265 && nal_type == 34) {
                int id = -1;
                const int dep = hevc_pps_dependent_slices(nal + 2, len - 2, &id);
                if (dep >= 0) pps_dep_slices[id] = (uint8_t)dep;
            }
            if (changed) {
                DvrRecorder::instance().set_parameter_sets(dvr_param_sets);
                if (codec == VideoCodec::H265) dvr_repair_.set_parameter_sets(dvr_param_sets);
                printf("ar8030: cached param set type=%d id=%d len=%zu (%zu total)\n",
                       nal_type, ps_id, unit.size(), dvr_param_sets.size());
            }
        } else {
            ps_dropped++;
            if (ps_dropped <= 5 || (ps_dropped % 100) == 0)
                printf("ar8030: dropped corrupt param-set NAL type=%d hdr=%02X %02X "
                       "(total %llu)\n", nal_type, nal[0], len > 1 ? nal[1] : 0,
                       (unsigned long long)ps_dropped);
        }
    }

    // Unconditional: `dvr` is the legacy --dvr shared_ptr and is always null
    // now that recording is driven by the REC button, so guarding on it here
    // meant DvrRecorder never received a single frame - the file was created
    // and the on-screen timer ran while nothing was written. DvrRecorder::feed()
    // drops frames cheaply when idle, so no guard is needed.
    //
    // Parameter sets are deliberately NOT forwarded: DvrRecorder replays the
    // cached ones once when recording starts. The air unit re-sends VPS/SPS/PPS
    // periodically, and feeding minimp4 the same set twice makes it append a
    // second, malformed entry to the hvcC arrays while leaving numNalus stale -
    // ffmpeg then rejects the file with "Invalid NAL unit size in extradata"
    // and QuickTime refuses it outright.
    // VCL NALs are accumulated and handed over as one access unit in
    // flush_access_unit(). Parameter sets are replayed by DvrRecorder at record
    // start. Anything else (SEI and friends) is left out: handed over on its
    // own it became an MP4 sample of its own - an empty "frame" with a frame's
    // duration between every two pictures - and nothing in it is needed to
    // play the file.

    update_stats(unit.size());

    nal_count++;
}

// Is the picture this header opens one from the past? Pictures are captured in
// order, so a live one is always newer than the newest seen - by a frame
// interval, or by the length of an outage. One that is not is a replay of the
// air unit's encoder ring (see live_cap_), or a new clock after the air unit
// restarted; the second keeps running forward, uninterrupted by any live
// picture, and after kStaleAdopt of those it is taken as the live stream.
bool Ar8030Source::stale_picture(uint32_t cap32) {
    // Differences in wrapping 32-bit microseconds: good for 35 minutes either way.
    if (!live_cap_valid_ || (int32_t)(cap32 - live_cap_) > 0) {
        live_cap_ = cap32;
        live_cap_valid_ = true;
        stale_run_ = 0;
        return false;
    }
    const int32_t step = (int32_t)(cap32 - stale_prev_cap_);
    stale_run_ = (stale_run_ > 0 && step > 0 && step < 1000000) ? stale_run_ + 1 : 1;
    stale_prev_cap_ = cap32;
    if (stale_run_ >= kStaleAdopt) {
        printf("ar8030: the air unit's capture clock went back %.1f s and stayed - "
               "a restart; following it\n", (double)(int32_t)(live_cap_ - cap32) / 1e6);
        live_cap_ = cap32;
        stale_run_ = 0;
        return false;
    }
    if (++stale_pics <= 3 || (stale_pics % 1000) == 0)
        printf("ar8030: dropped a picture captured %.3f s before the newest one "
               "(%llu so far)\n", (double)(int32_t)(live_cap_ - cap32) / 1e6, stale_pics);
    return true;
}

// Where a picture's last slice starts, learned from pictures completed by the
// next one's arrival: the same start address as the last slice of kLearnRun
// pictures in a row. The air unit cuts every picture the same way - two
// halves, the second at CTB 255 of 510 at 1080p, IDR or not - so this settles
// within a second of video; a new picture size (SPS) starts it over.
void Ar8030Source::learn_last_slice(int max_addr, int slices) {
    if (last_slice_addr > 0 || max_addr <= 0) return;
    // Only a larger end starts over: a picture that ends early (a slice lost or
    // unreadable) says nothing about where pictures end. With intra refresh a
    // GOP start every 17 pictures did that, the run never reached kLearnRun, and
    // every picture waited for the next one to be decoded - a frame of latency
    // (2026-10-05). The slice count is learned with it (streamed pictures need
    // it): the same end with more slices starts over too, with fewer it is a
    // picture that lost one.
    if (max_addr == last_addr_cand && slices == last_count_cand) last_addr_streak++;
    else if (max_addr > last_addr_cand || (max_addr == last_addr_cand && slices > last_count_cand)) {
        last_addr_cand = max_addr; last_count_cand = slices; last_addr_streak = 1;
    } else {
        static unsigned shorter = 0;
        if (++shorter <= 5 || shorter % 1000 == 0)
            printf("ar8030: a picture ended at CTB %d with %d slices, short of %d with %d - "
                   "not counted (%u so far)\n", max_addr, slices, last_addr_cand,
                   last_count_cand, shorter);
        return;
    }
    if (last_addr_streak >= kLearnRun) {
        last_slice_addr = max_addr;
        last_slice_count = slices;
        printf("ar8030: pictures end with the slice at CTB %d - decoding each as that "
               "slice arrives\n", last_slice_addr);
    }
}

// A slice after its picture was flushed early. The air unit's stray fragments
// do this about once in 600 pictures; more than a few in 64 pictures means
// pictures do not end where learned (a mode change), so stop flushing early
// and learn again.
void Ar8030Source::late_slice() {
    late_slices_total++;
    if (++late_slices > kLateMax && last_slice_addr > 0) {
        printf("ar8030: %u slices after their picture ended in %u pictures - relearning "
               "where pictures end\n", late_slices, late_window_pics);
        last_slice_addr = -1;
        last_addr_cand = -1; last_addr_streak = 0; last_count_cand = 0; last_slice_count = 0;
        late_slices = 0;
    }
}

// Hand the decoder a picture, or with stream_slices > 1 the first slice of one
// that many slices long (the rest by Vdec::stream_append), stamped as the
// picture open in au_*.
void Ar8030Source::feed_au(const uint8_t *data, size_t len, int stream_slices) {
    // This AU's OWN first-slice arrival, not the arrival of the read that
    // happens to be flushing it - see au_first_recv_us.
    const uint64_t au_recv = au_first_recv_us ? au_first_recv_us : now_us();
    if (ltrace::on()) ltrace::rec(ltrace::kAu, now_us(), (uint32_t)au_pts, au_recv);
    if (ltrace::on() && au_measured_ && au_air_us && au_recv > au_air_us)
        ltrace::rec(ltrace::kCap, au_recv, (uint32_t)au_pts, au_recv - au_air_us);
    // The air delay, capture stamp -> first slice here, in two parts.
    // Measured by the radio's clock: capture -> the encoder handing the
    // first slice out (the HUD's Enc), and from there to here (RF). By
    // the floor estimate (see air_delay_for) it is all in the second.
    // No exposure wait is added: an LED switched on in front of the
    // camera is in the frame whose capture stamp comes 0 ms later on the
    // median (-5..+5 ms, photodiode rig with the air unit and goggle
    // clocks synced), so the stamp already stands for the moment of
    // capture. The screen's own delay is not counted: the figure ends at
    // the vblank the picture goes out on.
    const uint32_t enc_us = au_air_us ? au_enc_us_ : 0;
    const uint32_t proc_us = au_air_unknown_ ? kAirUnknown : au_air_us - au_enc_us_;
    vdec->set_next_slice_times(au_times_);
    if (stream_slices > 1)
        vdec->stream_start((void *)data, (int)len, au_pts, au_recv, au_nal_type,
                           enc_us, proc_us, stream_slices);
    else
        vdec->feed_packet_to_decoder((void *)data, (int)len, au_pts, au_recv, au_nal_type,
                                     enc_us, proc_us);
}

// Drop the access unit being built. One already streamed to the decoder is
// ended there - not left waiting for the kernel's deadline with the decoder
// held.
void Ar8030Source::drop_au() {
    if (au_streamed && !au_stream_done && vdec) vdec->stream_end(au_pts);
    au_streamed = au_stream_done = false;
    au_slices.clear();
    // The next picture gets a pts of its own, as after a flush: with this
    // one's, MPP refuses its streamed parts as late and the renderer takes
    // it for the picture dropped here.
    if (au_open) frame_pts++;
    au_open = false;
}

// A slice of the open access unit, for its own latency (utils/slice_times.hpp):
// its first row, when that row was captured, when the encoder handed the slice
// out (its header, just before it) and when it got here. The capture of a row
// is the picture's capture stamp (row 0) plus the time the sensor takes to
// read down to it, which is learned here: a slice leaves the encoder a fixed
// tail after its last row is read, so the first and last slices' encoder-out
// times are apart by the readout of the rows between their ends.
void Ar8030Source::note_slice(int addr, bool first_slice, uint64_t recv_us) {
    const uint64_t out = hdr_out_us_;
    hdr_out_us_ = 0;
    const bool gop_start = first_slice && ps_since_pic_;   // parameter sets came just before it
    if (first_slice) ps_since_pic_ = false;
    SliceTimes *st = au_times_.get();
    if (!st || addr < 0 || (!first_slice && addr == 0)) return;
    const int i = st->n.load();
    if (i >= SliceTimes::kMax) return;
    const uint32_t row = (uint32_t)(addr / (int)ctbs_per_row_) * ctb_size_;
    if (row >= st->height) return;
    if (!readout_us_ && hdr_fps_) readout_us_ = 900000u / hdr_fps_;   // until learned: 90% of a frame
    if (first_slice) {
        st->expected = last_slice_count;
        uint32_t end = predict_first_end(au_hseq_);
        // Parameter sets just before it: a GOP start, where kestrel-air's
        // intra refresh restarts and cuts the first slice short. Taking the
        // shorter end is safe either way (fewer rows shown early).
        if (gop_start && cut_row_ && end && cut_row_ < end) end = cut_row_;
        st->split_row.store(end);
    }
    if (i == 1 && ctbs_per_row_ && last_slice_addr > 0) {
        // The first slice's end, as it turned out: learn the cut pictures.
        const uint32_t normal = (uint32_t)(last_slice_addr / (int)ctbs_per_row_) * ctb_size_;
        pics_since_cut_++;
        if (row != normal && au_hseq_ >= 0) {
            if (cut_last_hseq_ >= 0) {
                const int d = (au_hseq_ - cut_last_hseq_ + 65536) % 65536;
                if (d == cut_period_) cut_confirms_++;
                else { cut_period_ = d; cut_confirms_ = 0; }
            }
            cut_last_hseq_ = au_hseq_;
            cut_row_ = row;
            pics_since_cut_ = 0;
        }
    }
    SliceTimes::Slice &s = st->s[i];
    s.row = row;
    const uint32_t ro = st->readout_us ? st->readout_us : readout_us_;   // the air unit's, else learned
    s.cap_us = st->frame_cap_us + (uint64_t)ro * row / st->height;
    s.out_us = out;
    s.here_us = recv_us;
    st->n.store(i + 1);
    const bool last = last_slice_addr > 0 && addr == last_slice_addr;
    if (last) st->complete.store(true);
    if (last && i >= 1 && st->s[0].out_us && out > st->s[0].out_us) {
        const uint32_t span = st->height - st->s[1].row;      // last end - first end, in rows
        if (span > st->height / 8) {
            const uint64_t r = (out - st->s[0].out_us) * st->height / span;
            if (r >= 2000 && r <= 40000) {
                readout_win_.push_back((uint32_t)r);
                if (readout_win_.size() >= 64) {
                    std::vector<uint32_t> v = readout_win_;
                    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
                    readout_us_ = v[v.size() / 2];
                    readout_win_.clear();
                }
            }
        }
    }
}

// Where picture hseq's first slice should end, row; 0 if that cannot be said
// with confidence (then it is not shown in halves before its second slice).
uint32_t Ar8030Source::predict_first_end(int hseq) const {
    if (!ctbs_per_row_ || last_slice_addr <= 0 || hseq < 0) return 0;
    // Two slices only (1080p above 60 fps): with four (60 fps) the first one
    // ends a quarter down and that is not learned yet.
    if (last_slice_count != 2) return 0;
    const uint32_t normal = (uint32_t)(last_slice_addr / (int)ctbs_per_row_) * ctb_size_;
    // No cut for a long while (none at all yet, or the refresh is off now):
    // every picture is cut at the usual place - after enough of them to say so.
    if (cut_last_hseq_ < 0) return pics_since_cut_ > 100 ? normal : 0;
    if (pics_since_cut_ > 400) return normal;
    if (cut_period_ <= 0 || cut_confirms_ < 1) return 0;     // cuts seen, their rhythm not yet
    const int d = (hseq - cut_last_hseq_ + 65536) % 65536;
    return d % cut_period_ == 0 ? cut_row_ : normal;
}

// Feed the accumulated slices of one picture as a single access unit / PTS.
// A streamed one is in the decoder already: if its last slice never came, it
// is told there is no more.
void Ar8030Source::flush_access_unit(uint64_t recv_us) {
    if (!au_open) return;
    if (au_times_) au_times_->complete.store(true);   // no more slices for it
    if (!au_slices.empty() && vdec) {
        const uint64_t au_recv = au_first_recv_us ? au_first_recv_us : recv_us;
        if (!au_streamed) {
            feed_au(au_slices.data(), au_slices.size(), 0);
        } else if (!au_stream_done) {
            send_held_slices();
            vdec->stream_end(au_pts);
            if (++stream_ended <= 3 || (stream_ended % 1000) == 0)
                printf("ar8030: picture %lld ended without its last slice "
                       "(%llu of %llu streamed pictures)\n", (long long)au_pts,
                       (unsigned long long)stream_ended, (unsigned long long)stream_pics);
        }
        au_streamed = au_stream_done = false;
        note_ap_picture(au_last_recv_us);
        frame_pts++;

        // Reassembly span: first slice of THIS picture on the wire to its
        // last. Must use au_last_recv_us, not the flush-triggering read -
        // that read is the NEXT picture's first slice, so it would measure
        // the inter-frame interval (~10ms at 100fps) instead.
        //
        // Expect 0.0 much of the time: recv_us is stamped per socket read,
        // and a whole picture usually arrives inside one read, so all its
        // slices share a timestamp. Non-zero means the picture straddled
        // reads - which is exactly the case worth seeing, since that is when
        // waiting for a complete picture actually costs latency.
        if (prof::enabled() && au_last_recv_us >= au_recv) {
            // KESTREL_PROF: what the reassembly stage is made of - the span from a picture's
            // first slice to its last, against the picture's size, for key and other pictures.
            static std::vector<float> span[2];
            static double bytes[2] = {0, 0};
            static uint64_t last_print = 0;
            const bool key = (codec == VideoCodec::H265) ? (au_nal_type >= 19 && au_nal_type <= 21) : (au_nal_type == 5);
            span[key ? 1 : 0].push_back((float)(au_last_recv_us - au_recv) / 1000.0f);
            bytes[key ? 1 : 0] += (double)au_slices.size();
            const uint64_t tnow = now_us();
            if (tnow - last_print > 2000000) {
                if (last_print) {
                    for (int kk = 0; kk < 2; kk++) {
                        auto& s = span[kk];
                        if (s.empty()) continue;
                        std::sort(s.begin(), s.end());
                        int zero = 0;
                        for (float x : s) if (x < 0.05f) zero++;
                        printf("PROF AU %s: n=%zu span ms p50 %.1f p90 %.1f max %.1f, same-read %d, avg %.1f kB\n",
                               kk ? "key  " : "other", s.size(), s[s.size() / 2], s[s.size() * 9 / 10], s.back(), zero,
                               bytes[kk] / (double)s.size() / 1000.0);
                    }
                    fflush(stdout);
                }
                last_print = tnow;
                span[0].clear(); span[1].clear(); bytes[0] = bytes[1] = 0;
            }
        }
        if (osd && au_last_recv_us >= au_recv) {
            float rsm_ms = (float)(au_last_recv_us - au_recv) / 1000.0f;
            if (rsm_ms < 200.0f) {   // ignore absurd outliers
                static float rsm_ewma = -1.0f;
                rsm_ewma = (rsm_ewma < 0.0f) ? rsm_ms
                                             : 0.9f * rsm_ewma + 0.1f * rsm_ms;
                osd->set_reassembly_latency(rsm_ewma);
            }
        }
    }
    // The DVR gets the same complete access unit. Feeding it slice by slice
    // made minimp4 emit one MP4 sample per slice, so a player saw a "frame"
    // holding a single slice and dropped the rest with "PPS changed between
    // slices" - decoding only the top quarter of the picture and painting the
    // remainder green. One picture must be one sample.
    //
    if (!au_slices.empty()) update_dvr(std::make_shared<std::vector<uint8_t>>(au_slices), au_cap_us_);
    au_slices.clear();
    au_open = false;
}

// --debug-replay: the video stream from a file in place of the baseband's
// socket - the same splitter, picture assembly, decoder, DVR and web feeds,
// with no air unit and no link. For finding out what a damaged stream does to
// them (tools/ar8030_fuzz.py makes one from any H.265 file). Read in the air
// unit's ~4K bursts at about 11 Mbit/s, round and round until stopped, with
// two seconds of nothing between passes: the air app stopping and starting
// its stream again, which is how a restart looks from here.
void Ar8030Source::run_replay() {
    FILE* f = fopen(replay_path.c_str(), "rb");
    if (!f) {
        printf("ar8030: replay: cannot open %s\n", replay_path.c_str());
        return;
    }
    printf("ar8030: replaying %s in place of the baseband\n", replay_path.c_str());
    std::vector<uint8_t> buf(4096);
    unsigned passes = 0;
    bool read_any = false;
    while (!*should_stop) {
        const size_t n = fread(buf.data(), 1, buf.size(), f);
        if (n == 0) {
            if (!read_any) break;              // empty file
            rewind(f);
            printf("ar8030: replay: pass %u done\n", ++passes);
            for (int i = 0; i < 20 && !*should_stop; i++) usleep(100000);
            continue;
        }
        read_any = true;
        total_bytes += n;
        last_data_ms = now_ms();
        consume(buf.data(), n);
        usleep(3000);
    }
    fclose(f);
    printf("ar8030: replay stopped (%llu frames)\n", (unsigned long long)frames_seen);
}

// Split an Annex-B byte stream into NALs. Handles NALs straddling reads by
// keeping the tail in `accum` until the next start code arrives.
//
// That alone holds every packet's last NAL until the next packet: the first
// slice of a picture waited for the second (~5 ms at 1080p100, the encoder's
// gap between them), and the last for whatever came next. The air unit's own
// framing says where a packet ends - its header (first byte 0x80) gives the
// slice's length (bytes 4..7, with any SEI after it) and the extras' (byte
// 31), and the packet closes with ee 29 55 9f - so once that trailer is in,
// the packet's last NAL goes on at once, with the same bytes the next start
// code would have given it. A header that does not add up, or a trailer that
// is not there, leaves it to the start code.
// One read of the radio's video: its NALs, then the slices held from it.
void Ar8030Source::consume(const uint8_t* data, size_t len) {
    consume_bytes(data, len);
    send_held_slices();
}

// Two appends of the same picture within microseconds (its middle slices in
// one read, the last one later) made the RK3568 end the picture before its
// last slice now and then (11 of 78 such pictures at 1080p60): that slice was
// refused and the picture's lower part stayed wrong until the intra refresh
// passed. So the slices of one read go in one append - as a whole picture
// goes, all its slices in one part - and the last slice takes the held ones
// with it.
void Ar8030Source::send_held_slices(bool last) {
    if (held_slices_.empty() && !last) return;
    if (held_pts_ == au_pts && au_streamed && !au_stream_done && vdec) {
        if (!vdec->stream_append(held_slices_.data(), (int)held_slices_.size(), au_pts, last) &&
            (++stream_refused <= 3 || (stream_refused % 1000) == 0))
            printf("ar8030: the decoder did not take a slice of picture %lld "
                   "(%llu so far)\n", (long long)au_pts,
                   (unsigned long long)stream_refused);
        au_stream_done = last;
    }
    held_slices_.clear();
    held_pts_ = -1;
}

// The slice checksum of the packet whose header came last, once its payload
// is all in accum: XOR of its little-endian u32 words, then of the bytes left
// over (kestrel-air's ka_xor32). A mismatch is a slice the radio delivered
// damaged but whole - nothing else counts those: the decoder takes it and
// draws garbage blocks until intra refresh repairs them.
void Ar8030Source::check_slice_xor() {
    if (!xor_pending_) return;
    if (xor_start_ < accum_base_) { xor_pending_ = false; xor_unchecked_++; return; }
    const uint64_t end = xor_start_ + xor_len_;
    if (end > accum_base_ + accum.size()) return;          // not all here yet
    const uint8_t *b = accum.data() + (size_t)(xor_start_ - accum_base_);
    uint32_t x = 0;
    for (uint32_t i = 0; i + 4 <= xor_len_; i += 4)
        x ^= (uint32_t)b[i] | (uint32_t)b[i + 1] << 8 | (uint32_t)b[i + 2] << 16 | (uint32_t)b[i + 3] << 24;
    for (uint32_t i = xor_len_ & ~3u; i < xor_len_; i++) x ^= b[i];
    xor_pending_ = false;
    xor_checked_++;
    if (x != xor_want_) {
        xor_bad_++;
        if (xor_bad_ <= 5 || xor_bad_ % 100 == 0)
            printf("ar8030: a slice arrived damaged: checksum %08x, its packet says %08x (%u bytes; %llu of %llu)\n",
                   x, xor_want_, xor_len_, (unsigned long long)xor_bad_, (unsigned long long)xor_checked_);
    }
}

void Ar8030Source::consume_bytes(const uint8_t* data, size_t len) {
    accum.insert(accum.end(), data, data + len);

    if (accum.size() > AR_MAX_ACCUM) {
        printf("ar8030: no start code in %zu bytes - wrong port or non-AnnexB stream?\n",
               accum.size());
        accum_base_ += accum.size();
        pkt_end_pos_ = 0;
        accum.clear();
        return;
    }

    size_t i = 0, n = accum.size();
    size_t nal_start = SIZE_MAX;   // offset of current NAL payload

    while (i + 3 < n) {
        bool sc4 = (accum[i] == 0 && accum[i+1] == 0 && accum[i+2] == 0 && accum[i+3] == 1);
        bool sc3 = (accum[i] == 0 && accum[i+1] == 0 && accum[i+2] == 1);
        if (sc4 || sc3) {
            size_t sc_len = sc4 ? 4 : 3;
            if (nal_start != SIZE_MAX) {
                // NAL runs from nal_start up to this start code.
                emit_nal(accum.data() + nal_start, i - nal_start);
            }
            nal_start = i + sc_len;
            i += sc_len;
            if (nal_start >= n) break;         // the start code ends the buffer: its NAL comes next time
            if (accum[nal_start] != 0x80) continue;
            // A packet header is not Annex-B: its bytes can hold a start code.
            // The capture time's high bytes (18..25) are 01 00 00 00 or zeros,
            // so a radio clock (26..29) ending in 01 makes 00 00 00 01 - about
            // one header in 256. Cut there, the header's tail read as a header
            // of its own now and then (a mode of its own, a capture stamp from
            // nowhere: 16 pictures dropped as stale). So a header is skipped
            // whole, with its extras, once its checksum (byte 1, the sum of
            // bytes 2..41) says it is one; until it is all here, nothing past
            // its start is scanned.
            if (nal_start + 42 > n) break;
            const uint8_t *h = accum.data() + nal_start;
            uint8_t sum = 0;
            for (int k = 2; k < 42; k++) sum += h[k];
            const uint32_t ext = h[31];
            if (sum == h[1] && ext <= 64) {
                if (nal_start + 42 + ext > n) break;
                i = nal_start + 42 + ext;
                // The packet before it is all here by now: its checksum, then
                // this one's (06 02 <XOR32> in the extras), to check later.
                check_slice_xor();
                if (xor_pending_) { xor_pending_ = false; xor_unchecked_++; }
                const uint32_t pl = (uint32_t)h[4] | ((uint32_t)h[5] << 8) |
                                    ((uint32_t)h[6] << 16) | ((uint32_t)h[7] << 24);
                for (uint32_t e = 0; e + 6 <= ext && pl > 0 && pl < AR_MAX_ACCUM;) {
                    const uint8_t *t = h + 42 + e;
                    if (t[0] < 2) break;
                    if (t[0] == 6 && t[1] == 0x02) {
                        xor_want_ = (uint32_t)t[2] | (uint32_t)t[3] << 8 | (uint32_t)t[4] << 16 | (uint32_t)t[5] << 24;
                        xor_start_ = accum_base_ + nal_start + 42 + ext;
                        xor_len_ = pl;
                        xor_pending_ = true;
                        break;
                    }
                    e += t[0];
                }
            }
            // A packet header, whole: where its packet ends.
            const uint32_t plen = (uint32_t)h[4] | ((uint32_t)h[5] << 8) |
                                  ((uint32_t)h[6] << 16) | ((uint32_t)h[7] << 24);
            pkt_end_pos_ = (plen > 0 && plen < AR_MAX_ACCUM && ext <= 64)
                ? accum_base_ + nal_start + 42 + ext + plen + 4 : 0;
            continue;
        }
        i++;
    }

    // The open NAL is its packet's last and the packet is all here: on with it.
    if (nal_start != SIZE_MAX && pkt_end_pos_ > accum_base_ + nal_start &&
        pkt_end_pos_ <= accum_base_ + n) {
        const size_t pe = (size_t)(pkt_end_pos_ - accum_base_);
        if (accum[pe - 4] == 0xee && accum[pe - 3] == 0x29 && accum[pe - 2] == 0x55 &&
            accum[pe - 1] == 0x9f) {
            emit_nal(accum.data() + nal_start, pe - nal_start);
            check_slice_xor();
            accum.erase(accum.begin(), accum.begin() + pe);
            accum_base_ += pe;
            pkt_end_pos_ = 0;
            return;
        }
        pkt_end_pos_ = 0;
    }

    check_slice_xor();
    // Keep the unterminated tail (from the last start code we saw, or the last
    // 3 bytes which might be a partial start code).
    if (nal_start != SIZE_MAX) {
        size_t keep_from = nal_start - (nal_start >= 4 ? 4 : nal_start);
        accum.erase(accum.begin(), accum.begin() + keep_from);
        accum_base_ += keep_from;
    } else if (accum.size() > 3) {
        accum_base_ += accum.size() - 3;
        accum.erase(accum.begin(), accum.end() - 3);
    }
}

void Ar8030Source::run() {
    // stdout is block-buffered when redirected to a file; line-buffer it so
    // the connect/socket diagnostics below actually reach the log.
    setvbuf(stdout, nullptr, _IOLBF, 0);
    SchedulingHelper::set_thread_params_max_realtime("AR8030_RX", 20);
    printf("ar8030: source starting (host=%s:%d slot=%d port=%d) decode=%s\n",
           host.c_str(), host_port, slot, video_port,
           decode_enabled ? "ON" : "off (--ar8030-decode to enable)");
    if (!replay_path.empty()) { run_replay(); return; }

    std::vector<uint8_t> buf(AR_READ_CHUNK);

    while (!*should_stop) {
        bb_watchdog_alive();   // the loop is still going round
        bw_poll();
        if (sockfd < 0) {
            if (!connect_bb()) {
                disconnect_bb();
                if (*should_stop) break;
                if (++connect_fails == 1 || connect_fails % 10 == 0)
                    printf("ar8030: connect attempt %d failed, retrying\n", connect_fails);

                // Escalate. Retrying by itself never clears a daemon that has
                // stopped answering BB_INIT_REQ: measured, every attempt fails
                // the same way, because the fault is on its side - a previous
                // client died holding the owner slot and nothing hands it back.
                // Restarting the daemon is the only thing that does, which is
                // also why a bare re-exec of ourselves was never enough.
                //
                // Not on the first failure. At boot S60ar8030 backgrounds its
                // own bring-up, so an early attempt can fail simply because the
                // daemon is not up yet, and restarting it then would push the
                // thing we are waiting for further away.
                if (connect_fails >= kFailsBeforeDaemonRestart &&
                    (connect_fails % kFailsBeforeDaemonRestart) == 0) {
                    // On ar_libre this restarts the baseband itself: arlink.ko
                    // is unloaded and the radio power-cycled (bb_only.sh).
                    printf(on_ar_libre() ? "ar8030: %d bring-ups failed - restarting the baseband\n"
                                         : "ar8030: %d bring-ups failed - restarting the daemon\n",
                           connect_fails);
                    Settings::getInstance().flush();   // bb_only.sh reads it
                    if (system("/etc/init.d/S60ar8030 restart >/dev/null 2>&1") != 0)
                        printf("ar8030: S60ar8030 restart returned non-zero\n");
                    // It backgrounds its own bring-up: loading the driver and
                    // opening the device takes seconds, and asking too early
                    // just burns another failed attempt.
                    for (int i = 0; i < 100 && !*should_stop; i++) usleep(100000);
                }
                for (int i = 0; i < 10 && !*should_stop; i++) usleep(100000);
                continue;
            }
            connect_fails = 0;
        }

        // Keep the OSD repainting even with no video: its render is driven by
        // frame updates, so an unlinked air unit would otherwise leave the
        // last (empty) frame on screen forever. Pipeline does the same at 1Hz.
        drain_control_socket();
        drain_msp_socket();
        // (The channel scan is the stats thread's: a radio round trip here
        // would stall the video.)

        // A menu mode change parked by the OSD thread.
        int want = pending_mode_index.exchange(-1);
        if (want >= 0 && want < sky::kFpvModeCount) {
            const sky::VideoMode& m = sky::kFpvModes[want];
            if (m.w == sky_cfg.ch0_w && m.h == sky_cfg.ch0_h && m.fps == sky_cfg.ch0_fps) {
                // The mode the air unit runs: no switch (it restarts the encoder).
                printf("ar8030: menu mode %s - already this air unit's, nothing sent\n", m.name);
            } else {
                mode_w = m.w; mode_h = m.h; mode_fps = m.fps; mode_chn = 0;
                printf("ar8030: menu requested mode %s\n", m.name);
                apply_video_mode();
            }
        }
        apply_pending_settings();
        apply_pending_rf();
        // Binding, if the menu or the front-panel button asked for it. One
        // non-blocking step per pass; it owns the baseband only in short
        // bursts, so video and telemetry keep flowing while it runs.
        bind_step();

        // The radios associate on their own schedule, and after a restart of
        // either end that takes seconds. Everything sent before then - the
        // whole handshake, including the video configuration that makes the
        // air unit stream - goes out over a link with nobody on the far side.
        // That is why the acks time out on every start and why video appears
        // only when the air unit eventually decides to send some. Watch for
        // the link coming up and send it then, when there is something there
        // to hear it.
        {
            uint64_t tl = now_ms();
            {
                // The state comes from the stats thread's poll (400 ms on
                // Artosyn's library, 5 s on ar_libre) and, on ar_libre, from
                // the chip's link-state event the moment it changes. Reading it costs nothing, so every pass.
                int ls = link_state_.load();
                // (The HUD is told by the stats thread and the event handler.)
                if (ls >= 0 && ls != last_link_state) {
                    printf("ar8030: link state %d -> %d (%s)\n", last_link_state, ls,
                           ls == 2 ? "linked" : ls == 1 ? "connecting" : "not linked");
                    // The step to 40 MHz (kestrel-air's, after link-up) makes the
                    // radios re-form the link: down and back within about a
                    // second, the same air unit on the same stream. Taken as a
                    // new link it cost ~4.6 s of picture (resync gate, handshake)
                    // where the radio's own gap is ~0.1 s - and only when this
                    // loop happened to look while it was down. So a drop within
                    // 2 s of a bandwidth change that is back within 3 s keeps
                    // everything; only the picture cut in half goes.
                    if (last_link_state == 2 && ls != 2) {
                        const uint64_t bw = bw_change_ms_.load();
                        retune_down_ms_ = (bw && tl - bw <= 2000) ? tl : 0;
                        if (retune_down_ms_)
                            printf("ar8030: link re-forming after a bandwidth change\n");
                    }
                    const bool retune = ls == 2 && retune_down_ms_ && tl - retune_down_ms_ <= 3000;
                    if (ls == 2) retune_down_ms_ = 0;
                    // A channel pinned from the menu holds only while the link
                    // does. The air unit decides the channel - after a restart
                    // it is back on its own - and a ground left fixed on the
                    // pinned one never finds it: video stayed gone ~100 s, until
                    // the idle reconnect below happened to reset the mode. So a
                    // lost link puts the ground back to searching, as at start.
                    // --ar8030-chan-manual asked to stay fixed, and does.
                    if (last_link_state == 2 && ls != 2 && !retune_down_ms_ && !chan_auto && !chan_manual_cli && bb_dev) {
                        printf("ar8030: link lost on a pinned channel - searching again\n");
                        search_all_channels();
                    }
                    if (retune) {
                        drop_au();
                        printf("ar8030: link back after the bandwidth change - same link, video goes on\n");
                    } else if (ls == 2) {
                        // Whatever is still queued belongs to the link that just
                        // ended. Feeding it to the decoder replays history as
                        // fast as it arrives - the picture races, which is worse
                        // than no picture at the moment you are trying to work
                        // out where the aircraft is. Drop it and wait for a
                        // keyframe from the new link.
                        drop_au();
                        live_cap_valid_  = false;   // a new link may be a new clock
                        hdr_stale_       = false;
                        resync_          = true;
                        resync_live_     = 0;
                        resync_prev_us_  = 0;
                        resync_prev_cap_ = 0;
                        resync_start_ms_ = now_ms();
                        air_ver_seen_    = false;   // the next air unit may be the other kind
                        clear_air_info();
                        air_hdr_tag_     = false;
                        air_kestrel_     = false;
                        air_feat_        = 0;
                        air_stock_hdrs_  = 0;
                        if (osd) osd->set_air_stock(false);
                        frame_clock_.forget_air();
                        printf("ar8030: link up - dropping video until it arrives "
                               "at the rate it was captured\n");
                        printf("ar8030: link up - sending the air handshake now that "
                               "there is a link to carry it\n");
                        send_air_handshake();
                        last_rehandshake_ms = tl;
                        rehandshakes = 0;
                    }
                    if (ls == 2 && !frame_mode_due_ms_.load()) frame_mode_due_ms_.store(tl + 1000);   // no event (stock library)
                    last_link_state = ls;
                }
                if (frame_mode_changed.exchange(false)) frame_mode_due_ms_.store(tl);   // the menu
                const uint64_t fm_due = frame_mode_due_ms_.load();
                // Once the air unit has said what it runs (kestrel-air's first
                // tagged header, or the stock app's 200 untagged): a stock air
                // unit keeps the standard frame every link starts on.
                if (fm_due && ls == 2 && tl >= fm_due && tl - bw_change_ms_.load() >= 1000 && air_ver_seen_) {
                    frame_mode_due_ms_.store(0);
                    if (air_kestrel_) set_frame_mode();
                    else printf("ar8030: stock air app: the radio frame stays standard\n");
                }
            }
        }

        uint64_t now = now_ms();
        if (osd && now - last_tick_ms >= 1000) {
            last_tick_ms = now;
            // Only publish an idle (zero) bitrate once video has actually
            // stopped. update_stats() publishes the true figure once a second
            // while data flows, so zeroing unconditionally here made the OSD
            // alternate between the real bitrate and 0 every other second.
            if (now - last_data_ms >= 1500) {
                osd->update_video_bandwidth(0.0);   // idle OSD tick
                osd->log_csv_row();
                osd->signal_render(prof::kWakeLink);
            }
        }

        // timeout in ms; short enough that *should_stop is honoured promptly.
        // On ar_libre: what is queued, else one wait on the video, control
        // and MSP sockets together, so a slice is read the moment it lands.
        // Read in turn, each with its own timeout, the control and MSP drains
        // above held it up to 1 ms whenever they found nothing (their last
        // read always does): the stats' Dec wait, 0.8 ms (2026-10-12).
        // The video silent while the air unit still talks (see where it is
        // called): the handshake again. Also checked on a pass that only
        // found the control or MSP socket's data - an air unit sending its
        // telemetry every ~0.5 s kept the video read from ever timing out,
        // and this from ever running (2026-10-14, video gone after a step
        // to 40 MHz until kestrel-gnd restarted).
        auto video_silent_check = [&]() {
                uint64_t t_now = now_ms();
                // Tight enough that an air unit coming back on its own power is
                // streaming again inside ten seconds: about three to notice, then
                // the handshake itself.
                // Talking: anything from it in the last 8 s - with the video gone
                // a kestrel-air sends little else than its status, every ~5 s
                // (3 s missed it: video stayed off after a step to 40 MHz).
                bool air_talking = last_ctrl_ms && (t_now - last_ctrl_ms) < 8000;
                // No video yet since this start: counted from the link-up's
                // handshake. A handshake sent just before the link re-formed
                // (the radio's own step to 40 MHz right after link-up, as after
                // RADIO > Max Bandwidth restarts the radio) can be lost, and
                // the air unit then talks but never streams.
                bool video_gone  = last_data_ms ? (t_now - last_data_ms) > 2500
                                                : last_rehandshake_ms && (t_now - last_rehandshake_ms) > 5000;
                bool cooled      = (t_now - last_rehandshake_ms) > 6000;
                if (air_talking && video_gone && cooled && rehandshakes < 4) {
                    last_rehandshake_ms = t_now;
                    rehandshakes++;
                    printf("ar8030: video silent %llums but telemetry still arriving - "
                           "re-sending the air handshake (%d/4)\n",
                           (unsigned long long)(t_now - last_data_ms), rehandshakes);
                    send_air_handshake();
                }
        };
        int rd;
        if (poll_fd(sockfd) >= 0) {
            rd = arlink_socket_read_now(sockfd, buf.data(), (uint32_t)buf.size());
            if (rd == 0) {
                struct pollfd p[3];
                int np = 0;
                bool other = false;
                for (int s : { sockfd, ctrl_sockfd, msp_sockfd }) {
                    const int f = poll_fd(s);
                    if (f >= 0) p[np++] = { f, POLLIN, 0 };
                }
                if (poll(p, (nfds_t)np, 500) > 0)
                    for (int i = 1; i < np; i++) other |= (p[i].revents & POLLIN) != 0;
                rd = arlink_socket_read_now(sockfd, buf.data(), (uint32_t)buf.size());
                if (rd == 0 && other) { video_silent_check(); continue; }   // the control or MSP socket's: drained above, next pass
                if (rd == 0) rd = -1;             // nothing: as a read timeout
            }
        } else {
            rd = bb_socket_read(sockfd, buf.data(), (uint32_t)buf.size(), 500);
        }
        if (rd > 0) {
            read_rx_us = arlink_socket_rx_ns ? arlink_socket_rx_ns(sockfd) / 1000 : 0;
            read_timeouts = 0;
            last_data_ms = now_ms();
            rehandshakes = 0;
            total_bytes += (unsigned long long)rd;
            if (dumped_reads < 5) {
                dumped_reads++;
                char hex[3 * 32 + 1];
                int n = rd < 32 ? rd : 32;
                for (int i = 0; i < n; i++) sprintf(hex + i * 3, "%02X ", buf[i]);
                hex[n * 3] = 0;
                printf("ar8030: read[%d] %d bytes: %s\n", dumped_reads, rd, hex);
            }
            if (ltrace::on()) ltrace::rec(ltrace::kRead, now_us(), (uint32_t)rd, 0);
            // `echo N > /tmp/vdump.req`: the next N seconds of the radio's video
            // bytes, as read, into /tmp/vdump.bin - what --debug-video-dump does,
            // without restarting kestrel-gnd - and the parameter sets held now
            // into /tmp/vdump.ps.
            {
                static uint64_t req_check_ms = 0, dump_until_ms = 0;
                const uint64_t t = now_ms();
                if (t - req_check_ms >= 500) {
                    req_check_ms = t;
                    if (FILE *rq = fopen("/tmp/vdump.req", "r")) {
                        int secs = 0;
                        if (fscanf(rq, "%d", &secs) != 1) secs = 0;
                        fclose(rq);
                        unlink("/tmp/vdump.req");
                        if (secs > 0) {
                            if (video_dump_fp) { fclose((FILE *)video_dump_fp); video_dump_fp = nullptr; }
                            // With intra refresh the parameter sets only come with a
                            // keyframe: the ones we hold, to decode the dump with.
                            if (FILE *ps = fopen("/tmp/vdump.ps", "wb")) {
                                for (const auto &q : dvr_param_sets) fwrite(q.data(), 1, q.size(), ps);
                                fclose(ps);
                            }
                            video_dump_path = "/tmp/vdump.bin";
                            video_dump_written = 0;
                            dump_until_ms = t + (uint64_t)secs * 1000;
                        }
                    }
                }
                if (dump_until_ms && t >= dump_until_ms) {
                    if (video_dump_fp) { fclose((FILE *)video_dump_fp); video_dump_fp = nullptr; }
                    printf("ar8030: video dump done, %llu bytes in %s\n", video_dump_written,
                           video_dump_path.c_str());
                    video_dump_path.clear();
                    dump_until_ms = 0;
                }
            }
            if (!video_dump_path.empty() && video_dump_written < 64u * 1024 * 1024) {
                if (!video_dump_fp) {
                    // Appending after a reconnect, which closes it.
                    video_dump_fp = (void *)fopen(video_dump_path.c_str(),
                                                  video_dump_written ? "ab" : "wb");
                    if (video_dump_fp) printf("ar8030: dumping video to %s\n", video_dump_path.c_str());
                    else video_dump_path.clear();
                }
                if (video_dump_fp) {
                    fwrite(buf.data(), 1, (size_t)rd, (FILE *)video_dump_fp);
                    video_dump_written += (unsigned long long)rd;
                }
            }
            consume(buf.data(), (size_t)rd);
        } else {
            // bb_socket_read() returns -1 both for a plain read timeout (no
            // air unit transmitting) and for a dead socket, so a bare -1 is
            // not a reason to tear the link down. Only reconnect once reads
            // have been failing for long enough that the socket is suspect.
            if (read_timeouts == 6 && !link_reported) { link_reported = true; report_link_status(); }
            if (++read_timeouts == 1 || read_timeouts % 40 == 0)
                printf("ar8030: no video yet (%d idle reads) - is the air unit powered?\n",
                       read_timeouts);

            // Video silent while telemetry still arrives. The air unit is
            // powered, linked and answering - it has just stopped streaming,
            // which is what happens when it re-links or restarts and comes
            // back without the video configuration we sent at bring-up. The
            // handshake's two SET_CONFIG frames are that configuration, so
            // sending them again is the whole fix.
            //
            // Waiting for the socket to look dead instead (read_timeouts >=
            // 200) does eventually recover this, but only after ~100s of a
            // blank screen, and by tearing down a link that was never broken.
            video_silent_check();
            // NOT while binding. A bind window has no video by definition -
            // no air unit is associated yet, that is the entire point - so
            // this recovery reads the silence as a dead socket and tears down
            // the link the bind is running on. disconnect_bb() closes the
            // device, which loses BB_SET_PAIR_MODE, so any bind outliving
            // this timer was being killed from inside. Seen on hardware as
            // repeated "shutdown:" and "bring-ups failed - restarting the
            // daemon" in the middle of an open window.
            if (read_timeouts >= 200 && !bind_busy()) {
                printf("ar8030: link idle too long, reopening socket\n");
                disconnect_bb();
                accum.clear();
                read_timeouts = 0;
                sleep(1);
            }
        }
    }

    printf("ar8030: source stopped (%llu frames)\n", (unsigned long long)frames_seen);
    disconnect_bb();
}
