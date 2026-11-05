#ifndef AR8030_SKY_H
#define AR8030_SKY_H

#include <cstddef>
#include <cstdint>
#include <vector>

// The "sky" (air-unit / camera) control protocol.
//
// It rides inside the port-2 BB socket as a byte stream of FE A5 frames. The
// baseband knows nothing about them - they are payload written with
// bb_socket_write() and read back with bb_socket_read().
//
// Frame layout (verified against all 26 frames of a stock clean boot; the
// builder below reproduces every one of them byte-exactly):
//
//   FE A5 | hchk | seq | len16 | cmd | payload[] | 00 | crc32c[4]
//
//   hchk   = (seq + len16_lo + len16_hi) & 0xFF
//   len16  = little-endian u16, = (1 + payload_len) << 4.
//            The low nibble is a flag: 0 in commands, 8 in the air unit's
//            replies, so an ack is recognised by (len16 & 0xF) == 8.
//   00     = one pad byte the stock framer always appends
//   crc32c = CRC-32C (Castagnoli, poly 0x1EDC6F41 reflected), init 0,
//            xorout 0, taken over every preceding byte, stored little-endian.
//            Note init/xorout are 0, not the usual 0xFFFFFFFF.
//
// The air unit acknowledges each command with a frame carrying the same cmd
// byte, the ack flag, and a one-byte status (0 = accepted).
namespace sky {

// Wire command ids, recovered from ar_ldy_gnd's GUI command dispatcher.
enum Cmd : uint8_t {
    CMD_SET_SCENES        = 0x01,  // u32 scene index
    CMD_SET_EV            = 0x02,  // i32 EV x10 (-30..30 => -3.0..+3.0 EV)
    CMD_SET_SAT           = 0x03,  // u32 saturation
    CMD_SET_SHARPNESS     = 0x04,  // u32 sharpness
    CMD_SET_AWB           = 0x05,  // u32 colour temperature (0 = auto)
    CMD_SET_ANGLE         = 0x06,  // u32 - camera rotation, stock's "Rotate" row
    CMD_SET_IMAGE_RATIO   = 0x07,  // u32 aspect ratio index
    CMD_SET_CHN_RES       = 0x08,  // {u8 chn, u16 w, u16 h, u8 fps}  <- video mode
    CMD_SET_REC_LOOP      = 0x0B,  // u32 loop-record device
    CMD_FORMAT_SKY        = 0x0C,  // no payload
    CMD_SET_CHN_FOCUS     = 0x0E,  // {u8 chn, u8 enable}
    CMD_SET_BB_FREQ       = 0x21,  // {u8 hop_en, u8 slot, u32 freq_khz}  <- RF channel
    CMD_SET_WORK_CHAN_LIST= 0x29,  // u32 freq_khz[n]  - SG_MSG_ID_SET_WORK_CHAN_LIST
    CMD_RECORD_TRIG       = 0x0F,  // 5 bytes
    CMD_SET_CONFIG        = 0x10,  // full 49-byte config + TLV tail
    CMD_SKY_CFG_RESET     = 0x1E,  // u32
    CMD_SET_3DNR          = 0x1B,  // u32 0 off, 1 bin's, 2/3/4 low/mid/high (osd.cpp kDnrVals)
    // u32 boolean enable flag. ar_ldy_gnd has a handler for it
    // (GUI_CMD_SET_CAM_ANTI_FLICKER at 0x88e74) and the air unit ACKs it, but
    // stock's own Camera menu has no Anti-Flicker row at all - confirmed
    // against a photo of the stock UI (Scene/EV/Saturation/Sharpness/
    // Contrast/WB/Rotate/Ratio/3D DNR, nothing else). Not exposed in our menu
    // either, for the same reason: it is not a setting stock ever lets you
    // touch, so there is nothing to match it against.
    CMD_SET_ANTI_FLICKER  = 0x1F,
    CMD_SET_BB_PWR        = 0x22,  // u16 mW
    CMD_ENABLE_IDR        = 0x20,  // u8 1: send a keyframe (stock's "enable I-frames"; the handshake has it)
    CMD_SET_STANDBY       = 0x23,  // u8
    // Ask the air unit for its measured video delay. Stock's ground app polls
    // this rather than trying to reconcile clocks: the air computes the number
    // locally, where the capture and transmit timestamps share one clock
    // domain, so the constant offset that blocks a ground-side calculation
    // never arises. Dispatched by the air firmware's
    // sky handler tbh table at index 0x26 -> fpv_video_parser_delay_rsp(),
    // which scales the raw value by 8/10, 9/10 or 10/10 depending on the
    // pipeline mode before replying.
    CMD_GET_VIDEO_DELAY   = 0x26,  // u8 mode -> air replies with the delay
    CMD_SET_BB_BANDWIDTH  = 0x24,  // u8
    CMD_SET_VIDEO_STRATEGY= 0x25,  // u8
    CMD_SET_CONTRAST      = 0x27,  // u32 contrast
    CMD_KA_MAX_BITRATE    = 0x40,  // u32 kbps, 0 = none - kestrel-air only (stock logs and ignores it)
    CMD_KA_MAX_BW         = 0x41,  // u8 MHz, 20 or 40 (0 = none) - kestrel-air only
    CMD_KA_LOGS           = 0x42,  // u8 1/0: the air unit keeps flight logs on its flash - kestrel-air only
    CMD_KA_FRAME_TRIM     = 0x43,  // SPIKE: s32 ns a frame, u16 hold ms - kestrel-air only
};

// The FPV-channel (ch0) video modes this camera accepts, taken from the mode
// list in the stock GlassesUI binary rather than guessed. ch1 (the air unit's
// own recording channel) offers a different set - 1080P100, 2.7KP30, 4KP30,
// 4KP60 - which we do not expose here because ch1 is not what we decode.
struct VideoMode {
    const char* name;
    uint16_t    w;
    uint16_t    h;
    uint8_t     fps;
    uint16_t    need = 0;   // kestrel-air feature bits the mode takes (0: any air unit); 0x100 and
                            // up are said in its info, not its header (Ar8030Source::kAirFeat864p144)
};
extern const VideoMode kFpvModes[];
extern const int       kFpvModeCount;

// Index of the mode stock boots into (1080p100, sent as 120), used as the menu
// default.
int default_fpv_mode_index();

// The subset of the SET_CONFIG (cmd 0x10) body we understand and want to
// control. Offsets were recovered from the serialiser in ar_ldy_gnd, not
// guessed.
//
// The stock capture is still used as the template for the whole 59-byte body so
// every byte we have NOT identified keeps its stock value; only these fields are
// overwritten, plus the timestamp.
struct SkyConfig {
    uint16_t ch0_w = 1920, ch0_h = 1080;  uint8_t ch0_fps = 120;   // the FPV stream
    uint16_t ch1_w = 1920, ch1_h = 1080;  uint8_t ch1_fps = 60;    // air-unit recording
    int8_t   ev_x10        = 0;
    uint8_t  scenes        = 0;
    uint8_t  saturation    = 0;
    uint8_t  sharpness     = 0;
    uint16_t awb_cct       = 0;
    uint8_t  angle         = 0;
    uint8_t  image_ratio   = 1;
    uint8_t  focus_en      = 0;
    uint8_t  anti_flicker  = 0;
    uint8_t  video_strategy= 0;
    uint8_t  contrast      = 0;   // 0-15: the body packs it into a nibble
    // 3D DNR, body[35]. Levels, not a flag: the stock goggle stores
    // Off/Low/Mid/High as 1/2/3/4 and writes 0 only when it has never been
    // set (which its UI renders as Off) - though the air unit takes 1 as its
    // tuning bin's own, not off (osd.cpp kDnrVals). Proved on hardware -
    // switching stock to High moved exactly this byte 0 -> 4 in the air
    // unit's cmd 0x03 report.
    uint8_t  dnr_3d        = 0;
};

// Overwrite the known fields of a 59-byte SET_CONFIG body in place, and stamp it
// with the current time (stock sends a live gettimeofday here, so replaying a
// captured frame also replays a stale clock). Returns false if len is wrong.
bool patch_config_payload(uint8_t *payload, size_t len, const SkyConfig &cfg);

// Read the known fields back out of a 59-byte body (used to seed defaults from
// the stock capture).
bool read_config_payload(const uint8_t *payload, size_t len, SkyConfig *out);

// Frame builder / parser. Owns the outgoing sequence counter, which the air
// unit tracks: it starts at 1 and must increase monotonically per frame.
class Proto {
    public:
        // Wrap cmd+payload into a complete FE A5 frame, consuming one seq.
        std::vector<uint8_t> build(uint8_t cmd, const uint8_t* payload, size_t len);
        std::vector<uint8_t> build(uint8_t cmd) { return build(cmd, nullptr, 0); }

        // Convenience wrappers for the fixed-width setters.
        std::vector<uint8_t> build_u8 (uint8_t cmd, uint8_t v)  { return build(cmd, &v, 1); }
        std::vector<uint8_t> build_u32(uint8_t cmd, uint32_t v);

        // Video mode. chn 0 is the low-latency FPV stream that reaches us over
        // the port-3 socket; chn 1 is the air unit's own recording channel.
        // Stock boots at ch0 = 1920x1080@120, ch1 = 1920x1080@60.
        std::vector<uint8_t> set_chn_res(uint8_t chn, uint16_t w, uint16_t h, uint8_t fps);

        // RF channel. This is how stock moves the AIR unit: the ground side's
        // BB_SET_CHAN only retunes the ground, and stock issues no remote
        // ioctl at all. Note it carries the
        // frequency in kHz, not a channel index, so the two ends do not have to
        // agree on the ordering of their channel tables.
        std::vector<uint8_t> set_bb_freq(bool hop_en, uint8_t slot, uint32_t freq_khz);

        // The channel list the air unit is allowed to use, as raw kHz values.
        // ar_ldy_gnd:0x89318 logs this as SG_MSG_ID_SET_WORK_CHAN_LIST and
        // sends count*4 bytes. Distinct from the ground-side
        // BB_SET_WORK_CHAN_LIST ioctl, which carries channel *indices*.
        std::vector<uint8_t> set_work_chan_list(const uint32_t *freq_khz, size_t n);

        uint8_t seq() const { return seq_; }
        void set_seq(uint8_t s) { seq_ = s; }

        // Validate a received frame and pull out cmd/status. Returns false if
        // the frame is truncated, mis-sized or fails CRC.
        static bool parse(const uint8_t* buf, size_t len,
                          uint8_t* cmd, bool* is_ack, uint8_t* status);

        static uint32_t crc32c(const uint8_t* d, size_t n);

        // The air unit's status report (inbound cmd 0x03) is a 49-byte config
        // body followed by a TLV tail: {u8 len, u8 tag, u8 value[len-2]}, where
        // len counts itself and the tag. Stock's GlassesUI parses it the same
        // way, skipping 50 bytes (cmd + body) before walking the tags
        // (GlassesUI:0x869ec). Returns the first byte of the tag's value, or -1.
        static int find_tlv(const uint8_t* payload, size_t len, uint8_t tag);

        // TLV tags carried in cmd 0x03, named from stock's RSV_SKY_CAM_CFG log
        // line ("sky_prj_name=%d, sensor_type=%d, standby_mode=%d, ...").
        enum SkyTlv : uint8_t {
            TLV_SKY_PRJ_NAME  = 0x11,
            TLV_STANDBY_MODE  = 0x12,  // 1 = low-power/standby, 0 = normal
            TLV_SENSOR_TYPE   = 0x13,
            TLV_SKY_RF_HWVER  = 0x14,
        };
        static const size_t kStatusTlvOffset = 50;  // cmd byte + 49-byte body

    private:
        uint8_t seq_ = 1;
};

}  // namespace sky

#endif
