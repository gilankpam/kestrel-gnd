
#include <poll.h>
#include <sys/statvfs.h>
#include "artosyn/ar8030_source.hpp"
#include "frame_mode.hpp"
#include "osd.hpp"
#include "hud_theme.hpp"
#include "dvr.hpp"
#include "wifi_ap.hpp"
#include "renderer.hpp"
#include "settings.hpp"
#include "zones.hpp"
#include <iostream>
#include <unistd.h>
#include <xf86drm.h>
#include <algorithm>
#include <set>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <gbm.h>
#include <cairo.h>

// rockchip-mali ships a cut-down gbm.h that omits gbm_bo_get_modifier, even
// though libmali exports the symbol (verified: defined in the shipped
// libmali.so.1). Mesa's gbm.h declares it. Declare it here so kestrel builds
// against either header - a second identical declaration is harmless where the
// header already provides one.
extern "C" uint64_t gbm_bo_get_modifier(struct gbm_bo *bo);

#include "utils/time_util.h"
#include <ctime>   // the top-centre clock: time/localtime_r/strftime
#include "utils/math_utils.hpp"
#include "icons/logo.h"
#include "betaflight_glyphs.hpp"
#include "kestrel_gnd_config.h"


// The TX Power row holds the power chosen; it shows (and Enter sends) it
// fitted to the air unit connected, and -- until that unit has said what it is
// (its levels are not known before: a Lite+ would be offered a Lite's).
void OSD::set_air_power_levels() {
    if (!menu_open) menu_ar_power = Ar8030Source::tx_power_want_mw;
}

// AR8030 RF menu. The rows are Channel (the channel page, where AUTO is the
// old Channel Hop), TX Power and Standby Mode; MCS, Bandwidth and LNA Mode were
// removed - see ar_rf_items().
//
// Power levels live in common.hpp (kArPwrLevels): mW, with the dBm the radio
// wants and stock's "N+1 means auto capped at N" encoding, taken from
// ar_ldy_gnd's own table. Capped at 31 dBm - stock's table goes to 37 but
// bb_set_pwr_in_t documents [0-31], and the board browns out under load
// (section 11).
// Stock's work-channel table (kHz), plus AUTO scan/hop at index 0.
static const int kArChanVals[]   = { -1, 5740000, 5770000, 5805000, 5839000 };
static const char* kArChanLabels[] = { "AUTO", "5740", "5770", "5805", "5839" };
static const int kArChanCount = 5;
// Max Bitrate: kestrel-air's video bitrate cap, kbps (0 = AUTO: it follows the
// link, up to 20 Mbps; a cap above that lets kestrel-air go to it, 40 at most,
// on a link that carries it - 40 MHz). Below the link's own rate it buys a
// steadier picture and shorter slices on the air; stock air units ignore it.
static const int kArMaxBrVals[]     = { 0, 6000, 8000, 10000, 12000, 15000, 18000, 20000, 22000, 25000,
                                        30000, 35000, 40000 };
static const char* kArMaxBrLabels[] = { "AUTO", "6 Mbps", "8 Mbps", "10 Mbps", "12 Mbps",
                                        "15 Mbps", "18 Mbps", "20 Mbps", "22 Mbps", "25 Mbps",
                                        "30 Mbps", "35 Mbps", "40 Mbps" };
static const int kArMaxBrCount = 13;
static int ar_maxbr_index(int kbps) {
    for (int i = 0; i < kArMaxBrCount; i++) if (kArMaxBrVals[i] == kbps) return i;
    return 0;
}
// Max Bandwidth: how wide the video link may go. The goggle's radio widens it
// by itself on a good link (bw_auto), to 40 MHz with fpvOS's radio config -
// more throughput and a shorter frame, less range - or 20 with its 20 MHz one
// (bb_only.sh picks it from ar8030_max_bw_mhz). Any air unit follows. With
// Open8030 on the goggle, AUTO (the radio's own config, nothing sent), 5 and
// 10 too, taken live (SET 22).
static const int kArMaxBwVals[]     = { 0, 5, 10, 20, 40 };
static const char* kArMaxBwLabels[] = { "AUTO", "5 MHz", "10 MHz", "20 MHz", "40 MHz" };
static const int kArMaxBwCount = 5;
static int ar_maxbw_index(int mhz) { return mhz == 0 ? 0 : mhz == 5 ? 1 : mhz == 10 ? 2 : mhz == 20 ? 3 : 4; }
// Max MCS (Open8030 on the goggle): the highest MCS its receive policy may
// use - AUTO (none), or MCS 0..11. Lower: fewer blocks sent again (the radio
// part's latency tail), less throughput.
static const int kArMaxMcsCount = 13;   // AUTO, MCS 0..11
static const char *ar_maxmcs_label(int i) {
    static char b[8];
    if (i <= 0) return "AUTO";
    snprintf(b, sizeof(b), "MCS %d", i - 1);
    return b;
}
// Radio Frame: the TDD frame mode asked of the radio (Ar8030Source::frame_mode).
// SHORT, mode 2, is 3.25 ms at 40 MHz and 4.65 at 20; STANDARD, stock's mode
// 0, is 4.23 and 5.63, with an uplink twice as long. For field tests.
static const int kArFrameVals[]     = { 2, 0 };
static const char* kArFrameLabels[] = { "SHORT", "STANDARD" };
static const int kArFrameCount = 2;
static int ar_frame_index(int mode) { return mode == 0 ? 1 : 0; }
// VRX Radio Firmware: the baseband firmware the goggle's radio runs (bb_only.sh
// uploads it from ar8030_firmware): Open8030 when it is installed (the
// default), or stock's.
static const char* kArFwLabels[] = { "STOCK", "OPEN8030" };
static const int kArFwCount = 2;
// DEBUG (decode stalls): when the OSD last started and finished drawing, us
std::atomic<uint64_t> g_dbg_osd_t0{0}, g_dbg_osd_t1{0};

// DISPLAY > Refresh Sync's values (g_display_lock).
static const char *refresh_sync_label(int v) {
    return v == 3 ? "CAMERA" : v == 2 ? "FREE" : v == 1 ? "ON" : "OFF";
}

std::vector<std::pair<const char*, int>> OSD::ar_rf_items() const {
    std::vector<std::pair<const char*, int>> v;
    // Unlinked: binding is the only thing this tab can do, and the only time
    // it makes sense - it is how an air unit becomes associated in the first
    // place. Linked: an air unit is already associated, so Bind is hidden and
    // the tuning rows take over. Pairing is a baseband operation and does not
    // depend on the rf_caps bits, which only describe what the radio lets us
    // tune, so this row never looks at them.
    if (osd_vars.artosyn.state != 2) {
        v.push_back({"Bind", 9});
        // Also unlinked: a firmware that does not link is the time to switch.
        v.push_back({"VRX Radio Firmware", 13});
        return v;
    }
    // Channel opens the channel page. Channel Hop (id 7) was the other half of
    // the same decision - picking a channel turns hopping off - and is AUTO on
    // that page now. Every consumer switches on the item id (.second), not the
    // row index, so this order is free to change.
    if (rf_caps & 16) v.push_back({"Channel",   5});  // RF_CHAN, RF_HOP
    if (rf_caps & 1)  v.push_back({"TX Power",  1});  // RF_TX_POWER
    // Max Bandwidth (11), Radio Frame (12) and Steady Latency (15) are
    // hidden: their settings (ar8030_max_bw_mhz, ar8030_frame_mode,
    // ar8030_steady) still apply from the yaml, and their handlers stay, so
    // listing them again is one line each here. Steady Latency needs an
    // Open8030 with steady mode (request 0x0c5a0037), which its main lacks.
    // Open8030 on the goggle: the MCS its receive policy may go up to.
    if (Ar8030Source::radio_fw == 1) v.push_back({"Max MCS", 14});
    v.push_back({"VRX Radio Firmware", 13});
    // MCS, Bandwidth and LNA Mode were removed rather than left visible.
    // BB_SET_MCS has no direction field, so it can only set our own transmit -
    // the video MCS is the air unit's choice, driven by the policy table in
    // section 38.2, and no ground-side setting moves it. BB_SET_BANDWIDTH is
    // not how this firmware sets bandwidth at all and measurably costs
    // throughput (section 38.4). LNA does work, but it only trims our own
    // front end and AUTO is the right answer outside a bench.
    // Standby is not an rf_caps bit - it is a sky command to the air unit
    // (0x23), so it is offered whenever the link is up rather than gated on
    // what the local baseband's ioctls answer.
    //
    // "Auto Standby", not "Standby Mode": this row reflects/toggles the
    // air's enable/disable SETTING for parking itself in low power while
    // idle - not whether it is in standby right now, which the row cannot
    // show reliably (TLV 0x12 was confirmed stuck reporting ON through an
    // entire arm/disarm cycle - see ar8030-power-and-standby-verified.md).
    // The live state is the canopy's own STANDBY label instead.
    v.push_back({"Auto Standby", 8});
    return v;
}

// AR8030 camera menu option tables. EV, the mode list and the anti-flicker
// choices come from the stock GlassesUI; contrast is 0-15 because the config
// frame packs it into a nibble. Saturation/sharpness ranges are NOT confirmed -
// the camera acks every setting, so watch the log for a non-zero status.
static const int   kEvSteps[]      = { -10, -7, -3, 0, 3, 7, 10 };
static const char* kEvLabels[]     = { "-1.0", "-0.7", "-0.3", "0", "+0.3", "+0.7", "+1.0" };
static const int   kEvCount        = 7;
// Scene. Captured off the wire from stock (cmd 0x01): Race = 0, Standard = 2.
// We used to send the menu index (0 = Standard, 1 = Race), i.e. both values
// wrong. The gap at 1 is presumably a scene this unit does not offer.
static const char* kSceneLabels[]  = { "Race", "Standard" };
static const int   kSceneVals[]    = { 0, 2 };
static const int   kSceneCount     = 2;
// 3D DNR, sky cmd 0x1B. The stock goggle sends Off/Low/Mid/High/Auto as
// 1/2/3/4/5, but the air unit - the stock app and kestrel-air alike - takes
// 0 off, 1 its tuning bin's own noise reduction, 2/3/4 low/mid/high, and
// anything else as 1: stock's Off (1) left it on. Off is 0 here, which both
// turn off; Auto stays 5, the bin's.
static const char* kDnrLabels[]    = { "Off", "Low", "Mid", "High", "Auto" };
static const int   kDnrVals[]      = { 0, 2, 3, 4, 5 };
static const int   kDnrCount       = 5;
// Anti-Flicker (sky cmd 0x1F) was here but is gone: stock's own Camera menu
// has no such row at all - confirmed against a photo of the stock UI - so it
// was never a real setting to expose, whatever encoding the command itself
// takes on the wire. See CMD_SET_ANTI_FLICKER in ar8030_sky.hpp.
// White balance. The wire carries the colour temperature in Kelvin (cmd 0x05),
// and stock offers Auto plus 4000K..7000K in 100K steps - 32 entries. The old
// five-value table was a guess and did not match any of stock's steps except
// 4000K. fact_env.json stores the *index*, which is why 7000K reads as 31.
static const int   kWbCount        = 32;
static inline int  wb_kelvin(int i) { return i <= 0 ? 0 : 3900 + i * 100; }
static inline const char* wb_label(int i) {
    static char buf[8];
    if (i <= 0) return "Auto";
    snprintf(buf, sizeof(buf), "%dK", wb_kelvin(i));
    return buf;
}
// Camera rotation, stock's Camera > Rotate row (sky cmd 0x06).
static const char* kRotateLabels[] = { "0", "180" };
// Saturation, sharpness and contrast are Auto plus 1..10 on stock, and 0 is
// what Auto puts on the wire - the same scheme white balance uses (0 = Auto,
// otherwise a real value). Rendering 0 as "0" made Auto look like "minimum".
// Contrast used to allow 15: that is the width of the nibble the config body
// packs it into, not stock's range.
static const int   kCamSatMax      = 10;
static const int   kCamSharpMax    = 10;
static const int   kCamContrastMax = 10;
// 0 -> "Auto", anything else the number itself.
static inline const char* cam_auto_label(int v) {
    static char buf[8];
    if (v <= 0) return "Auto";
    snprintf(buf, sizeof(buf), "%d", v);
    return buf;
}


static int freq_to_channel(int freq) {
    if (freq == 2484) return 14;
    if (freq < 2484) return (freq - 2407) / 5;
    if (freq >= 4910 && freq <= 4980) return (freq - 4000) / 5;
    if (freq < 5935) return (freq - 5000) / 5;
    if (freq >= 5955 && freq <= 7115) return (freq - 5950) / 5; // 6GHz
    return 0; 
}

#define OSD_BUF_COUNT 2

static std::map<void*, uint32_t> bo_to_fb;

// Console Color Definitions
#define RESET       "\033[0m"
#define BOLDCYAN    "\033[1m\033[36m"
#define BOLDYELLOW  "\033[1m\033[33m"
#define BOLDGREEN   "\033[1m\033[32m"
#define BOLDMAGENTA "\033[1m\033[35m"
#define BLUE        "\033[34m"
#define GREEN       "\033[32m"
#define YELLOW      "\033[33m"
#define RED         "\033[31m"
#define CYAN        "\033[36m"

static std::string get_braille_char(unsigned char dots) {
    // Unicode Braille Patterns start at U+2800 (represented in UTF-8 as E2 A0 80)
    std::string s;
    s += (char)0xE2;
    s += (char)(0xA0 | (dots >> 6));
    s += (char)(0x80 + (dots & 0x3F));
    return s;
}

// Helper for console graphing
static void print_metric_line(const char* label, double value, const char* unit, double max_scale, int width, const char* label_color, const char* bar_color, const char* value_color) {
    int fill = (int)((value / max_scale) * width);
    if (fill > width) fill = width;
    if (fill < 0) fill = 0;

    printf("%s%-5s%s [", label_color, label, RESET);
    printf("%s", bar_color);
    for (int i = 0; i < width; i++) {
        if (i < fill) printf("█");
        else printf(" ");
    }
    printf("%s] %s%6.2f %s%s\n", RESET, value_color, value, unit, RESET);
}

static void print_latency_dist(const std::vector<float>& values, double avg, double min, double max, double max_scale, int width, const char* bar_color) {
    std::vector<int> bins(width, 0);
    int max_bin_count = 0;
    if (!values.empty()) {
        for (float v : values) {
            int bin = (int)((v / max_scale) * width);
            if (bin >= width) bin = width - 1;
            if (bin < 0) bin = 0;
            bins[bin]++;
            if (bins[bin] > max_bin_count) max_bin_count = bins[bin];
        }
    }

    printf("Dist: [");
    printf("%s", bar_color);
    // Fixed: blocks[1] was a space, now it's a 1/8th block.
    const char* blocks[] = {" ", " ", "▂", "▃", "▄", "▅", "▆", "▇", "█"};
    for (int i = 0; i < width; i++) {
        if (bins[i] == 0) {
            printf(" ");
        } else {
            int level = (bins[i] * 8) / max_bin_count;
            if (level > 8) level = 8;
            
            if (level == 0 && bins[i] > 0) {
                // Outlier/Single-frame marker
                printf("."); 
            } else {
                printf("%s", blocks[level]);
            }
        }
    }
    printf("%s] Avg: %s%6.2f%s | Max: %s%6.2f%s | Min: %s%6.2f%s\n", RESET, YELLOW, avg, RESET, RED, max, RESET, GREEN, min, RESET);
}

// PNG loading to GL Texture helper
typedef struct {
    const unsigned char* data;
    unsigned int size;
    unsigned int offset;
} cairo_png_reader_t;

cairo_status_t read_png_from_mem(void *closure, unsigned char *data, unsigned int length) {
    cairo_png_reader_t *reader = (cairo_png_reader_t *)closure;
    if (reader->offset + length > reader->size) return CAIRO_STATUS_READ_ERROR;
    memcpy(data, reader->data + reader->offset, length);
    reader->offset += length;
    return CAIRO_STATUS_SUCCESS;
}

GLuint OSD::load_texture_from_png(const unsigned char* png, unsigned int length) {
    cairo_png_reader_t reader = { png, length, 0 };
    cairo_surface_t *surface = cairo_image_surface_create_from_png_stream(read_png_from_mem, &reader);
    if (cairo_surface_status(surface) != CAIRO_STATUS_SUCCESS) {
        fprintf(stderr, "Failed to load PNG to Cairo surface\n");
        return 0;
    }

    int w = cairo_image_surface_get_width(surface);
    int h = cairo_image_surface_get_height(surface);
    unsigned char* data = cairo_image_surface_get_data(surface);

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

    cairo_surface_destroy(surface);
    return tex;
}

// ---------------------------------------------------------------------------
// Async asset prefetch.
//
// The two heaviest startup items are pure CPU/Cairo work with no GL dependency:
//   * decoding /etc/kestrel/background.png  (~260 ms for a 5.7 MB image)
//   * rendering the "KESTREL" idle title via Cairo/FreeType (~50 ms)
// They used to run on the OSD thread *after* eglInitialize() (~285 ms on Mali)
// had already returned, so the big costs were serialized.
//
// osd_prefetch_assets_async() is called once from main() before DRM/EGL
// bring-up, so the Cairo decode runs on another core while the GPU driver
// initializes. init_gl_buffers() then only uploads the already-decoded pixels.
// ---------------------------------------------------------------------------
// Same idea for --menu: set before the OSD exists, read once when it starts.
static bool s_menu_at_start = false;
static int  s_menu_start_tab = -1;
void osd_set_menu_at_start(bool on) { s_menu_at_start = on; }
// Which tab --menu opens on. -1 leaves whatever the menu would pick.
void osd_set_menu_start_tab(int t) { s_menu_start_tab = t; }

static pthread_t        s_prefetch_thread;
static bool             s_prefetch_started = false;
static cairo_surface_t* s_bg_surface    = nullptr; // decoded background.png (null on failure)
// The wordmark, in two layers. "fpv" is the subject and stays in the type
// colour; "OS" takes the theme's accent - the same hue the HUD spends on live
// state, which is the one colour on this screen that already means something
// is running.
//
// Two textures rather than one coloured bitmap, because the accent is not
// known at render time: it follows whichever theme is selected, and a theme is
// chosen by looking at it. Both layers hold the whole wordmark's geometry with
// only their own half inked, so drawing them over each other reassembles the
// word exactly - no second layout to keep in step with the first.
static cairo_surface_t* s_title_surface    = nullptr; // "fpv"
static cairo_surface_t* s_title_os_surface = nullptr; // "OS"
static const int        s_title_w = 2048, s_title_h = 256;
static const char*      kTitleHead = "fpv";
static const char*      kTitleTail = "OS";

static void render_idle_title(cairo_surface_t* surf, int part) {
    cairo_t* cr = cairo_create(surf);
    // The face the rest of the OSD is drawn in, so the app's name is set in
    // the same voice as everything it says.
    cairo_select_font_face(cr, "Chakra Petch SemiBold",
                           CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);

    char full[16];
    snprintf(full, sizeof(full), "%s%s", kTitleHead, kTitleTail);

    // Fit to the surface rather than trust a point size: the size that suited
    // a seven-letter name is not the size that suits a five-letter one, and
    // the quad on screen is sized from this texture's aspect.
    double size = 168.0;
    cairo_text_extents_t all;
    cairo_set_font_size(cr, size);
    cairo_text_extents(cr, full, &all);
    if (all.width > 1.0) {
        size *= (s_title_w * 0.50) / all.width;
        cairo_set_font_size(cr, size);
        cairo_text_extents(cr, full, &all);
    }
    if (all.height > s_title_h * 0.52) {
        size *= (s_title_h * 0.52) / all.height;
        cairo_set_font_size(cr, size);
        cairo_text_extents(cr, full, &all);
    }

    // Centre the whole word, then place each half where the whole word put it.
    double tx = (s_title_w - all.width)  * 0.5 - all.x_bearing;
    double ty = (s_title_h - all.height) * 0.5 - all.y_bearing;

    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
    if (part == 0) {
        cairo_move_to(cr, tx, ty);
        cairo_show_text(cr, kTitleHead);
    } else {
        cairo_text_extents_t head;
        cairo_text_extents(cr, kTitleHead, &head);
        cairo_move_to(cr, tx + head.x_advance, ty);
        cairo_show_text(cr, kTitleTail);
    }
    cairo_destroy(cr);
}

static void* osd_prefetch_func(void*) {
    cairo_surface_t* s = cairo_image_surface_create_from_png("/etc/kestrel/background.png");
    if (s && cairo_surface_status(s) == CAIRO_STATUS_SUCCESS) {
        s_bg_surface = s;
    } else {
        if (s) cairo_surface_destroy(s);
        fprintf(stderr, "[OSD] prefetch: cannot decode /etc/kestrel/background.png\n");
    }
    for (int part = 0; part < 2; part++) {
        cairo_surface_t* ts = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                        s_title_w, s_title_h);
        if (ts && cairo_surface_status(ts) == CAIRO_STATUS_SUCCESS) {
            render_idle_title(ts, part);
            (part == 0 ? s_title_surface : s_title_os_surface) = ts;
        } else if (ts) {
            cairo_surface_destroy(ts);
        }
    }
    return nullptr;
}

void osd_prefetch_assets_async() {
    if (s_prefetch_started) return;
    if (pthread_create(&s_prefetch_thread, nullptr, osd_prefetch_func, nullptr) == 0)
        s_prefetch_started = true; // init_gl_buffers() joins; falls back to sync load otherwise
}

// Upload an already-decoded Cairo ARGB32 surface as a GL_TEXTURE_2D
// (matches load_texture_from_file's parameters).
static GLuint texture_from_cairo_surface(cairo_surface_t* surface) {
    int w = cairo_image_surface_get_width(surface);
    int h = cairo_image_surface_get_height(surface);
    unsigned char* data = cairo_image_surface_get_data(surface);
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    return tex;
}

OSD::OSD(std::shared_ptr<DrmDevice> dev_, int refresh_frequency_ms_, volatile bool* signal_stop, bool console_stats_) :
    dev(dev_), 
    refresh_frequency_ms(refresh_frequency_ms_),
    console_stats(console_stats_),
    signal_stop(signal_stop) {
    tid_dvr_menu_active = false;
    tid_dvr_menu = 0;
    
    display = (EGLDisplay)dev->get_egl_display();
    context = (EGLContext)dev->get_egl_context();
    
    pthread_mutex_init(&osd_mutex, NULL);
    // Monotonic: the render loop's deadlines come from get_time_us(), and a
    // wall-clock step must not stall (or rush) HUD animation.
    {
        pthread_condattr_t ca;
        pthread_condattr_init(&ca);
        pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
        pthread_cond_init(&osd_cond, &ca);
        pthread_condattr_destroy(&ca);
    }
    dev->flip_hook_ctx = this;
    dev->flip_hook = [](void* c) { static_cast<OSD*>(c)->on_flip(); };

    osd_vars.ui_scale = 1.0f;
    osd_vars.sky_exposure_us = 5000;
    osd_vars.sky_framerate = 120;
    osd_vars.slices_received = false;
    menu_ui_scale = 1.0f;
    applied_ui_scale = 1.0f;

    bg_player_ = new BgVideoPlayer("/etc/kestrel/background.mp4", dev);
    // Wake the OSD render loop when a new background frame is ready, but cap
    // idle redraws to ~30 fps. The whole HUD is recomposited on every wake, so
    // tracking the 60 fps cinematic loop doubled idle CPU for no real benefit —
    // 30 fps looks identical for an idle background. Menu/stats/transition
    // wakeups call signal_render() directly and are unaffected by this throttle.
    bg_player_->on_new_frame = [this]() {
        uint64_t now = get_time_us();
        if (now - last_bg_signal_us_ < 32000) return; // ~31 fps
        last_bg_signal_us_ = now;
        signal_render();
    };

    // Standby is not persisted or commanded: the air unit dictates it (auto-parks
    // to low power when disarmed/idle). The menu row reflects the air's reported
    // air_standby state, read-only.
    // The OSD keeps its own copy of dvr_screen for the menu row. It was never
    // loaded from settings, so the menu could read OFF while DvrRecorder (which
    // main() does initialise from the same key) was in screen mode - two views
    // of one setting, silently disagreeing.
    dvr_screen = Settings::getInstance().getBool("dvr_screen", kDvrScreenDefault);
    menu_brightness = Settings::getInstance().getInt("brightness", 50);
    if (dev) dev->set_display_property("brightness", menu_brightness);
    menu_dvr_source = dvr_source_now();
    // Stock subtracts a per-index calibration from the raw ranging result and
    // clamps at zero (ar_ldy_gnd 0x9be20). Its table, built in that function's
    // prologue, is {0, 0, 115, 90, 75, 0, 0, 0} indexed by a byte at ctx+1212 -
    // the offsets fall as the index rises, consistent with an RF-bandwidth
    // index (wider bandwidth = finer time-of-flight resolution = smaller base).
    // This board measures 109-111 with the units together, i.e. stock's index-2
    // entry, so 115 is the default. Recalibrate from HUD > Calib Distance if the
    // RF bandwidth changes, since the correct offset changes with it.
    dist_offset   = Settings::getInstance().getInt("dist_offset", 115);
    show_all_adapters = true;
    bg_video_enabled = Settings::getInstance().getBool("bg_video_enabled", true);
    link_edge_on_ = Settings::getInstance().getBool("link_edge_warning", true);
    hud_debug = Settings::getInstance().getBool("hud_debug", false);
    // 0=OFF 1=SMALL 2=MEDIUM 3=EXTREME. This was a bool before the intensity
    // levels existed, so migrate the stored "true"/"false" rather than letting
    // getInt() fail to parse them - stoi throws on both and would hand back the
    // default, quietly turning the feature back ON for anyone who had it off.
    {
        std::string hr = Settings::getInstance().getString("hud_reactivity", "");
        if (hr == "true")       hud_reactivity = 2;
        else if (hr == "false") hud_reactivity = 0;
        else                    hud_reactivity = Settings::getInstance().getInt("hud_reactivity", 2);
        if (hud_reactivity < 0 || hud_reactivity > 3) hud_reactivity = 2;
        // How far up the air unit's camera is tilted on the airframe: found each time the aircraft is
        // armed (it is level then), saved, and used to put the camera's rates into the aircraft's axes.
        msp_osd.set_cam_tilt(Settings::getInstance().getInt("imu_cam_tilt_deg", 0));
    }

    // The clock has two independent questions - WHEN it shows and WHAT FORMAT
    // it shows in - and clock_mode used to answer both with one enum
    // (OFF|12H|24H). That worked while "when" was hard-coded to idle-only; it
    // stops working the moment the answer can be "always". So clock_show now
    // owns visibility and clock_mode is format alone.
    //
    // Migration runs off the old enum: its 0 meant off, which is a visibility
    // answer, so it becomes clock_show=0 with the format left at 24H. Older
    // still is "show_clock", a plain bool - and getInt() cannot read "false"
    // (it throws and returns the default, which would switch the clock back on
    // for anyone who had turned it off), so the new key is read first.
    {
        int cm = Settings::getInstance().getInt("clock_mode", -1);
        if (cm < 0) cm = Settings::getInstance().getBool("show_clock", true) ? 2 : 0;

        int cs = Settings::getInstance().getInt("clock_show", -1);
        if (cs < 0) cs = (cm == 0) ? 0 : 1;          // old off -> off, else idle-only
        clock_show = (cs >= 0 && cs <= 2) ? cs : 1;

        if (cm <= 0) cm = 2;                         // 0 is no longer a format
        clock_mode = (cm == 1 || cm == 2) ? cm : 2;
    }
    menu_clock_mode = clock_mode;
    menu_clock_show = clock_show;
    {
        int t = Settings::getInstance().getInt("hud_theme", 0);
        hud_theme_idx = (t >= 0 && t < hud_theme_count()) ? t : 0;
        hud_theme_set(hud_theme_idx);
    }
    menu_hud_theme = hud_theme_idx;
    menu_open_at_start = s_menu_at_start;
    if (s_menu_start_tab >= 0 && s_menu_start_tab < kMenuTabs) {
        menu_tab = s_menu_start_tab;
        menu_first_open_ = false;   // asked for by name; do not second-guess it
    }
    // HUD style and the Betaflight OSD. Both are new keys; the three they
    // replaced (hud_detail, hud_style, msp_mode) are read once when the new
    // key is absent, so a goggle comes up after the update looking as it did.
    {
        // "hud_overlay", not the first name this had ("hud_layout"): the
        // ordinals were reordered (OFF moved from 3 to 0) before this ever
        // shipped, and a goggle that had already run the first version would
        // silently misread its own old CANOPY (0) as the new OFF (also 0)
        // otherwise. A fresh name forces the same safe fallback a goggle
        // that never had either key takes.
        int hl = Settings::getInstance().getInt("hud_overlay", -1);
        if (hl < 0 || hl >= kHudStyleCount) {
            int hd = Settings::getInstance().getInt("hud_detail", 2);   // 0 MINIMAL 1 MEDIUM 2 FULL
            int hs = Settings::getInstance().getInt("hud_style", 1);    // 0 LEGACY 1 CANOPY
            if (hd == 0)      hl = kHudOff;
            else if (hs != 0) hl = kHudCanopy;
            else              hl = (hd == 2) ? kHudArenaFull : kHudArena;
        }
        hud_style = hl;
        int bf = Settings::getInstance().getInt("bf_osd", -1);
        if (bf < 0) bf = Settings::getInstance().getInt("msp_mode", 1) != 0 ? 1 : 0;
        bf_osd = (bf != 0);
        int vm = Settings::getInstance().getInt("volt_mode", 0);
        volt_mode = (vm == 0 || vm == 1) ? vm : 0;
    }
    menu_bf_osd = bf_osd;
    menu_hud_style = hud_style;
    menu_volt_mode = volt_mode;
    menu_wifi_ap = wifi_ap_on() ? 1 : 0;   // off after boot; see wifi_ap.hpp
    menu_show_all_adapters = true;
    menu_bg_video = bg_video_enabled;
    menu_frame_mode = g_frame_mode.load();
    menu_refresh_sync = g_display_lock.load();
    menu_sync_slice = g_display_lock_slice.load();
    menu_sync_margin = g_display_lock_margin_us.load() / 1000;
    menu_hud_reactivity = hud_reactivity;
    menu_hud_debug = hud_debug;

    // Restore picture size. 100 is the untouched full-screen path.
    int saved_scale = Settings::getInstance().getInt("picture_scale", 100);
    if (saved_scale >= 60 && saved_scale < 100) {
        menu_picture_scale = saved_scale;
        dev->set_picture_scale(saved_scale);
    }

    menu_zone_ = zones::selected();
    menu_zone_gen_ = zones::generation();
}

void OSD::on_flip() {
    last_flip_seen_us_.store(get_time_us(), std::memory_order_relaxed);
    if (!phase_lock_.load(std::memory_order_relaxed)) return;
    pthread_mutex_lock(&osd_mutex);
    flip_tick_ = true;
    pthread_cond_signal(&osd_cond);
    pthread_mutex_unlock(&osd_mutex);
}

OSD::~OSD() {
    dev->flip_hook = nullptr;
    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context);
    glDeleteProgram(shader_program);
    glDeleteBuffers(1, &vbo);
    
    glDeleteTextures(1, &fps_tex);
    glDeleteTextures(1, &lat_tex);
    glDeleteTextures(1, &net_tex);
    glDeleteTextures(1, &logo_tex);
    if (bg_tex != 0) glDeleteTextures(1, &bg_tex);
    if (bg_video_tex_ != 0) glDeleteTextures(1, &bg_video_tex_);
    if (bg_ext_tex_   != 0) glDeleteTextures(1, &bg_ext_tex_);
    clip_player_.reset();    // joins its decoder before the textures go
    if (clip_ext_tex_ != 0) glDeleteTextures(1, &clip_ext_tex_);
    if (live_ext_tex_ != 0) glDeleteTextures(1, &live_ext_tex_);
    if (gallery_snap_tex_ != 0) glDeleteTextures(1, &gallery_snap_tex_);
    if (gallery_snap_shader_ != 0) glDeleteProgram(gallery_snap_shader_);
    if (gallery_flat_shader_ != 0) glDeleteProgram(gallery_flat_shader_);
    for (auto& th : gallery_thumbs_) if (th.second.tex) glDeleteTextures(1, &th.second.tex);
    if (warp_shader_prog_ != 0) glDeleteProgram(warp_shader_prog_);
    if (bg_ext_shader_    != 0) glDeleteProgram(bg_ext_shader_);
    if (warp_ext_shader_  != 0) glDeleteProgram(warp_ext_shader_);
    if (bg_player_) { bg_player_->stop(); delete bg_player_; bg_player_ = nullptr; }

    // /tmp/kestrel-adapt.csv was lazy-opened and never closed, so the last rows
    // of a run were lost in stdio's buffer on exit.
    if (csv_fp) { fclose((FILE *)csv_fp); csv_fp = nullptr; }

    if (dvr) {
        dvr->stop();
        if (tid_dvr_menu_active) {
            pthread_join(tid_dvr_menu, NULL);
            tid_dvr_menu_active = false;
        }
        dvr = nullptr;
    }

    for (auto& buf : gl_buffers) {
        if (buf.surface != EGL_NO_SURFACE) eglDestroySurface(display, buf.surface);
    }
    pthread_mutex_destroy(&osd_mutex);
    pthread_cond_destroy(&osd_cond);
}

void OSD::signal_render(prof::Wake why) {
    prof::wake(why);
    // A video frame alone is no reason to redraw: the video has its own plane
    // and the HUD is not redrawn to show it. This used to render the HUD once
    // per video frame, 60-120 times a second, mostly to identical pictures.
    // notify_video_frame() wakes the loop on the video edges that do change
    // the HUD (first keyframe, resume after a stall, video confirmed).
    if (why == prof::kWakeVideo) return;

    pthread_mutex_lock(&osd_mutex);
    if (why == prof::kWakeAnim) {
        // From inside a frame: another frame is wanted, one refresh from now,
        // not the instant this one finishes.
        anim_pending_ = true;
    } else {
        render_requested = true;
        pthread_cond_signal(&osd_cond);
    }
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::init_gl_buffers() {
    auto output = dev->output_list;
    // For OSD, we use a single GBM surface and lock the buffers as they come
    struct gbm_surface *gs = gbm_surface_create(dev->get_gbm_device(), output->mode.hdisplay, output->mode.vdisplay,
                                             GBM_FORMAT_ARGB8888, GBM_BO_USE_RENDERING | GBM_BO_USE_SCANOUT);
    if (!gs) {
        fprintf(stderr, "Failed to create GBM surface\n");
        return;
    }

    EGLSurface surf = eglCreateWindowSurface(display, dev->egl_config, (EGLNativeWindowType)gs, NULL);
    if (surf == EGL_NO_SURFACE) {
        fprintf(stderr, "Failed to create EGL surface from GBM surface (error: 0x%x)\n", eglGetError());
        return;
    }

    // Since we'll be using gbm_surface_lock_front_buffer, we don't pre-create FB IDs.
    // We'll create them on the fly and cache them if needed.
    // We'll store the gbm_surface in a GlBuffer slot.
    gl_buffers.push_back({(struct gbm_bo*)gs, surf, 0});
    printf("Created GL Surface: gs=%p, surf=%p\n", gs, surf);

    // Initialize base icons (small embedded PNGs).
    fps_tex  = load_texture_from_png(framerate_icon, framerate_icon_length);
    lat_tex  = load_texture_from_png(latency_icon, latency_icon_length);
    net_tex  = load_texture_from_png(bandwidth_icon, bandwidth_icon_length);
    logo_tex = load_texture_from_png(kestrel_logo_png, kestrel_logo_png_len);
    clock_tex = load_texture_from_png(clock_icon, clock_icon_length);

    // background.png: consume the copy decoded by the prefetch thread (which
    // overlapped EGL init); fall back to a synchronous load if unavailable.
    if (s_prefetch_started) { pthread_join(s_prefetch_thread, nullptr); s_prefetch_started = false; }
    if (s_bg_surface) {
        bg_tex = texture_from_cairo_surface(s_bg_surface);
        cairo_surface_destroy(s_bg_surface);
        s_bg_surface = nullptr;
    } else {
        bg_tex = load_texture_from_file("/etc/kestrel/background.png");
    }

    // CPU-path texture (GL_TEXTURE_2D, BGRA — software decode fallback)
    glGenTextures(1, &bg_video_tex_);
    glBindTexture(GL_TEXTURE_2D, bg_video_tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    uint32_t black = 0xFF000000u;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, &black);

    // DRM/EGL zero-copy texture (GL_TEXTURE_EXTERNAL_OES — hardware decode)
    glGenTextures(1, &bg_ext_tex_);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, bg_ext_tex_);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // The gallery's recording playback gets one of its own, so the idle
    // background and a recording never share a picture.
    glGenTextures(1, &clip_ext_tex_);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, clip_ext_tex_);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenTextures(1, &live_ext_tex_);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, live_ext_tex_);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Idle title textures — the "fpvOS" wordmark rendered at high resolution so
    // it stays crisp when scaled up on screen (no magnification blur from the
    // small draw_text cache surface). One layer per half of the word, so the
    // accent on "OS" can follow the theme without re-rendering anything.
    // Prefer the copies rendered by the prefetch thread.
    for (int part = 0; part < 2; part++) {
        cairo_surface_t*& src = (part == 0) ? s_title_surface : s_title_os_surface;
        cairo_surface_t* surf = src;
        src = nullptr;
        if (!surf) { // prefetch unused/failed — render synchronously now
            surf = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, s_title_w, s_title_h);
            render_idle_title(surf, part);
        }

        unsigned char* data = cairo_image_surface_get_data(surf);
        GLuint& tex = (part == 0) ? idle_title_tex_ : idle_title_os_tex_;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, s_title_w, s_title_h, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, data);
        idle_title_w_ = s_title_w;
        idle_title_h_ = s_title_h;
        cairo_surface_destroy(surf);
    }

    if (bg_player_ && bg_video_enabled) bg_player_->start();
}

GLuint OSD::load_texture_from_file(const char* filename) {
    cairo_surface_t *surface = cairo_image_surface_create_from_png(filename);
    if (cairo_surface_status(surface) != CAIRO_STATUS_SUCCESS) {
        fprintf(stderr, "Failed to load PNG file %s to Cairo surface\n", filename);
        return 0;
    }

    int w = cairo_image_surface_get_width(surface);
    int h = cairo_image_surface_get_height(surface);
    unsigned char* data = cairo_image_surface_get_data(surface);

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

    cairo_surface_destroy(surface);
    return tex;
}

static const char* v_shader_str = 
    "attribute vec3 pos;\n"
    "attribute vec2 uv;\n"
    "attribute float v_alpha_factor;\n" 
    "varying vec2 v_uv;\n"
    "varying float v_alpha_varying;\n" 
    "uniform mat4 mvp;\n"
    "void main() {\n"
    "    v_uv = uv;\n"
    "    v_alpha_varying = v_alpha_factor;\n" 
    "    gl_Position = mvp * vec4(pos, 1.0);\n"
    "}\n";

static const char* f_shader_str = 
    "precision mediump float;\n"
    "varying vec2 v_uv;\n"
    "varying float v_alpha_varying;\n" 
    "uniform sampler2D tex;\n"
    "uniform float alpha;\n"
    "uniform vec4 color;\n"
    "uniform bool is_text;\n"
    "uniform bool use_shading;\n"
    "void main() {\n"
    "    if (is_text) {\n"
    "        vec4 t = texture2D(tex, v_uv).bgra;\n" 
    "        gl_FragColor = t * color * alpha * v_alpha_varying;\n" 
    "    } else {\n"
    "        float final_alpha = alpha;\n"
    "        vec3 final_color = color.rgb;\n"
    "        \n"
    "        if (use_shading) {\n"
    "            float dist = distance(v_uv, vec2(0.5));\n"
    "            float shade = 1.0 - smoothstep(0.0, 0.5, dist);\n"
    "            final_color = mix(color.rgb, color.rgb + 0.1, shade);\n"
    "            final_alpha = mix(alpha * 0.4, alpha, shade);\n"
    "        }\n"
    "        \n"
    "        gl_FragColor = vec4(final_color, final_alpha) * alpha * v_alpha_varying;\n" 
    "    }\n"
    "}\n";

void OSD::init_shaders() {
    auto compile = [](GLenum type, const char* src) {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, NULL);
        glCompileShader(s);
        // Check compile log
        GLint success;
        glGetShaderiv(s, GL_COMPILE_STATUS, &success);
        if (!success) {
            char log[512];
            glGetShaderInfoLog(s, 512, NULL, log);
            fprintf(stderr, "Shader Compile Error: %s\n", log);
        }
        return s;
    };
    
    GLuint vs = compile(GL_VERTEX_SHADER, v_shader_str);
    GLuint fs = compile(GL_FRAGMENT_SHADER, f_shader_str);
    
    shader_program = glCreateProgram();
    glAttachShader(shader_program, vs);
    glAttachShader(shader_program, fs);
    glLinkProgram(shader_program);
    glUseProgram(shader_program);

    // Cache hot-path uniform/attribute locations once (see osd.hpp). A local
    // alias `sp` is used so the mechanical replacement of the
    // glGet*Location(shader_program, ...) call sites elsewhere leaves these
    // one-time lookups intact.
    {
        GLuint sp = shader_program;
        u_mvp_         = glGetUniformLocation(sp, "mvp");
        u_is_text_     = glGetUniformLocation(sp, "is_text");
        u_use_shading_ = glGetUniformLocation(sp, "use_shading");
        u_color_       = glGetUniformLocation(sp, "color");
        u_alpha_       = glGetUniformLocation(sp, "alpha");
        u_tex_         = glGetUniformLocation(sp, "tex");
        a_pos_          = glGetAttribLocation(sp, "pos");
        a_uv_           = glGetAttribLocation(sp, "uv");
        a_alpha_factor_ = glGetAttribLocation(sp, "v_alpha_factor");
    }

    glGenBuffers(1, &vbo);
    
    // Set default value for the new attribute so existing draw calls (which don't enable it) work fine
    GLint alpha_loc = a_alpha_factor_;
    if (alpha_loc != -1) {
        glVertexAttrib1f(alpha_loc, 1.0f);
    }

    // ---- Warp-zoom transition shader ----
    static const char* warp_vert =
        "attribute vec3 pos;\n"
        "attribute vec2 uv;\n"
        "varying vec2 v_uv;\n"
        "uniform mat4 mvp;\n"
        "void main() {\n"
        "    v_uv = uv;\n"
        "    gl_Position = mvp * vec4(pos, 1.0);\n"
        "}\n";

    // t=0: full frame, no distortion
    // t=1: fully zoomed / barrel-distorted / transparent → reveals FPV beneath
    //
    // Distortion model:
    //   - Centre zooms in hard (× up to 9)
    //   - Horizontal edges barrel outward much more than vertical
    //     → sides appear to stretch/warp outward as the centre rushes through
    //   - Clamp-to-edge instead of black so extreme UV wraps smear rather than cut
    static const char* warp_frag =
        "precision mediump float;\n"
        "varying vec2 v_uv;\n"
        "uniform sampler2D tex;\n"
        "uniform float warp_t;\n"
        "void main() {\n"
        "    float t2   = warp_t * warp_t;\n"
        "    float zoom = 1.0 + t2 * 8.0;\n"
        // Asymmetric barrel: horizontal ×20, vertical ×4
        // At screen edge (u=±0.5): horizontal expansion ratio ≈ (1+20×0.25)/zoom
        // >> vertical (1+4×0.25)/zoom → sides visibly stretch outward
        "    float bx = t2 * 20.0;\n"
        "    float by = t2 *  4.0;\n"
        "    vec2 uv = v_uv - 0.5;\n"
        "    uv.x = uv.x * (1.0 + bx * uv.x * uv.x) / zoom;\n"
        "    uv.y = uv.y * (1.0 + by * uv.y * uv.y) / zoom;\n"
        // Clamp to [0,1] — edge pixels smear rather than going black
        "    uv = clamp(uv + 0.5, 0.0, 1.0);\n"
        // Chromatic aberration — BGRA texture: .b=R, .g=G, .r=B
        "    float fringe = t2 * 0.04;\n"
        "    float r = texture2D(tex, clamp(uv + vec2(fringe, 0.0), 0.0, 1.0)).b;\n"
        "    float g = texture2D(tex, uv).g;\n"
        "    float b = texture2D(tex, clamp(uv - vec2(fringe, 0.0), 0.0, 1.0)).r;\n"
        "    float a = texture2D(tex, uv).a;\n"
        "    float alpha = max(0.0, 1.0 - t2 * 1.35);\n"
        "    gl_FragColor = vec4(r, g, b, a) * alpha;\n"
        "}\n";

    auto compile_s = [](GLenum type, const char* src) {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, NULL);
        glCompileShader(s);
        GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[256]; glGetShaderInfoLog(s, 256, NULL, log);
            fprintf(stderr, "[warp shader] %s\n", log);
        }
        return s;
    };
    GLuint wvs = compile_s(GL_VERTEX_SHADER,   warp_vert);
    GLuint wfs = compile_s(GL_FRAGMENT_SHADER, warp_frag);
    warp_shader_prog_ = glCreateProgram();
    glAttachShader(warp_shader_prog_, wvs);
    glAttachShader(warp_shader_prog_, wfs);
    glLinkProgram(warp_shader_prog_);
    GLint wok; glGetProgramiv(warp_shader_prog_, GL_LINK_STATUS, &wok);
    if (!wok) {
        char log[256]; glGetProgramInfoLog(warp_shader_prog_, 256, NULL, log);
        fprintf(stderr, "[warp shader link] %s\n", log);
        glDeleteProgram(warp_shader_prog_);
        warp_shader_prog_ = 0;
    }
    glUseProgram(shader_program); // restore main program

    // ---- EGL external-texture shaders (NV12 zero-copy, DRM_PRIME path) ----
    // These use samplerExternalOES so the Mali GPU converts NV12→RGB in hardware.

    static const char* ext_vert =
        "attribute vec3 pos;\n"
        "attribute vec2 uv;\n"
        "varying vec2 v_uv;\n"
        "uniform mat4 mvp;\n"
        "void main() {\n"
        "    v_uv = uv;\n"
        "    gl_Position = mvp * vec4(pos, 1.0);\n"
        "}\n";

    // Plain full-opacity draw (BACKGROUND state)
    static const char* ext_frag_plain =
        "#extension GL_OES_EGL_image_external : require\n"
        "precision mediump float;\n"
        "varying vec2 v_uv;\n"
        "uniform samplerExternalOES tex;\n"
        "void main() {\n"
        "    gl_FragColor = texture2D(tex, v_uv);\n"
        "}\n";

    // Warp-zoom + chromatic aberration (TRANSITION state)
    // samplerExternalOES on Mali outputs standard RGB — no .bgra swizzle needed.
    // Same distortion model as warp_frag above.
    static const char* ext_frag_warp =
        "#extension GL_OES_EGL_image_external : require\n"
        "precision mediump float;\n"
        "varying vec2 v_uv;\n"
        "uniform samplerExternalOES tex;\n"
        "uniform float warp_t;\n"
        "void main() {\n"
        "    float t2   = warp_t * warp_t;\n"
        "    float zoom = 1.0 + t2 * 8.0;\n"
        "    float bx = t2 * 20.0;\n"
        "    float by = t2 *  4.0;\n"
        "    vec2 uv = v_uv - 0.5;\n"
        "    uv.x = uv.x * (1.0 + bx * uv.x * uv.x) / zoom;\n"
        "    uv.y = uv.y * (1.0 + by * uv.y * uv.y) / zoom;\n"
        "    uv = clamp(uv + 0.5, 0.0, 1.0);\n"
        // samplerExternalOES → standard RGB (NV12→RGB done in hardware by Mali)
        "    float fringe = t2 * 0.04;\n"
        "    float r = texture2D(tex, clamp(uv + vec2(fringe, 0.0), 0.0, 1.0)).r;\n"
        "    float g = texture2D(tex, uv).g;\n"
        "    float b = texture2D(tex, clamp(uv - vec2(fringe, 0.0), 0.0, 1.0)).b;\n"
        "    float alpha = max(0.0, 1.0 - t2 * 1.35);\n"
        "    gl_FragColor = vec4(r, g, b, 1.0) * alpha;\n"
        "}\n";

    auto build_prog = [&](const char* vs_src, const char* fs_src) -> GLuint {
        GLuint vs = compile_s(GL_VERTEX_SHADER,   vs_src);
        GLuint fs = compile_s(GL_FRAGMENT_SHADER, fs_src);
        GLuint p  = glCreateProgram();
        glAttachShader(p, vs); glAttachShader(p, fs);
        glLinkProgram(p);
        GLint ok; glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[256]; glGetProgramInfoLog(p, 256, NULL, log);
            fprintf(stderr, "[ext shader link] %s\n", log);
            glDeleteProgram(p); return 0;
        }
        return p;
    };
    bg_ext_shader_   = build_prog(ext_vert, ext_frag_plain);
    warp_ext_shader_ = build_prog(ext_vert, ext_frag_warp);
    // A texture of premultiplied RGBA as it is, for the gallery's copy of the
    // screen. (The main program reads its textures as BGRA.)
    static const char* snap_frag =
        "precision mediump float;\n"
        "varying vec2 v_uv;\n"
        "uniform sampler2D tex;\n"
        "uniform float alpha;\n"
        "void main() {\n"
        "    gl_FragColor = texture2D(tex, v_uv) * alpha;\n"
        "}\n";
    gallery_snap_shader_ = build_prog(ext_vert, snap_frag);
    // Flat colour per vertex (premultiplied, as the OSD plane holds it): the stats charts.
    static const char* flat_vert =
        "attribute vec2 pos;\n"
        "attribute vec4 col;\n"
        "uniform mat4 mvp;\n"
        "varying vec4 v_col;\n"
        "void main() {\n"
        "    gl_Position = mvp * vec4(pos, 0.0, 1.0);\n"
        "    v_col = col;\n"
        "}\n";
    static const char* flat_frag =
        "precision mediump float;\n"
        "varying vec4 v_col;\n"
        "void main() { gl_FragColor = v_col; }\n";
    gallery_flat_shader_ = build_prog(flat_vert, flat_frag);
    if (gallery_flat_shader_) {
        flat_mvp_ = glGetUniformLocation(gallery_flat_shader_, "mvp");
        flat_pos_ = glGetAttribLocation(gallery_flat_shader_, "pos");
        flat_col_ = glGetAttribLocation(gallery_flat_shader_, "col");
    }

    // Load EGL DMA-BUF extension function pointers
    pfn_eglCreateImageKHR = (PFNEGLCREATEIMAGEKHRPROC)
        eglGetProcAddress("eglCreateImageKHR");
    pfn_eglDestroyImageKHR = (PFNEGLDESTROYIMAGEKHRPROC)
        eglGetProcAddress("eglDestroyImageKHR");
    pfn_glEGLImageTargetTexture2DOES = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)
        eglGetProcAddress("glEGLImageTargetTexture2DOES");

    {
        const char* ext = eglQueryString(display, EGL_EXTENSIONS);
        if (ext && strstr(ext, "EGL_ANDROID_native_fence_sync")) {
            pfn_eglCreateSyncKHR  = (PFNEGLCREATESYNCKHRPROC)eglGetProcAddress("eglCreateSyncKHR");
            pfn_eglDestroySyncKHR = (PFNEGLDESTROYSYNCKHRPROC)eglGetProcAddress("eglDestroySyncKHR");
            pfn_eglDupNativeFenceFDANDROID =
                (PFNEGLDUPNATIVEFENCEFDANDROIDPROC)eglGetProcAddress("eglDupNativeFenceFDANDROID");
            if (!pfn_eglDestroySyncKHR || !pfn_eglDupNativeFenceFDANDROID) pfn_eglCreateSyncKHR = nullptr;
        }
        if (prof::enabled())
            printf("[OSD] prof: native fences %s\n", pfn_eglCreateSyncKHR ? "available" : "NOT available");
    }

    if (pfn_eglCreateImageKHR && pfn_glEGLImageTargetTexture2DOES)
        fprintf(stderr, "[OSD] EGL DMA-BUF import available — zero-copy bg video enabled\n");
    else
        fprintf(stderr, "[OSD] EGL DMA-BUF import NOT available — CPU fallback\n");

    glUseProgram(shader_program);
}

void OSD::draw_panel(float x, float y, float w, float h, float alpha, float r, float g, float b) {
    float verts[] = {
        x,   y,   0,   0, 0,
        x+w, y,   0,   1, 0,
        x,   y+h, 0,   0, 1,
        x+w, y+h, 0,   1, 1
    };
    
    glUniform1i(u_is_text_, 0);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, r, g, b, 0.85); 
    glUniform1f(u_alpha_, alpha);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);
    
    GLint pos_loc = a_pos_;
    glEnableVertexAttribArray(pos_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);
    
    GLint uv_loc = a_uv_;
    glEnableVertexAttribArray(uv_loc);
    glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void OSD::draw_icon(GLuint tex, float x, float y, float w, float h, float r, float g, float b, float alpha) {
    if (tex == 0) return;
    float verts[] = {
        x,   y,   0,   0, 1,
        x+w, y,   0,   1, 1,
        x,   y+h, 0,   0, 0,
        x+w, y+h, 0,   1, 0
    };
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);
    glUniform1i(u_tex_, 0);
    glUniform1i(u_is_text_, 1);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, r, g, b, 1.0); 
    glUniform1f(u_alpha_, alpha);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);
    
    GLint pos_loc = a_pos_;
    glEnableVertexAttribArray(pos_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);
    
    GLint uv_loc = a_uv_;
    glEnableVertexAttribArray(uv_loc);
    glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

// Draw a texture over an arbitrary quad (bl, br, tr, tl) rather than an
// axis-aligned rect, so text can be warped by the menu's perspective: the
// bottom edge of a line ends up wider than its top edge, matching the panel.
void OSD::draw_icon_quad(GLuint tex, math::Vec2 bl, math::Vec2 br,
                         math::Vec2 tr, math::Vec2 tl, float r, float g, float b) {
    if (tex == 0) return;
    float verts[] = {
        bl.x, bl.y, 0,   0, 1,
        br.x, br.y, 0,   1, 1,
        tl.x, tl.y, 0,   0, 0,
        tr.x, tr.y, 0,   1, 0
    };

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);
    glUniform1i(u_tex_, 0);
    glUniform1i(u_is_text_, 1);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, r, g, b, 1.0);
    glUniform1f(u_alpha_, 1.0);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);

    GLint pos_loc = a_pos_;
    glEnableVertexAttribArray(pos_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);

    GLint uv_loc = a_uv_;
    glEnableVertexAttribArray(uv_loc);
    glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void OSD::draw_trapezoid(float x, float y, float w, float h, float top_scale, float slant, float alpha, float r, float g, float b) {
    float top_w = w * top_scale;
    float inset = (w - top_w) / 2.0f;
    
    // Slant shifts the whole top row
    float slant_x = h * slant;

    float verts[] = {
        x,           y,   0,   0, 0,
        x+w,         y,   0,   1, 0,
        x+inset+slant_x,       y+h, 0,   0, 1,
        x+w-inset+slant_x,     y+h, 0,   1, 1
    };
    
    glUniform1i(u_is_text_, 0);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, r, g, b, 0.85); 
    glUniform1f(u_alpha_, alpha);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);
    
    GLint pos_loc = a_pos_;
    glEnableVertexAttribArray(pos_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);
    
    GLint uv_loc = a_uv_;
    glEnableVertexAttribArray(uv_loc);
    glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void OSD::draw_poly(const std::vector<math::Vec2>& points, float alpha, float r, float g, float b) {
    if (points.size() < 3) return;
    
    std::vector<float> verts;
    verts.reserve(points.size() * 3 * 5); 
    
    math::Vec2 center = {0,0};
    for(const auto& p : points) { center.x += p.x; center.y += p.y; }
    center.x /= points.size(); center.y /= points.size();

    for (size_t i = 0; i < points.size(); ++i) {
        size_t next = (i + 1) % points.size();
        verts.insert(verts.end(), {center.x, center.y, 0, 0.5f, 0.5f});
        verts.insert(verts.end(), {points[i].x, points[i].y, 0, 0, 0});
        verts.insert(verts.end(), {points[next].x, points[next].y, 0, 1, 1});
    }

    glUniform1i(u_is_text_, 0);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, r, g, b, 1.0f);
    glUniform1f(u_alpha_, alpha);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_DYNAMIC_DRAW);

    GLint pos_loc = a_pos_;
    glEnableVertexAttribArray(pos_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);

    GLint uv_loc = a_uv_;
    glEnableVertexAttribArray(uv_loc);
    glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

    glDrawArrays(GL_TRIANGLES, 0, points.size() * 3);
}

// The video link warning's glow: a rail along the top of the picture and one
// along the bottom, each a solid line at the frame edge whose light falls off
// quickly and then trails away inboard. The sides are left alone - that is
// where the canopy's wings sit - and each rail fades out before the corners
// rather than stopping square, so it reads as light, not as a painted bar.
void OSD::draw_rail_glow(float fw, float fh, float depth, float alpha,
                         float r, float g, float b) {
    if (depth <= 0.0f || alpha <= 0.0f) return;
    // Down the rail, from the edge inboard: x depth, x alpha.
    static const float kInset[] = { 0.0f, 0.04f, 0.12f, 0.45f, 1.0f };
    static const float kAlpha[] = { 1.0f, 1.00f, 0.55f, 0.18f, 0.0f };
    // Along it, as a fraction of fw: the rail spans 88% of the width and
    // ramps up over its outer 14% at each end.
    static const float kX[]  = { -0.88f, -0.6336f, 0.6336f, 0.88f };
    static const float kXa[] = {  0.0f,   1.0f,    1.0f,    0.0f  };
    const int nv = (int)(sizeof(kInset) / sizeof(kInset[0]));
    const int nh = (int)(sizeof(kX) / sizeof(kX[0]));

    std::vector<float> v;
    v.reserve(2 * (nv - 1) * (nh - 1) * 6 * 6);
    for (int side = 0; side < 2; side++) {
        const float edge = side ? fh : -fh, dir = side ? -1.0f : 1.0f;
        for (int k = 0; k + 1 < nv; k++) {
            float y0 = edge + dir * kInset[k] * depth;
            float y1 = edge + dir * kInset[k + 1] * depth;
            for (int c = 0; c + 1 < nh; c++) {
                float x0 = kX[c] * fw, x1 = kX[c + 1] * fw;
                float a00 = alpha * kAlpha[k]     * kXa[c];
                float a10 = alpha * kAlpha[k]     * kXa[c + 1];
                float a01 = alpha * kAlpha[k + 1] * kXa[c];
                float a11 = alpha * kAlpha[k + 1] * kXa[c + 1];
                const float q[6][3] = {{x0, y0, a00}, {x1, y0, a10}, {x1, y1, a11},
                                       {x0, y0, a00}, {x1, y1, a11}, {x0, y1, a01}};
                for (int m = 0; m < 6; m++)
                    v.insert(v.end(), {q[m][0], q[m][1], 0.0f, 0.5f, 0.5f, q[m][2]});
            }
        }
    }

    glUniform1i(u_is_text_, 0);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, r, g, b, 1.0f);
    glUniform1f(u_alpha_, 1.0f);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(a_pos_);
    glVertexAttribPointer(a_pos_, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);
    glEnableVertexAttribArray(a_uv_);
    glVertexAttribPointer(a_uv_, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    if (a_alpha_factor_ != -1) {
        glEnableVertexAttribArray(a_alpha_factor_);
        glVertexAttribPointer(a_alpha_factor_, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(5 * sizeof(float)));
    }
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(v.size() / 6));
    if (a_alpha_factor_ != -1) {
        glDisableVertexAttribArray(a_alpha_factor_);
        glVertexAttrib1f(a_alpha_factor_, 1.0f);
    }
}

int OSD::link_warn_hold(int raw) {
    // Two seconds: longer than the radio takes to try the next rung up and
    // fall back, so a link bouncing between two rungs holds the worse colour
    // instead of flashing between them.
    constexpr uint64_t kStepDownUs = 2000000ULL;
    uint64_t now = get_time_us();
    if (raw >= link_warn_level_) {
        link_warn_level_ = raw;
        link_warn_better_us_ = 0;
    } else if (!link_warn_better_us_) {
        link_warn_better_us_ = now;
    } else if (now - link_warn_better_us_ >= kStepDownUs) {
        link_warn_level_ = raw;
        link_warn_better_us_ = 0;
    }
    return link_warn_level_;
}

// Helper to draw a glowing line by stacking transparent lines
void OSD::draw_glow_line(const std::vector<math::Vec2>& points, float width, float alpha, float r, float g, float b) {
    if (points.size() < 2) return;
    
    // Disable shading logic for lines
    glUniform1i(u_use_shading_, 0);

    // Compute normals at each point
    std::vector<math::Vec2> normals(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        float dx, dy;
        if (i == 0) {
            dx = points[1].x - points[0].x;
            dy = points[1].y - points[0].y;
        } else if (i == points.size() - 1) {
            dx = points[i].x - points[i-1].x;
            dy = points[i].y - points[i-1].y;
        } else {
            float dx1 = points[i].x - points[i-1].x;
            float dy1 = points[i].y - points[i-1].y;
            float dx2 = points[i+1].x - points[i].x;
            float dy2 = points[i+1].y - points[i].y;
            float len1 = sqrtf(dx1*dx1 + dy1*dy1);
            float len2 = sqrtf(dx2*dx2 + dy2*dy2);
            if (len1 > 0) { dx1 /= len1; dy1 /= len1; }
            if (len2 > 0) { dx2 /= len2; dy2 /= len2; }
            dx = dx1 + dx2;
            dy = dy1 + dy2;
        }
        float len = sqrtf(dx*dx + dy*dy);
        if (len > 0) {
            normals[i].x = -dy / len;
            normals[i].y = dx / len;
        } else {
            normals[i].x = 0.0f;
            normals[i].y = 1.0f;
        }
    }

    // Render 3 passes for glow (each pass is a SINGLE draw call)
    for (int j = 0; j < 3; j++) {
        float w_scale = 1.0f + j * 2.0f;
        float w = width * w_scale * 0.001f;
        float a = alpha / (j * 2.0f + 1.0f);
        if (j == 0) a = alpha;

        std::vector<float> verts;
        verts.reserve(points.size() * 2 * 5);

        for (size_t i = 0; i < points.size(); i++) {
            float nx = normals[i].x * w;
            float ny = normals[i].y * w;
            
            verts.push_back(points[i].x + nx);
            verts.push_back(points[i].y + ny);
            verts.push_back(0.0f);
            verts.push_back(0.0f);
            verts.push_back(0.0f);
            
            verts.push_back(points[i].x - nx);
            verts.push_back(points[i].y - ny);
            verts.push_back(0.0f);
            verts.push_back(0.0f);
            verts.push_back(0.0f);
        }

        glUniform1i(u_is_text_, 0);
        glUniform4f(u_color_, r, g, b, a);
        glUniform1f(u_alpha_, a);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_DYNAMIC_DRAW);
        GLint pos_loc = a_pos_;
        glEnableVertexAttribArray(pos_loc);
        glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);
        GLint uv_loc = a_uv_;
        glEnableVertexAttribArray(uv_loc);
        glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
        glDrawArrays(GL_TRIANGLE_STRIP, 0, (int)verts.size() / 5);
    }
}

void OSD::draw_hex_panel(float x, float y, float w, float h, float alpha, float r, float g, float b, bool outline, int num_h, int num_v, float grid_y, float grid_h, bool uniform_x) {
    if (!outline) return; // Background now handled inside the outline block for integration
    
    // side estimation: left wing x is negative, right wing x is positive
    bool is_left = (x < 0);
    
    auto get_pos = [&](float tx, float ty) {
        float ratio_y = (ty - y) / h;
        float span_scale = 1.0f - (ratio_y * 0.3f);
        float sw = w * span_scale;
        float sx = x + (w - sw) / 2.0f;
        float ratio_x = (tx - x) / w;
        return math::Vec2{sx + ratio_x * sw, ty};
    };

    // 4. Smooth Gradient Background
    // We draw a single quad (or two depending on wing shape) with vertex colors for smooth interpolation.
    // Left Wing: Fades out towards Top-Right. Max alpha at Bottom-Left.
    // Right Wing: Fades out towards Top-Left. Max alpha at Bottom-Right.
    
    // Coordinates
    float x1 = x;
    float x2 = x + w;
    float y1 = y;
    float y2 = y + h;
    
    // Get actual polygon corners based on trapezoid shape
    math::Vec2 p_bl = get_pos(x1, y1);
    math::Vec2 p_br = get_pos(x2, y1);
    math::Vec2 p_tr = get_pos(x2, y2);
    math::Vec2 p_tl = get_pos(x1, y2);

    float max_a = alpha * 0.85f;

    // Vertex Alphas based on Wing Side
    float a_bl, a_br, a_tr, a_tl;
    
    if (is_left) {
        // Max at Bottom-Left (p_bl)
        a_bl = max_a;
        a_br = max_a * 0.4f;
        a_tr = 0.0f;          // Fade out top-right
        a_tl = max_a * 0.4f;
    } else {
        // Max at Bottom-Right (p_br)
        a_bl = max_a * 0.4f;
        a_br = max_a;
        a_tr = max_a * 0.4f;
        a_tl = 0.0f;          // Fade out top-left
    }
    
    // Draw directly using gl buffer to support per-vertex color/alpha if shader supports it.
    // Our 'draw_poly' uses uniform alpha. We need a way to pass vertex attributes or just standard lerp.
    // Since our simple shader uses uniform alpha, drawing a smooth gradient requires modifying the drawing logic 
    // OR just drawing a bunch of overlapping quads which is what caused the blockiness.
    // Wait, the shader 'f_shader_str' currently handles 'v_uv' but uses uniform color/alpha.
    
    // Hack: We can simulate a gradient by drawing many thin overlapping slices? No that's what we just removed.
    // Better: Update the shader to support vertex colors or use UVs to create a gradient.
    // The shader has v_uv. We can use UVs to generate the gradient in the Fragment Shader.
    // UVs are (0,0) to (1,1).
    
    // Let's use 'draw_poly' but rely on a new shader uniform or just custom implementation here?
    // 'draw_poly' sets UVs: (0,0) (0,1) (1,1)... wait, draw_poly sets UVs:
    // center (0,0.5), p1(0,0), pnext(1,1)?? No draw_poly sets weird UVs.
    
    // Look at 'draw_trapezoid' or 'draw_panel' - they set specific UVs.
    // Let's implement 'draw_gradient_quad' locally here or use a custom block.
    
    // Actually, 'draw_panel' sets UVs (0,0)..(1,1) correctly?
    // In 'draw_hex_panel':
    // We already calculated p_bl, p_br, p_tr, p_tl. 
    // We can feed these into a VBO.
    // NOTE: Our fragment shader "f_shader_str" lines 215-217 has some "use_shading" logic.
    // If we enable 'use_shading', it creates a radial gradient from center 0.5. This isn't what we want.
    
    // Simplest fix without changing shader:
    // Create a dense mesh (e.g. 10x10) for the background so vertex interpolation (if we had vertex colors) works? 
    // No, we only have uniform color.
    
    // Real fix: Update Shader to support a 'gradient_mode'.
    // BUT modifying shader might break other things or require recompile of everything.
    
    // Alternate: Use 100s of very small overlapping transparent layers? Expensive.
    
    auto smoothstep = [](float edge0, float edge1, float x) {
        float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    };


    // BATCHED SMOOTH BACKGROUND RENDER (Gouraud Shading)
    // We generate a mesh where each vertex has its own calculated alpha factor.
    // This removes the "squares" artifact by letting GL interpolate alpha.
    
    int bg_steps = 20; // Enough for smooth interpolation
    std::vector<float> bg_verts;
    bg_verts.reserve(bg_steps * bg_steps * 6 * 6); // x,y,z,u,v,a * 6 verts/quad
    
    glUniform1i(u_is_text_, 0);
    glUniform1i(u_use_shading_, 0);
    // Use the base color, alpha will be modulated by the attribute
    glUniform4f(u_color_, r, g, b, 1.0f);
    glUniform1f(u_alpha_, alpha); // Base alpha, attribute multiplies this

    for (int j = 0; j < bg_steps; j++) {
        for (int i = 0; i < bg_steps; i++) {
            float rx1 = (float)i/bg_steps;
            float rx2 = (float)(i+1)/bg_steps;
            float ry1 = (float)j/bg_steps;
            float ry2 = (float)(j+1)/bg_steps;

            float x1_l = x + rx1 * w;
            float x2_l = x + rx2 * w;
            float y1_l = y + ry1 * h;
            float y2_l = y + ry2 * h;

            math::Vec2 p1 = get_pos(x1_l, y1_l); // BL
            math::Vec2 p2 = get_pos(x2_l, y1_l); // BR
            math::Vec2 p3 = get_pos(x2_l, y2_l); // TR
            math::Vec2 p4 = get_pos(x1_l, y2_l); // TL

            // Helper to calc alpha factor for a logical coordinate (0..1)
            auto calc_a = [&](float rx, float ry) {
                float fade_x = uniform_x ? 0.5f : (is_left ? (1.0f - rx) : rx);
                float fade_top = 1.0f - ry;
                return 1.25f * fade_x * fade_top;
            };

            float a1 = calc_a(rx1, ry1);
            float a2 = calc_a(rx2, ry1);
            float a3 = calc_a(rx2, ry2);
            float a4 = calc_a(rx1, ry2);

            // Quad as 2 triangles: p1-p2-p3, p1-p3-p4
            auto push = [&](math::Vec2 p, float a) {
                bg_verts.push_back(p.x); bg_verts.push_back(p.y); bg_verts.push_back(0); // pos
                bg_verts.push_back(0.5f); bg_verts.push_back(0.5f); // dummy UV
                bg_verts.push_back(a); // alpha factor
            };

            push(p1, a1); push(p2, a2); push(p3, a3);
            push(p1, a1); push(p3, a3); push(p4, a4);
        }
    }

    if (!bg_verts.empty()) {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, bg_verts.size() * sizeof(float), bg_verts.data(), GL_DYNAMIC_DRAW);

        GLint pos_loc = a_pos_;
        glEnableVertexAttribArray(pos_loc);
        glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);

        GLint uv_loc = a_uv_;
        glEnableVertexAttribArray(uv_loc);
        glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

        GLint alpha_loc = a_alpha_factor_;
        if (alpha_loc != -1) {
            glEnableVertexAttribArray(alpha_loc);
            glVertexAttribPointer(alpha_loc, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(5 * sizeof(float)));
        }

        glDrawArrays(GL_TRIANGLES, 0, bg_verts.size() / 6);

        if (alpha_loc != -1) {
            glDisableVertexAttribArray(alpha_loc);
            glVertexAttrib1f(alpha_loc, 1.0f); // Reset to default
        }
    }

    // 2. Converging Grid
    float grid_a_base = alpha * 0.40f; // Toned down grid (0.45 -> 0.40)
    float gy = (grid_y >= 0.0f) ? grid_y : y;
    float gh = (grid_h >= 0.0f) ? grid_h : h;

    // Horizontal Grid Lines
    for (int i = 0; i <= num_h; i++) {
        float ry = (float)i / num_h;
        float ly = gy + ry * gh;
        
        // Horizontal lines fade towards the far side and top
        int h_steps = 10;
        for (int k = 0; k < h_steps; k++) {
            float rx1 = (float)k / h_steps;
            float rx2 = (float)(k+1) / h_steps;
            math::Vec2 p1 = get_pos(x + rx1 * w, ly);
            math::Vec2 p2 = get_pos(x + rx2 * w, ly);
            
            float mid_rx = (rx1 + rx2) * 0.5f;
            float fade_x = uniform_x ? 0.5f : (is_left ? (1.0f - mid_rx) : mid_rx);
            
            float fade_top = 1.0f - ry;
            float a = grid_a_base * fade_x * fade_top;
            
            if (a > 0.01f) {
                draw_poly({p1, {p1.x, p1.y + 0.003f}, {p2.x, p2.y + 0.003f}, p2}, a, 1.0f, 1.0f, 1.0f);
            }
        }
    }

    // Vertical Grid Lines
    for (int i = 0; i <= num_v; i++) {
        float rx = (float)i / num_v;
        float lx = x + rx * w;
        
        int v_segments = 8;
        for (int j = 0; j < v_segments; j++) {
            float ry1 = (float)j / v_segments;
            float ry2 = (float)(j+1) / v_segments;
            math::Vec2 p1 = get_pos(lx, gy + ry1 * gh);
            math::Vec2 p2 = get_pos(lx, gy + ry2 * gh);
            float mid_ry = (ry1 + ry2) * 0.5f;

            float fade_x = uniform_x ? 0.5f : (is_left ? (1.0f - rx) : rx);
            
            float fade_top = 1.0f - mid_ry;
            float a = grid_a_base * fade_x * fade_top;
            
            if (a > 0.01f) {
                draw_poly({p1, {p1.x + 0.002f, p1.y}, {p2.x + 0.002f, p2.y}, p2}, a, 1.0f, 1.0f, 1.0f);
            }
        }
    }
}

void OSD::draw_arc(float cx, float cy, float r_in, float r_out, float start_angle, float end_angle, float alpha, float r, float g, float b) {
    int segments = 24;
    float step = (end_angle - start_angle) / segments;
    
    std::vector<float> verts;
    verts.reserve(segments * 6 * 5);

    for (int i = 0; i < segments; i++) {
        float a1 = start_angle + i * step;
        float a2 = start_angle + (i + 1) * step;
        
        float cos1 = cosf(a1), sin1 = sinf(a1);
        float cos2 = cosf(a2), sin2 = sinf(a2);
        
        float x1_in = cx + r_in * cos1, y1_in = cy + r_in * sin1;
        float x1_out = cx + r_out * cos1, y1_out = cy + r_out * sin1;
        float x2_in = cx + r_in * cos2, y2_in = cy + r_in * sin2;
        float x2_out = cx + r_out * cos2, y2_out = cy + r_out * sin2;
        
        verts.insert(verts.end(), {x1_in, y1_in, 0, 0, 0});
        verts.insert(verts.end(), {x1_out, y1_out, 0, 1, 0});
        verts.insert(verts.end(), {x2_in, y2_in, 0, 0, 1});
        
        verts.insert(verts.end(), {x1_out, y1_out, 0, 1, 0});
        verts.insert(verts.end(), {x2_out, y2_out, 0, 1, 1});
        verts.insert(verts.end(), {x2_in, y2_in, 0, 0, 1});
    }

    glUniform1i(u_is_text_, 0);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, r, g, b, 0.85);
    glUniform1f(u_alpha_, alpha);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_DYNAMIC_DRAW);

    GLint pos_loc = a_pos_;
    glEnableVertexAttribArray(pos_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);

    GLint uv_loc = a_uv_;
    glEnableVertexAttribArray(uv_loc);
    glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

    glDrawArrays(GL_TRIANGLES, 0, segments * 6);
}

void OSD::draw_grid_arc(float cx, float cy, float r_inner, float r_outer, float start_angle, float end_angle, float alpha) {
    int num_h = 10;
    int num_v = 24;
    
    // 1. Concentric Arcs (Continuous Flow)
    for (int i = 0; i <= num_h; i++) {
        float ratio = (float)i / num_h;
        float r = r_inner + (r_outer - r_inner) * ratio;
        // Exponential fade top-wards
        float a = alpha * 0.25f * (1.0f - ratio * ratio);
        draw_arc(cx, cy, r, r + 0.004f, start_angle, end_angle, a, 1.0f, 1.0f, 1.0f);
    }
    
    // 2. Radial Spokes (Perspective Convergence)
    for (int i = 0; i <= num_v; i++) {
        float ratio = (float)i / num_v;
        float angle = start_angle + (end_angle - start_angle) * ratio;
        float c = cosf(angle), s = sinf(angle);
        
        float rx1 = cx + r_inner * c, ry1 = cy + r_inner * s;
        float rx2 = cx + r_outer * c, ry2 = cy + r_outer * s;
        
        // Slightly fade outer edges and top
        float v_alpha = alpha * 0.12f * (1.0f - (fabsf(ratio - 0.5f) * 1.5f));
        if (v_alpha < 0.01f) continue;

        std::vector<math::Vec2> line = {
            {rx1, ry1}, {rx1 + 0.003f, ry1},
            {rx2 + 0.003f, ry2}, {rx2, ry2}
        };
        glUniform1i(u_use_shading_, 0);
        draw_poly(line, v_alpha, 1.0f, 1.0f, 1.0f);
    }
}

// A cairo context set up exactly as ensure_text_texture renders, for
// measuring strings without rasterising them.
static cairo_t* text_measure_context() {
    static cairo_surface_t* surf = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 1, 1);
    static cairo_t* m = nullptr;
    // A cairo_t's error state is permanent (an invalid UTF-8 string sets it),
    // and a broken one measures everything as zero: start a fresh one.
    if (m && cairo_status(m) != CAIRO_STATUS_SUCCESS) { cairo_destroy(m); m = nullptr; }
    if (!m) {
        m = cairo_create(surf);
        cairo_select_font_face(m, "Chakra Petch SemiBold",
                               CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        cairo_matrix_t fm;
        cairo_matrix_init_scale(&fm, 40.0, 40.0);
        cairo_set_font_matrix(m, &fm);
    }
    return m;
}

void OSD::ensure_text_texture(const char* text) {
    // One lookup, not two: this used to count() here and operator[] again at the
    // end. On a hit, just stamp the entry so eviction knows it is warm.
    auto hit = text_cache.find(text);
    if (hit != text_cache.end()) {
        hit->second.last_used = ++text_cache_clock;
        return;
    }
    prof::count(prof::kCountTextMiss);
    {
        // Sized to the text. Every string used to get a 2048x64 texture and be
        // drawn as a quad of that shape - 32 times wider than tall, for "5" as
        // much as for a sentence - so the GPU blended, and the upload moved,
        // mostly transparent pixels: 512 KB per string. Measure first, then
        // make the surface just wide enough (the glyphs start at x = 2).
        cairo_t* measure = text_measure_context();
        cairo_text_extents_t ext;
        cairo_text_extents(measure, text, &ext);
        double right = std::max(ext.x_advance, ext.x_bearing + ext.width);
        int w = ((int)std::ceil(2.0 + right) + 4 + 15) & ~15;   // 2 px lead-in, 4 px margin, 16-aligned
        if (w < 16)   w = 16;
        if (w > 2048) w = 2048;
        int h = 64;
        cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, w, h);
        cairo_t *cr = cairo_create(surface);
        
        // Chakra Petch, shipped in the rootfs overlay. The HUD and the menu were
        // designed in it; the goggle previously had only DejaVu Sans, so every
        // mockup was drawn in one face and rendered in another - which is most
        // of why the built menu never looked like the picture of it.
        //
        // "Rajdhani" here before it was a font that was never installed, so it
        // silently fell back to DejaVu and nobody noticed for a long time. The
        // family name below is the one fontconfig reports, so a missing font
        // shows up as a visibly wrong face rather than an invisible fallback.
        cairo_select_font_face(cr, "Chakra Petch SemiBold",
                               CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

        // Use shared OSD font configuration
        cairo_matrix_t font_matrix;
        // 0.75 was condensing DejaVu, which is a wide face. Chakra Petch is
        // already narrow by design; condensing it again closes the counters and
        // is exactly what made the menu look cramped next to the mockup.
        cairo_matrix_init_scale(&font_matrix, 1.0, 1.0);
        cairo_matrix_scale(&font_matrix, 40.0, 40.0);    
        cairo_set_font_matrix(cr, &font_matrix);
        
        cairo_text_extents_t extents;
        cairo_text_extents(cr, text, &extents);

        cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
        cairo_move_to(cr, 2, 48); 
        cairo_show_text(cr, text);
        
        cairo_destroy(cr);
        unsigned char* data = cairo_image_surface_get_data(surface);
        
        GLuint tex;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // Clamp: with the texture now ending just past the last glyph, the
        // default REPEAT would filter the left edge into the right one.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        
        // Evict BEFORE inserting so the new entry is never the eviction victim.
        // Pick the least-recently-used entry. The previous version took
        // text_cache.begin() - the lexicographically smallest key - so it
        // preferentially discarded the shortest, hottest strings and paid a full
        // cairo render plus glTexImage2D to rebuild them almost immediately. It
        // also built a std::string per comparison while scanning; comparing the
        // key against the char* directly needs no allocation.
        // We are on the miss path here, so the O(n) scan runs rarely.
        if (text_cache.size() >= 500) {
            auto victim = text_cache.end();
            for (auto it = text_cache.begin(); it != text_cache.end(); ++it) {
                if (it->first == text) continue;   // never evict what we are adding
                if (victim == text_cache.end() ||
                    it->second.last_used < victim->second.last_used)
                    victim = it;
            }
            if (victim != text_cache.end()) {
                glDeleteTextures(1, &victim->second.tex);
                text_cache.erase(victim);
            }
        }

        text_cache[text] = {tex, w, h, (float)extents.x_advance, std::string(text),
                            ++text_cache_clock};
        cairo_surface_destroy(surface);
    }
}

float OSD::text_width(const char* text, float scale) {
    if (!text || !text[0]) return 0.0f;
    // A cached string knows its width; anything else is measured, not
    // rendered - the menu's word wrap tries many strings it never draws.
    auto it = text_cache.find(text);
    if (it != text_cache.end()) return (it->second.text_w / 64.0f) * scale;
    cairo_text_extents_t ext;
    cairo_text_extents(text_measure_context(), text, &ext);
    return ((float)ext.x_advance / 64.0f) * scale;
}

void OSD::draw_text(const char* text, float x, float y, float scale, bool right_align, float r, float g, float b) {
    if (!text || !text[0]) return;
    ensure_text_texture(text);

    auto cit = text_cache.find(text);
    if (cit == text_cache.end()) return;
    auto& cached = cit->second;
    float aspect = (float)cached.w / cached.h;
    
    float tx = x;
    if (right_align) {
        float text_width_units = (cached.text_w / 64.0f) * scale;
        tx = x - text_width_units;
    }
    
    draw_icon(cached.tex, tx, y, scale * aspect, scale, r, g, b);
}

// Same as draw_text, but only the left `reveal` fraction (0..1) of the
// rendered string is drawn - a texture crop, not an alpha fade, so a state
// label sweeps into view left-to-right instead of fading or popping in
// whole. draw_text caches each string as one texture (ensure_text_texture),
// so "revealing" it is just drawing a narrower sub-rect of that same quad
// rather than re-rendering glyph-by-glyph.
void OSD::draw_text_wipe(const char* text, float x, float y, float scale,
                         bool right_align, float r, float g, float b,
                         float reveal) {
    if (!text || !text[0]) return;
    reveal = fmaxf(0.0f, fminf(1.0f, reveal));
    if (reveal <= 0.0f) return;
    ensure_text_texture(text);

    auto cit = text_cache.find(text);
    if (cit == text_cache.end()) return;
    auto& cached = cit->second;

    // ensure_text_texture renders each string with the glyphs starting at
    // x=2 and a few pixels of margin after them, so the texture (w) is a
    // little wider than the word (text_w). Both numbers matter here and they
    // are NOT interchangeable - right-alignment is measured on the glyph
    // advance (as draw_text does), and the quad spans the whole texture.
    const float px_to_units = scale / (float)cached.h;   // 64 texture px == scale
    float tx = x;
    if (right_align) tx = x - cached.text_w * px_to_units;

    if (reveal >= 1.0f) {
        draw_icon(cached.tex, tx, y, (float)cached.w * px_to_units, scale, r, g, b);
        return;
    }

    // Crop to the revealed part of the GLYPHS, not of the texture, so the
    // sweep ends exactly at the last letter.
    const float cut_px = 2.0f + cached.text_w * reveal;  // 2 = cairo's left margin
    const float w  = cut_px * px_to_units;
    const float u1 = cut_px / (float)cached.w;

    // Same vertex/uniform setup as draw_icon, just a narrower quad and a
    // matching narrower U range instead of the full 0..1 texture.
    float verts[] = {
        tx,     y,       0,  0.0f, 1,
        tx + w, y,       0,  u1,   1,
        tx,     y+scale, 0,  0.0f, 0,
        tx + w, y+scale, 0,  u1,   0
    };

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, cached.tex);
    glUniform1i(u_tex_, 0);
    glUniform1i(u_is_text_, 1);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, r, g, b, 1.0);
    glUniform1f(u_alpha_, 1.0);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(a_pos_);
    glVertexAttribPointer(a_pos_, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);
    glEnableVertexAttribArray(a_uv_);
    glVertexAttribPointer(a_uv_, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}


void OSD::draw_stacked_bar(float x, float y, float w, float h, float slant, bool reverse, float val1, float r1, float g1, float b1, float val2, float r2, float g2, float b2, float val3, float r3, float g3, float b3, float val4, float r4, float g4, float b4, float val5, float r5, float g5, float b5, float val6, float r6, float g6, float b6) {
    float seg_w = 0.02f;
    float spacing = 0.005f;
    
    int total_segments = (int)(w / (seg_w + spacing));
    int c1 = (int)(val1 * total_segments);
    int c2 = (int)(val2 * total_segments);
    int c3 = (int)(val3 * total_segments);
    int c4 = (int)(val4 * total_segments);
    int c5 = (int)(val5 * total_segments);
    int c6 = (int)(val6 * total_segments);
    
    if (c1 + c2 + c3 + c4 + c5 + c6 > total_segments) {
        float scale = (float)total_segments / (c1 + c2 + c3 + c4 + c5 + c6);
        c1 *= scale; c2 *= scale; c3 *= scale; c4 *= scale; c5 *= scale; c6 *= scale;
    }

    float cx = reverse ? (x + w - seg_w) : x;
    float step = reverse ? -(seg_w + spacing) : (seg_w + spacing);

    auto draw_seg = [&](int count, float r, float g, float b, float a = 0.9f) {
        if (count <= 0) return;
        for(int i=0; i<count; i++) {
             draw_trapezoid(cx, y, seg_w, h, 1.0f, 0.0f, a, r, g, b);
             cx += step;
        }
    };
    
    draw_seg(c1, r1, g1, b1);
    draw_seg(c2, r2, g2, b2);
    draw_seg(c3, r3, g3, b3);
    draw_seg(c4, r4, g4, b4);
    draw_seg(c5, r5, g5, b5);
    draw_seg(c6, r6, g6, b6);
}

void OSD::draw_signal_bar(float x, float y, float w, float h, float value, float min_val, float max_val) {
    float norm = (value - min_val) / (max_val - min_val);
    if (norm < 0) norm = 0;
    if (norm > 1) norm = 1;

    // Color based on strength
    float r = 1.0f - norm;
    float g = norm;
    float b = 0.2f;
    
    draw_panel(x, y, w * norm, h, 0.9f, r, g, b);
}

// Hand buffers back to GBM - so the GPU may draw into them again - once the
// display can no longer read them: not pending, not in a flip in flight, not
// on screen. It used to be "keep the newest two", which with frames that never
// reached the screen (several renders between two video flips) could hand
// back the very buffer being scanned out. Run at both ends of a frame, so a
// flip that landed while the loop slept frees its buffer before EGL wants one.
void OSD::release_unused_bos(struct gbm_surface* gs) {
    for (size_t i = 0; i < bo_release_queue.size(); ) {
        struct gbm_bo* old_bo = bo_release_queue[i];
        auto it = bo_to_fb.find(old_bo);
        if (it != bo_to_fb.end() && dev->osd_fb_in_use(it->second)) { i++; continue; }
        gbm_surface_release_buffer(gs, old_bo);
        bo_release_queue.erase(bo_release_queue.begin() + i);
    }
    // Backstop: never hold more than the old code did (two queued plus the
    // current one). Correct tracking never needs more; stale tracking (a lost
    // flip event) must not be able to lock every buffer and stall EGL.
    while (bo_release_queue.size() > 2) {
        static bool warned = false;
        if (!warned) { warned = true; fprintf(stderr, "[OSD] releasing a buffer still marked in use (backstop)\n"); }
        gbm_surface_release_buffer(gs, bo_release_queue.front());
        bo_release_queue.erase(bo_release_queue.begin());
    }
}

void OSD::render_gl() {
    if (gl_buffers.empty()) return;
    prof::frame_begin();
    release_unused_bos((struct gbm_surface *)gl_buffers[0].bo);
    auto& gb = gl_buffers[0];
    struct gbm_surface *gs = (struct gbm_surface *)gb.bo;

    eglMakeCurrent(display, gb.surface, gb.surface, context);
    
    int screen_w = dev->output_list->mode.hdisplay;
    int screen_h = dev->output_list->mode.vdisplay;
    glViewport(0, 0, screen_w, screen_h);
    
    glClearColor(0, 0, 0, 0); 
    // The clear must be the whole buffer, every frame. Scissor state persists
    // across frames and this surface is double-buffered, so a clear clipped to
    // last frame's rectangle would leave the other buffer's outer band holding
    // whatever was drawn there before the picture shrank - stale HUD pixels
    // flickering outside the picture while the setting is being nudged.
    glDisable(GL_SCISSOR_TEST);
    glClear(GL_COLOR_BUFFER_BIT);

    // The gallery (Back on the live picture) takes the whole frame: no HUD,
    // no picture scale. It hands back false once it has closed.
    frame_tile_ = DrmDevice::Tile();
    if (!gallery_.active()) phase_lock_.store(false, std::memory_order_relaxed);
    if (prof::enabled() && !gallery_.active()) stats_prof(get_time_us());
    if (gallery_.active() && render_gallery(screen_w, screen_h)) {
        present_gl_frame();
        return;
    }

    // Clip the whole overlay to the scaled picture. The canopy's wings are
    // drawn to run PAST the physical screen edge on purpose, so the edge
    // clips their outer ends - and shrinking everything in clip space brought
    // that overhang inside the picture. Rather than teach each element about
    // the new edge, scissor to the picture rectangle: nothing outside it is
    // drawn, whatever it is. After the clear, so the area outside is still
    // cleared each frame and cannot hold stale pixels; at 100% this is the
    // full screen and changes nothing. Same rectangle the video plane uses.
    {
        int px, py, pw, ph;
        if (dev) dev->picture_rect(screen_w, screen_h, px, py, pw, ph);
        else { px = 0; py = 0; pw = screen_w; ph = screen_h; }
        glEnable(GL_SCISSOR_TEST);
        glScissor(px, py, pw, ph);   // GL y is bottom-up; the rect is centred, so symmetric
    }
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); // Premultiplied alpha

    math::Mat4 projection, view, tmp;
    math::Mat4 model_c, model_l, model_r, model_hud_c;
    math::Mat4 mvp_c, mvp_l, mvp_r, mvp_hud_c;
    
    math::perspective(projection, 0.785f, (float)screen_w / screen_h, 0.1f, 100.0f);

    // Whole-picture scale. The video plane is shrunk by the display
    // controller (drm.cpp); the OSD plane cannot be - it sits on a Smart
    // window with no scaler - so the overlay shrinks itself to match, here,
    // in the one projection every MVP on this frame is built from. Scaling
    // the projection's x and y rows is exactly "gl_Position.xy *= s".
    //
    // It is done on the CPU on purpose. The first version did the multiply
    // in the vertex shaders, and on this Mali driver that line alone - even
    // at a scale of 1.0 - left white text and the idle title drawn faint,
    // as if their alpha had been cut. Keeping the shaders untouched avoids
    // whatever the compiler made of it; the matrix costs nothing.
    {
        const float s = dev ? dev->picture_scale_pct / 100.0f : 1.0f;
        projection[0] *= s;   // x row: the only non-zero entry is [0]
        projection[5] *= s;   // y row: the only non-zero entry is [5]
    }
    math::lookAt(view, {0, 0, wrap_depth}, {0, 0, 0}, {0, 1, 0});
    
    math::multiply(tmp, view, projection);
    
    // Fetch dynamic HUD displacement bounce offsets from active MSP telemetry.
    // hud_bank is a view-plane tilt from the craft's roll - a bank angles the
    // HUD rather than sliding it, which would read as yaw.
    float hud_dx = 0.0f, hud_dy = 0.0f, hud_bank = 0.0f;
    // Only while there is an aircraft. In IDLE the last attitude describes a
    // craft that is no longer there, and holding the panels at its offset (or
    // springing them about on whatever stale reading survived) reads as a
    // stuck HUD - so centre them and leave them still. hud_connected_ is a
    // frame behind here, which is not visible.
    msp_osd.set_motion_wanted(hud_reactivity > 0 && hud_connected_);
    if (hud_reactivity > 0 && hud_connected_) {
        // 1=SMALL 2=MEDIUM 3=EXTREME. MEDIUM is 1.0 - the amount tuned
        // against a real craft, so the other two hang off it.
        static const float kReactScale[4] = { 0.0f, 0.5f, 1.0f, 3.0f };
        bool moving = false;
        msp_osd.get_hud_offset(hud_dx, hud_dy, hud_bank, kReactScale[hud_reactivity], moving);
        // Still settling: keep frames coming until it is at rest.
        if (moving) signal_render(prof::kWakeAnim);
    } else {
        msp_osd.reset_hud_motion();   // leaves dx/dy/bank at 0
    }

    // 1. Center Matrix (No rotation) - Background remains stable (mvp_c)
    math::rotateY(model_c, 0.0f);
    math::multiply(mvp_c, model_c, tmp);

    // 1b. Center HUD Matrix (dynamic bank + translation shift)
    math::rotateZ(model_hud_c, hud_bank);
    model_hud_c[12] = hud_dx;
    model_hud_c[13] = hud_dy;
    math::multiply(mvp_hud_c, model_hud_c, tmp);

    // --- DYNAMIC BOUNDARY CALCULATION ---
    float fov_y = 0.785f; // ~45 deg
    float aspect = (float)screen_w / (float)screen_h;
    
    // Absolute 100% screen boundaries in 3D units at wrap_depth
    float frustum_h = tanf(fov_y / 2.0f) * wrap_depth;
    float frustum_w = frustum_h * aspect;

    // HUD Margins (Distance from the physical screen edges)
    // 0.0f = Absolute edge. Increase to move elements inward.
    // A fixed inset from the edge. (There used to be a user "overscan" here
    // that pushed the HUD inward; Picture Size replaced it, since shrinking
    // the whole picture is what actually helps optics that clip the frame.)
    float hud_margin_l = 0.02f;
    float hud_margin_r = 0.02f;
    float hud_margin_b = 0.02f;

    // --- THREAD SAFETY: Copy shared state ---
    pthread_mutex_lock(&osd_mutex);
    // Keep the history graphs SCROLLING through dead air: when no frame has
    // arrived for 150ms, backfill zero "gap" entries at ~30/s so an outage
    // draws as a flat gap in the timeline instead of freezing the graph on
    // its last shape (the render loop still ticks via the 1s refresh/bg feed).
    {
        uint64_t now_us = get_time_us();
        uint64_t last = osd_vars.latency_history_last_us;
        if (last != 0 && now_us - last > 150000) {
            int fill = (int)((now_us - last) / 33333);   // ~30 entries per second
            if (fill > 90) fill = 90;                    // cap backlog per pass
            for (int k = 0; k < fill; k++) {
                LatencyFrame gap{};
                gap.t_us = now_us - (uint64_t)(fill - k) * 33333ULL;
                osd_vars.latency_ring.push(gap);
            }
            osd_vars.latency_history_last_us = now_us;
        }
    }
    // Only what this frame shows: the newest entry and the median.
    LatencyFrame last_latency_frame{};
    const bool have_latency_frame = !osd_vars.latency_ring.empty();
    if (have_latency_frame) last_latency_frame = osd_vars.latency_ring.back();
    float latency_median_ms = 0.0f;
    const bool have_latency_median = osd_vars.latency_ring.median(latency_median_ms);
    packets_stats link_stats_copy = osd_vars.link_stats;
    uint32_t v_width = osd_vars.video_width;
    uint32_t v_height = osd_vars.video_height;
    int cur_fps = osd_vars.current_framerate;
    float v_bw = osd_vars.video_bandwidth;
    float lat_avg = osd_vars.total_latency_avg;
    float lat_max = osd_vars.total_latency_max;
    bool slices_received = osd_vars.slices_received;
    if (slices_received) {
        // the slowest slice's: enc = capture -> encoder out (air side), rf = encoder out to here, dec, disp
        lat_avg = osd_vars.tx_latency_avg
                + osd_vars.proc_latency_avg
                + osd_vars.decoding_latency_avg + osd_vars.display_latency_avg;
        if (lat_max < lat_avg) lat_max = lat_avg;
    }
    // SYSTEM > Time Zone, picked in the menu or from a phone: its TZ goes into
    // force here, on the thread that draws the clock (setenv() must not race
    // the time functions), and the menu's mirror follows a change made elsewhere.
    zones::apply_pending_tz();
    if (zones::generation() != menu_zone_gen_) {
        menu_zone_gen_ = zones::generation();
        if (menu_zone_ != zones::selected()) {
            menu_zone_ = zones::selected();
            menu_opt_tab_ = -1;
            // A zone picked elsewhere (the web page) can take the Timezone row
            // away from under the selection.
            if (menu_open && menu_tab == kTabSystem) {
                const int n = (int)menu_items(kTabSystem).size();
                if (menu_index >= n) {
                    menu_index = n - 1;
                    if (menu_focus == 2) menu_focus = 1;
                }
            }
            if (menu_open) menu_refresh_options();
        }
    }
    // AIR UNIT > SERIAL counts up while it is open: redraw four times a second.
    if (menu_open && menu_tab == kTabAirSerial) {
        const uint64_t t = get_time_us();
        if (t - serial_tab_us_ >= 250000ULL) { serial_tab_us_ = t; render_requested = true; }
        // The frame rate: the count a second apart.
        if (t - serial_rate_us_ >= 500000ULL) {      // twice a second, and only with this tab open
            const MspOsd::MspCounts g = msp_osd.msp_counts();
            const uint32_t n = g.frames;
            const float dt = serial_rate_us_ ? (t - serial_rate_us_) / 1e6f : 0.0f;
            serial_rate_ = (dt > 0.0f && n >= serial_prev_) ? (n - serial_prev_) / dt : 0.0f;
            serial_prev_ = n;
            serial_bps_ = (dt > 0.0f && g.bytes >= serial_prev_bytes_) ? (g.bytes - serial_prev_bytes_) / dt : 0.0f;
            serial_prev_bytes_ = g.bytes;
            serial_rate_us_ = t;
        }
    }
    {
        int tilt_deg;
        if (msp_osd.take_cam_tilt_to_save(&tilt_deg)) {
            Settings::getInstance().set("imu_cam_tilt_deg", tilt_deg);
            printf("hud: camera tilt %d deg, taken at arming, saved\n", tilt_deg);
        }
    }
    // Refresh Sync on trial that nobody kept: back to the kept value.
    if (sync_confirm_ && sync_confirm_left_s() <= 0) {
        sync_confirm_ = false;
        printf("menu: refresh sync %s not kept - back to %s\n", refresh_sync_label(g_display_lock.load()),
               refresh_sync_label(sync_confirm_prev_));
        g_display_lock = sync_confirm_prev_;
        menu_refresh_sync = sync_confirm_prev_;
        if (dev) dev->refollow();
        // Back to OFF takes the Sync Margin row away, maybe from under the selection.
        if (menu_open && menu_tab == 3) {
            const int n = (int)menu_items(3).size();
            if (menu_index >= n) {
                menu_index = n - 1;
                if (menu_focus == 2) menu_focus = 1;
            }
        }
        if (menu_open) menu_refresh_options();   // the list's position too, not just the row's label
        render_requested = true;
    }
    // A screen mode on trial that nobody kept: back to the screen's kept mode.
    if (screen_confirm_ && screen_confirm_left_s() <= 0) {
        screen_confirm_ = false;
        printf("menu: screen mode %s not kept - restarting in the previous one\n",
               screen_confirm_mode_.c_str());
        kestrel_request_restart();
    }
    if (menu_open_at_start) {
        menu_open = true;
        menu_open_at_start = false;
        if (screen_confirm_) {           // straight onto the question
            menu_first_open_ = false;
            menu_tab = kTabSystem;
            menu_index = 3;
            menu_focus = 1;
        }
        if (menu_first_open_) {          // --menu with no tab named
            menu_first_open_ = false;
            menu_tab = kTabSystem;
            menu_index = 0;
            menu_focus = 0;
        }
        // Straight onto the goggle's blade, unfolded, on SYSTEM.
        ui_side_ = 0;
        ui_side_tab_[0] = menu_tab = kTabSystem;
        ui_stage_ = 1;
        if (menu_focus == 0) menu_focus = 1;
        // --menu opens without a keypress, so nothing has worked out the value
        // column yet. Safe to do from here: this runs under the same lock the
        // key handler takes, so no press can be halfway through a row.
        menu_refresh_options();
    }
    // The canopy's motion (ui_* in osd.hpp), eased the way the gallery's zoom
    // is - each value closes the gap to its target by 1 - e^(-dt/tau) a frame -
    // and kept moving a frame at a time, started right after a flip as the
    // gallery's are.
    bool menu_moving = false;
    {
        const uint64_t t_us = get_time_us();
        float dt = menu_anim_us_ ? (float)(t_us - menu_anim_us_) / 1e6f : 1.0f / 60.0f;
        menu_anim_us_ = t_us;
        if (dt < 0.0f) dt = 0.0f;
        if (dt > 1.0f / 30.0f) dt = 1.0f / 30.0f;   // a stall is not a leap
        auto follow = [&](float& v, float target, float tau) {
            v += (target - v) * (1.0f - std::exp(-dt / tau));
            if (std::fabs(target - v) < 0.002f) v = target;
            if (v != target) menu_moving = true;
        };
        const bool band = menu_open && menu_scan_open;
        follow(ui_wake_, menu_open ? 1.0f : 0.0f, 0.09f);
        for (int i = 0; i < 2; i++) {
            const bool picking = menu_open && ui_stage_ == 0;
            follow(ui_lift_[i], picking ? (ui_side_ == i ? 1.0f : -0.6f) : 0.0f, 0.08f);
            follow(ui_open_[i], (menu_open && ui_stage_ == 1 && ui_side_ == i && !(band && i == 1)) ? 1.0f : 0.0f, 0.09f);
        }
        follow(ui_band_, band ? 1.0f : 0.0f, 0.09f);
    }
    // Drawn while it wakes or settles, not only while it has the keys.
    const bool is_menu_open = menu_open || ui_wake_ > 0.0f;
    float s = osd_vars.ui_scale;

    // Which overlay. OFF leaves the screen to the Betaflight OSD; CANOPY
    // replaces both wings wholesale, so every arena panel element is gated
    // on `legacy_panels` while the shared measurements above them still run
    // - the canopy reads the same numbers. ARENA FULL adds the arena's
    // verbose rows; the canopy has no such level.
    const bool hud_panels     = (hud_style != kHudOff);
    const bool canopy_on      = (hud_style == kHudCanopy);
    const bool legacy_panels  = (hud_style == kHudArena || hud_style == kHudArenaFull);
    const bool hud_verbose    = (hud_style == kHudArenaFull);
    const bool legacy_verbose = hud_verbose;
    (void)hud_panels;
    CanopyIn canopy;

    artosyn_stats artosyn_copy = osd_vars.artosyn;
    adapt_stats adapt_copy = osd_vars.adapt;
    pthread_mutex_unlock(&osd_mutex);
    prof::mark(prof::kSnapshot);
    if (menu_moving) {
        phase_lock_.store(true, std::memory_order_relaxed);
        signal_render(prof::kWakeAnim);
    }
    // What sits along the top: the clock strip.
    math::Mat4 model_top, mvp_top;
    std::copy(model_hud_c, model_hud_c + 16, model_top);
    math::multiply(mvp_top, model_top, tmp);
    
    msp_osd.set_scale(s);
    // The panel layout's cell size. Full-canvas mode overrides it below, so it
    // is reset here every frame rather than once at startup.
    msp_osd.set_cell_size(0.03375f * s, 0.045f * s);
    char buf[128]; // Shared buffer

    // --- COLOR PALETTE (Cyberpunk / Midnight) ---
    float cyan_r = 0.0f, cyan_g = 0.9f, cyan_b = 1.0f;
    float neon_r = 0.7f, neon_g = 1.0f, neon_b = 0.0f;
    // Deep Midnight Blue backgrounds
    float bg_r = 0.005f, bg_g = 0.015f, bg_b = 0.04f; 

    // --- BACKGROUND VIDEO / WARP-ZOOM STATE MACHINE ---
    uint64_t now_us = get_time_us();
    VideoState vs;
    VideoState prev_vs;
    float trans_t = 0.0f;
    {
        pthread_mutex_lock(&osd_mutex);
        prev_vs = video_state_; // snapshot BEFORE any changes this frame
        // State is one-way: BACKGROUND → TRANSITION → LIVE.
        // Once FPV is received we stay in LIVE and show the frozen last frame on
        // disconnect. The background video is never restarted after first FPV lock.
        // Advance transition.
        if (video_state_ == VideoState::TRANSITION) {
            if (TRANSITION_DURATION_US <= 0.0f) {
                // Zero duration means the warp is switched off. Take the branch
                // explicitly rather than dividing by it: on the frame where the
                // keyframe lands the elapsed time is also 0, and 0/0 is a NaN
                // that fails every comparison - which would park the state
                // machine in TRANSITION for good.
                trans_t = 1.0f;
                video_state_ = VideoState::LIVE;
            } else {
                trans_t = (float)(now_us - transition_start_us_) / TRANSITION_DURATION_US;
                if (trans_t >= 1.0f) {
                    trans_t = 1.0f;
                    video_state_ = VideoState::LIVE;
                }
            }
        }
        vs = video_state_;
        pthread_mutex_unlock(&osd_mutex);
    }

    // Stop bg video decode once warp-zoom completes (outside mutex — stop() blocks).
    // It is never restarted: after FPV lock the frozen last frame stays on screen.
    if (bg_video_enabled && bg_player_ &&
        prev_vs == VideoState::TRANSITION && vs == VideoState::LIVE) {
        fprintf(stderr, "[BgVideo] FPV connected — stopping background decode\n");
        bg_player_->stop();
    }

    // true only when FPV is actively displayed AND frames decoded recently —
    // VideoState latches LIVE, so without the freshness check the bottom-left
    // fps/resolution/bitrate labels freeze on stale values when video stops.
    bool video_active = (vs == VideoState::LIVE) &&
                        (get_time_us() - last_fpv_frame_us_ < 1500000ULL);

    // Upload / import latest bg video frame while not fully live (video path only)
    if (!video_active && bg_video_enabled && bg_player_) {
        if (bg_player_->is_drm() &&
            pfn_eglCreateImageKHR && pfn_glEGLImageTargetTexture2DOES) {
            // --- Zero-copy DRM path: import NV12 DMA-BUF as EGL external texture ---
            import_drm_frame(bg_player_, bg_ext_tex_);
        } else {
            // --- CPU path: sws BGRA → GL_TEXTURE_2D ---
            bg_player_->upload_latest_frame(bg_video_tex_);
        }
    }

    // Draw background or transition
    glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_c);
    // Acquiring: there is an aircraft, and no picture from it yet. Deliberately
    // not gated on VideoState - that is one-way (BACKGROUND -> TRANSITION ->
    // LIVE) and never returns, so keying off it meant the lock played on the
    // first acquisition of a session and never again. A link that drops and
    // comes back is acquiring video just as much as one that never had it.
    //
    // Reaching LIVE with stale frames is exactly the reacquire case: the video
    // plane still holds the frozen last frame, and painting over it is right -
    // that picture is history, and leaving it up while the radio hunts for a
    // new one is the same lie as a panel at full brightness over a dead link.
    //
    // The lock screen then owns the whole frame. The background plate is
    // scenery for an idle goggle; behind an acquisition it is just busy ground
    // for thin rings and hairline brackets to lose themselves in.
    //
    // "Acquiring" is not the same as "no picture". On the way OUT of a link the
    // video stops seconds before the HUD concedes the aircraft is gone - frames
    // go stale after 1.5s, the three evidence sources need 3s, and the drop
    // hold adds 1.5s more - so a rule of "connected and no video" would raise a
    // lock screen on every dropout, on the way to IDLE_2. What separates the
    // two is whether a frame has ever arrived from THIS connection.
    const bool seen_this_link = frames_since_connect_ >= kFramesForVideo;
    // Acquiring: connected, and no video worth the name yet.
    //
    // Deliberately NOT gated on video_active. That goes true on a single frame
    // and stays true for 1.5s after it, so one straggler arriving ahead of the
    // real stream was enough to drop the black ground - leaving the animation
    // playing over the frozen last frame, then that one stale picture, and only
    // then the black it should have had from the start.
    //
    // seen_this_link is the honest test and the one the rest of this already
    // uses: ten frames from THIS connection. It also behaves correctly the
    // other way round - once a connection has produced real video the black
    // never returns, so a mid-flight stall dims the panels rather than blacking
    // out a picture that is merely late.
    // Not while the menu is up. The lock screen owns the centre of the frame
    // and so does the menu; drawn together the reticle runs straight through
    // the rows, which the first capture of the two at once showed plainly.
    // The menu is the one the pilot is looking at, so it wins.
    // Gated on vs==LIVE, not just "not TRANSITION": vs is one-way and
    // BACKGROUND only until the very first connection's first keyframe ever
    // decodes, so hud_connected_ can go true well before that (telemetry
    // proves an aircraft while the decoder is still hunting for a clean
    // frame). That gap used to raise the black panel and reticle before
    // there was anything to acquire yet; now the plain background loop
    // stays up through it, same as before there was ever an aircraft, and
    // the warp takes over the instant
    // video actually arrives, unfought (974c86e turned the warp off instead
    // of fixing this fight; fixed here instead). Every later reconnect
    // finds vs already at LIVE, since it never reverts, so acquiring still
    // plays on every one of those - only the very first connection skips it.
    const bool acquiring = hud_connected_ && !seen_this_link && !is_menu_open &&
                            vs == VideoState::LIVE;
    // ...and once video does arrive, the screen still owes the eye a moment.
    // It fades out over the live picture rather than being cut mid-animation,
    // so nothing is ever hidden to buy the time.
    const uint64_t lk_now = get_time_us();
    const bool holding = hud_connected_ && !acquiring && lk_now < lock_hold_until_us_ &&
                          vs == VideoState::LIVE;
    const bool lock_screen = acquiring || holding;
    const float lock_fade =
        acquiring ? 1.0f
                  : (lock_hold_until_us_ > lk_now
                         ? fminf(1.0f, (float)(lock_hold_until_us_ - lk_now) / (float)kLockFadeUs)
                         : 0.0f);
    if (acquiring) {
        // Opaque black, drawn rather than left clear: the OSD plane is blended
        // over the video plane at scanout, and leaving it transparent shows
        // whatever that plane last held. Only while acquiring - once there is a
        // picture the fading lock rides over it, because holding black to buy
        // the animation time would hide video we already have.
        draw_panel(-frustum_w, -frustum_h, frustum_w * 2.0f, frustum_h * 2.0f,
                   1.0f, 0.0f, 0.0f, 0.0f);
    } else if (!lock_screen && vs == VideoState::BACKGROUND) {
        if (bg_video_enabled) {
            draw_bg_video(frustum_w, frustum_h, mvp_c, 0.0f);
            // No signal_render() here — on_new_frame callback drives wakeups
        } else {
            draw_bg_png(frustum_w, frustum_h, mvp_c);
        }
    } else if (vs == VideoState::TRANSITION) {
        if (bg_video_enabled) {
            draw_bg_video(frustum_w, frustum_h, mvp_c, trans_t);
        } else {
            draw_bg_png(frustum_w, frustum_h, mvp_c, trans_t);
        }
        signal_render(prof::kWakeAnim); // keep redrawing to animate the warp
    }
    // VideoState::LIVE draws nothing here. The FPV frame is on its own DRM
    // plane, which the display controller composites with this one at scanout;
    // a screen recording rebuilds the same from both with RGA (screen_tap.hpp).
    //
    // This used to call draw_live_video() when dvr_screen was set, because
    // glReadPixels could not see the video plane and the capture needed a copy
    // of the video inside the GL layer. The side effect was that the video was
    // drawn TWICE - once by the hardware plane, once here on top - which made
    // the picture visibly brighter and the OSD grid show through as soon as the
    // Record Screen toggle was switched on.

    // --- IDLE TITLE: "KESTREL" ---
    // BACKGROUND: plain draw with shadow (main shader).
    // TRANSITION: routed through warp_shader_prog_ — the same barrel distortion,
    //             chromatic aberration and alpha fade that the background video
    //             gets, so the letters visibly stretch from the sides.
    // The idle title and the lock screen are the same slot in two different
    // states. KESTREL means "nothing to fly"; once there is an aircraft it says
    // the opposite of what is happening, and the lock takes the slot instead.
    // Only BACKGROUND - the transition is already the picture arriving, and
    // drawing a lock over it would be acquiring something we have.
    if (lock_screen) {
        // The lock plays when video actually arrives, not on a timer. That is
        // precisely the acquiring -> holding edge: `holding` only becomes true
        // once a frame from THIS connection has been decoded.
        if (holding && !lock_locked_us_)  lock_locked_us_ = get_time_us();
        if (acquiring)                    lock_locked_us_ = 0;
        draw_signal_lock(frustum_w, frustum_h, mvp_c, lock_fade,
                         1.0f, 0.09f, 1.0f, nullptr, nullptr, nullptr,
                         lock_locked_us_);
    } else {
        lock_locked_us_ = 0;
        // Restart the sequence next time rather than resuming mid-lock: a
        // second aircraft should get a whole lock, not the tail of the last one.
        lock_since_us_ = 0;
    }

    // It stays with the canopy awake: a blade unfolding up its side moves the
    // wordmark into the part of the picture it leaves free, and back as it
    // folds (draw_canopy_ui: an unfolded blade is 0.34 of the width).
    // "Ready" sits where the channel band unfolds, so it goes while that is up.
    if (idle_title_tex_ && !lock_screen) {
        const HudTheme& TT = hud_theme_current();
        auto ease = [](float t) { return t * t * (3.0f - 2.0f * t); };
        const float open_l = ease(ui_open_[0]), open_r = ease(ui_open_[1]);
        const float ready_a = 1.0f - ease(ui_band_);
        const float kPanel = 0.34f;
        const float title_dx = (open_l - open_r) * kPanel * frustum_w;
        float base_h = frustum_h * 0.663f;
        float base_w = base_h * (float)idle_title_w_ / (float)idle_title_h_;
        {
            // The ink is half the texture's width (render_idle_title); fit it
            // to 80% of what the panel leaves.
            const float fit = std::min(1.0f, 0.8f * (1.0f - kPanel) * 2.0f * frustum_w / (base_w * 0.5f));
            const float s = 1.0f + (fit - 1.0f) * std::max(open_l, open_r);
            base_h *= s;
            base_w *= s;
        }
        float cx = title_dx - base_w * 0.5f;
        float cy = -base_h * 0.5f + frustum_h * 0.08f; // slightly above centre

        if (vs == VideoState::BACKGROUND) {
            glUseProgram(shader_program);
            glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_c);
            glUniform1i(u_is_text_, 1);
            glUniform1i(u_use_shading_, 0);

            // Softer than the old title's: the wordmark is set larger, so the
            // same fraction of its height threw a slab rather than a shadow.
            float sh = base_h * 0.018f;
            GLint alpha_loc = a_alpha_factor_;
            struct { float dx, dy, r, g, b; } shadow_passes[] = {
                { sh * 2.0f, -sh * 2.0f, 0.00f, 0.00f, 0.05f },
                { sh,        -sh,        0.00f, 0.00f, 0.03f },
            };
            // The shadow is cast by the word, not by either half of it, so
            // both layers drop it before either is inked.
            for (auto& p : shadow_passes) {
                if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, 1.0f);
                draw_icon(idle_title_tex_,    cx + p.dx, cy + p.dy, base_w, base_h, p.r, p.g, p.b);
                draw_icon(idle_title_os_tex_, cx + p.dx, cy + p.dy, base_w, base_h, p.r, p.g, p.b);
            }
            if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, 1.0f);
            draw_icon(idle_title_tex_,    cx, cy, base_w, base_h,
                      TT.text[0],   TT.text[1],   TT.text[2]);
            draw_icon(idle_title_os_tex_, cx, cy, base_w, base_h,
                      TT.accent[0], TT.accent[1], TT.accent[2]);

            // "Ready" in the theme's accent, sitting where the boot splash's
            // progress bar does (splash.c: 85% down the frame) - so the
            // splash's loading bar and this label read as the same spot
            // finishing its job, not two unrelated elements.
            if (ready_a > 0.003f) {
                float ready_scale    = 0.18f * osd_vars.ui_scale;   // 3x - the first size read too small
                float ready_y_centre = -frustum_h * 0.7f;
                float ready_w        = text_width("Ready", ready_scale);
                float ready_x        = title_dx - ready_w * 0.5f;
                float ready_y        = ready_y_centre - ready_scale * 0.5f;

                // Same soft two-pass shadow as the wordmark above, scaled to
                // this label's own (much smaller) size - without it "Ready"
                // washed out over the brighter parts of the photo. Lighter
                // than the wordmark's: at a 6% offset and full opacity it
                // read as a second, dark "Ready" rather than a shadow, so the
                // offset is shorter and both passes are part-transparent.
                float rsh = ready_scale * 0.035f;
                struct { float dx, dy, r, g, b, a; } ready_shadow[] = {
                    { rsh * 2.0f, -rsh * 2.0f, 0.00f, 0.00f, 0.05f, 0.35f },
                    { rsh,        -rsh,        0.00f, 0.00f, 0.03f, 0.60f },
                };
                for (auto& p : ready_shadow) {
                    if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, p.a * ready_a);
                    draw_text("Ready", ready_x + p.dx, ready_y + p.dy, ready_scale, false, p.r, p.g, p.b);
                }
                if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, ready_a);
                draw_text("Ready", ready_x, ready_y, ready_scale, false,
                          TT.accent[0], TT.accent[1], TT.accent[2]);
                if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, 1.0f);
            }

            glUseProgram(shader_program);

        } else if (vs == VideoState::TRANSITION) {
            // "Stretched from sides" effect:
            //   - Quad is pinned to full screen width (no geometry expansion).
            //   - UV range in X shrinks toward the centre over time, so the same
            //     texture content is spread across a wider screen area → letters
            //     appear to be pulled outward from both sides.
            //   - At t=0 the UV exactly matches what BACKGROUND draws, so there
            //     is no visual pop at the moment the transition starts.
            //   - Height grows slowly so letters become taller too.
            //
            // u_half: at t=0 = frustum_w / base_w (0.335 at 16:9 with no blade
            //         unfolded: the same UV slice BACKGROUND shows via its large
            //         overflowing quad), centred where BACKGROUND has the word
            //         (title_dx). Shrinks as 1/(1+8t²) so letters are ≈2× wider
            //         at t=0.4, ≈5× at t=0.7.
            float t2 = trans_t * trans_t;
            float title_alpha = std::max(0.0f, 1.0f - t2 * 1.35f);

            float u_half = (frustum_w / base_w) / (1.0f + t2 * 8.0f);
            float u_mid  = 0.5f - title_dx / base_w;
            float u0 = u_mid - u_half;
            float u1 = u_mid + u_half;

            // Full-screen-width quad; height grows
            float tw = frustum_w * 2.0f;
            float th = base_h * (1.0f + t2 * 2.0f);
            float qx = -frustum_w;
            float qy = -th * 0.5f + frustum_h * 0.08f;

            float verts[] = {
                qx,      qy,      0.0f,  u0, 1.0f,
                qx + tw, qy,      0.0f,  u1, 1.0f,
                qx,      qy + th, 0.0f,  u0, 0.0f,
                qx + tw, qy + th, 0.0f,  u1, 0.0f,
            };

            glUseProgram(shader_program);
            glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_hud_c);
            glUniform1i(u_is_text_,    1);
            glUniform1i(u_use_shading_, 0);
            glUniform1f(u_alpha_, 1.0f);

            GLint alpha_loc = a_alpha_factor_;
            if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, title_alpha);

            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);
            GLint pos_loc = a_pos_;
            GLint uv_loc  = a_uv_;
            glEnableVertexAttribArray(pos_loc);
            glEnableVertexAttribArray(uv_loc);
            glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)0);
            glVertexAttribPointer(uv_loc,  2, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)(3*sizeof(float)));

            // Both halves ride the same quad and the same stretch, so the word
            // comes apart as one thing - only their colours differ.
            const GLuint parts[2] = { idle_title_tex_, idle_title_os_tex_ };
            const float* cols[2]  = { TT.text, TT.accent };
            for (int k = 0; k < 2; k++) {
                if (!parts[k]) continue;
                glUniform4f(u_color_, cols[k][0], cols[k][1], cols[k][2], 1.0f);
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, parts[k]);
                glUniform1i(u_tex_, 0);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
            glUniform4f(u_color_, 1.0f, 1.0f, 1.0f, 1.0f);

            if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, 1.0f);
            glUseProgram(shader_program);
        }
    }

    // --- GLOBAL BACKGROUND SHADE ---
    // A large, extremely subtle hexagonal shade behind everything
    glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_hud_c);
    draw_hex_panel(-frustum_w, -frustum_h, frustum_w * 2.0f, frustum_h * 2.0f, 0.5f, bg_r, bg_g, bg_b, false);
    
    // 2/3. Left and Right Matrices (panel yaw +/-0.26 rad / ~15 deg), each with
    // the dynamic bank and translation shift. The panel's own yaw goes on
    // first and the bank after it, so the bank stays a view-plane tilt - the
    // whole HUD leans as one with the horizon, rather than each panel rolling
    // about its own yawed axis.
    {
        math::Mat4 bank_m;
        math::rotateZ(bank_m, hud_bank);

        math::Mat4 yaw_l;
        math::rotateY(yaw_l, 0.26f);
        math::multiply(model_l, yaw_l, bank_m);
        // The canopy awake: the picked blade lifts a little and the other one
        // sinks; a blade unfolding into its settings (or the right one into
        // the channel band, which takes the whole bottom) sinks out of the way
        // under them, since what unfolds is that blade.
        const float lift_l = ui_lift_[0] * 0.06f * frustum_h -
                             std::max(ui_open_[0], ui_band_) * 0.9f * frustum_h;
        const float lift_r = ui_lift_[1] * 0.06f * frustum_h -
                             std::max(ui_open_[1], ui_band_) * 0.9f * frustum_h;
        model_l[12] = hud_dx;
        model_l[13] = hud_dy + lift_l;
        math::multiply(mvp_l, model_l, tmp);

        math::Mat4 yaw_r;
        math::rotateY(yaw_r, -0.26f);
        math::multiply(model_r, yaw_r, bank_m);
        model_r[12] = hud_dx;
        model_r[13] = hud_dy + lift_r;
        math::multiply(mvp_r, model_r, tmp);
    }
    
    // --- MSP OSD ---
    // Betaflight's canvas was always drawn flat white, the one thing on
    // screen a theme change did nothing to. Every glyph here is white on
    // transparent already (BetaflightGlyphs::draw_glyph, and the ASCII path
    // below) bar the battery icon's own fill colours, so tinting the whole
    // canvas in the theme's text role - the same role every other reading
    // uses - carries it along with everything else without a second table
    // of which glyph is which.
    const HudTheme& msp_theme = hud_theme_current();
    auto draw_msp_span_func = [&](float x, float y, const std::vector<uint16_t>& span) {
        float cw, ch;
        msp_osd.get_cell_size(cw, ch);

        // Each cell's texture is rendered at the size it is drawn at, on screen: a 64x64
        // texture squashed into a cell 36x54 (a third narrower than tall) made the letters
        // condensed and soft, next to the canopy's, which are drawn at their own pixel size.
        // The canvas spans the picture, so a cell is the picture's pixels over the grid.
        const float pic = dev ? dev->picture_scale_pct / 100.0f : 1.0f;
        const float px_per_unit_x = (float)screen_w * pic / (2.0f * frustum_w);
        const float px_per_unit_y = (float)screen_h * pic / (2.0f * frustum_h);
        const int tw = std::max(8, (int)lroundf(cw * px_per_unit_x));
        const int th = std::max(8, (int)lroundf(ch * px_per_unit_y));
        if (tw != char_tex_w_ || th != char_tex_h_) {      // a new size: draw them all again
            for (int i = 0; i < 1024; i++)
                if (char_tex_cache[i]) { glDeleteTextures(1, &char_tex_cache[i]); char_tex_cache[i] = 0; }
            char_tex_w_ = tw;
            char_tex_h_ = th;
            for (int i = 0; i < 1024; i++) { char_w_px_[i] = 0; char_adv_px_[i] = 0; }
        }

        auto ensure = [&](uint16_t char_idx) {
            if (char_adv_px_[char_idx] != 0) return;           // done (a blank one has an advance too)
            // The glyph table first, for every char_idx, not just the ones outside 32-126: a few
            // reserved codes Betaflight uses as icon markers (0x77-0x7B - arrows, link quality)
            // land inside that range. draw_glyph knows which in-range codes are really reserved
            // and declines (returns false) for everything else, so it is the one place that decides.
            // An icon is a whole cell wide. Text is as wide as the letter itself.
            double adv = 0;
            int w = tw;
            cairo_surface_t *probe = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 4, 4);
            cairo_t *pc = cairo_create(probe);
            const bool icon = BetaflightGlyphs::draw_glyph(pc, char_idx, tw, th);
            cairo_destroy(pc);
            cairo_surface_destroy(probe);
            cairo_text_extents_t extents;
            char buf[2] = {(char)char_idx, 0};
            if (icon) {
                adv = tw;
            } else {
                // Text: the canopy's face, sized to the cell's height and never stretched, spaced
                // as the rest of the HUD spaces it - each letter its own advance, a space its own
                // width - not one cell apiece.
                cairo_surface_t *m = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 4, 4);
                cairo_t *mc = cairo_create(m);
                cairo_select_font_face(mc, "Chakra Petch SemiBold", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
                cairo_set_font_size(mc, 0.84 * th);
                cairo_text_extents(mc, buf, &extents);
                cairo_destroy(mc);
                cairo_surface_destroy(m);
                adv = extents.x_advance;
                if (char_idx == ' ') adv = std::max(adv, 0.30 * th);
                w = std::max(4, (int)std::ceil(std::max(adv, extents.x_bearing + extents.width)) + 3);
            }
            char_w_px_[char_idx] = w;
            char_adv_px_[char_idx] = (float)std::max(adv, 1.0);
            if (char_idx == ' ') return;                        // nothing to draw

            cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, w, th);
            cairo_t *cr = cairo_create(surface);
            if (icon) {
                BetaflightGlyphs::draw_glyph(cr, char_idx, tw, th);
            } else {
                cairo_select_font_face(cr, "Chakra Petch SemiBold", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
                cairo_set_font_size(cr, 0.84 * th);
                cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
                cairo_move_to(cr, 1.0, 0.78 * th);              // the pen, as in any line of text
                cairo_show_text(cr, buf);
            }
            unsigned char* data = cairo_image_surface_get_data(surface);
            GLuint tex;
            glGenTextures(1, &tex);
            glBindTexture(GL_TEXTURE_2D, tex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
            char_tex_cache[char_idx] = tex;
            cairo_destroy(cr);
            cairo_surface_destroy(surface);
        };

        // A soft drop shadow, as the canopy's type has, so it reads on bright video; the whole
        // span's shadows first, so a glyph's shadow never lies over its neighbour. Then the
        // glyphs, slightly dimmed like the canopy's.
        const float sx = 1.5f / px_per_unit_x, sy = 1.5f / px_per_unit_y;
        for (int pass = 0; pass < 2; pass++) {
            float pen_px = 0;                                    // along the run, pixels
            for (uint16_t char_idx : span) {
                if (char_idx >= 1024) { pen_px += tw; continue; }
                ensure(char_idx);
                const float cx = x + pen_px / px_per_unit_x;
                if (char_tex_cache[char_idx] != 0) {
                    const float qw = char_w_px_[char_idx] / px_per_unit_x;
                    if (pass == 0)
                        draw_icon(char_tex_cache[char_idx], cx + sx, y - sy, qw, ch, 0.0f, 0.0f, 0.0f, 0.55f);
                    else
                        draw_icon(char_tex_cache[char_idx], cx, y, qw, ch,
                                  msp_theme.text[0], msp_theme.text[1], msp_theme.text[2], 0.92f);
                }
                pen_px += char_adv_px_[char_idx];
            }
        }
    };

    // --- MENU RENDERING ---
    // --- TOP CENTRE: distance, clock, temperature ---
    // These three sit above the video rather than in the RF panel because they
    // are wanted at every detail level - including MINIMAL, where the panels
    // are gone entirely and the Betaflight OSD has the screen. Distance and
    // temperature are the two readings a pilot acts on in the air; the rest of
    // the RF panel is diagnostics.
    //
    // Laid out as one centred group so the clock does not shift as the strings
    // beside it change width.
    {
        float clk_scale    = 0.075f * s;
        float hud_margin_t = 0.02f;
        float clk_y        = frustum_h - hud_margin_t - clk_scale;
        float gap          = clk_scale * 0.55f;
        float icon_sz      = clk_scale;
        float icon_gap     = clk_scale * 0.35f;

        char clock_buf[16] = {0};
        if (clock_show != 0) {
            time_t     now_t = time(nullptr);
            struct tm  tm_now;
            if (localtime_r(&now_t, &tm_now)) {
                if (clock_mode == 2) {
                    // 24-hour pads the hour ("09:11"); 12-hour does not
                    // ("9:11 AM"). That is the convention each form is read with.
                    snprintf(clock_buf, sizeof(clock_buf), "%02d:%02d", tm_now.tm_hour, tm_now.tm_min);
                } else {
                    int h12 = tm_now.tm_hour % 12;
                    if (h12 == 0) h12 = 12;          // midnight and noon are 12, not 0
                    snprintf(clock_buf, sizeof(clock_buf), "%d:%02d %s", h12, tm_now.tm_min,
                             tm_now.tm_hour < 12 ? "AM" : "PM");
                }
            }
        }

        // Temperature. VRX is this unit's RK3568 (/sys/class/thermal); VTX is
        // the air unit's SoC from sky cmd 0x05 byte 1. Thresholds are this
        // board's own judgement: RK3568 designs commonly throttle at 85-95C,
        // so 80C leaves margin; 90C for the VTX is a placeholder pending a
        // real stock threshold.
        static float    vrx_temp_c  = 0.0f;
        static uint64_t vrx_temp_ms = 0;

        // This goggle's own supply voltage.
        //
        // It is NOT on the baseband, despite the AR8030 having an ADC - that
        // channel is an NTC thermistor stock uses to drive the fan. The
        // battery is on the RK3568's own SARADC, channel 5, read as a bare
        // IIO attribute with no consumer node, which is why the device tree
        // shows no battery anywhere and /sys/class/power_supply is empty.
        //
        // Recovered from ar_ldy_gnd (read at 0xa7ee8, scaled at 0x98678):
        //
        //   pin_mV  = raw * 1800 / 1024        10-bit SARADC, 1.8V reference
        //   avg     = mean of the last 10 pin_mV samples
        //   batt_mV = avg * 16 - 300           16:1 divider, 300mV offset trim
        //
        // Full scale is 1800*16-300 = 28.5V, which is the 1S-6S range stock's
        // own cell-count auto-detect covers. Stock samples about every 400ms
        // and discards the first conversion of each cycle; both are copied
        // here because a divider this steep turns one bad count into 16mV.
        static float    vrx_volts   = 0.0f;
        static uint64_t vrx_volts_ms = 0;
        static int      vbuf[10] = {0};
        static int      vcount = 0, vhead = 0;
        uint64_t temp_now_ms = get_time_ms();
        if (temp_now_ms - vrx_temp_ms > 2000) {   // sysfs read - not every frame
            vrx_temp_ms = temp_now_ms;
            FILE *tf = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
            if (tf) {
                int millideg = 0;
                if (fscanf(tf, "%d", &millideg) == 1) vrx_temp_c = millideg / 1000.0f;
                vrx_temp_cached_ = vrx_temp_c;
                fclose(tf);
            }
        }
        if (temp_now_ms - vrx_volts_ms > 400) {
            vrx_volts_ms = temp_now_ms;
            FILE *vf = fopen("/sys/bus/iio/devices/iio:device0/in_voltage5_raw", "r");
            if (vf) {
                int raw = -1, first = -1;
                // Discard the first conversion, as stock does.
                if (fscanf(vf, "%d", &first) == 1) {
                    rewind(vf);
                    if (fscanf(vf, "%d", &raw) != 1) raw = first;
                }
                fclose(vf);
                if (raw >= 0) {
                    int pin_mv = raw * 1800 / 1024;
                    vbuf[vhead] = pin_mv;
                    vhead = (vhead + 1) % 10;
                    if (vcount < 10) vcount++;
                    int sum = 0;
                    for (int i = 0; i < vcount; i++) sum += vbuf[i];
                    int avg = sum / vcount;          // live from the first sample
                    int batt_mv = avg * 16 - 300;
                    vrx_volts = batt_mv > 0 ? batt_mv / 1000.0f : 0.0f;
                }
            }
        }
        vrx_volts_cached_ = vrx_volts;
        // The strip is the IDLE state, not a permanent status bar. Distance
        // and both temperatures moved into the panels, where they sit beside
        // the thing they describe; what is left here says only that there is
        // nothing to fly yet. Once there is, the bar goes away entirely -
        // knowing the time is worth a line of the screen when nothing is
        // happening, and worth none of it when something is.
        static uint64_t last_conn_ms = 0;
        uint64_t strip_ms = get_time_ms();
        if (artosyn_copy.state == 2) last_conn_ms = strip_ms;
        BfTelem strip_bf = msp_osd.get_telem();
        // Evidence that there is an aircraft, from three independent sources: a
        // radio status of 2, frames arriving, or a flight controller talking.
        // Any one of them is proof enough - the AR8030 does fail to reach state
        // 2 with video on screen, and gating on it alone flagged idle over a
        // HUD full of live readings.
        bool raw_aircraft = (last_conn_ms && strip_ms - last_conn_ms < 3000) ||
                            video_active ||
                            (strip_bf.stamp_us && get_time_us() - strip_bf.stamp_us < 3000000ULL);

        // The hold belongs on the way OUT, not on the way in.
        //
        // Holding the rising edge was worse than the flicker it guarded
        // against: for the length of the hold the link is genuinely back, so
        // video is flowing and the readings are live, but the HUD is still
        // calling itself idle - and then it tears the panels down and rebuilds
        // them on top of a picture that was already working. You watched the
        // OSD come alive and then reassemble itself.
        //
        // Debouncing the drop instead gets the same protection for free. A link
        // that blinks never leaves CONNECTED at all, so there is no rebuild to
        // suppress; only a drop long enough to be a real dropout reaches IDLE_2,
        // and coming back from that fires the rebuild on the first frame of
        // evidence, with nothing shown before it.
        uint64_t strip_us = get_time_us();
        if (raw_aircraft) {
            hud_raw_down_us_ = 0;
            if (!hud_connected_) {
                hud_connected_since_us_ = strip_us;
                frames_since_connect_   = 0;
                lock_hold_until_us_     = strip_us + kLockMinUs;
            }
            hud_connected_   = true;
        } else if (hud_connected_) {
            if (!hud_raw_down_us_) hud_raw_down_us_ = strip_us;
            if (strip_us - hud_raw_down_us_ >= kLinkDropHoldUs) hud_connected_ = false;
            // Nothing else need be driving repaints while the link is gone, and
            // the drop has to land on time.
            else signal_render(prof::kWakeAnim);
        }
        bool aircraft_here = hud_connected_;

        // IDLE ONLY (1) gives the line back to the video the moment there is
        // something to fly; ON (2) keeps the time on screen throughout. The
        // IDLE flag itself belongs to the state, not to the setting, so it
        // appears whenever there is no aircraft and never once there is - the
        // canopy's own IDLE label says the same thing at the same moment.
        // IDLE ONLY (1) gives the line back to the video the moment there is
        // something to fly; ON (2) keeps the time on screen throughout.
        //
        // The strip carries no IDLE word any more. Before the first link there
        // are no panels on screen, and an empty frame with a clock in it says
        // "nothing to fly" more plainly than a label does; after a dropout the
        // canopy's own IDLE sits on the right panel's rail, where it is next to
        // the readings it is qualifying. The word was only ever needed when it
        // was the sole indication, and it no longer is.
        // The recording indicator has to live here too, not only in the canopy.
        // The canopy is not drawn at all before the first link, so pressing REC
        // on an idle goggle started a recording and showed nothing whatsoever -
        // the only way to know it was running was to look for the file.
        char rec_buf[32]; rec_buf[0] = '\0';
        // Only when no panel is already saying it. The canopy carries its own
        // REC on the left blade and the legacy HUD has one too, so the strip is
        // the fallback for the one case neither covers: IDLE_1, where the
        // canopy has never drawn. In IDLE_2 the blades are back - dimmed, but
        // drawn - and two indicators for one state is worse than none, because
        // the second one invites you to wonder what it means that is different.
        const bool panel_has_rec = (canopy_on && canopy_link_up_) || legacy_panels;
        bool rec_now = DvrRecorder::instance().is_recording() && !panel_has_rec;
        if (rec_now) {
            if (rec_started_us == 0) rec_started_us = get_time_us();
            unsigned rs = (unsigned)((get_time_us() - rec_started_us) / 1000000ULL);
            snprintf(rec_buf, sizeof(rec_buf), "REC %u:%02u", rs / 60, rs % 60);
        }
        const bool want_clock = clock_buf[0] && (clock_show == 2 || !aircraft_here);

        // Recording shows the strip even with the clock switched off: the
        // setting is about wanting the time on screen, not about hiding that
        // the goggle is writing to the card.
        if (want_clock || rec_now) {
            const float clk_w = want_clock ? text_width(clock_buf, clk_scale) : 0.0f;
            const float sep_w = (want_clock && rec_now) ? text_width("\xE2\x80\xA2", clk_scale) : 0.0f;
            const float rec_w = rec_now ? text_width(rec_buf, clk_scale) : 0.0f;
            const float dot_w = rec_now ? icon_sz * 0.45f : 0.0f;

            float total = 0.0f;
            if (want_clock) total += icon_sz + icon_gap + clk_w;
            if (want_clock && rec_now) total += gap + sep_w + gap;
            if (rec_now) total += dot_w + icon_gap + rec_w;

            float x = -total * 0.5f;
            glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_top);
            if (want_clock) {
                draw_icon(clock_tex, x, clk_y, icon_sz, icon_sz, 1.0f, 1.0f, 1.0f);
                x += icon_sz + icon_gap;
                draw_text(clock_buf, x, clk_y, clk_scale, false, 1.0f, 1.0f, 1.0f);
                x += clk_w;
            }
            if (want_clock && rec_now) {
                x += gap;
                draw_text("\xE2\x80\xA2", x, clk_y, clk_scale, false, 0.45f, 0.5f, 0.55f);
                x += sep_w + gap;
            }
            if (rec_now) {
                // The same red the canopy's indicator uses, and a dot rather
                // than a word so it reads as a state at a glance.
                draw_panel(x, clk_y + clk_scale * 0.30f, dot_w, dot_w, 1.0f, 1.0f, 0.25f, 0.25f);
                x += dot_w + icon_gap;
                draw_text(rec_buf, x, clk_y, clk_scale, false, 1.0f, 0.35f, 0.35f);
            }
        }

    }


    // --- Binding ---------------------------------------------------------
    //
    // Reuses the acquiring-video lock rather than having its own animation.
    // Rings contracting onto an aircraft is exactly what a bind is, and two
    // separate figures meaning almost the same thing would be a worse HUD -
    // this way there is one visual language for "the radio is hunting for an
    // aircraft", whichever end of the process you are at.
    //
    // Smaller and lower than the acquiring screen, so it sits under the fpvOS
    // wordmark instead of taking the whole frame, with the captions kept at
    // full size. Its own clock, so a bind does not restart an acquiring pass
    // that happens to be running.
    {
        const int bst = Ar8030Source::bind_state.load();
        static uint64_t bind_anim_us = 0, bind_locked_us = 0;
        if (bst != Ar8030Source::BIND_IDLE) {
            if (!bind_anim_us) bind_anim_us = get_time_us();
            // The lock plays once, on the bind actually landing. A failed
            // window never plays it - nothing was found, and a snap would say
            // otherwise.
            if (bst == Ar8030Source::BIND_OK) {
                if (!bind_locked_us) bind_locked_us = get_time_us();
            } else {
                bind_locked_us = 0;
            }

            char head[48], sub[64];
            const unsigned p = Ar8030Source::bind_peer.load();
            if (bst == Ar8030Source::BIND_RUNNING) {
                snprintf(head, sizeof(head), "BINDING   %ds",
                         Ar8030Source::bind_secs_left.load());
                snprintf(sub, sizeof(sub), "Push the bind button on the air unit");
            } else if (bst == Ar8030Source::BIND_OK) {
                snprintf(head, sizeof(head), "BOUND");
                snprintf(sub, sizeof(sub), "%02X:%02X:%02X:%02X   saved",
                         (p >> 24) & 0xFF, (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF);
            } else if (bst == Ar8030Source::BIND_LINKED) {
                // Not a failure. The radio started hearing an air unit, so
                // the window had nothing left to look for.
                snprintf(head, sizeof(head), "ALREADY LINKED");
                snprintf(sub, sizeof(sub), "an air unit is already connected");
            } else {
                snprintf(head, sizeof(head), "NO AIR UNIT FOUND");
                snprintf(sub, sizeof(sub), "Push the bind button on the air unit");
            }
            // 0.34 puts the outer ring at about the orbit radius this replaces;
            // -0.50 clears the wordmark above it.
            draw_signal_lock(frustum_w, frustum_h, mvp_hud_c, 1.0f,
                             0.34f, -0.50f, 1.30f, head, sub, &bind_anim_us,
                             bind_locked_us);
        } else {
            // Next bind gets a whole scan, not the tail of the last one.
            bind_anim_us = 0;
            bind_locked_us = 0;
        }
    }

    // --- PANEL LAYOUT VARS ---
    float panel_width = 1.2f * s;
    float panel_height = 0.90f * s;
    float slant_offset_x = 0.30f; 
    float slant_offset_y = 0.15f; 
    
    float bx = -frustum_w + hud_margin_l + slant_offset_x; 
    float rx = frustum_w - hud_margin_r - panel_width - slant_offset_x;
    
    float bg_bx = -frustum_w; 
    float bg_rx = frustum_w - panel_width - 0.05f * s; 
    float panel_bottom_y = -frustum_h; 
    float content_margin = 2.0f * hud_margin_l;
    
    float bar_h_offset = 0.02f * s + 0.02f * s; 
    float current_ly = -frustum_h + content_margin + slant_offset_y + bar_h_offset;

    // --- BOTTOM LEFT: Health-Bar Latency & Stats ---
    glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_l);
    // 0.95, not 0.8: over a bright FPV feed the wing panels washed out. Their
    // gradient still fades towards the screen edge, so this raises the strong
    // end rather than flattening the look.
    if (legacy_panels)
        draw_hex_panel(bg_bx, panel_bottom_y, panel_width, panel_height, 0.95f, bg_r, bg_g, bg_b, true);
    
    float data_mbps = video_active ? v_bw : 0.0f;
    int mcs = (!link_stats_copy.antennas.empty()) ? link_stats_copy.antennas[0].mcs_index : 0;

    // (Removed) A "fake placeholder" here substituted 15 Mbps / MCS5 when the
    // real values were unavailable. It rendered invented numbers in the same
    // style as measurements, which is worse than showing nothing.

    float max_lat = 50.0f;
    // Zero until a real frame supplies a breakdown. These used to default to
    // 5/21/8/10ms - a plausible-looking invented profile that summed to the
    // invented 44ms "max" below, so the bar drew a convincing shape from
    // nothing.
    float r_proc = 0.0f, r_net = 0.0f,
          r_dec = 0.0f, r_disp = 0.0f;
    
    if (have_latency_frame) {
        const auto& last_frame = last_latency_frame;
        float total = last_frame.processing_ms + last_frame.net_ms + last_frame.dec_ms + last_frame.disp_ms;
        if (total > 0.1f) {
            r_proc = last_frame.processing_ms / max_lat;
            r_net = last_frame.net_ms / max_lat;
            r_dec = last_frame.dec_ms / max_lat;
            r_disp = last_frame.disp_ms / max_lat;
        }
    }

    float bar_h = 0.04f * s;
    float m_slant = 0.0f; 

    // (Removed) lat_avg/lat_max used to be forced to 30.0/44.0 whenever the
    // real measurement read 0 while video was active - fabricated numbers
    // presented identically to measured ones. Now 0 stays 0 and the row
    // renders "--", so "no measurement" is visibly different from "12.3ms".

    // Left panel dim factor: grey when no active video (mirrors right panel behaviour)
    float ldim = video_active ? 1.0f : 0.5f;

    if (legacy_panels)
    draw_stacked_bar(bx + hud_margin_l, current_ly, 0.85f * s, bar_h, m_slant, false,
                     r_proc, cyan_r * ldim, cyan_g * ldim, cyan_b * ldim,
                     r_net, 0.012f * ldim, 0.808f * ldim, 0.643f * ldim, // Teal for RF
                     r_dec, 0.918f * ldim, 0.769f * ldim, 0.208f * ldim, // Yellow for Dec
                     r_disp, 0.9f * ldim, 0.2f * ldim, 0.2f * ldim,
                     0.0f, 0.0f, 0.0f, 0.0f,                             // (a fifth and sixth part: none)
                     0.0f, 0.0f, 0.0f, 0.0f);

    current_ly += 0.07f * s;
    // With the air delay measured (the AR8030 path: see air_delay_for) this is
    // the slowest slice's, from its first row's capture to the scan-out lighting
    // that row (Renderer::update_stats). Without it, arrival to vblank only,
    // and labelled so.
    if (video_active && lat_avg > 0.0f) {
        sprintf(buf, !air_delay_valid ? "LATENCY (gnd): %.1fms | MAX: %.1fms"
                                      : "LATENCY (to scan-out): %.1fms | MAX: %.1fms",
                lat_avg, lat_max);
    } else if (video_active) {
        sprintf(buf, "LATENCY (gnd): -- | MAX: --");
    } else {
        sprintf(buf, "LATENCY (gnd): 0.0ms | MAX: 0.0ms");
    }
    if (legacy_panels) {
        draw_text(buf, bx + hud_margin_l, current_ly, 0.07f * s, false, ldim, ldim, ldim);
        current_ly += 0.085f * s;
    }

    if (legacy_verbose) {
    if (video_active) {
        // Only components with a live producer. cap/enc/net came from the
        // kestrel-air RTP headers and have read a hard 0 since that pipeline
        // was removed - printing them implied a measurement that no longer
        // exists. The air unit's own encode/RF timings are not recoverable
        // from the AR8030 link: the baseband SDK exposes no delay field, the
        // sky telemetry frame (cmd 0x05) carries none, and stock's own debug
        // OSD shows its sky delay triple as "S: 0,0,0" on this hardware - so
        // stock does not have them here either.
        float dec_avg  = osd_vars.decoding_latency_avg;
        float disp_avg = osd_vars.display_latency_avg;
        if (air_delay_valid)
            // "air" is capture stamp to the first slice's arrival: camera
            // readout, encode, the air app, the radio and the goggle's daemon.
            sprintf(buf, "[air:%.1f, dec:%.1f, disp:%.1f]",   // dec includes the slices' arrival
                    air_delay_ms, dec_avg, disp_avg);
        else
            sprintf(buf, "[dec:%.1f, disp:%.1f]  (ground)", dec_avg, disp_avg);
    } else {
        sprintf(buf, "[dec:0.0, disp:0.0]  (ground)");
    }
    draw_text(buf, bx + hud_margin_l, current_ly, 0.058f * s, false, ldim * 0.8f, ldim * 0.8f, ldim * 0.8f);
    current_ly += 0.09f * s;
    }   // hud_verbose

    // Kept at MEDIUM: what the link is actually delivering. The greyed
    // per-stage breakdown above is diagnostics and stays FULL-only.
    if (legacy_panels) {
        if (video_active) {
            sprintf(buf, "%dx%d | %d FPS | %.1f Mbps", v_width, v_height, cur_fps, data_mbps);
        } else {
            sprintf(buf, "- | 0 FPS | 0.0 Mbps");
        }
        draw_text(buf, bx + hud_margin_l, current_ly, 0.07f * s, false, ldim, ldim, ldim);
        current_ly += 0.09f * s;
    }

    // Recording indicator: a red dot plus elapsed time, sitting directly above
    // the resolution/fps/bitrate line. current_ly grows upward, so drawing it
    // after that line's advance puts it above on screen. Driven straight off
    // DvrRecorder, which owns the writer's lifetime.
    if (DvrRecorder::instance().is_recording()) {
        if (rec_started_us == 0) rec_started_us = get_time_us();
        unsigned secs = (unsigned)((get_time_us() - rec_started_us) / 1000000ULL);
        canopy.rec_active = true;
        canopy.rec_secs   = secs;
        float dot_x = bx + hud_margin_l;
        sprintf(buf, "   REC  %u:%02u", secs / 60, secs % 60);
        if (legacy_panels) {
            draw_poly({{dot_x, current_ly + 0.012f},
                       {dot_x + 0.018f * s, current_ly + 0.012f},
                       {dot_x + 0.018f * s, current_ly + 0.030f},
                       {dot_x, current_ly + 0.030f}}, 1.0f, 0.95f, 0.15f, 0.15f);
            draw_text(buf, dot_x, current_ly, 0.07f * s, false, 1.0f, 0.35f, 0.35f);
        }
        current_ly += 0.09f * s;
    } else {
        rec_started_us = 0;
    }

    // MSP OSD's corner-panel render (the old "PANELS" mode) lived here, split
    // across this hex panel and its counterpart below. Removed along with
    // PANELS - MSP OSD is now ON/OFF, always the full canvas. See the
    // full-canvas block below.

    float current_ry = -frustum_h + content_margin + slant_offset_y + bar_h_offset;
    float outer_x = rx + panel_width - hud_margin_r; 
    float bar_x = outer_x - 0.85f * s; 
    m_slant = 0.0f;

    // --- BOTTOM RIGHT: AR803x RF Link Stats ---
    glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_r);
    if (legacy_panels)
        draw_hex_panel(bg_rx, panel_bottom_y, panel_width, panel_height, 0.95f, bg_r, bg_g, bg_b, true);

    {
        // Debounce the badge: the baseband's status RPC blips for a poll or two
        // while the radio reconfigures an MCS change — flashing IDLE for that
        // sub-second reads as "link dropped" when nothing was lost. Only show
        // IDLE after 3s without a good status.
        static uint64_t last_connected_ms = 0;
        uint64_t badge_now = get_time_ms();
        if (artosyn_copy.state == 2) last_connected_ms = badge_now;
        bool connected = (last_connected_ms != 0 &&
                          badge_now - last_connected_ms < 3000);
        float dim = connected ? 1.0f : 0.50f;
        float total_gauge_h = 0.045f * s;
        float rs = 0.085f * s; // row step

        // TX power text, shared: at MEDIUM it rides on the LINK row, at FULL it
        // sits on its own SNR row, and the two must not drift apart.
        //
        // This is the AIR VTX power - the power we commanded the air unit to
        // transmit the video at (cmd 0x22), which is what "TX power" means on an
        // FPV goggle. It is deliberately NOT the ground radio's own uplink power
        // (info.self.tx_power): that is the goggle-to-air control-link power, and
        // its adaptation loop drifts to the ceiling, which is what made the OSD
        // read 500mW while the menu said 25mW.
        //   (A) - auto: the air runs adaptive power capped at the shown level.
        auto pwr_text = [&](char* out, size_t n) {
            int cmd_mw = artosyn_copy.air_pwr_mw;
            if (cmd_mw <= 0) { snprintf(out, n, "--"); return; }
            const ar_pwr_level &lv = kArPwrLevels[ar_pwr_index(cmd_mw)];
            int mw = lv.automode ? lv.mw - 1 : lv.mw;   // 501 (auto cap 500) -> 500
            const char *aflag = lv.automode ? " (A)" : "";
            if (mw >= 1000) snprintf(out, n, "%.1fW%s", mw / 1000.0f, aflag);
            else            snprintf(out, n, "%dmW%s", mw, aflag);
        };

        auto get_snr_color = [&](float val) -> std::array<float, 3> {
            if (val > 0.6f) return {cyan_r, cyan_g, cyan_b};
            if (val > 0.35f) return {1.0f, 0.8f, 0.0f};
            return {0.9f, 0.2f, 0.2f};
        };

        // The bar is the MCS rung, not SNR. SNR is an input to link quality,
        // not a measure of it: the SNR a link needs rises with its MCS, so the
        // same 20 dB is comfortable at MCS5 and about to fail at MCS12, and a
        // raw-SNR bar reads identically in both. The rung is what actually
        // moves when quality degrades - a downshift IS the bitrate dropping.
        //
        // This radio does not use MCS 0..12 continuously. It walks seven rungs,
        // {1,2,5,7,8,10,12}, which is what the SET 36 adaptation policy in the
        // stock capture we replay installs (see kStockRfSetup) and what section
        // 24 found it accepts. Scaling by raw MCS/12 would make a downshift
        // 12->10 move the bar by 17%; over the rungs it is a clear seventh.
        static const int kMcsLadder[] = { 1, 2, 5, 7, 8, 10, 12 };
        const int kMcsRungs = (int)(sizeof(kMcsLadder) / sizeof(kMcsLadder[0]));
        int rung = -1;
        for (int i = 0; i < kMcsRungs; i++)
            if (artosyn_copy.rx_mcs_val >= kMcsLadder[i]) rung = i;   // nearest rung at or below
        // A working link never shows an empty bar: the lowest rung is 1/7, and
        // "no rung" (an MCS below the ladder, or none reported) is what reads
        // as empty.
        float snr_norm = (rung < 0) ? 0.0f : (float)(rung + 1) / (float)kMcsRungs;

        // The composite, drawn as a thin second strip inside the same gauge so
        // the two can be watched against each other in real time. The blocks
        // above are what the link IS doing; this is what it is ABOUT to do.
        //
        //   margin  - SNR above the downshift threshold for the current rung.
        //             Thresholds are stock's own, byte 8..9 of the SET 36
        //             entries we replay (kStockRfSetup), read as dB x100.
        //             That reading of the scale is inferred, not confirmed -
        //             it is monotonic across all seven rungs, which is good
        //             evidence, but check it first if the strip misbehaves
        //             against the blocks.
        //   ldpc    - pre-FEC codeword errors, which rise while FEC is still
        //             repairing them, i.e. before any frame is lost. 3-8% is
        //             NORMAL (measured at 2m with zero post-FEC loss) - that is
        //             the FEC working, so health only falls above 10% and is
        //             dead at 50%.
        //
        // min() of the two: either one alone ruins the picture, and averaging
        // would let a fat SNR margin mask interference - the exact blind spot
        // raw SNR had.
        static const float kMcsDownDb[] =
            { 0.29f, 0.65f, 1.19f, 2.41f, 4.75f, 9.47f, 14.66f };  // per rung
        float quality = 0.0f;
        if (rung >= 0) {
            float margin_db     = artosyn_copy.snr - kMcsDownDb[rung];
            float margin_health = std::max(0.0f, std::min(1.0f, margin_db / 6.0f));
            float ldpc_health   = 1.0f - std::max(0.0f, std::min(1.0f,
                                        (artosyn_copy.ldpc_error - 0.10f) / 0.40f));
            quality = std::min(margin_health, ldpc_health);
        }
        // Smoothed once, here, so the strip and the LINK figure below cannot
        // disagree - they are the same measurement shown two ways. Only while
        // connected: a dropped link would otherwise decay the average with
        // readings that mean nothing.
        static float quality_ewma = -1.0f;
        if (connected) {
            quality_ewma = (quality_ewma < 0.0f) ? quality
                                                 : 0.9f * quality_ewma + 0.1f * quality;
            quality = quality_ewma;
        }
        if (legacy_panels) {
        // The gauge is split: MCS rung on top, composite as a thin strip under
        // it. Same segment width, so both read as one gauge rather than two.
        float h_thin = total_gauge_h * 0.26f;
        float h_gap  = total_gauge_h * 0.14f;
        float h_main = total_gauge_h - h_thin - h_gap;
        if (connected) {
            auto snr_rgb = get_snr_color(snr_norm);
            draw_stacked_bar(bar_x, current_ry + h_thin + h_gap, 0.85f * s, h_main, 0.0f, true,
                             snr_norm, snr_rgb[0] * dim, snr_rgb[1] * dim, snr_rgb[2] * dim);
            // The theme's neon green, fixed rather than value-coloured: while
            // the two strips are being compared it has to be obvious at a
            // glance which is which, and green reads as distinct from the
            // cyan/amber/red the MCS blocks cycle through.
            draw_stacked_bar(bar_x, current_ry, 0.85f * s, h_thin, 0.0f, true,
                             quality, neon_r * dim, neon_g * dim, neon_b * dim);
        } else {
            draw_stacked_bar(bar_x, current_ry + h_thin + h_gap, 0.85f * s, h_main, 0.0f, true,
                             1.0f, 1.0f * dim, 0.75f * dim, 0.20f * dim);
        }
        }
        current_ry += total_gauge_h + 0.03f * s;

        // Row 1b: LINK HEALTH - the numeric form of the green strip in the
        // gauge above, not a second opinion. Both come from `quality`.
        //
        // The +/-dB beside it is margin above the downshift threshold for the
        // rung the radio is CURRENTLY on, so it reads as "this much before the
        // bitrate steps down". It used to come from MCS_SNR_FLOOR, a table
        // estimated from walk-test downshifts, which stock's own captured
        // thresholds put roughly 8 dB too high at MCS 12 - the readout was
        // pessimistic enough to show danger while the radio was comfortable.
        if (legacy_panels && connected) {
            float margin_db = (rung >= 0) ? artosyn_copy.snr - kMcsDownDb[rung] : 0.0f;
            float health    = quality * 100.0f;
            auto hc = get_snr_color(health / 100.0f);
            if (hud_verbose) {
                sprintf(buf, "LINK %d%%  %+.1f dB", (int)(health + 0.5f), margin_db);
            } else {
                // Two rows at MEDIUM, so power rides here rather than being
                // dropped with the SNR row: health, headroom and what the
                // radio is spending to hold it, on one line.
                char pw[24]; pwr_text(pw, sizeof(pw));
                sprintf(buf, "LINK %d%%  %+.1f dB  %s", (int)(health + 0.5f), margin_db, pw);
            }
            draw_text(buf, outer_x, current_ry, 0.07f * s, true,
                      hc[0] * dim, hc[1] * dim, hc[2] * dim);
            // "STANDBY | " sits on the same line but keeps its own amber, so
            // it reads as a mode flag rather than part of the link health. Only
            // drawn when the VTX is actually in standby (vtx_low_power, from
            // cmd 0x05 bytes 29-30) - NOT air_standby, the TLV 0x12 value, which
            // only echoes the Auto Standby setting. The line is right-aligned,
            // so it is placed by measuring the LINK text.
            if (vtx_low_power) {
                float link_w = text_width(buf, 0.07f * s);
                draw_text("STANDBY | ", outer_x - link_w, current_ry, 0.07f * s, true,
                          1.0f, 0.75f, 0.20f);
            }
            current_ry += rs;
        } else if (legacy_panels) {
            // Not connected: same line, same shape, amber "IDLE | " flag. The
            // row used to vanish entirely, leaving a gap above the gauge.
            sprintf(buf, "LINK 0%%");
            draw_text(buf, outer_x, current_ry, 0.07f * s, true, dim, dim, dim);
            float link_w = text_width(buf, 0.07f * s);
            // Same amber as the bar below it, dim included - without the * dim
            // the label read noticeably brighter than the gauge it labels.
            draw_text("IDLE | ", outer_x - link_w, current_ry, 0.07f * s, true,
                      1.0f * dim, 0.75f * dim, 0.20f * dim);
            current_ry += rs;
        }

        // Row 2: SNR + TX power
        // The radio answers in dBm; show mW, which is what the menu speaks and
        // what a pilot thinks in. Under AUTO this tracks the level the radio
        // has actually settled on, not the ceiling that was requested.
        //
        // "?" suffix when the air unit reports standby: the value the SDK
        // returns is the configured/requested TX power, not necessarily the
        // actual output. fpv_sky_standby_mode_thread in the air firmware
        // gates the sensor-off + PA-bypass transition behind an internal
        // check (0x6abf0, precise condition not identified), so the air can
        // ACK "standby=ON" while still transmitting at full power - in which
        // case 500mW may be honest, but paired with the STANDBY flag it
        // reads as if it dropped. The "?" makes clear we do not know.
        if (legacy_panels && hud_verbose) {
            char pw[24]; pwr_text(pw, sizeof(pw));
            sprintf(buf, "SNR: %.1f dB  PWR: %s", artosyn_copy.snr, pw);
            draw_text(buf, outer_x, current_ry, 0.07f * s, true, dim, dim, dim);
            current_ry += rs;
        }

        // Row 5: antenna gains + LDPC success rate. ldpc_error is already the
        // pre-FEC codeword error RATIO (baseband reports it x10000, scaled on
        // receive) — success is simply 1 - ratio. The old formula divided one
        // ratio by the other and pinned the display to 100 or 0.
        if (legacy_panels && hud_verbose) {
            float ldpc_ok = std::max(0.0f, std::min(1.0f, 1.0f - artosyn_copy.ldpc_error)) * 100.0f;
            sprintf(buf, "Gain: %d/%d  LDPC: %.2f%%",
                    artosyn_copy.gains_cur, artosyn_copy.gains_max, ldpc_ok);
            draw_text(buf, outer_x, current_ry, 0.07f * s, true, dim, dim, dim);
            current_ry += rs;
        }


        // Row 3: RX MCS + RX data rate. Kept at MEDIUM alongside SNR/PWR:
        // between them they say how hard the radio is working and what it is
        // getting for it, which is the pair worth watching in flight. Gain,
        // LDPC, frequency and bandwidth are diagnostics and stay FULL-only.
        if (legacy_panels) {
            sprintf(buf, "RX MCS%d  %.2f Mbps", ar_mcs_label(artosyn_copy.rx_mcs_val),
                    artosyn_copy.rx_data_rate_kbps / 1000.0f);
            draw_text(buf, outer_x, current_ry, 0.07f * s, true, dim, dim, dim);
            current_ry += rs;
        }

        // (Removed) Row 3b used to show the kestrel-air adaptation telemetry
        // ("AIR T0 0.0Mbps 0fps WLd0 0ms"). The AR8030 link carries no such
        // feed, so it only ever rendered zeros.

        // Row 4: TX/RX frequency + RF bandwidth
        // tx_freq only. rx_status reports 2100 MHz on this radio while the link
        // is plainly on 5839, so showing both just raised the question.
        // The gear the radio reports, as MHz. Stock's own fpv_bb_set_bw
        // (0x9fee0) maps 5->2, 10->3, 20->4, 40->5, which matches our
        // bb_bandwidth_e, so the enum and the reading are both sound. Do not
        // put throughput here - the RX MCS row below already carries it.
        if (legacy_panels && hud_verbose) {
            if (artosyn_copy.tx_freq > 0)
                sprintf(buf, "%d MHz  BW: %s MHz",
                        artosyn_copy.tx_freq, ar_bw_label(artosyn_copy.rf_bw_idx));
            else
                sprintf(buf, "BW: %s MHz", ar_bw_label(artosyn_copy.rf_bw_idx));
            draw_text(buf, outer_x, current_ry, 0.07f * s, true, dim, dim, dim);
            current_ry += rs;
        }

        // Distance and temperature used to sit here. They moved to the top
        // status row, where they stay visible at every detail level.

        // Video link warning, for the screen-edge glow and the canopy's VIDEO
        // track. Same rung thresholds as the arena gauge's colours above, so
        // every layout calls the same link amber at the same moment:
        //   amber - MCS 5-7: the bitrate is being squeezed
        //   red   - MCS 1-2: a trickle, the picture breaks up; or no frame
        //           for 400 ms on a link that was delivering them - frozen
        // No MCS reading at all gives no verdict rather than a red one.
        int warn = 0;
        if (connected) {
            if (rung >= 0 && rung <= 1)      warn = 2;
            else if (rung >= 2 && rung <= 3) warn = 1;
            if (seen_this_link && get_time_us() - last_fpv_frame_us_ > 400000ULL)
                warn = 2;
        }

        // The canopy reads the same measurements this panel just computed
        // rather than re-deriving them, so the two layouts can never disagree.
        canopy.link_warn = link_warn_hold(warn);
        canopy.link_up  = connected;
        canopy.mcs_norm = snr_norm;
        canopy.quality  = quality;
        pwr_text(canopy.vtx_pwr, sizeof(canopy.vtx_pwr));
        // tx_freq, not rx_freq: rx_status reports a fixed 2100 MHz gear value
        // on this radio rather than the actual channel (see the RF tab's own
        // frequency row for the same substitution).
        canopy.link_freq_mhz = artosyn_copy.tx_freq;
        canopy.link_bw_idx = artosyn_copy.rf_bw_idx;
        canopy.link_mbps = artosyn_copy.state == 2 ? artosyn_copy.rx_data_rate_kbps / 1000.0f : 0.0f;
        // HUD > Debug Info: the frame our radio runs, read off its state word,
        // and the mode that length is - each of the six is its own length:
        // short (2), standard (0), long (1), at 40 MHz and at 20 or narrower.
        canopy.debug[0] = 0;
        if (hud_debug) {
            static const struct { float us; int mode; } kFrames[] = {
                { 3250.56f, 2 }, { 4233.60f, 0 }, { 5398.40f, 1 },
                { 4650.88f, 2 }, { 5633.92f, 0 }, { 7964.80f, 1 },
            };
            const float T = artosyn_copy.radio_frame_us;
            const int want = artosyn_copy.radio_frame_mode_want;
            int mode = -1;
            for (const auto& f : kFrames) if (fabsf(T - f.us) < 2.0f) mode = f.mode;
            const char* name = mode == 2 ? "SHORT" : mode == 0 ? "STANDARD" : mode == 1 ? "LONG" : "?";
            if (T > 0.0f)
                snprintf(canopy.debug, sizeof(canopy.debug), "FRAME %.2fms %s%s", T / 1000.0f, name,
                         mode < 0 || mode == want ? "" : want == 2 ? " (ASKED SHORT)" : " (ASKED STANDARD)");
            else
                snprintf(canopy.debug, sizeof(canopy.debug), "FRAME --");
        }
    }

    if (canopy_on) {
        canopy.frustum_w = frustum_w;
        canopy.frustum_h = frustum_h;
        canopy.overscan  = 0.02f;
        canopy.s         = s;
        canopy.model_l   = model_l;
        canopy.model_r   = model_r;
        canopy.vp        = tmp;
        canopy.video_active   = video_active;
        canopy.vtx_temp_valid = vtx_temp_valid;
        canopy.vtx_temp_c     = vtx_cpu_c;
        canopy.vtx_low_power  = vtx_low_power;
        canopy.vrx_temp_c     = vrx_temp_cached_;
        canopy.vrx_volts      = vrx_volts_cached_;
        {
            const uint64_t t = get_time_us();
            if (t - hud_wifi_us_ > 1000000) { hud_wifi_us_ = t; hud_wifi_on_ = wifi_ap_on(); }
            canopy.wifi_on = hud_wifi_on_;
        }
        canopy.bf             = msp_osd.get_telem();
        int d = link_distance_raw;
        if (d >= 0) { d -= dist_offset; if (d < 0) d = 0; }
        canopy.dist_m = d;
        canopy.v_w  = (int)v_width;
        canopy.v_h  = (int)v_height;
        canopy.fps  = cur_fps;
        canopy.mbps = video_active ? v_bw : 0.0f;
        // Median, not mean: one stalled frame drags an average somewhere the
        // link never actually was, and the figure is there to say what the
        // link normally feels like.
        if (video_active && have_latency_median) {
            canopy.lat_med_ms = latency_median_ms;
            // One per slice in every frame mode: a picture put up whole still
            // has its lower slice lit later, mid-screen.
            canopy.lat_nslices = osd_vars.latency_ring.slice_medians(canopy.lat_slice_ms);
            canopy.have_lat = true;
        }
        prof::mark(prof::kDrawPre);
        draw_canopy_hud(canopy);
        prof::mark(prof::kCanopy);
    }

    // Full 53x20 Betaflight canvas, centred and fitted to the visible area.
    // Drawn on the centre matrix so it stays flat: the side panels are rotated
    // into perspective, which is wrong for a character grid the pilot reads.
    // With the canopy up, the elements whose values it shows read off this
    // canvas are left blank, so a figure is never on screen twice: the
    // voltage it shows (cell or pack, HUD > Canopy Voltage), current, flight
    // mode, speed, Link Quality, DISARMED, and armed the timer it counts. A
    // value the air unit got from the flight controller over MSP instead
    // leaves its element drawn, as do RSSI, altitude, attitude and the rest.
    // With the canopy off the canvas is drawn whole - it is the whole picture
    // then.
    //
    // Never while the canopy is awake: its unfolded blades are meant to be
    // the topmost thing on screen, but this canvas is 53x20
    // cells of dense text with no gaps to read through - over the menu it
    // just looked like noise laid across it. The FC's own OSD is still
    // running underneath; it comes back the moment the menu closes.
    if (bf_osd && !is_menu_open) {
        glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_hud_c);

        const float avail_w = 2.0f * frustum_w;
        const float avail_h = 2.0f * frustum_h;
        const int   gw = MspOsd::grid_w();
        const int   gh = MspOsd::grid_h();

        // Independently, not aspect-locked: Betaflight's HD canvas is
        // defined to map its 53x20 grid onto the WHOLE video frame, one
        // axis at a time, not onto a fixed-aspect block centred in it. The
        // previous line kept the aspect ratio the old side-panel layout
        // used (0.045 / 0.03375, tuned for a narrow strip down each side)
        // and shrank whichever axis ran out first to preserve it. On this
        // screen's 16:9 the width branch was already exact, but the height
        // it derived from that aspect came out to ~89% of the frame - a gap
        // close to a full row's height at both the top and the bottom - so
        // anything near row 0 or row 19 landed inside that margin instead
        // of on the physical edge, however precisely its column was placed.
        // Dividing each axis by its own cell count fills the frame exactly,
        // in whatever aspect that produces per cell - the one the real
        // canvas has - so both ends of both axes land on the true edges.
        const float cw = avail_w / (float)gw;
        const float ch = avail_h / (float)gh;

        msp_osd.set_cell_size(cw, ch);
        // draw_region()'s y is the first row's baseline and rows descend from
        // it, so start one cell below the top of the fitted block.
        unsigned hide = 0;
        if (canopy_on) {
            hide = (volt_mode == 1 ? BfTelem::kElPackV : BfTelem::kElCellV) | BfTelem::kElAmps |
                   BfTelem::kElMode | BfTelem::kElSpeed | BfTelem::kElLq | BfTelem::kElArm;
            if (canopy_arm_prev_ == 1) hide |= BfTelem::kElTimer;   // the canopy's timer slot, armed
            // Only where the canopy's value is that element's: one the flight
            // controller sends over MSP leaves its element on the canvas.
            hide &= msp_osd.get_telem().osd_src;
        }
        msp_osd.draw_region(0, 0, gh, gw,
                            -((float)gw * cw) / 2.0f,
                            ((float)gh * ch) / 2.0f - ch,
                            draw_msp_span_func, hide);
    }

    prof::mark(prof::kBfGrid);
    // Video link warning: rails along the picture's top and bottom edges glow
    // amber, then red, as the link degrades - where the eye already is, without reading anything.
    // Fades in fast and out slowly; red breathes so it cannot be mistaken
    // for a tint in the footage.
    {
        uint64_t t = get_time_us();
        float dt = link_edge_ts_ ? (float)(t - link_edge_ts_) / 1e6f : 0.0f;
        if (dt > 0.1f) dt = 0.1f;
        // At rest (fully off or fully on) the loop draws at 10 Hz, so the time
        // since the last frame is no measure of the fade: start it with one
        // animation frame, not a 100 ms jump.
        if ((link_edge_a_ <= 0.001f || link_edge_a_ >= 0.999f) && dt > 1.0f / 60.0f) dt = 1.0f / 60.0f;
        link_edge_ts_ = t;
        int lvl = link_edge_on_ ? canopy.link_warn : 0;
        if (lvl > 0) link_edge_shown_ = lvl;
        float target = lvl > 0 ? 1.0f : 0.0f;
        float rate   = (target > link_edge_a_) ? 1.0f / 0.15f : 1.0f / 0.60f;
        float step   = rate * dt;
        if (link_edge_a_ < target) link_edge_a_ = fminf(target, link_edge_a_ + step);
        else                       link_edge_a_ = fmaxf(target, link_edge_a_ - step);

        if (link_edge_a_ > 0.001f) {
            float a = link_edge_a_, r, g, b;
            if (link_edge_shown_ >= 2) {
                r = 1.0f; g = 0.30f; b = 0.25f;
                a *= 0.65f + 0.35f * (0.5f + 0.5f * sinf((float)(t % 10000000ULL) / 1e6f * 7.54f));
            } else {
                r = 1.0f; g = 0.72f; b = 0.15f;
            }
            glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp_c);
            // 15% of the picture's height deep, at 75% strength.
            draw_rail_glow(frustum_w, frustum_h, 0.30f * frustum_h, 0.75f * a, r, g, b);
        }
        link_watch_ = canopy.link_up;
        // Fading in or out, or the red pulse: keep frames coming.
        if ((link_edge_a_ > 0.001f && link_edge_a_ < 0.999f) ||
            (link_edge_a_ > 0.001f && link_edge_shown_ >= 2))
            signal_render(prof::kWakeAnim);
    }

    // The canopy awake: labels over the blades, a blade unfolded into its
    // settings, the channel band. Last, over the HUD.
    if (is_menu_open) {
        CanopyIn geo;
        geo.frustum_w = frustum_w;
        geo.frustum_h = frustum_h;
        geo.overscan  = 0.02f;
        geo.s         = s;
        geo.model_l   = model_l;
        geo.model_r   = model_r;
        geo.vp        = tmp;
        draw_canopy_ui(screen_w, screen_h, geo);
        glUseProgram(shader_program);
        if (a_alpha_factor_ != -1) glVertexAttrib1f(a_alpha_factor_, 1.0f);
    }

    if (gallery_capture_) gallery_capture_frame(screen_w, screen_h);   // before the swap
    present_gl_frame();
}

// The end of a frame: swap, wrap the buffer as a DRM framebuffer, and hand it to
// the display. Shared by the HUD and the gallery.
void OSD::present_gl_frame() {
    auto& gb = gl_buffers[0];
    struct gbm_surface *gs = (struct gbm_surface *)gb.bo;
    const int screen_w = dev->output_list->mode.hdisplay;
    const int screen_h = dev->output_list->mode.vdisplay;

    // The display size, whatever REC records: a screen recording and the web
    // view's encoder (a phone watching) are both made at this size.
    DvrRecorder::instance().set_screen_size(screen_w, screen_h);

    prof::mark(prof::kDraw);
    // Profiling only: a native fence for this frame's GPU work, so its real
    // completion time can be read from the kernel without waiting on it.
    EGLSyncKHR prof_sync = EGL_NO_SYNC_KHR;
    if (prof::enabled() && pfn_eglCreateSyncKHR) {
        const EGLint a[] = { EGL_SYNC_NATIVE_FENCE_FD_ANDROID, EGL_NO_NATIVE_FENCE_FD_ANDROID, EGL_NONE };
        prof_sync = pfn_eglCreateSyncKHR(display, EGL_SYNC_NATIVE_FENCE_ANDROID, a);
    }
    // The frame is offered to the display only once the GPU has drawn it. A
    // video flip carries the newest offered OSD frame in its own commit, and
    // the kernel waits for that frame's rendering before the commit takes
    // effect: with the OSD redrawn every refresh (the stats screen) half the
    // video flips missed their vblank by waiting on the GPU - 5 ms more
    // latency than on the live view. Waiting here costs the OSD thread, not
    // the video.
    EGLSyncKHR done_sync = EGL_NO_SYNC_KHR;
    if (pfn_eglCreateSyncKHR && pfn_eglDupNativeFenceFDANDROID) {
        const EGLint a[] = { EGL_SYNC_NATIVE_FENCE_FD_ANDROID, EGL_NO_NATIVE_FENCE_FD_ANDROID, EGL_NONE };
        done_sync = pfn_eglCreateSyncKHR(display, EGL_SYNC_NATIVE_FENCE_ANDROID, a);
    }
    if (done_sync == EGL_NO_SYNC_KHR) glFinish();
    uint64_t prof_submit = get_time_us();
    eglSwapBuffers(display, gb.surface);
    if (prof_sync != EGL_NO_SYNC_KHR) {
        prof::gpu_fence(pfn_eglDupNativeFenceFDANDROID(display, prof_sync), prof_submit);
        pfn_eglDestroySyncKHR(display, prof_sync);
    }
    // closed on every way out of here, the early returns below included
    struct FenceFd { int fd = -1; ~FenceFd() { if (fd >= 0) close(fd); } } done;
    if (done_sync != EGL_NO_SYNC_KHR) {
        done.fd = pfn_eglDupNativeFenceFDANDROID(display, done_sync);
        pfn_eglDestroySyncKHR(display, done_sync);
    }
    prof::mark(prof::kSwap);
    last_render_ts = get_time_us();

    struct gbm_bo *bo = gbm_surface_lock_front_buffer(gs);
    if (!bo) {
        fprintf(stderr, "Failed to lock front buffer\n");
        return;
    }

    uint32_t fb_id;
    if (bo_to_fb.count(bo) == 0) {
        int prime_fd = gbm_bo_get_fd(bo);
        if (prime_fd < 0) {
            fprintf(stderr, "Failed to get prime FD from gbm_bo: %d\n", errno);
            gbm_surface_release_buffer(gs, bo);
            return;
        }
        uint32_t handle = 0;
        if (drmPrimeFDToHandle(dev->drm_fd, prime_fd, &handle) < 0) {
            perror("drmPrimeFDToHandle (OSD)");
            close(prime_fd);
            gbm_surface_release_buffer(gs, bo);
            return;
        }
        // prime_fd is kept: the screen recorder reads the OSD through it
        // (dev->register_osd_fb below). The BOs live as long as the app.

        uint32_t stride = gbm_bo_get_stride(bo);
        uint32_t handles[4] = {handle, 0, 0, 0};
        uint32_t pitches[4] = {stride, 0, 0, 0};
        uint32_t offsets[4] = {0, 0, 0, 0};
        // Mesa allocates the GBM scanout buffer with the GPU's native tiling
        // (e.g. AMD GFX9), which carries a DRM format modifier. A plain
        // drmModeAddFB2 declares no modifier, so the kernel assumes the buffer
        // is linear and amdgpu rejects the mismatch with EINVAL. Pass the bo's
        // real modifier via drmModeAddFB2WithModifiers — matching the video
        // DMA-BUF import path in renderer.cpp. Rockchip/mali returns
        // MOD_INVALID here, in which case we fall back to the plain call.
        uint64_t modifier = gbm_bo_get_modifier(bo);
        uint64_t modifiers[4] = {modifier, 0, 0, 0};
        uint32_t bo_w = gbm_bo_get_width(bo);
        uint32_t bo_h = gbm_bo_get_height(bo);
        uint32_t bo_fmt = gbm_bo_get_format(bo);
        printf("drmModeAddFB2 OSD: dev->drm_fd=%d, screen_w=%d, screen_h=%d, bo_w=%u, bo_h=%u, bo_fmt=0x%08x, handle=%u, stride=%u, modifier=0x%llx\n",
               dev->drm_fd, screen_w, screen_h, bo_w, bo_h, bo_fmt, handle, stride,
               (unsigned long long)modifier);
        int addfb_ret;
        if (modifier != DRM_FORMAT_MOD_INVALID) {
            addfb_ret = drmModeAddFB2WithModifiers(dev->drm_fd, screen_w, screen_h,
                            DRM_FORMAT_ARGB8888, handles, pitches, offsets, modifiers,
                            &fb_id, DRM_MODE_FB_MODIFIERS);
        } else {
            addfb_ret = drmModeAddFB2(dev->drm_fd, screen_w, screen_h,
                            DRM_FORMAT_ARGB8888, handles, pitches, offsets, &fb_id, 0);
        }
        if (addfb_ret) {
            perror("drmModeAddFB2 (OSD)");
            close(prime_fd);
            gbm_surface_release_buffer(gs, bo);
            return;
        }
        bo_to_fb[bo] = fb_id;
        dev->register_osd_fb(fb_id, prime_fd, stride);
    } else {
        fb_id = bo_to_fb[bo];
    }

    if (locked_bo) {
        bo_release_queue.push_back(locked_bo);
    }
    locked_bo = bo;
    prof::mark(prof::kLock);
    if (done.fd >= 0) {
        struct pollfd pfd = { done.fd, POLLIN, 0 };
        poll(&pfd, 1, 100);   // the GPU has finished this frame (see done_sync)
    }
    dev->stage_tile(frame_tile_);   // the picture's rectangle goes with this frame
    dev->set_osd_fb(fb_id);
    prof::mark(prof::kHandoff);

    release_unused_bos(gs);
    prof::frame_end();
}

void OSD::set_ui_scale(float v) {
    pthread_mutex_lock(&osd_mutex);
    osd_vars.ui_scale = v;
    menu_ui_scale = v;
    applied_ui_scale = v;
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::run() {
    SchedulingHelper::configure_thread(SchedulingHelper::ThreadRole::OSD);

    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context);
    init_gl_buffers();
    init_shaders();

    printf("OSD::run GL start (Handled on video frame update).\n");

    // Paint the idle screen immediately instead of waiting for the first
    // background-video frame (or the 1 s refresh timeout) — gets pixels on the
    // display ~200 ms sooner at cold start.
    if (!gl_buffers.empty()) {
        g_dbg_osd_t0.store(get_time_us());   // DEBUG (decode stalls)
        render_gl();
        g_dbg_osd_t1.store(get_time_us());
    }

    // Render pacing. The HUD is redrawn when something on it changes, at a
    // rate that fits what is changing, and never just because a video frame
    // arrived (the video is on its own plane):
    //   - an animation in progress asks for its next frame 16.7 ms out
    //   - input, link events and video edges are drawn promptly, but never
    //     closer than 16 ms to the previous frame (bursts coalesce)
    //   - new MSP data is parsed, and drawn only if it changed the HUD
    //   - otherwise a steady refresh: see next_render_interval_us()
    // Each frame costs a full-screen GPU pass, and on battery the frames not
    // drawn are the saving.
    constexpr uint64_t kMinGapUs = 16000;
    while(!*signal_stop) {
        uint64_t interval = next_render_interval_us();
        // Phase-locked to the display's flips while the gallery animates: a flip starts the
        // next frame, and the timer is only the fallback for when none comes (video stalled).
        const bool lock_flips = phase_lock_.load(std::memory_order_relaxed);
        if (lock_flips && interval < 30000) interval = 30000;
        uint64_t due = loop_start_us_ + interval;

        pthread_mutex_lock(&osd_mutex);
        // MSP data that arrived without a signal (throttled): come back for
        // it one throttle period after the last signal.
        uint64_t wake_at = due;
        if (msp_dirty_ && !msp_wake_)
            wake_at = std::min<uint64_t>(wake_at, last_msp_signal_us_ + 33000);
        uint64_t now = get_time_us();
        uint64_t wait_us = wake_at > now ? wake_at - now : 0;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        long ns = ts.tv_nsec + (long)(wait_us * 1000);
        ts.tv_sec += ns / 1000000000L;
        ts.tv_nsec = ns % 1000000000L;
        int rc = 0;
        while (!render_requested && !msp_wake_ && !(lock_flips && flip_tick_ && anim_pending_) &&
               !*signal_stop && rc != ETIMEDOUT && wait_us > 0)
            rc = pthread_cond_timedwait(&osd_cond, &osd_mutex, &ts);
        now = get_time_us();
        bool requested = render_requested;
        bool msp       = msp_wake_ || (msp_dirty_ && now >= last_msp_signal_us_ + 33000);
        const bool tick = lock_flips && flip_tick_ && anim_pending_;
        bool heartbeat = now >= due || tick;
        flip_tick_ = false;
        render_requested = false;
        msp_wake_ = false;
        if (msp) { msp_dirty_ = false; last_msp_signal_us_ = now; }
        pthread_mutex_unlock(&osd_mutex);

        if (*signal_stop) break;

        // Parse whatever the RX thread staged since the last repaint, so the
        // grid is current before it is drawn.
        drain_msp();
        uint32_t msp_ver = msp_osd.content_version();
        if (!requested && !heartbeat && msp_ver == last_msp_ver_)
            continue;   // MSP arrived, nothing the HUD shows changed
        bool timed_out = heartbeat && !requested;

        if (timed_out) prof::wake(prof::kWakeTimeout);
        else if (!requested) prof::wake(prof::kWakeOther);   // MSP change

        now = get_time_us();
        // Locked to the flips, the flip decides when a frame starts: the usual 16 ms minimum
        // is shorter than a 60 fps flip cycle, so any frame started a little late would set
        // the next by the clock and the phase would slide off the flips for good.
        const uint64_t min_gap = lock_flips ? 12000 : kMinGapUs;
        if (now - loop_start_us_ < min_gap) usleep((useconds_t)(min_gap - (now - loop_start_us_)));

        last_msp_ver_ = msp_ver;
        pthread_mutex_lock(&osd_mutex);
        anim_pending_ = false;   // this frame answers it; render_gl asks again if still animating
        pthread_mutex_unlock(&osd_mutex);
        loop_start_us_ = get_time_us();
        if (!gl_buffers.empty()) {
            g_dbg_osd_t0.store(get_time_us());   // DEBUG (decode stalls)
            render_gl();
            g_dbg_osd_t1.store(get_time_us());
        }
    }
}

// How long the loop may go without drawing when nothing asks it to.
uint64_t OSD::next_render_interval_us() {
    pthread_mutex_lock(&osd_mutex);
    bool anim = anim_pending_;
    pthread_mutex_unlock(&osd_mutex);
    if (anim) return 16667;                                  // animating: 60 Hz

    uint64_t us = (uint64_t)refresh_frequency_ms * 1000;    // idle: 1 s
    // (The HUD moving with the craft asks for frames itself while its spring
    // is settling - see render_gl.)
    // Connected: figures, freshness and stall detection (the red rails come
    // up 400 ms into a stall) - 10 Hz is ample for all of them.
    if (link_watch_ || hud_connected_)
        us = std::min<uint64_t>(us, 100000);
    // DEBUG (decode stalls): echo <ms> > /tmp/kestrel-osd-ms - draw this often
    // instead, looked at every 2 s
    {
        static uint64_t checked = 0;
        static int ms = 0;
        const uint64_t t = get_time_us();
        if (t - checked > 2000000) {
            checked = t;
            int v = 0;
            if (FILE *f = fopen("/tmp/kestrel-osd-ms", "r")) {
                if (fscanf(f, "%d", &v) != 1) v = 0;
                fclose(f);
            }
            if (v != ms) printf("osd: DEBUG drawing every %d ms (0: as usual)\n", v);
            ms = v;
        }
        if (ms > 0 && !anim) us = (uint64_t)ms * 1000;
    }
    return us;
}

void OSD::set_video_resolution(int w, int h) {
    pthread_mutex_lock(&osd_mutex);
	osd_vars.video_width = w;
	osd_vars.video_height = h;
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::update_video_bandwidth(float bw) {
    pthread_mutex_lock(&osd_mutex);
	osd_vars.video_bandwidth = bw;
    
    osd_vars.bitrate_history.push_back(bw);
    if (osd_vars.bitrate_history.size() > 50) osd_vars.bitrate_history.erase(osd_vars.bitrate_history.begin());

    pthread_mutex_unlock(&osd_mutex);
}

void OSD::set_slices_received(bool val) {
    pthread_mutex_lock(&osd_mutex);
    osd_vars.slices_received = val;
    pthread_mutex_unlock(&osd_mutex);
}

bool OSD::get_slices_received() {
    pthread_mutex_lock(&osd_mutex);
    bool val = osd_vars.slices_received;
    pthread_mutex_unlock(&osd_mutex);
    return val;
}

void OSD::update_artosyn_rx_mcs(int mcs) {
    pthread_mutex_lock(&osd_mutex);
    osd_vars.artosyn.rx_mcs_val = mcs;
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::update_artosyn_stats(artosyn_stats v) {
    pthread_mutex_lock(&osd_mutex);
    osd_vars.artosyn = v;
    // Link dropped: re-arm the re-asserts so the next settings broadcast
    // (requested on reconnect) re-applies the AIR-owned values.
    if (v.state != 2) { rf_bw_applied = -1; rf_chan_asserted = false; }

    // A DIFFERENT aircraft: forget the screen, on top of the plain reconnect
    // clear update_artosyn_link_state already does on every link-down ->
    // link-up edge. This one is keyed on the peer MAC instead, for the same
    // aircraft's MAC turning up on a fresh link_state edge this function
    // never saw in between - belt and braces, and reset_screen() is cheap to
    // call twice.
    if (v.state == 2 && v.peer_mac[0] &&
        strncmp(v.peer_mac, osd_peer_mac_, sizeof(osd_peer_mac_)) != 0) {
        const bool had_previous = osd_peer_mac_[0] != 0;
        snprintf(osd_peer_mac_, sizeof(osd_peer_mac_), "%s", v.peer_mac);
        if (had_previous) msp_osd.reset_screen();
        // Calib Distance is per air unit (its radio's MAC): one never
        // calibrated starts from the last calibration made.
        dist_offset = Settings::getInstance().getInt(dist_offset_key(),
                                                     Settings::getInstance().getInt("dist_offset", 115));
        printf("[OSD] air unit %s: distance offset %d\n", osd_peer_mac_, dist_offset);
    }
    pthread_mutex_unlock(&osd_mutex);
}

// Calib Distance's setting for the linked air unit: dist_offset_<its radio's
// MAC>, or the plain dist_offset before any link.
std::string OSD::dist_offset_key() const {
    std::string k = "dist_offset";
    if (!osd_peer_mac_[0]) return k;
    k += '_';
    for (const char *c = osd_peer_mac_; *c; c++) if (*c != ':') k += *c;
    return k;
}
// Saved for this air unit, and as the starting value for one never calibrated.
void OSD::save_dist_offset() {
    Settings::getInstance().set(dist_offset_key(), dist_offset);
    Settings::getInstance().set("dist_offset", dist_offset);
}

// Refresh only the link state, leaving every other figure at its last value.
// The full snapshot is published on frame arrival, which is no use for
// reporting that the frames have stopped; this is called from the source's own
// 400ms poll, which runs whether or not anything is arriving.
void OSD::update_artosyn_link_state(int state) {
    pthread_mutex_lock(&osd_mutex);
    const int prev = osd_vars.artosyn.state;
    osd_vars.artosyn.state = state;
    if (state != 2) { rf_bw_applied = -1; rf_chan_asserted = false; }
    // Any reconnect, not just a different aircraft (update_artosyn_stats
    // handles that case by peer MAC). The "same air unit keeps its baseline,
    // so don't clear" reasoning there assumed the FC's DisplayPort layer
    // always resends every element after a drop - in practice a real RF
    // dropout does not reliably do that, and cells our grid held from before
    // the drop stay on screen forever if the reconnect's writes land
    // somewhere else, or simply fewer of them. A clean grid costs one blank
    // frame; a screen that is still showing the last connection's numbers
    // costs nobody noticing they are stale.
    if (prev != 2 && state == 2) msp_osd.reset_screen();
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::notify_handshake_sent() {
    pthread_mutex_lock(&osd_mutex);
    handshake_sent_us_ = get_time_us();
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::update_adapt_stats(adapt_stats v) {
    pthread_mutex_lock(&osd_mutex);
    osd_vars.adapt = v;
    pthread_mutex_unlock(&osd_mutex);
}

// Append one row correlating the RF link (RX MCS/throughput/SNR), the result
// (received video Mbps, latency, loss) and the air's adaptation decisions
// (tier, applied bitrate, water-level drops/latency). Pull /tmp/kestrel-adapt.csv
// after a run to graph the adaptation — debuggable without a network to the air.
void OSD::log_csv_row() {
    pthread_mutex_lock(&osd_mutex);
    FILE* fp = (FILE*)csv_fp;
    if (!fp) {
        fp = fopen("/tmp/kestrel-adapt.csv", "w");
        csv_fp = fp;
        if (fp)
            fprintf(fp, "t_ms,video_mbps,rx_mbps,rx_mcs,snr_db,lost,lat_avg_ms,lat_max_ms,"
                        "tier,applied_kbps,thr_kbps,wl_drops,wl_lat_ms,fps,"
                        "gap_p99_ms,gap_max_ms,stutters,ldpc_err\n");
    }
    if (fp) {
        // Render cadence from hardware flip completions — the objective "did the
        // viewer see a hiccup" measure. stutters is cumulative; p99/max cover the
        // rolling ~2s gap window.
        RenderCadence rc = dev ? dev->get_render_cadence() : RenderCadence{0, 0, 0};
        fprintf(fp, "%llu,%.2f,%.2f,%d,%.1f,%u,%.1f,%.1f,%d,%u,%u,%u,%u,%u,%.1f,%.1f,%u,%.4f\n",
            (unsigned long long)get_time_ms(),
            osd_vars.video_bandwidth,
            osd_vars.artosyn.rx_data_rate_kbps / 1000.0f,
            ar_mcs_label(osd_vars.artosyn.rx_mcs_val),
            osd_vars.artosyn.snr,
            osd_vars.link_stats.count_p_lost,
            osd_vars.total_latency_avg,
            osd_vars.total_latency_max,
            osd_vars.adapt.tier,
            osd_vars.adapt.applied_bitrate_kbps,
            osd_vars.adapt.link_throughput_kbps,
            osd_vars.adapt.wl_drops,
            osd_vars.adapt.wl_latency_ms,
            osd_vars.adapt.fps,
            rc.p99_ms,
            rc.max_ms,
            rc.stutters,
            osd_vars.artosyn.ldpc_error);
        fflush(fp);
    }
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::update_stats(int current_framerate, latency_stats stats) {
    pthread_mutex_lock(&osd_mutex);
	osd_vars.proc_latency_avg = stats.proc_latency.avg;
	osd_vars.proc_latency_max = stats.proc_latency.max;
	osd_vars.proc_latency_min = stats.proc_latency.min;
    osd_vars.proc_latency_values = stats.proc_latency.values;

	osd_vars.decoding_latency_avg = stats.decoding_latency.avg;
	osd_vars.decoding_latency_max = stats.decoding_latency.max;
	osd_vars.decoding_latency_min = stats.decoding_latency.min;
    osd_vars.decoding_latency_values = stats.decoding_latency.values;

	osd_vars.display_latency_avg = stats.display_latency.avg;
	osd_vars.display_latency_max = stats.display_latency.max;
	osd_vars.display_latency_min = stats.display_latency.min;
    osd_vars.display_latency_values = stats.display_latency.values;

	osd_vars.tx_latency_avg = stats.tx_latency.avg;
	osd_vars.tx_latency_max = stats.tx_latency.max;
	osd_vars.tx_latency_min = stats.tx_latency.min;
    osd_vars.tx_latency_values = stats.tx_latency.values;


	osd_vars.total_latency_avg = stats.total_latency.avg;
	osd_vars.total_latency_max = stats.total_latency.max;
	osd_vars.total_latency_min = stats.total_latency.min;
    osd_vars.total_latency_values = stats.total_latency.values;

	osd_vars.frame_pace_avg = stats.frame_pace.avg;
	osd_vars.frame_pace_max = stats.frame_pace.max;
	osd_vars.frame_pace_min = stats.frame_pace.min;
    osd_vars.frame_pace_values = stats.frame_pace.values;


    osd_vars.proc_latency_worst = stats.proc_latency_worst;
    osd_vars.decoding_latency_worst = stats.decoding_latency_worst;
    osd_vars.display_latency_worst = stats.display_latency_worst;
    osd_vars.tx_latency_worst = stats.tx_latency_worst;
    osd_vars.latency_history.push_back(stats.total_latency.avg);
    if (osd_vars.latency_history.size() > 50) osd_vars.latency_history.erase(osd_vars.latency_history.begin());

	osd_vars.current_framerate = current_framerate;
    pthread_mutex_unlock(&osd_mutex);

    if (console_stats) {
        printf("\033[H\033[2J");

        const double MAX_LATENCY_SCALE = 45.0; 
        const int GRAPH_BAR_WIDTH = 40;        

        printf(BOLDCYAN "--- Performance Metrics ---\n" RESET);
        printf(BOLDYELLOW "Current Framerate: %s%s%d FPS%s\n", RESET, BOLDGREEN, current_framerate, RESET);
        printf(BOLDYELLOW "Video Bandwidth:   %s%s%.2f Mbps%s\n", RESET, BOLDGREEN, osd_vars.video_bandwidth, RESET);

        const char* link_color = RESET;
        if (osd_vars.link_stats.p_lost_rate > 0) link_color = RED;
        else if (osd_vars.link_stats.p_fec_recovered_rate > 0) link_color = YELLOW;
        
        printf(BOLDYELLOW "Link Stats:        %s%s%.2f Mbps | PPS: %u | FEC %d:%d | Loss: %u | Recovered: %u%s\n", 
            RESET, link_color,
            osd_vars.video_bandwidth, osd_vars.link_stats.p_all_rate, 
            osd_vars.link_stats.fec_k, osd_vars.link_stats.fec_n,
            osd_vars.link_stats.p_lost_rate, osd_vars.link_stats.p_fec_recovered_rate,
            RESET);
        printf("\n");

        printf(BOLDMAGENTA "RF - encoder out to the slice here (ms):%s\n", RESET);
        print_latency_dist(osd_vars.proc_latency_values, osd_vars.proc_latency_avg, osd_vars.proc_latency_min, osd_vars.proc_latency_max, MAX_LATENCY_SCALE, GRAPH_BAR_WIDTH, BOLDCYAN);

        printf(BOLDMAGENTA "Decoding - the slice here to its rows decoded (ms):%s\n", RESET);
        print_latency_dist(osd_vars.decoding_latency_values, osd_vars.decoding_latency_avg, osd_vars.decoding_latency_min, osd_vars.decoding_latency_max, MAX_LATENCY_SCALE, GRAPH_BAR_WIDTH, BOLDYELLOW);
        
        printf(BOLDMAGENTA "\nDisplay (VSync Aging) (ms):%s\n", RESET);
        print_latency_dist(osd_vars.display_latency_values, osd_vars.display_latency_avg, osd_vars.display_latency_min, osd_vars.display_latency_max, MAX_LATENCY_SCALE, GRAPH_BAR_WIDTH, RED);
        
        printf(BOLDMAGENTA "\nCapture -> encoder out (ms):%s\n", RESET);
        print_latency_dist(osd_vars.tx_latency_values, osd_vars.tx_latency_avg, osd_vars.tx_latency_min, osd_vars.tx_latency_max, MAX_LATENCY_SCALE, GRAPH_BAR_WIDTH, CYAN);
        
        printf(BOLDMAGENTA "\nGlass-to-glass estimate (ms):%s\n", RESET);
        print_latency_dist(osd_vars.total_latency_values, osd_vars.total_latency_avg, osd_vars.total_latency_min, osd_vars.total_latency_max, MAX_LATENCY_SCALE, GRAPH_BAR_WIDTH, GREEN);

        printf(BOLDMAGENTA "\nFrame Pace (ms per frame):%s\n", RESET);
        print_latency_dist(osd_vars.frame_pace_values, osd_vars.frame_pace_avg, osd_vars.frame_pace_min, osd_vars.frame_pace_max, MAX_LATENCY_SCALE, GRAPH_BAR_WIDTH, GREEN);

        int width_chars = 80;
        int height_chars = 6;
        float max_ms = 70.0f; 
        std::vector<LatencyFrame> frames;
        osd_vars.latency_ring.copy_to(frames, current_framerate > 0 ? current_framerate * 5 : 600);

        if (!frames.empty()) {
            printf(BOLDCYAN "\nLatency History Timeline (Braille, 5s Window):\n" RESET);
            int num_samples = (current_framerate > 0) ? current_framerate * 5 : 600;
            std::vector<LatencyFrame> data;
            if ((int)frames.size() >= num_samples) {
                data.assign(frames.end() - num_samples, frames.end());
            } else {
                data = frames;
                while ((int)data.size() < num_samples) data.insert(data.begin(), {0,0,0,0,0,0});
            }

            for (int r = height_chars - 1; r >= 0; --r) {
                printf("%3.0fms %s│", (max_ms / height_chars) * (r + 1), RESET);
                float row_lat_base = (max_ms / height_chars) * r;

                for (int c = 0; c < width_chars; ++c) {
                    unsigned char dots = 0;
                    const char* sample_color = RESET;
                    
                    for (int sub_c = 0; sub_c < 2; ++sub_c) {
                        int dot_column = c * 2 + sub_c;
                        int sample_idx = (dot_column * (int)data.size()) / (width_chars * 2);
                        if (sample_idx >= (int)data.size()) sample_idx = (int)data.size() - 1;

                        const auto& f = data[sample_idx];
                        float total = f.processing_ms + f.net_ms + f.dec_ms + f.disp_ms;
                        float total_dots = (total / max_ms) * (height_chars * 4);
                        
                        int row_start_dot = r * 4;
                        for (int d = 0; d < 4; ++d) {
                            if (total_dots > (row_start_dot + d)) {
                                if (sub_c == 0) {
                                     if (d == 0) dots |= 0x40; else if (d == 1) dots |= 0x04; else if (d == 2) dots |= 0x02; else if (d == 3) dots |= 0x01;
                                } else {
                                     if (d == 0) dots |= 0x80; else if (d == 1) dots |= 0x20; else if (d == 2) dots |= 0x10; else if (d == 3) dots |= 0x08;
                                }
                                
                                float cur = 0;
                                if (row_lat_base < (cur += f.processing_ms)) sample_color = CYAN;
                                else if (row_lat_base < (cur += f.net_ms)) sample_color = BOLDCYAN;
                                else if (row_lat_base < (cur += f.dec_ms)) sample_color = BOLDYELLOW;
                                else sample_color = RED;
                            }
                        }
                    }
                    if (dots == 0) printf(" ");
                    else printf("%s%s%s", sample_color, get_braille_char(dots).c_str(), RESET);
                }
                printf("\n");
            }
            printf("        └");
            for (int i = 0; i < width_chars; ++i) printf("─");
            printf("\n");
            printf("Legend: %sCap%s %sEnc%s %sNet%s %sDec%s %sDisp%s\n", BOLDGREEN, RESET, CYAN, RESET, BOLDCYAN, RESET, BOLDYELLOW, RESET, RED, RESET);
        }

        printf(BOLDCYAN "---------------------------\n" RESET);
        fflush(stdout);
    }
}

void OSD::update_link_stats(packets_stats v) {
    pthread_mutex_lock(&osd_mutex);
	osd_vars.link_stats = v;
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::add_latency_frame(LatencyFrame frame) {
    pthread_mutex_lock(&osd_mutex);
    
    static uint64_t last_frame_time_us = 0;
    static float smoothed_pace = 0.0f;
    uint64_t now_us = get_time_us();
    
    bool is_dummy = (frame.processing_ms == 0.0f && 
                     frame.net_ms == 0.0f && frame.dec_ms == 0.0f && frame.disp_ms == 0.0f);
    if (is_dummy) {
        frame.pace_ms = 0.0f;
        last_frame_time_us = 0;
        smoothed_pace = 0.0f;
    } else {
        float raw_pace = 0.0f;
        if (last_frame_time_us != 0) {
            raw_pace = (now_us - last_frame_time_us) / 1000.0f;
        }
        last_frame_time_us = now_us;
        
        if (smoothed_pace == 0.0f) {
            smoothed_pace = raw_pace;
        } else {
            smoothed_pace = 0.08f * raw_pace + 0.92f * smoothed_pace;
        }
        frame.pace_ms = smoothed_pace;
    }
    
    // Snapshot current bitrate alongside the latency frame
    frame.video_mbps = osd_vars.video_bandwidth;
    frame.rf_mbps = (float)osd_vars.artosyn.rx_data_rate_kbps / 1000.0f;
    frame.t_us = now_us;
    frame.lost = osd_vars.link_stats.count_p_lost;
    frame.snr = (int8_t)std::max(-127.0f, std::min(127.0f, osd_vars.artosyn.snr));
    frame.mcs = (uint8_t)std::max(0, std::min(255, osd_vars.artosyn.rx_mcs_val));
    frame.bw_idx = osd_vars.artosyn.state == 2 && osd_vars.artosyn.rf_bw_idx >= 0 && osd_vars.artosyn.rf_bw_idx < 6
                   ? (int8_t)osd_vars.artosyn.rf_bw_idx : (int8_t)-1;

    osd_vars.latency_ring.push(frame);
    osd_vars.latency_history_last_us = now_us;
    if (!is_dummy) latency_frames_total_.fetch_add(1, std::memory_order_relaxed);
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::set_camera_config(int ev_x10, int sat, int contrast, int sharp,
                            int scene, int awb, int angle,
                            int dnr3d, int focus) {
    pthread_mutex_lock(&osd_mutex);
    for (int i = 0; i < kEvCount; i++)      if (kEvSteps[i]    == ev_x10)  menu_cam_ev      = i;
    for (int i = 0; i < kWbCount; i++)      if (wb_kelvin(i)   == awb)     menu_cam_wb      = i;
    menu_cam_sat      = std::min(kCamSatMax,      std::max(0, sat));
    menu_cam_contrast = std::min(kCamContrastMax, std::max(0, contrast));
    menu_cam_sharp    = std::min(kCamSharpMax,    std::max(0, sharp));
    for (int i = 0; i < kSceneCount; i++) if (kSceneVals[i] == scene) menu_cam_scene = i;
    menu_cam_flip = angle ? 1 : 0;
    for (int i = 0; i < kDnrCount; i++)   if (kDnrVals[i]   == dnr3d) menu_cam_3dnr  = i;
    if (dnr3d == 1) menu_cam_3dnr = 4;   // the old Off: the air runs the bin's, as Auto
    menu_cam_focus = focus ? 1 : 0;
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::set_camera_angle(int angle) {
    pthread_mutex_lock(&osd_mutex);
    menu_cam_flip = angle ? 1 : 0;
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::set_camera_dnr(int dnr3d) {
    pthread_mutex_lock(&osd_mutex);
    for (int i = 0; i < kDnrCount; i++) if (kDnrVals[i] == dnr3d) menu_cam_3dnr = i;
    if (dnr3d == 1) menu_cam_3dnr = 4;   // the bin's own: Auto
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::menu_mark_dirty(bool on) {
    if (menu_tab < 0 || menu_tab >= kMenuTabs) return;
    if (menu_index < 0 || menu_index >= kMenuRows) return;   // -1 is the tab strip
    menu_dirty[menu_tab][menu_index] = on;
    if (!on) menu_pending_from[menu_tab][menu_index][0] = '\0';
}

// One step on SYSTEM > Time Zone's Area, Country or Timezone row, from the zone
// in menu_zone_: the zone it lands on. Area walks NOT SET and the areas and
// lands on the area's first country and zone; Country walks the area's
// countries and lands on its first zone; Timezone walks the country's zones.
int OSD::menu_step_zone(int row, int dir) {
    const int z = menu_zone_;
    if (row == kSysArea) {
        const int cur = z < 0 ? 0 : zones::zone(z).area + 1;
        const int v = menu_step(cur, dir, zones::area_count() + 1);
        if (v == cur) return z;
        if (v == 0) return -1;
        const std::vector<int> cs = zones::countries_in(v - 1);
        if (cs.empty()) return z;
        const std::vector<int> zs = zones::zones_in(v - 1, cs[0]);
        return zs.empty() ? z : zs[0];
    }
    if (z < 0) return z;
    const int area = zones::zone(z).area, c = zones::zone(z).country;
    if (row == kSysCountry) {
        const std::vector<int> cs = zones::countries_in(area);
        const int cur = (int)(std::find(cs.begin(), cs.end(), c) - cs.begin());
        if (cur >= (int)cs.size()) return z;
        const int v = menu_step(cur, dir, (int)cs.size());
        if (v == cur) return z;
        const std::vector<int> zs = zones::zones_in(area, cs[v]);
        return zs.empty() ? z : zs[0];
    }
    const std::vector<int> zs = zones::zones_in(area, c);
    const int cur = (int)(std::find(zs.begin(), zs.end(), z) - zs.begin());
    if (cur >= (int)zs.size()) return z;
    return zs[menu_step(cur, dir, (int)zs.size())];
}

// One press on a row that holds a set of values. For the arrows it wraps, so
// a row never dead-ends. While a row is being enumerated (menu_stepping_) it
// stops at the ends instead: that is what lets menu_options walk to the start
// of a set and along it to the end, and so learn a row's values in their own
// order without a table of every row's domain.
int OSD::menu_step(int v, int dir, int n) const {
    if (n <= 1) return 0;
    if (menu_stepping_) {
        int r = v + dir;
        return r < 0 ? 0 : (r >= n ? n - 1 : r);
    }
    return (v + dir + n) % n;
}

// What Enter does on an action row, and how the action is going. The hint
// is the item's own unless the action is under way or has just finished,
// when the verb changes with it: a running bind offers the cancel, a failed
// one the retry. The status is empty for anything that finishes at once.
void OSD::menu_action_status(int tab, int i, char* hint, size_t hcap,
                             char* status, size_t scap) {
    hint[0] = '\0'; status[0] = '\0';
    std::vector<MenuItem> rows = menu_items(tab);
    if (i < 0 || i >= (int)rows.size() || rows[i].type != 3) return;
    snprintf(hint, hcap, "%s", rows[i].hint ? rows[i].hint : "ENTER");

    if (tab == 1) {
        auto rfi = ar_rf_items();
        if (i < (int)rfi.size() && rfi[i].second == 9) {
            const unsigned p = Ar8030Source::bind_peer.load();
            switch (Ar8030Source::bind_state.load()) {
                case Ar8030Source::BIND_RUNNING:
                    snprintf(hint, hcap, "ENTER: CANCEL");
                    snprintf(status, scap, "BINDING  %ds", Ar8030Source::bind_secs_left.load());
                    break;
                case Ar8030Source::BIND_OK:
                    snprintf(status, scap, "BOUND  %02X:%02X:%02X:%02X",
                             (p >> 24) & 0xFF, (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF);
                    break;
                case Ar8030Source::BIND_FAILED:
                    snprintf(hint, hcap, "ENTER: RETRY");
                    snprintf(status, scap, "NO AIR UNIT FOUND");
                    break;
                default: break;
            }
        }
    }
}

// The canopy has no telemetry link of its own: every figure it shows is read
// off the Betaflight OSD canvas (msp_osd.cpp, scrape_locked). So the help for
// HUD Style, while CANOPY is the value under the cursor, is the list of
// elements the pilot has to enable in Betaflight for the canopy to be
// complete - said here, where the choice is made, rather than in a manual.
// And how they must sit: an element is recognised by its glyphs, so one that
// overlaps a neighbour can lose them - a voltage whose V was blanked by the
// next element's padding stayed on screen as a bare figure under the canopy.
// SYSTEM > Screen Mode's choices: the modes the connector in use advertises,
// progressive only (interlaced ones do not composite here), each size and
// rate once, largest and fastest first. Read without probing the connector -
// a probe re-reads the EDID - and kept once there is a list.
const std::vector<OSD::ScreenModeOpt>& OSD::screen_modes() {
    if (!screen_modes_.empty() || !dev || !dev->output_list || dev->drm_fd < 0) return screen_modes_;
    drmModeConnector* c = drmModeGetConnectorCurrent(dev->drm_fd, dev->output_list->connector.id);
    if (!c) return screen_modes_;
    for (int i = 0; i < c->count_modes; i++) {
        const drmModeModeInfo& m = c->modes[i];
        if (m.flags & DRM_MODE_FLAG_INTERLACE) continue;
        const ScreenModeOpt o{m.hdisplay, m.vdisplay, (int)m.vrefresh};
        if (std::none_of(screen_modes_.begin(), screen_modes_.end(), [&](const ScreenModeOpt& x) {
                return x.w == o.w && x.h == o.h && x.hz == o.hz; }))
            screen_modes_.push_back(o);
    }
    drmModeFreeConnector(c);
    std::sort(screen_modes_.begin(), screen_modes_.end(), [](const ScreenModeOpt& a, const ScreenModeOpt& b) {
        return a.w * a.h != b.w * b.h ? a.w * a.h > b.w * b.h : a.hz > b.hz; });
    return screen_modes_;
}

// screen_mode as a menu index: 0 when unset (Auto). A mode the screen does not
// advertise - set by hand, or from another display - reads as the mode the
// display was actually given, which is what kestrel fell back to.
int OSD::screen_mode_saved() {
    const std::vector<ScreenModeOpt>& modes = screen_modes();
    unsigned w, h, hz;
    const std::string s = screen_confirm_
        ? (screen_confirm_mode_ == "auto" ? std::string() : screen_confirm_mode_)
        : Settings::getInstance().getString(screen_key_, "");
    if (sscanf(s.c_str(), "%ux%u@%u", &w, &h, &hz) != 3) return 0;
    for (size_t i = 0; i < modes.size(); i++)
        if (modes[i].w == (int)w && modes[i].h == (int)h && modes[i].hz == (int)hz) return (int)i + 1;
    if (dev && dev->output_list)
        for (size_t i = 0; i < modes.size(); i++)
            if (modes[i].w == dev->output_list->mode.hdisplay && modes[i].h == dev->output_list->mode.vdisplay &&
                modes[i].hz == (int)dev->output_list->mode.vrefresh)
                return (int)i + 1;
    return 0;
}

void OSD::begin_screen_mode_confirm(const std::string& mode) {
    pthread_mutex_lock(&osd_mutex);
    screen_confirm_ = true;
    screen_confirm_mode_ = mode;
    // Long enough to find the button once the picture is back; short enough
    // that a black screen is not a long wait.
    screen_confirm_deadline_ms_ = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count()) + 15000;
    menu_open_at_start = true;
    render_requested = true;
    pthread_cond_signal(&osd_cond);
    pthread_mutex_unlock(&osd_mutex);
}

int OSD::sync_confirm_left_s() {
    const uint64_t now = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return now >= sync_confirm_deadline_ms_ ? 0 : (int)((sync_confirm_deadline_ms_ - now + 999) / 1000);
}

int OSD::screen_confirm_left_s() {
    const uint64_t now = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return now >= screen_confirm_deadline_ms_ ? 0 : (int)((screen_confirm_deadline_ms_ - now + 999) / 1000);
}

int OSD::screen_mode_menu() {
    if (menu_screen_mode < 0) menu_screen_mode = screen_mode_saved();
    return menu_screen_mode;
}

std::string OSD::screen_mode_label(int idx) {
    const std::vector<ScreenModeOpt>& modes = screen_modes();
    if (idx <= 0 || idx > (int)modes.size()) return "Auto";
    char b[32];
    snprintf(b, sizeof(b), "%dx%d @ %d Hz", modes[idx - 1].w, modes[idx - 1].h, modes[idx - 1].hz);
    return b;
}

// The WiFi board's internal antenna cannot hold a link for the video stream,
// and the radio sits next to the RC link: say so on the row, on or off.

const char* OSD::menu_help_warning(int tab, int i) {
    if (tab == kTabSystem && i == 3 && screen_confirm_) {
        static char ask[96];
        snprintf(ask, sizeof(ask), "Keep this mode? Enter keeps it - going back in %d s.",
                 screen_confirm_left_s());
        return ask;
    }
    if (tab == 3 && i == kDispRefreshSync && sync_confirm_) {
        static char ask[96];
        snprintf(ask, sizeof(ask), "Keep it? Enter keeps it - back to %s in %d s.",
                 refresh_sync_label(sync_confirm_prev_), sync_confirm_left_s());
        return ask;
    }
    if (tab == kTabSystem && i == 3 && menu_dirty[kTabSystem][3])
        return "Enter tries it: the app restarts, then asks whether to keep it.";
    if (tab == kTabSystem && i == 4)
        return "Warning: Before enabling, plug an external antenna to the wifi uFL connector.";
    return nullptr;
}

// The DVR Source row's value as the menu shows it: FPV or SCREEN.
int OSD::dvr_source_now() const {
    return dvr_screen ? 1 : 0;
}

int OSD::hud_row(int shown) const {
    const bool voltage_listed = menu_hud_style == kHudCanopy;
    return (!voltage_listed && shown >= kHudRowVoltage) ? shown + 1 : shown;
}

const char* OSD::menu_help_text(int tab, int i, const char* fallback) {
    if (tab == 3 && i == kDispFrameMode) {
        // Lowest latency first; what each one costs.
        switch (menu_frame_mode) {
        case kFrameSplit:
            return "Lowest latency: the top of the picture goes up before the bottom.\n"
                   "May tear in the middle on fast motion. No gain on a screen that buffers frames.";
        case kFrameWholeDecode:
            return "Highest latency, the most conservative: the decoder only gets\n"
                   "whole pictures.";
        default:
            return "Low latency, whole pictures: slices are decoded as they land, and\n"
                   "only whole pictures go up. No tearing. The default.";
        }
    }
    if (tab == 3 && i == kDispRefreshSync) {
        // With the screen's refresh and the camera's rate known (the display
        // lock's, measured): what the value shown does to this screen in this
        // video mode, and while it is the one running, whether it holds now.
        const float hz = g_dl_mode_hz.load(), fps = g_dl_camera_fps.load();
        if (hz > 0 && fps > 0) {
            static char t[256];
            const bool live = menu_refresh_sync == g_display_lock.load();
            const char* why = g_dl_why.load();
            char now[96] = "";
            if (live && menu_refresh_sync) {
                if (g_dl_holding.load()) snprintf(now, sizeof(now), " Holding now.");
                else if (why) snprintf(now, sizeof(now), " Now: %s.", why);
            }
            switch (menu_refresh_sync) {
            case 1: {
                const float k = std::round(hz / fps);
                if (k >= 1 && std::fabs(hz - k * fps) <= 0.01f * hz)
                    snprintf(t, sizeof(t), "Holds this screen's %.2f Hz just after each picture, %.0f refresh%s a\n"
                             "camera frame (%.2f fps). Off if the screen blanks.%s", hz, k, k > 1 ? "es" : "", fps, now);
                else
                    snprintf(t, sizeof(t), "Cannot hold here: %.2f Hz is not a whole multiple of the camera's\n"
                             "%.2f fps. Screen Mode Auto picks one that is, or use FREE.", hz, fps);
                return t;
            }
            case 2: {
                if (fps > hz / 0.999f) {
                    snprintf(t, sizeof(t), "Cannot slow this screen: the camera (%.2f fps) is faster than\n"
                             "its %.2f Hz. Pick a faster screen mode.", fps, hz);
                    return t;
                }
                const float k = std::max(1.0f, std::floor(hz / fps + 1e-6f));
                char per[40];
                if (k > 1) snprintf(per, sizeof(per), "%.0f refreshes a camera frame", k);
                else snprintf(per, sizeof(per), "a refresh a camera frame");
                snprintf(t, sizeof(t), "Slows this screen from %.2f Hz to %.2f Hz in this video mode, %s.\n"
                         "Only for screens that take any frame length.%s", hz, k * fps, per, now);
                return t;
            }
            default:
                snprintf(t, sizeof(t), "The screen refreshes on its own timing: %.2f Hz, the camera %.2f fps.\n"
                         "ON or FREE hold it in step with the pictures.", hz, fps);
                return t;
            }
        }
        switch (menu_refresh_sync) {
        case 1:  return "Holds the refresh just after each picture arrives, a line at a time.\n"
                        "Needs a refresh in step with the camera (Screen Mode Auto). Off if the screen blanks.";
        case 2:  return "Each frame made as long as the camera's, at the fastest mode: for screens that\n"
                        "show their refresh change (118, 100 Hz...) without blanking. Lowest latency.";
        default: return "The screen refreshes on its own timing. ON or FREE hold it in step with\n"
                        "the pictures - FREE only for a screen that takes any frame length.";
        }
    }
    if (tab == kTabDvr && i == 0) {
        switch (menu_dvr_source) {
        case 1:  return "Records the screen - video and HUD, as you see them - 60 times a second.\n"
                        "Adds about 1 ms to the FPV feed's latency while recording.";
        default: return "Records the FPV stream as the air unit sends it: full quality, no HUD.\n"
                        "No impact on the FPV feed's latency.";
        }
    }
    if (tab == kTabSystem && i == 3) {
        if (menu_dirty[kTabSystem][3] || screen_confirm_) return "";   // the warning says it
        static std::string mode;
        char now[64] = "--";
        if (dev && dev->output_list)
            snprintf(now, sizeof(now), "%dx%d @ %d Hz", dev->output_list->mode.hdisplay,
                     dev->output_list->mode.vdisplay, dev->output_list->mode.vrefresh);
        mode = std::string("Showing ") + now + ". Auto refreshes in step with the camera's frame rate\n"
               "where the screen can, else at the highest refresh it offers.";
        return mode.c_str();
    }
    if (tab == kTabSystem && i == kSysTimezone && menu_zone_ >= 0) {
        // Which zone exactly, and whose rules the channel page uses.
        static std::string zone_help;
        const zones::Zone& z = zones::zone(menu_zone_);
        const zones::Country& c = zones::country(z.country);
        zone_help = std::string(z.name) + ". Only the clock follows this; the channel rules are " +
                    c.name + "'s either way" +
                    (zones::has_rules(z.country) ? "." : std::string(", and none are known for it yet."));
        return zone_help.c_str();
    }
    if (tab == kTabSystem && i == 4) {
        // While a change is pending, the "Pending: OFF -> ON" line says it
        // all; the antenna warning above it comes from menu_help_warning().
        if (menu_dirty[kTabSystem][4]) return "";
        static std::string wifi;          // rebuilt each frame the row is selected
        wifi = wifi_ap_describe();
        return wifi.c_str();
    }
    if (tab == 2 && i == kHudRowStyle && menu_hud_style == kHudCanopy)
        return "Battery, flight mode and GPS come from the flight controller "
               "(polled over MSP) and the HUD's movement from the camera's IMU.\n"
               "Link quality is still read off Betaflight's OSD: enable that "
               "element there.";
    return fallback;
}

// A row's value as words: menu_value_text without the "< >" chevrons and
// without the dirty mark, so it can be compared and quoted.
void OSD::menu_plain_value(int tab, int i, char* out, size_t cap) {
    char raw[64];
    menu_value_text(tab, i, raw, sizeof(raw));
    const char* b = raw;
    if (b[0] == '<' && b[1] == ' ') b += 2;
    size_t n = strlen(b);
    if (n >= 2 && b[n - 2] == ' ' && b[n - 1] == '*') n -= 2;
    if (n >= 2 && b[n - 2] == ' ' && b[n - 1] == '>') n -= 2;
    if (n >= cap) n = cap - 1;
    memcpy(out, b, n); out[n] = '\0';
}

// The Video tab's rows as the handlers below number them, from where they sit
// on screen: Max Bitrate (10) is listed second, right after Video Mode.
static int video_row(int shown) {
    if (shown == 1) return 10;
    return shown >= 2 && shown <= 10 ? shown - 1 : shown;
}

// AIR UNIT > INFO's capabilities rows: their help, which also marks them.
static const char kAirCapsHelp[] =
    "What this air unit's kestrel-air has on. Air Timing: its times in each slice. "
    "Radio Clock: pictures stamped on the radio's clock. Frame Clock: its radio's frame, for exact latency. "
    "Sensor Timing: readout and exposure, for STATS. Intra Refresh: no keyframe spikes. "
    "Camera IMU: the camera's gyro with the video. FC IMU: the flight controller's gyro instead. "
    "Bandwidth Cap: takes the VRX's limit. "
    "864p120, 864p144: Video Mode offers a 1536x864 window at 120 or 144 fps.";

// A row label made at run time (the capabilities list), kept for good: a
// MenuItem holds only the pointer, and a set's strings never move.
const char* OSD::menu_intern(const std::string& s) {
    static std::mutex m;
    static std::set<std::string> pool;
    std::lock_guard<std::mutex> g(m);
    return pool.insert(s).first->c_str();
}

std::vector<OSD::MenuItem> OSD::menu_items(int tab) const {
    std::vector<MenuItem> items;
    if (tab == 0) { // Video Tab
        {
            // Grouped by what the setting acts on: the format block first,
            // then the capture settings, then everything applied to the image
            // after it has been captured. Rows render bottom-up, so index 0
            // sits nearest the tab strip.
            items = {
                // Ratio (aspect ratio, sky cmd 0x07) was here but is gone:
                // ACKed on send, but this air unit never reflects it in its
                // own status echo the way every other camera field does, and
                // there is no other way to tell it landed - looks unsupported
                // on this hardware. See ar8030-camera-settings-verification.
                //
                // Anti-Flicker was here too and is also gone: stock's own
                // Camera menu simply has no such row (Scene/EV/Saturation/
                // Sharpness/Contrast/WB/Rotate/Ratio/3D DNR, confirmed
                // against a photo of stock's UI) - it was never a real
                // setting to expose, whatever cmd 0x1F does on the wire.
                // The air unit's video bitrate: AUTO follows the link; a cap is
                // a kestrel-air command (sky cmd 0x40), ignored by stock. Next to
                // Video Mode, the other setting that decides the stream; listed
                // here, handled as row 10 (video_row).
                {"Video Mode", 1},
                air_stock_
                    ? MenuItem{"Max Bitrate", 0, "Needs an fpvOS air unit: this one runs the stock "
                                                 "air app, which ignores the cap.", nullptr, true}
                    : MenuItem{"Max Bitrate", 1, "Cap on the video bitrate. AUTO follows the link. A cap "
                                                 "below it gives a steadier picture and less time on the "
                                                 "air per frame. fpvOS air units only."},
                {"Rotate", 1},
                {"Scene", 1}, {"EV", 1}, {"White Balance", 1},
                {"Saturation", 1}, {"Contrast", 1}, {"Sharpness", 1}, {"3D DNR", 1},
                // Stock's own Display tab (photo-confirmed), not Camera - but
                // it is still a sky-wire command to the air unit (cmd 0x0E,
                // CMD_SET_CHN_FOCUS), so it lives with the other AR8030
                // camera settings here rather than in our DISPLAY tab, which
                // only holds goggle-local drawing settings.
                {"Focus Mode", 1}
            };
        }
    } else if (tab == 1) { // RF Tab
        // Bind and Channel are actions (type 3): Enter does the thing, the
        // arrows do nothing, and the value column says what Enter will do
        // rather than echoing a value there is not.
        for (auto& it : ar_rf_items()) {
            switch (it.second) {
                case 9: items.push_back({it.first, 3,
                    "Pairs this VRX with an air unit. Put the air unit in its "
                    "pairing mode first: hold its bind button until the LED turns red.",
                    "ENTER: BIND"}); break;
                case 5: items.push_back({it.first, 3,
                    "Where the link sits, what is on every channel and what your "
                    "country allows. AUTO there lets the link hop by itself.",
                    "ENTER: OPEN"}); break;
                case 1: items.push_back({it.first, 1,
                    "Radio output. More reaches further and runs hotter."}); break;
                case 8: items.push_back({it.first, 1,
                    "Force the air unit's low-power standby on or off for now."}); break;
                case 11: items.push_back({it.first, 1,
                    "How wide the video link may go. 40 widens it on a strong link: more "
                    "bitrate, a shorter radio frame, less range. 20 reaches further. "
                    "Enter applies it: the radio restarts, the picture is back in ~10 s."}); break;
                case 12: items.push_back({it.first, 1,
                    "How long the radio's frame is. SHORT gets each slice out sooner; "
                    "STANDARD is stock's, with a longer, sturdier uplink, and what a stock "
                    "air unit always gets. "
                    "HUD > Debug Info shows the frame in use."}); break;
                case 13: items.push_back({it.first, 1,
                    "This VRX's own radio firmware: STOCK, or OPEN8030 (open source) when "
                    "it is installed. The air unit's is not changed. Enter applies it: the "
                    "radio restarts, the picture is back in ~10 s."}); break;
                case 15: items.push_back({it.first, 1,
                    "Each block the air radio has to send again takes the link one MCS "
                    "lower at once, and it climbs back only after 5 s without one: steadier "
                    "latency, less bitrate. OFF, the default: the MCS follows the signal."}); break;
                default: items.push_back({it.first, 1}); break;
            }
        }
    } else if (tab == 2) { // HUD Tab - what is drawn over the video
        items = {
            // Order matches HudRow in osd.hpp - change both together.
            {"HUD Style", 1, "Which overlay is drawn over the video."},
            {"Betaflight OSD", 1, "Draw Betaflight's own OSD across the screen."},
            {"Canopy Voltage", 1, "The canopy's headline voltage: per cell reads the same on any pack size."},
            {"Calib Distance", 3, "Zero the range reading where you are standing. "
                                  "The arrows trim the offset by hand.", "ENTER: ZERO HERE"},
            {"Clock", 1, "When the time is shown at the top of the screen, and how it is written."},
            {"Dynamic HUD", 1, "Let the panels move with the aircraft."},
            {"Debug Info", 1, "A line of radio details under the right panel, for field "
                              "tests: the radio's frame, and the mode it means."}
        };
        // Only the canopy has a headline voltage to set; the rows after it close up (hud_row()).
        if (menu_hud_style != kHudCanopy) items.erase(items.begin() + kHudRowVoltage);
    } else if (tab == 3) { // DISPLAY Tab - how the screen looks
        // This tab was PHYSICS, two cosmetic toggles on their own. The screen
        // and decoration settings that were scattered across HUD and SYSTEM
        // join them, which is what retires a tab holding two rows.
        // Order matches DisplayRow in osd.hpp - change both together.
        items = {
            {"Theme", 1, "Which hues carry which meaning."},
            {"Background Video", 1, "Play the idle scene when nothing is linked."},
            {"Picture Size", 1, "Shrink the whole picture if your optics clip the edges."},
            {"UI Scaling", 1, "Size of the HUD's type. The panels stay put."},
            {"Brightness", 1, "Panel backlight. Lifts video and HUD together."},
            // How pictures are timed onto the screen: worth leaving alone
            // unless chasing latency or smoothness.
            {"ADVANCED", 2},
            {"Frame Mode", 1, "How pictures go from the radio to the screen."},  // see menu_help_text
            {"Refresh Sync", 1, ""},                                              // see menu_help_text
        };
        // Only while Refresh Sync is ON or FREE: with it OFF the margin does
        // nothing. Sync To (kDispSyncSlice, after it: which slice the lock
        // times the screen to) is hidden; display_lock_slice still applies.
        if (menu_refresh_sync != 0)
            items.push_back({"Sync Margin", 1, "Refresh Sync's refresh this much later. Fewer pictures miss their refresh (a hitch each), for that much latency: 2 MS, about a third fewer."});
    } else if (tab == kTabDvr) {
        items = {
            {"Source", 1, "What REC captures: the FPV stream, or the screen."},  // see menu_help_text
            {"Folder", 0, "Where recordings are written."},
            {"Space Left", 0, "Room remaining on the recordings partition."}
        };
    } else if (tab == kTabAirInfo) { // AIR UNIT > INFO - what the linked air unit says it is
        // The first rows are stock's Device Info for the air unit (TX HW / SN / SW
        // Version), worked out as stock does, so they read the same for either air
        // app. The rest only when kestrel-air answers: what makes it fpvOS. Rows
        // come and go with that, so the values go by label (menu_value_text).
        const Ar8030Source::AirInfo a = Ar8030Source::air_info();
        items = {
            {"AIR UNIT", 2},
            {"Model", 0, "The air unit on the other end of the link, as it reports itself."},
            {"HW Version", 0, "As stock shows it: the board's version, then the RF board's."},
            {"SN", 0, "As stock shows it: the model, then the radio chip's ID. Caddx's air app "
                      "always reports 0 there, so an Ascent reads H_Sky_000000."},
            {"SW Version", 0, "The Caddx firmware on the air unit, as stock shows it. With fpvOS it "
                              "is the stock firmware fpvOS runs on."},
        };
        if (osd_vars.artosyn.state == 2 && (a.kestrel || !a.kv.empty())) {
            items.push_back({"FPVOS", 2});
            items.push_back({"fpvOS", 0, "The fpvOS air image the unit was flashed with."});
            items.push_back({"kestrel-air", 0, "fpvOS's air app: it runs the camera and the radio in "
                                               "place of Caddx's."});
            items.push_back({"Radio Driver", 0, "What drives the AR8030 radio on the air unit."});
            items.push_back({"Radio Firmware", 0, "What runs on the air unit's AR8030 radio chip: the stock "
                                                  "firmware, or Open8030, the open one (Lite and Lite+)."});
            items.push_back({"Kernel", 0, "The air unit's Linux kernel (the stock one)."});
            items.push_back({"Camera", 0, "The camera's image sensor."});
            // What it has on, as one comma-separated list: a row of type 4,
            // which the blade wraps over as many lines as it takes.
            const std::vector<std::string> caps = Ar8030Source::air_cap_names(a);
            if (!caps.empty()) {
                items.push_back({"CAPABILITIES", 2});
                std::string list;
                for (size_t k = 0; k < caps.size(); k++) list += (k ? ", " : "") + caps[k];
                items.push_back({menu_intern(list), 4, kAirCapsHelp});
            }
        }
    } else if (tab == kTabAirSerial) { // AIR UNIT > SERIAL - what the flight controller stream carries
        // Read-only, and counted here at the goggle from the telemetry stream the air
        // unit relays (nothing extra is sent for it). Values go by label (menu_value_text).
        items = {
            {"FLIGHT CONTROLLER", 2},
            {"Receiving", 0, "Whether MSP frames from the flight controller are coming in now."},
            {"Frames", 0, "MSP frames received and understood, with the rate."},
            {"Data", 0, "Flight controller frames arriving, in bytes a second. Measured while this tab is open."},
        };
    } else { // SYSTEM
        items = {
            {"Build", 0},
            {"Version", 0},
            {"Decoder", 0},
            // Help is built live by menu_help_text: the mode in use.
            {"Screen Mode", 1, ""}
        };
        // Help is built live by menu_help_text: name, password, channel.
        items.push_back({"WiFi AP", 1, ""});
        // Where you fly: the clock's time zone and the country whose rules the
        // channel page shows. Order matches SysRow in osd.hpp.
        items.push_back({"WHERE YOU FLY", 2});
        items.push_back({"Amateur Licence", 1, "Whether you hold an amateur radio licence. With it, the amateur "
                                               "band of your country's ITU region reads OK on the channel page; "
                                               "your licence sets the power."});
        items.push_back({"Area", 1, "Where you fly, starting with the part of the world. Sets the clock and "
                                    "which channels the channel page allows. Or set it from a phone, on the "
                                    "VRX's web page."});
        items.push_back({"Country", 1, "The country you fly in: its rules for the radio's channels."});
        // Only where the country has more than one time zone (in this area):
        // which one the clock shows. The channel rules are per country and do
        // not depend on it. Last, so hiding it moves no other row.
        if (menu_zone_ >= 0) {
            const zones::Zone& z = zones::zone(menu_zone_);
            if (zones::zones_in(z.area, z.country).size() > 1)
                items.push_back({"Timezone", 1, "Which of the country's time zones the clock shows. "
                                                "The channel rules are the country's either way."});
        }
    }
    return items;
}

// See the header. Steps the row's own handler to find its values, then puts it
// back exactly where it was.
//
// Three rows are excluded because there is nothing to enumerate: Brightness
// and UI Scaling are ranges far longer than the cap, and Calib Distance is a
// capture rather than a choice. Walking them would spend a dozen steps to
// find that out. The walk itself is harmless now - menu_stepping_ keeps the
// steps off the HUD and out of the settings file.
int OSD::menu_options(int tab, int index, char out[kMenuOptMax][kMenuOptLen], int* cur) {
    if (cur) *cur = 0;
    const bool self_saving = (tab == 3 && index == kDispBrightness) ||
                             (tab == 3 && index == kDispUiScale) ||
                             (tab == 2 && hud_row(index) == kHudRowCalib);
    if (self_saving) return 0;

    if (tab == menu_opt_tab_ && index == menu_opt_idx_) {
        if (cur) *cur = menu_opt_cur_;
        memcpy(out, menu_opt_, sizeof(menu_opt_));
        return menu_opt_n_;
    }

    // Stepping, not setting: the walk moves the row's mirror to read the text
    // off it and puts it back, and neither the HUD nor the settings file wants
    // to hear about the values it passed through on the way.
    const bool was_stepping = menu_stepping_;
    menu_stepping_ = true;
    // Stepping back is not undoing for the time zone rows (a step on Area or
    // Country starts the next one over at its first zone), so its walk puts
    // the zone itself back.
    const int zone_before = menu_zone_;

    // Stepping stops at the ends while menu_stepping_ is set (menu_step), so
    // a set and a clamped range look the same from here: walk down until the
    // value stops moving - that is the start, and the number of steps it
    // took is where the value in force sits - then walk up collecting until
    // it stops again. A walk that never reaches an end inside the cap is a
    // long range, which has no list to draw; the ladder shows it instead.
    char t[64], prev[64];
    menu_value_text(tab, index, prev, sizeof(prev));
    int back = 0;
    for (; back < kMenuOptMax; back++) {
        menu_apply_change(tab, index, -1);
        menu_value_text(tab, index, t, sizeof(t));
        if (!strcmp(t, prev)) break;
        snprintf(prev, sizeof(prev), "%s", t);
    }
    int n = 0, fwd = 0;
    if (back < kMenuOptMax) {
        menu_value_text(tab, index, out[0], kMenuOptLen);
        n = 1;
        while (n < kMenuOptMax) {
            menu_apply_change(tab, index, +1);
            menu_value_text(tab, index, t, sizeof(t));
            if (!strcmp(t, out[n - 1])) break;
            fwd++;
            snprintf(out[n++], kMenuOptLen, "%s", t);
        }
        if (n >= kMenuOptMax) n = 0;    // never found the top: a range
    }
    // Put it back exactly where it was: the walk is fwd steps up from the
    // start, and the value in force is back steps up from it.
    for (; fwd > back; fwd--) menu_apply_change(tab, index, -1);
    for (; fwd < back; fwd++) menu_apply_change(tab, index, +1);
    menu_zone_ = zone_before;
    menu_stepping_ = was_stepping;

    menu_opt_tab_ = tab; menu_opt_idx_ = index;
    menu_opt_n_ = n; menu_opt_cur_ = (n > 0) ? back : 0;
    memcpy(menu_opt_, out, sizeof(menu_opt_));
    if (cur) *cur = menu_opt_cur_;
    return n;
}

// The values either side of the current one, for a row that is a range rather
// than a set.
//
// A range has no list to draw, so the value column used to show one number in
// the middle of an empty panel - which says what the setting reads but nothing
// about where that sits, or how far a press moves it. A short ladder either
// side answers both, and costs nothing to find: the same step-and-read trick
// menu_options uses, walked a few steps instead of all the way round.
//
// The self-saving rows are the exception. Brightness writes to the panel and
// UI Scaling rewrites the layout this very frame, so walking them to fill a
// ladder would strobe the screen. Those two are computed from their own
// domains instead - the one place a setting's range is spelled out twice, and
// worth it to keep the draw free of side effects.
int OSD::menu_range_ladder(int tab, int index, int span,
                           char out[kMenuOptMax][kMenuOptLen]) {
    if (span < 1) span = 1;
    const int n = span * 2 + 1;
    if (n > kMenuOptMax) return 0;
    // Whole rows, not just the terminator: the render thread reads this buffer
    // while it is being filled, and a partial write over longer leftovers is
    // the one way it could come back without a terminator at all.
    memset(out, 0, (size_t)kMenuOptMax * kMenuOptLen);

    // menu_value_text writes "< X >" for a nudgeable row; the chevrons said
    // which key to press on a menu that no longer works that way.
    // The pending mark goes too: it belongs to the row, not to each of the
    // values it could take, and menu_value_text stamps every reading it makes
    // while a row is dirty - including the ones this walk asks for.
    auto strip = [](char* t) {
        size_t len = strlen(t);
        if (len >= 2 && t[len-1] == '*' && t[len-2] == ' ') {
            t[len - 2] = '\0';
            len -= 2;
        }
        if (len >= 4 && t[0] == '<' && t[1] == ' ' && t[len-2] == ' ' && t[len-1] == '>') {
            memmove(t, t + 2, len - 4);
            t[len - 4] = '\0';
        }
    };

    if (tab == 2 && hud_row(index) == kHudRowCalib) return 0;   // an action

    if (tab == 3 && index == kDispBrightness) {   // 0..100 in fives
        for (int k = -span; k <= span; k++) {
            int v = menu_brightness + k * 5;
            if (v < 0 || v > 100) continue;
            snprintf(out[k + span], kMenuOptLen, "%d", v);
        }
        return n;
    }
    if (tab == 3 && index == kDispUiScale) {      // 0.5x..2.0x in tenths
        for (int k = -span; k <= span; k++) {
            float v = menu_ui_scale + (float)k * 0.1f;
            if (v < 0.5f - 0.001f || v > 2.0f + 0.001f) continue;
            snprintf(out[k + span], kMenuOptLen, "%.1fx", v);
        }
        return n;
    }

    const bool was_stepping = menu_stepping_;
    menu_stepping_ = true;
    const int zone_before = menu_zone_;   // see menu_options

    char centre[64];
    menu_value_text(tab, index, centre, sizeof(centre));
    snprintf(out[span], kMenuOptLen, "%s", centre);
    strip(out[span]);

    // Outwards in both directions, stopping where the value stops moving -
    // which is how a clamped range says it has reached its end.
    for (int dir = -1; dir <= 1; dir += 2) {
        char prev[64];
        snprintf(prev, sizeof(prev), "%s", centre);
        int steps = 0;
        for (int k = 1; k <= span; k++) {
            menu_apply_change(tab, index, dir);
            char t[64];
            menu_value_text(tab, index, t, sizeof(t));
            // A step that changed nothing is the end of the range - and it is
            // NOT counted, because it also needs no undoing. Counting it was
            // the whole bug in Overscan: pressing down at 10px walked to 0,
            // stopped against the clamp, and then unwound two steps from 0,
            // which put the setting back at 10px and ate the press.
            if (!strcmp(t, prev)) break;
            steps++;
            snprintf(out[span + dir * k], kMenuOptLen, "%s", t);
            strip(out[span + dir * k]);
            snprintf(prev, sizeof(prev), "%s", t);
        }
        while (steps-- > 0) menu_apply_change(tab, index, -dir);
        menu_zone_ = zone_before;
    }
    menu_stepping_ = was_stepping;
    return n;
}

// Work out what the selected row's value column will show, for the render
// thread to read.
//
// Both ways of discovering a row's values - menu_options walking a set until
// it comes back round, menu_range_ladder walking a few steps either side of a
// range - find them by stepping the row's own handler and putting it back
// afterwards. That is a write, and it used to happen inside the draw: a key
// that landed mid-walk was applied to whatever temporary value the walk had
// left behind, and then the walk's own restore put its result back. Overscan
// would not go below 10px because the ladder was standing two steps below it
// every frame and undoing each press.
//
// So both walks run here instead, on the thread that owns the menu state and
// under the lock its keys are handled with, whenever the selection or a value
// changes. The draw only reads.
void OSD::menu_refresh_options() {
    // Drop the set cache first. It is keyed by the row, not by what the row
    // reads, and after a press the cached position would be stale.
    menu_opt_tab_ = -1;
    menu_opt_idx_ = -1;
    menu_lad_tab_ = menu_tab;
    menu_lad_idx_ = menu_index;
    menu_lad_n_   = 0;
    menu_lad_cur_ = 0;

    // Chevrons and the pending mark off, so the list holds bare values.
    auto strip = [](char* t) {
        size_t len = strlen(t);
        if (len >= 2 && t[len-1] == '*' && t[len-2] == ' ') { t[len - 2] = '\0'; len -= 2; }
        if (len >= 4 && t[0] == '<' && t[1] == ' ' && t[len-2] == ' ' && t[len-1] == '>') {
            memmove(t, t + 2, len - 4);
            t[len - 4] = '\0';
        }
    };

    // A set: every value, in the row's own order, and where the current one
    // sits in it. This is the list the column draws.
    char opts[kMenuOptMax][kMenuOptLen];
    int cur = 0;
    int n = menu_options(menu_tab, menu_index, opts, &cur);
    if (n > 0) {
        for (int k = 0; k < n; k++) {
            snprintf(menu_lad_[k], kMenuOptLen, "%s", opts[k]);
            strip(menu_lad_[k]);
        }
        menu_lad_n_   = n;
        menu_lad_cur_ = cur;
        return;
    }

    // A range too long to list: the values either side of the current one,
    // ascending, with the ends left off rather than wrapped.
    char lad[kMenuOptMax][kMenuOptLen];
    const int m = menu_range_ladder(menu_tab, menu_index, kMenuLadderSpan, lad);
    for (int k = 0; k < m; k++) {
        if (!lad[k][0]) continue;
        if (k == kMenuLadderSpan) menu_lad_cur_ = menu_lad_n_;
        snprintf(menu_lad_[menu_lad_n_++], kMenuOptLen, "%s", lad[k]);
    }
}

// The Clock row is one choice made of two settings - when the time shows
// (clock_show: 0 off, 1 idle only, 2 always) and how it is written
// (clock_mode: 1 = 12H, 2 = 24H): OFF, then idle only and always, each in 12H
// and 24H. OFF leaves the format as it was, so coming back to it keeps it.
static const int kClockChoices = 5;
static const char* const kClockLabels[kClockChoices] =
    { "OFF", "IDLE ONLY 12H", "IDLE ONLY 24H", "ON 12H", "ON 24H" };
static int clock_choice(int show, int mode) {
    return show <= 0 ? 0 : 1 + (show - 1) * 2 + (mode == 2 ? 1 : 0);
}

// Rows that take effect the moment the arrows move them.
//
// These write the setting as they go, so there is nothing for Enter to apply
// and nothing pending to mark: the "*" would be claiming the screen and the
// file disagree with each other when they do not. Everything else stays on
// Enter, either because applying it costs something real (the air unit has to
// be told, the background player has to be started or stopped) or because it
// is a capture the row makes at the moment you press it.
bool OSD::menu_row_applies_live(int tab, int index) {
    if (tab == 2) {
        // Every HUD row writes as it goes, Calib Distance's arrows included;
        // its Enter is a capture, not an apply.
        // (The headers cannot be selected, so which row `index` is makes no difference.)
        return true;
    }
    if (tab == 3)   // DISPLAY: all but Background Video, which owns a player
        return index != kDispBgVideo;
    // SYSTEM: Screen Mode restarts the app and WiFi AP waits for Enter, after
    // its antenna warning has been shown; neither applies as it is stepped.
    // The time zone applies as it is walked: the clock follows it.
    if (tab == kTabSystem) return index >= kSysLicence && index <= kSysTimezone;
    return false;
}

// Section headers (type 2) sit in the row list to break it into groups but
// are not a row anyone can land on - there is nothing to set. Called
// wherever a tab is entered or changed and the row index gets reset, so the
// cursor opens on the first real setting rather than a label.
int OSD::menu_first_selectable_row(int tab) {
    std::vector<MenuItem> rows = menu_items(tab);
    for (int i = 0; i < (int)rows.size(); i++)
        if (rows[i].type != 2) return i;
    return 0;
}

// Apply one left/right step to a menu row.
//
// Extracted from the key handler so it can be driven from somewhere other than
// a keypress - menu_options() below steps a row through its own values to find
// out what they are, which is how a value column gets a list without a second
// table of every setting's domain to keep in sync.
void OSD::menu_apply_change(int tab, int index, int dir) {
    const int menu_tab = tab, menu_index = index;
            if (menu_tab == 0) {
       switch (video_row(menu_index)) {
           case 0: {
               int n = video_mode_names.empty() ? 1 : (int)video_mode_names.size();
               // Past the modes this air unit does not have (864p120 needs kestrel-air's).
               int v = menu_video_mode;
               for (int k = 0; k < n; k++) {
                   const int s = menu_step(v, dir, n);
                   if (s == v) break;
                   v = s;
                   if (video_mode_ok(v)) break;
               }
               if (video_mode_ok(v)) menu_video_mode = v;
               break;
           }
           case 1: menu_cam_flip = menu_step(menu_cam_flip ? 1 : 0, dir, 2) != 0; break;
           case 2: menu_cam_scene    = menu_step(menu_cam_scene, dir, kSceneCount); break;
           case 3: menu_cam_ev       = menu_step(menu_cam_ev, dir, kEvCount); break;
           case 4: menu_cam_wb       = menu_step(menu_cam_wb, dir, kWbCount); break;
           case 5: menu_cam_sat      = std::min(kCamSatMax,      std::max(0, menu_cam_sat + dir)); break;
           case 6: menu_cam_contrast = std::min(kCamContrastMax, std::max(0, menu_cam_contrast + dir)); break;
           case 7: menu_cam_sharp    = std::min(kCamSharpMax,    std::max(0, menu_cam_sharp + dir)); break;
           case 8: menu_cam_3dnr    = menu_step(menu_cam_3dnr, dir, kDnrCount); break;
           case 9: menu_cam_focus = menu_step(menu_cam_focus ? 1 : 0, dir, 2) != 0; break;
           case 10: menu_ar_maxbr = menu_step(menu_ar_maxbr, dir, kArMaxBrCount); break;
           default: break;
       }
    } else if (menu_tab == 1) {
       auto rfi = ar_rf_items();
       if (menu_index >= 0 && menu_index < (int)rfi.size()) {
           switch (rfi[menu_index].second) {
               case 5: break;   // Channel is read-only (see above)
               case 1:
                   {
                       // The levels this air unit is offered (common.hpp);
                       // none before it has said what it is.
                       const int prj = Ar8030Source::air_prj.load();
                       if (prj <= 0) break;
                       ar_pwr_offer o = ar_pwr_offered(prj);
                       int i = ar_pwr_offer_index(prj, ar_pwr_fit(prj, menu_ar_power));
                       i = menu_step(i < 0 ? 0 : i, dir, o.n);
                       menu_ar_power = o.mw[i];
                   }
                   break;
               case 8: menu_ar_standby = menu_step(menu_ar_standby ? 1 : 0, dir, 2) != 0; break;  // forced on Enter
               case 11:   // applied on Enter; stock's radio: 20 or 40 only
                   if (Ar8030Source::radio_fw == 1) menu_ar_maxbw = menu_step(menu_ar_maxbw, dir, kArMaxBwCount);
                   else menu_ar_maxbw = menu_ar_maxbw == 4 ? 3 : 4;
                   break;
               case 14: menu_ar_maxmcs = menu_step(menu_ar_maxmcs, dir, kArMaxMcsCount); break;  // applied on Enter
               case 15: menu_ar_steady = menu_ar_steady ? 0 : 1; break;  // applied on Enter
               case 12: menu_ar_frame = menu_step(menu_ar_frame, dir, kArFrameCount); break;  // applied on Enter
               case 13: menu_ar_fw = menu_step(menu_ar_fw, dir, kArFwCount); break;  // applied on Enter
               default: break;
           }
       }
    } else if (menu_tab == 2) { // HUD - rows as listed in menu_items(), see hud_row()
        if (hud_row(menu_index) == kHudRowStyle) {
            menu_hud_style = menu_step(menu_hud_style, dir, kHudStyleCount);
            if (!menu_stepping_) {
                hud_style = menu_hud_style;
                Settings::getInstance().set("hud_overlay", hud_style);
            }
        } else if (hud_row(menu_index) == kHudRowBfOsd) {
            menu_bf_osd = menu_step(menu_bf_osd ? 1 : 0, dir, 2) != 0;
            if (!menu_stepping_) {
                bf_osd = menu_bf_osd;
                Settings::getInstance().set("bf_osd", bf_osd ? 1 : 0);
            }
        } else if (hud_row(menu_index) == kHudRowVoltage) {
            menu_volt_mode = menu_step(menu_volt_mode, dir, 2);
            if (!menu_stepping_) {
                volt_mode = menu_volt_mode;
                Settings::getInstance().set("volt_mode", volt_mode);
            }
        } else if (hud_row(menu_index) == kHudRowCalib) {
            // left/right nudges the calibration by 1
            if (!menu_stepping_) {
                dist_offset = std::max(0, dist_offset + dir);
                save_dist_offset();
            }
        } else if (hud_row(menu_index) == kHudRowClock) {
            const int c = menu_step(clock_choice(menu_clock_show, menu_clock_mode), dir, kClockChoices);
            menu_clock_show = c == 0 ? 0 : 1 + (c - 1) / 2;
            if (c > 0) menu_clock_mode = ((c - 1) % 2) ? 2 : 1;
            if (!menu_stepping_) {
                clock_show = menu_clock_show;
                clock_mode = menu_clock_mode;
                Settings::getInstance().set("clock_show", clock_show);
                Settings::getInstance().set("clock_mode", clock_mode);
            }
        } else if (hud_row(menu_index) == kHudRowDynamic) {
            menu_hud_reactivity = menu_step(menu_hud_reactivity, dir, 4);
            if (!menu_stepping_) {
                hud_reactivity = menu_hud_reactivity;
                Settings::getInstance().set("hud_reactivity", hud_reactivity);
            }
        } else if (hud_row(menu_index) == kHudRowDebug) {
            menu_hud_debug = menu_step(menu_hud_debug ? 1 : 0, dir, 2) != 0;
            if (!menu_stepping_) {
                hud_debug = menu_hud_debug;
                Settings::getInstance().set("hud_debug", hud_debug);
                if (cmd_cb) cmd_cb(0x201, hud_debug ? 1 : 0);   // the logs follow it
            }
        }
    } else if (menu_tab == 3) { // DISPLAY
        if (menu_index == kDispBrightness) {
            // Hardware picture control on the HDMI connector: lifts the
            // video and the OSD together, which is what made the grid
            // read better when the video was accidentally drawn twice.
            menu_brightness += dir * 5;
            if (menu_brightness < 0)   menu_brightness = 0;
            if (menu_brightness > 100) menu_brightness = 100;
            if (!menu_stepping_) {
                if (dev) dev->set_display_property("brightness", menu_brightness);
                Settings::getInstance().set("brightness", menu_brightness);
            }
        } else if (menu_index == kDispUiScale) {
            // Applies and saves as it is nudged, like Brightness beside it.
            // A scale is judged by looking at what it did to the HUD, and the
            // menu is drawn at this scale too - holding it back until Enter
            // meant choosing a size without being able to see it.
            menu_ui_scale = std::min(2.0f, std::max(0.5f, menu_ui_scale + (float)dir * 0.1f));
            if (!menu_stepping_) {
                osd_vars.ui_scale = menu_ui_scale;
                Settings::getInstance().set("ui_scale", menu_ui_scale);
            }
        } else if (menu_index == kDispBgVideo) {
            menu_bg_video = menu_step(menu_bg_video ? 1 : 0, dir, 2) != 0;
        } else if (menu_index == kDispTheme) {
            int n = hud_theme_count();
            menu_hud_theme = menu_step(menu_hud_theme, dir, n);
            // A theme is chosen by looking at it, and the menu is drawn over
            // the same HUD it recolours.
            if (!menu_stepping_) {
                hud_theme_idx = menu_hud_theme;
                hud_theme_set(hud_theme_idx);
                Settings::getInstance().set("hud_theme", hud_theme_idx);
            }
        } else if (menu_index == kDispPicture) {
            // Whole-picture scale: video and HUD shrink together, black all
            // round. Not Overscan, which only pulls the HUD's margins in. For
            // optics that clip the frame, this is the one that shows the lot.
            // Applied as it is nudged, like Overscan: you set it by looking.
            menu_picture_scale = std::min(100, std::max(60, menu_picture_scale + dir * 5));
            if (!menu_stepping_ && dev) {
                dev->set_picture_scale(menu_picture_scale);
                Settings::getInstance().set("picture_scale", menu_picture_scale);
            }
        } else if (menu_index == kDispFrameMode) {
            // Applied as it is stepped: the difference is seen in the video.
            menu_frame_mode = menu_step(menu_frame_mode, dir, kFrameModeCount);
            if (!menu_stepping_) {
                g_frame_mode = menu_frame_mode;
                Settings::getInstance().set("frame_mode", menu_frame_mode);
                printf("menu: frame mode %s\n", frame_mode_label(menu_frame_mode));
            }
        } else if (menu_index == kDispSyncMargin) {
            // Applied as it is stepped, like Sync To.
            menu_sync_margin = menu_step(menu_sync_margin, dir, 5);
            if (!menu_stepping_) {
                g_display_lock_margin_us = menu_sync_margin * 1000;
                Settings::getInstance().set("display_lock_margin_ms", menu_sync_margin);
                printf("menu: refresh sync margin %d ms\n", menu_sync_margin);
            }
        } else if (menu_index == kDispSyncSlice) {
            menu_sync_slice = menu_step(menu_sync_slice - 1, dir, 2) + 1;
            if (!menu_stepping_) {
                g_display_lock_slice = menu_sync_slice;
                Settings::getInstance().set("display_lock_slice", menu_sync_slice);
                printf("menu: refresh sync to slice %d\n", menu_sync_slice);
            }
        } else if (menu_index == kDispRefreshSync) {
            // The screen's own, like its mode: one that blanks on it keeps it off
            // while another has it on. FREE also moves Screen Mode Auto to the
            // fastest mode (and back), so Auto chooses again.
            // ON and FREE go on trial (sync_confirm_): live at once, kept on
            // Enter, the kept value back after 10 s without it. OFF, or the
            // kept value itself, is kept at once.
            const int was = g_display_lock.load();
            menu_refresh_sync = menu_step(menu_refresh_sync, dir, 4);   // SPIKE: CAMERA
            if (!menu_stepping_) {
                const std::string key = screen_id_.empty() ? std::string("display_lock")
                                                           : "display_lock_" + screen_id_;
                if (!sync_confirm_) sync_confirm_prev_ = was;
                g_display_lock = menu_refresh_sync;
                if (dev) dev->refollow();
                if (menu_refresh_sync == 0 || menu_refresh_sync == sync_confirm_prev_) {
                    Settings::getInstance().set(key, menu_refresh_sync);
                    sync_confirm_ = false;
                    printf("menu: refresh sync %s\n", refresh_sync_label(menu_refresh_sync));
                } else {
                    sync_confirm_ = true;
                    sync_confirm_deadline_ms_ = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count()) + 10000;
                    printf("menu: refresh sync %s on trial - Enter keeps it\n", refresh_sync_label(menu_refresh_sync));
                }
            }
        }
    } else if (menu_tab == kTabDvr) {
        if (menu_index == 0) {
            menu_dvr_source = menu_step(menu_dvr_source, dir, 2);
        }
    } else if (menu_tab == kTabSystem) {
        if (menu_index == 3) {
            // Only stages the choice: the display is set up once, at start,
            // so Enter saves it and restarts kestrel (below).
            menu_screen_mode = menu_step(screen_mode_menu(), dir, (int)screen_modes().size() + 1);
        } else if (menu_index == 4 && wifi_ap_available()) {
            // Only stages the choice. Enter applies it (below, with DVR
            // Source), so turning the radio on always passes through the
            // pending state, whose help line is the antenna warning.
            menu_wifi_ap = menu_step(menu_wifi_ap ? 1 : 0, dir, 2) != 0;
        } else if (menu_index == kSysLicence) {
            menu_licence_ = menu_step(menu_licence_, dir, 2);
            if (!menu_stepping_) zones::set_licensed(menu_licence_ != 0);
        } else if (menu_index >= kSysArea && menu_index <= kSysTimezone) {
            menu_zone_ = menu_step_zone(menu_index, dir);
            if (!menu_stepping_) {
                zones::select(menu_zone_);
                menu_opt_tab_ = -1;   // Country's and Timezone's lists follow the zone
            }
        }
    }
}

// Where recordings are written, and how much room is left there.
//
// The path is a setting rather than a constant - main() hands the same one to
// the recorder - so the menu asks for it rather than repeating the default.
// statvfs is a syscall and the value column asks once a frame while the DVR
// tab is open, so the answer is held for a couple of seconds; free space does
// not move faster than that, and while recording it moves slowly.
static const char* dvr_dir_path() {
    static std::string dir = Settings::getInstance().getString("dvr_dir", "/media/dvr");
    return dir.c_str();
}

static const char* dvr_space_left() {
    static char buf[32] = "--";
    static uint64_t last_us = 0;
    const uint64_t now = get_time_us();
    if (last_us == 0 || now - last_us > 2000000) {
        last_us = now;
        struct statvfs st;
        if (statvfs(dvr_dir_path(), &st) == 0) {
            // Binary units, so this agrees with what df on the goggle says
            // rather than being a gigabyte larger for no visible reason.
            const double gb = (double)st.f_bavail * (double)st.f_frsize / 1073741824.0;
            if (gb >= 10.0)     snprintf(buf, sizeof(buf), "%.0f GB", gb);
            else if (gb >= 1.0) snprintf(buf, sizeof(buf), "%.1f GB", gb);
            else                snprintf(buf, sizeof(buf), "%.0f MB", gb * 1024.0);
        } else {
            snprintf(buf, sizeof(buf), "--");   // not mounted
        }
    }
    return buf;
}

// The text shown for one menu row's current value.
//
// Lifted out of draw_menu so more than one thing can ask what a row reads. It
// was 200 lines inline, which meant the value could only ever be rendered in
// the one place the loop happened to be - and a value column needs the same
// answer from somewhere else entirely.
void OSD::menu_value_text(int tab, int i, char* val_buf, size_t cap) {
    const int menu_tab = tab;
    val_buf[0] = '\0';
    
    if (menu_tab == 0) {
        switch (video_row(i)) {
            case 0: {
                int idx = menu_video_mode;
                if (video_mode_names.empty()) { sprintf(val_buf, "< -- >"); break; }
                if (idx < 0 || idx >= (int)video_mode_names.size()) idx = 0;
                sprintf(val_buf, "< %s >", video_mode_names[idx].c_str());
                break;
            }
            case 1: sprintf(val_buf, "< %s >", kRotateLabels[menu_cam_flip ? 1 : 0]); break;
            case 2: sprintf(val_buf, "< %s >", kSceneLabels[menu_cam_scene]); break;
            case 3: sprintf(val_buf, "< %s EV >", kEvLabels[menu_cam_ev]); break;
            case 4: sprintf(val_buf, "< %s >", wb_label(menu_cam_wb)); break;
            case 5: sprintf(val_buf, "< %s >", cam_auto_label(menu_cam_sat)); break;
            case 6: sprintf(val_buf, "< %s >", cam_auto_label(menu_cam_contrast)); break;
            case 7: sprintf(val_buf, "< %s >", cam_auto_label(menu_cam_sharp)); break;
            case 8: sprintf(val_buf, "< %s >", kDnrLabels[menu_cam_3dnr]); break;
            case 9: sprintf(val_buf, "< %s >", menu_cam_focus ? "On" : "Off"); break;
            case 10:
                if (air_stock_) snprintf(val_buf, cap, "FPVOS AIR ONLY");
                else sprintf(val_buf, "< %s >", kArMaxBrLabels[menu_ar_maxbr]);
                break;
            default: break;
        }
    } else if (menu_tab == 1) { // AR8030 RF items
        auto rfi = ar_rf_items();
        if (i < (int)rfi.size()) {
            switch (rfi[i].second) {
                case 9:     // Bind - an action. The row is the verb; what
                    // Enter does and how the bind is going are the value
                    // column's (menu_action_status), the countdown is the
                    // animation's.
                    val_buf[0] = '\0';
                    break;
                case 5: {   // Channel: where the link is, and whether it hops.
                    // No chevrons, so it does not look adjustable; Enter opens
                    // the channel page, where it is moved.
                    const chan_scan_info &cs = osd_vars.chan_scan;
                    int f = 0;
                    bool hop = Ar8030Source::chan_auto;
                    if (cs.last_update_ms && cs.work_chan < cs.chan_num) {
                        f = cs.freq_mhz[cs.work_chan];
                        hop = cs.auto_mode != 0;
                    }
                    if (!f) f = osd_vars.artosyn.tx_freq;
                    if (f) sprintf(val_buf, "%d MHz  %s", f, hop ? "AUTO" : "FIXED");
                    else   sprintf(val_buf, "--");
                    break;
                }
                case 1: {   // TX power: -- until the air unit has said what it is
                    const int prj = Ar8030Source::air_prj.load();
                    if (prj <= 0) sprintf(val_buf, "--");
                    else sprintf(val_buf, "< %s >", kArPwrLevels[ar_pwr_index(ar_pwr_fit(prj, menu_ar_power))].label);
                    break;
                }
                case 8:     // Standby. Shows the air's current state (menu value is
                    // seeded from it); the arrows + Enter force it on/off as a
                    // momentary override. The air still dictates it on its own
                    // (auto-parks when idle, exits when flying), so a force does
                    // not necessarily stick - see fpv_sky_standby_mode_thread.
                    sprintf(val_buf, "< %s >", menu_ar_standby ? "ON" : "OFF");
                    break;
                case 11:    // Max Bandwidth
                    sprintf(val_buf, "< %s >", kArMaxBwLabels[menu_ar_maxbw]);
                    break;
                case 12:    // Radio Frame
                    sprintf(val_buf, "< %s >", kArFrameLabels[menu_ar_frame]);
                    break;
                case 14:    // Max MCS
                    sprintf(val_buf, "< %s >", ar_maxmcs_label(menu_ar_maxmcs));
                    break;
                case 15:    // Steady Latency
                    sprintf(val_buf, "< %s >", menu_ar_steady ? "ON" : "OFF");
                    break;
                case 13:    // VRX Radio Firmware
                    sprintf(val_buf, "< %s >", kArFwLabels[menu_ar_fw]);
                    break;
                default: break;
            }
        }
        } else if (menu_tab == 2) { // HUD Tab - see menu_items() and hud_row()
            if (hud_row(i) == kHudRowStyle) {
                static const char* kStyleLabels[kHudStyleCount] = { "OFF", "CANOPY", "ARENA", "ARENA FULL" };
                int m = (menu_hud_style >= 0 && menu_hud_style < kHudStyleCount) ? menu_hud_style : 0;
                sprintf(val_buf, "< %s >", kStyleLabels[m]);
            } else if (hud_row(i) == kHudRowBfOsd) {
                sprintf(val_buf, "< %s >", menu_bf_osd ? "ON" : "OFF");
            } else if (hud_row(i) == kHudRowVoltage) {
                sprintf(val_buf, "< %s >", menu_volt_mode == 1 ? "PACK" : "PER CELL");
            } else if (hud_row(i) == kHudRowCalib) {
                // Enter zeroes the reading; show what is being subtracted.
                if (link_distance_raw < 0) sprintf(val_buf, "ofs %d", dist_offset);
                else sprintf(val_buf, "raw %d  ofs %d", link_distance_raw, dist_offset);
            } else if (hud_row(i) == kHudRowClock) {
                const int show = (menu_clock_show >= 0 && menu_clock_show < 3) ? menu_clock_show : 1;
                sprintf(val_buf, "< %s >", kClockLabels[clock_choice(show, menu_clock_mode)]);
            } else if (hud_row(i) == kHudRowDynamic) {
                const char* react_labels[4] = { "OFF", "SMALL", "MEDIUM", "EXTREME" };
                int r = (menu_hud_reactivity >= 0 && menu_hud_reactivity < 4) ? menu_hud_reactivity : 2;
                sprintf(val_buf, "< %s >", react_labels[r]);
            } else if (hud_row(i) == kHudRowDebug) {
                sprintf(val_buf, "< %s >", menu_hud_debug ? "ON" : "OFF");
            }
        } else if (menu_tab == 3) { // DISPLAY Tab
            if (i == kDispBrightness) {
                sprintf(val_buf, "< %d >", menu_brightness);
            } else if (i == kDispUiScale) {
                sprintf(val_buf, "< %.1fx >", menu_ui_scale);
            } else if (i == kDispBgVideo) {
                sprintf(val_buf, "< %s >", menu_bg_video ? "ON" : "OFF");
            } else if (i == kDispTheme) {
                sprintf(val_buf, "< %s >", hud_theme_at(menu_hud_theme).name);
            } else if (i == kDispPicture) {
                if (menu_picture_scale >= 100) sprintf(val_buf, "< FULL >");
                else                           sprintf(val_buf, "< %d%% >", menu_picture_scale);
            } else if (i == kDispFrameMode) {
                sprintf(val_buf, "< %s >", frame_mode_label(menu_frame_mode));
            } else if (i == kDispRefreshSync) {
                sprintf(val_buf, "< %s >", refresh_sync_label(menu_refresh_sync));
            } else if (i == kDispSyncSlice) {
                sprintf(val_buf, "< SLICE %d >", menu_sync_slice);
            } else if (i == kDispSyncMargin) {
                sprintf(val_buf, "< %d MS >", menu_sync_margin);
            }
        } else if (menu_tab == kTabDvr) {
            if (i == 0) {
                // Says what the DVR records, which "Record OSD" as a bare
                // ON/OFF never did.
                static const char* kSource[2] = { "FPV", "SCREEN" };
                int src = (menu_dvr_source >= 0 && menu_dvr_source < 2) ? menu_dvr_source : 0;
                sprintf(val_buf, "< %s >", kSource[src]);
            } else if (i == 1) {
                sprintf(val_buf, "%s", dvr_dir_path());
            } else if (i == 2) {
                sprintf(val_buf, "%s", dvr_space_left());
            }
        } else if (menu_tab == kTabAirInfo) {
            // Stock's placeholders until the air unit has said ("-.-", "--------",
            // "-.-.-"), and its formats after (GlassesUI's Device Info).
            const std::vector<MenuItem> rows = menu_items(tab);
            const char* L = (i >= 0 && i < (int)rows.size()) ? rows[i].label : "";
            const bool linked = osd_vars.artosyn.state == 2;
            const Ar8030Source::AirInfo a = linked ? Ar8030Source::air_info() : Ar8030Source::AirInfo();
            const int prj = linked ? Ar8030Source::air_prj.load() : 0;
            std::string v;
            if (!strcmp(L, "Model")) {
                v = a.get("model");
                if (v.empty() && prj > 0) v = ar_air_name(prj);
                if (v.empty()) v = "--";
            } else if (!strcmp(L, "HW Version")) {
                if (a.hw > 0 && a.rf_hw >= 0)
                    snprintf(val_buf, cap, "%d.%d-%d.%d", a.hw >> 4, a.hw & 15, a.rf_hw >> 4, a.rf_hw & 15);
                else
                    snprintf(val_buf, cap, "-.-");
                return;
            } else if (!strcmp(L, "SN")) {
                if (a.hw > 0 && prj > 0) snprintf(val_buf, cap, "%s%06X", ar_air_sn_prefix(prj), a.chipid & 0xffffff);
                else snprintf(val_buf, cap, "--------");
                return;
            } else if (!strcmp(L, "SW Version")) {
                if (a.hw > 0) snprintf(val_buf, cap, "%d.%d.%d", a.stock[0], a.stock[1], a.stock[2]);
                else snprintf(val_buf, cap, "-.-.-");
                return;
            } else if (!strcmp(L, "Radio Firmware")) {
                // What kestrel-air loaded (an older one does not say: --).
                const std::string fw = a.get("radiofw");
                if (fw == "open8030") v = "Open8030";
                else if (fw == "stock") v = "Stock";
                else v = fw;
            } else if (!strcmp(L, "fpvOS")) {
                v = a.get("os");
            } else if (!strcmp(L, "kestrel-air")) {
                v = a.get("ver");
                if (!v.empty() && a.proto) v += " (protocol " + std::to_string(a.proto) + ")";
            } else if (!strcmp(L, "Radio Driver")) {
                v = a.get("radio");
                if (!v.empty()) v = "ar_libre " + v;
            } else if (!strcmp(L, "Kernel")) {
                v = a.get("kernel");
            } else if (!strcmp(L, "Camera")) {
                v = a.get("sensor");
            } else if (i >= 0 && i < (int)rows.size() && rows[i].help == kAirCapsHelp) {
                return;                                // a line of the capabilities list
            }
            snprintf(val_buf, cap, "%s", v.empty() ? "--" : v.c_str());
        } else if (menu_tab == kTabAirSerial) {
            const std::vector<MenuItem> rows = menu_items(tab);
            const char* L = (i >= 0 && i < (int)rows.size()) ? rows[i].label : "";
            const bool linked = osd_vars.artosyn.state == 2;
            const MspOsd::MspCounts g = msp_osd.msp_counts();
            snprintf(val_buf, cap, "--");
            if (false) {
            } else if (!strcmp(L, "Receiving")) {
                if (linked) snprintf(val_buf, cap, "%s", g.active ? "MSP" : "NOTHING");
            } else if (!strcmp(L, "Data")) {
                if (linked) {
                    if (serial_bps_ >= 1000.0f) snprintf(val_buf, cap, "%.1f KB/s", serial_bps_ / 1000.0f);
                    else snprintf(val_buf, cap, "%.0f B/s", serial_bps_);
                }
            } else if (!strcmp(L, "Frames")) {
                if (linked) snprintf(val_buf, cap, "%u  (%.0f/s)", (unsigned)g.frames, serial_rate_);
            }
        } else if (menu_tab == kTabSystem) {
            if (i == 0) {
                sprintf(val_buf, "%s", KESTREL_GND_BUILD_TIME);
            } else if (i == 1) {
                sprintf(val_buf, "v%s", KESTREL_GND_VERSION);
            } else if (i == 2) {
                sprintf(val_buf, "%s", decoder_name.c_str());
            } else if (i == 3) {
                if (screen_modes().empty())
                    sprintf(val_buf, "--");
                else
                    sprintf(val_buf, "< %s >", screen_mode_label(screen_mode_menu()).c_str());
            } else if (i == 4) {
                if (wifi_ap_available())
                    sprintf(val_buf, "< %s >", menu_wifi_ap ? "ON" : "OFF");
                else
                    sprintf(val_buf, "N/A");
            } else if (i == kSysLicence) {
                sprintf(val_buf, "< %s >", menu_licence_ ? "YES" : "NO");
            } else if (i >= kSysArea && i <= kSysTimezone) {
                const int z = menu_zone_;
                if (z < 0) sprintf(val_buf, i == kSysArea ? "< NOT SET >" : "--");
                else if (i == kSysArea) snprintf(val_buf, cap, "< %s >", zones::area_name(zones::zone(z).area));
                else if (i == kSysCountry) snprintf(val_buf, cap, "< %s >", zones::country(zones::zone(z).country).name);
                else snprintf(val_buf, cap, "< %s >", zones::zone(z).city);
            }
        }

    // "*" marks a row the arrows have changed but Enter has not applied.
    //
    // The room test used to be against sizeof(val_buf), which is the size of
    // the pointer - so the mark only ever appeared on values shorter than six
    // characters, and rows like Overscan reading "NONE" never wore it. cap is
    // what the caller passed for exactly this.
    if (menu_tab >= 0 && menu_tab < kMenuTabs && i >= 0 && i < kMenuRows &&
        menu_dirty[menu_tab][i] && val_buf[0]) {
        size_t n = strlen(val_buf);
        if (n + 3 <= cap) { val_buf[n] = ' '; val_buf[n + 1] = '*'; val_buf[n + 2] = '\0'; }
    }

}

// The sections each blade holds, as menu_tab numbers, in the order its tabs
// are listed. VIDEO is the air unit's camera and needs a link to reach.
std::vector<int> OSD::ui_side_tabs(int side) const {
    if (side == 0) return { 2, 3, kTabDvr, kTabSystem };
    std::vector<int> v;
    if (osd_vars.artosyn.state == 2) v.push_back(0);
    v.push_back(1);
    v.push_back(+kTabAirSerial);   // by value: a static const int has no definition to bind to
    v.push_back(+kTabAirInfo);
    return v;
}

// Where a folded blade is, in pixels from the top left: its side of the
// picture, from the canopy's top rail (canopy_top_y_, frustum units) down.
void OSD::ui_blade_rect(int side, int W, int H, float fh, float& x, float& y, float& w, float& h) {
    int px, py, pw, ph;
    dev->picture_rect(W, H, px, py, pw, ph);
    const float top = py + (fh - canopy_top_y_) / (2.0f * fh) * ph;
    w = 0.31f * pw;
    x = side == 0 ? (float)px : px + pw - w;
    y = top;
    h = py + ph - top;
}

// The rows under the cursor change by themselves: the link dropping takes
// VIDEO away (draw_canopy_ui) and RADIO down to Bind, and an air unit's
// announce adds or removes a RADIO row. A value being changed acts only on
// the row it started on - OK on whatever row is there now would do that
// row's action instead: a bind that keeps the air unit out for 120 s, a
// standby.
bool OSD::menu_edit_row_intact() const {
    const std::vector<MenuItem> rows = menu_items(menu_tab);
    return menu_tab == menu_edit_tab_ && menu_index >= 0 && menu_index < (int)rows.size() &&
           rows[menu_index].type == 1 && rows[menu_index].label &&
           menu_edit_label_ == rows[menu_index].label;
}

// The canopy awake, drawn over the HUD: a light dimming, the two blades'
// names while one is being picked, a blade unfolded into its settings, the
// right blade unfolded into the channel band. fh is the frustum's half height.
void OSD::draw_canopy_ui(int W, int H, const CanopyIn& geo) {
    const float fh = geo.frustum_h;
    // VIDEO needs a link: unlinked, the air unit's side holds RADIO (Bind) only.
    if (osd_vars.artosyn.state != 2 && menu_tab == 0) {
        pthread_mutex_lock(&osd_mutex);
        menu_tab = ui_side_tab_[1] = 1;
        menu_index = menu_first_selectable_row(menu_tab);
        menu_refresh_options();
        pthread_mutex_unlock(&osd_mutex);
    }
    if (menu_index < 0) menu_index = 0;

    const HudTheme& T = hud_theme_current();
    int px, py, pw, ph;
    dev->picture_rect(W, H, px, py, pw, ph);
    const float u0 = ph / 1080.0f;
    const float u = u0 * osd_vars.ui_scale;
    auto ease = [](float t) { return t * t * (3.0f - 2.0f * t); };
    auto mix = [](float a, float b, float t) { return a + (b - a) * t; };
    const float open_any = std::max(ui_open_[0], ui_open_[1]);

    px_begin(W, H, px + pw * 0.5f, py + ph * 0.5f, 1.0f, 1.0f);
    // The picture stays the thing to look at: dimmed only a little.
    px_fill(px, py, pw, ph, T.ground, ui_wake_ * 0.10f + open_any * 0.06f);

    // Picking: each blade's name over it, slanted and in depth with it
    // (draw_canopy_pick), between the dimming and anything unfolding.
    const float pick = ui_wake_ * (1.0f - open_any) * (1.0f - ui_band_);
    if (pick > 0.003f) {
        px_flush();
        draw_canopy_pick(geo, pick);
        px_begin(W, H, px + pw * 0.5f, py + ph * 0.5f, 1.0f, 1.0f);
    }

    // A blade unfolded up its side into its settings.
    for (int side = 0; side < 2; side++) {
        const float t = ui_open_[side];
        if (t <= 0.003f) continue;
        float bx, by, bw, bh;
        ui_blade_rect(side, W, H, fh, bx, by, bw, bh);
        const float ow = 0.34f * pw, oy = py + 0.06f * ph, oh = 0.92f * ph;
        const float ox = side == 0 ? (float)px : px + pw - ow;
        const float e = ease(t);
        const float x = mix(bx, ox, e), y = mix(by, oy, e), w = mix(bw, ow, e), h = mix(bh, oh, e);
        px_fill(x, y, w, h, T.ground, 0.94f * std::min(1.0f, t * 1.6f));
        px_fill(x, y, w, 3 * u0, T.accent, t);
        if (t > 0.5f) {
            px_begin(W, H, px + pw * 0.5f, py + ph * 0.5f, 1.0f, (t - 0.5f) * 2.0f);
            draw_side_panel(side, x, y, w, h, u);
            px_begin(W, H, px + pw * 0.5f, py + ph * 0.5f, 1.0f, 1.0f);
        }
    }

    // The right blade unfolded along the bottom, into the channel band.
    if (ui_band_ > 0.003f) {
        const float t = ui_band_, e = ease(t);
        const float ow = 0.34f * pw, fx = px + pw - ow, fy = py + 0.06f * ph, fh2 = 0.92f * ph;   // from the unfolded blade
        const float bx = (float)px, by = py + 0.59f * ph, bw = (float)pw, bh = 0.41f * ph;
        const float x = mix(fx, bx, e), y = mix(fy, by, e), w = mix(ow, bw, e), h = mix(fh2, bh, e);
        // Soft at the top, so the picture runs into it.
        px_fill(x, y, w, h * 0.14f, T.ground, 0.45f * t);
        px_fill(x, y + h * 0.14f, w, h * 0.86f, T.ground, 0.9f * t);
        if (t > 0.5f && draw_channel_band(W, H, x, y, w, h, (t - 0.5f) * 2.0f)) {
            phase_lock_.store(true, std::memory_order_relaxed);
            signal_render(prof::kWakeAnim);
        }
        px_begin(W, H, px + pw * 0.5f, py + ph * 0.5f, 1.0f, 1.0f);
    }
    px_flush();
}


void OSD::handle_key(int key) {
    // Flags set under the mutex; acted on AFTER unlock to avoid deadlock.
    // (bg_player_->stop() calls pthread_join which would block while holding
    // osd_mutex, and the decode thread calls on_new_frame → signal_render →
    // pthread_mutex_lock(&osd_mutex) → deadlock.)
    bool do_start_player = false;
    bool do_stop_player  = false;

    pthread_mutex_lock(&osd_mutex);

    // Refresh Sync on trial: Enter, anywhere in the menu, keeps it.
    if (sync_confirm_ && (key == 13 || key == '\n')) {
        sync_confirm_ = false;
        Settings::getInstance().set(screen_id_.empty() ? std::string("display_lock")
                                                       : "display_lock_" + screen_id_, g_display_lock.load());
        printf("menu: refresh sync %s kept for %s\n", refresh_sync_label(g_display_lock.load()),
               screen_id_.empty() ? "this screen" : screen_id_.c_str());
        render_requested = true;
        pthread_cond_signal(&osd_cond);
        pthread_mutex_unlock(&osd_mutex);
        return;
    }
    // A screen mode on trial: Enter, anywhere in the menu, keeps it.
    if (screen_confirm_ && (key == 13 || key == '\n')) {
        screen_confirm_ = false;
        Settings::getInstance().set(screen_key_,
            screen_confirm_mode_ == "auto" ? std::string() : screen_confirm_mode_);
        printf("menu: screen mode %s kept for %s\n", screen_confirm_mode_.c_str(),
               screen_id_.empty() ? "this screen" : screen_id_.c_str());
        menu_open = false;
        render_requested = true;
        pthread_cond_signal(&osd_cond);
        pthread_mutex_unlock(&osd_mutex);
        return;
    }
    
    // The buttons. On the live picture the D-pad's Enter opens the menu and Back
    // opens the gallery. With the menu open they are what they were: Enter
    // applies, Back closes (as M does). With the gallery open they are its keys.
    if (gallery_.active()) {
        Gallery::Key gk = Gallery::kUp;
        bool is_gallery_key = true;
        if      (key == 'w' || key == 'W' || key == KEY_UP    || key == 0x101) gk = Gallery::kUp;
        else if (key == 's' || key == 'S' || key == KEY_DOWN  || key == 0x102) gk = Gallery::kDown;
        else if (key == 'a' || key == 'A' || key == KEY_LEFT  || key == 0x104) gk = Gallery::kLeft;
        else if (key == 'd' || key == 'D' || key == KEY_RIGHT || key == 0x103) gk = Gallery::kRight;
        else if (key == 13 || key == '\n')                                     gk = Gallery::kEnter;
        else if (key == kKeyBack || key == 27)                                  gk = Gallery::kBack;
        else is_gallery_key = false;   // M and the rest wait until it is closed
        if (is_gallery_key) {
            gallery_.key(gk);
            playback_ui_key_us_ = get_time_us();   // playback's overlay comes back
        }
        render_requested = true;
        pthread_cond_signal(&osd_cond);
        pthread_mutex_unlock(&osd_mutex);
        return;
    }
    if (menu_open && menu_scan_open) {
        chan_key(key);
        render_requested = true;
        pthread_cond_signal(&osd_cond);
        pthread_mutex_unlock(&osd_mutex);
        return;
    }
    if (key == kKeyBack || key == 27) {
        if (!menu_open) {
            // The next frame copies the screen, then opens the gallery on it
            // (gallery_capture_frame).
            gallery_capture_ = true;
            render_requested = true;
            pthread_cond_signal(&osd_cond);
            pthread_mutex_unlock(&osd_mutex);
            return;
        }
        // The canopy folds one step at a time: a value being changed (its
        // pending change stays, as it always has), then the blade.
        if (ui_stage_ == 1) {
            if (menu_focus == 2) menu_focus = 1;
            else ui_stage_ = 0;
            menu_refresh_options();
            render_requested = true;
            pthread_cond_signal(&osd_cond);
            pthread_mutex_unlock(&osd_mutex);
            return;
        }
        key = 'm';   // the canopy goes back to sleep
    } else if ((key == 13 || key == '\n') && !menu_open) {
        key = 'm';   // Enter on the live picture opens the menu
    }

    if (key == 'm' || key == 'M') {
        menu_open = !menu_open;
        if (menu_open) {
             // The canopy wakes with a blade picked - the one used last - and
             // nothing unfolded yet.
             ui_stage_ = 0;
             // SYSTEM > Screen Mode lists what the connector offers at this
             // open: a display plugged in since brings modes of its own. A
             // choice staged against the old list goes with it.
             screen_modes_.clear();
             menu_screen_mode = -1;
             menu_dirty[kTabSystem][3] = false;
             menu_pending_from[kTabSystem][3][0] = '\0';
             menu_opt_tab_ = -1;
             // Focus opens on the section column (menu_focus = 0, below) so
             // the first press goes somewhere useful, the same on every open
             // - but the row list underneath it must already have a real,
             // selectable row lit, not the -1 this used to leave behind.
             // That -1 was fine for drawing (the render loop clamped it to 0
             // before the next frame), but the key handler runs on its own
             // thread and reads menu_index too, under the same lock, and a
             // key pressed in the gap before that clamp ran acted on a row
             // that did not exist - and a header row is just as invalid a
             // "selection" as -1 is, if the tab or the settings under it
             // changed while the menu was closed. Reuse the same check a tab
             // switch already does.
             std::vector<MenuItem> rows = menu_items(menu_tab);
             if (menu_tab < 0 || menu_tab >= kMenuTabs ||
                 menu_index < 0 || menu_index >= (int)rows.size() ||
                 rows[menu_index].type == 2)
                 menu_index = menu_first_selectable_row(menu_tab);
             menu_focus = 0;
             // First open of the boot: start somewhere deliberate rather than
             // on whatever tab the enum happens to begin with. After that the
             // menu reopens where it was left.
             if (menu_first_open_) {
                 menu_first_open_ = false;
                 menu_tab = kTabSystem;
                 menu_index = menu_first_selectable_row(menu_tab);
                 menu_focus = 0;
             }
             menu_scan_open = false;
             menu_synced = false;
             if (cmd_cb) cmd_cb(0x0E, 0);
             memset(menu_dirty, 0, sizeof(menu_dirty));   // re-seeded = nothing pending
             menu_bf_osd = bf_osd;
             menu_clock_mode = clock_mode;
             menu_clock_show = clock_show;
             menu_hud_theme = hud_theme_idx;
             menu_zone_ = zones::selected();
             menu_zone_gen_ = zones::generation();
             menu_licence_ = zones::licensed() ? 1 : 0;
             menu_hud_style = hud_style;
             menu_volt_mode = volt_mode;
             menu_ui_scale = osd_vars.ui_scale;
             menu_picture_scale = dev->picture_scale_pct;
             menu_bg_video = bg_video_enabled;
             menu_frame_mode = g_frame_mode.load();
             menu_refresh_sync = g_display_lock.load();
             menu_sync_slice = g_display_lock_slice.load();
             menu_sync_margin = g_display_lock_margin_us.load() / 1000;
             menu_hud_reactivity = hud_reactivity;
             menu_hud_debug = hud_debug;
             menu_dvr_source = dvr_source_now();
             // Seed the RF rows from what the radio and the air unit report.
             // This air unit's own (Ar8030Source::load_sky_link), which the
             // radio is running with.
             menu_ar_power = Ar8030Source::tx_power_want_mw;   // shown fitted to the air unit (or --)
             menu_ar_standby = air_standby;   // start the row at the air's real state
             menu_ar_hop = Ar8030Source::chan_auto;
             menu_ar_maxbr = ar_maxbr_index(Ar8030Source::max_kbps);
             menu_ar_maxbw = ar_maxbw_index(Ar8030Source::max_bw_mhz);
             menu_ar_frame = ar_frame_index(Ar8030Source::frame_mode);
             menu_ar_maxmcs = Ar8030Source::max_mcs < 0 ? 0 : Ar8030Source::max_mcs + 1;
             menu_ar_steady = Ar8030Source::steady;
             menu_ar_fw = Ar8030Source::radio_fw;
             menu_refresh_options();      // seeded: work out what row 0 shows
        }
        render_requested = true;
        pthread_cond_signal(&osd_cond);
        pthread_mutex_unlock(&osd_mutex);
        return;
    }
    
    if (!menu_open) {
        pthread_mutex_unlock(&osd_mutex);
        return;
    }

    // The canopy (osd.hpp, ui_*). Picking: Left and Right are the two blades,
    // OK unfolds one. Unfolded: Left and Right walk its sections; Up and Down
    // its rows; OK on a setting starts changing it (Up/Down) and OK again
    // applies it; OK on an action (Bind, Calib Distance, Channel) does it.
    {
        const bool up    = (key == 'w' || key == 'W' || key == KEY_UP    || key == 0x101);
        const bool down  = (key == 's' || key == 'S' || key == KEY_DOWN  || key == 0x102);
        const bool left  = (key == 'a' || key == 'A' || key == KEY_LEFT  || key == 0x104);
        const bool right = (key == 'd' || key == 'D' || key == KEY_RIGHT || key == 0x103);
        const bool enter = (key == 13 || key == '\n');
        auto done = [&]() {
            render_requested = true;
            pthread_cond_signal(&osd_cond);
            pthread_mutex_unlock(&osd_mutex);
        };
        if (ui_stage_ == 0) {
            if (left) ui_side_ = 0;
            else if (right) ui_side_ = 1;
            else if (enter) {
                const std::vector<int> tabs = ui_side_tabs(ui_side_);
                int t = ui_side_tab_[ui_side_];
                if (std::find(tabs.begin(), tabs.end(), t) == tabs.end()) t = tabs[0];
                ui_side_tab_[ui_side_] = menu_tab = t;
                menu_index = menu_first_selectable_row(menu_tab);
                menu_focus = 1;
                ui_stage_ = 1;
                menu_refresh_options();
            }
            (void)up; (void)down;
            done();
            return;
        }
        if (left || right) {
            if (menu_focus == 2) {
                if (left) menu_focus = 1;            // out of the value, the change stays pending
            } else {
                const std::vector<int> tabs = ui_side_tabs(ui_side_);
                int i = (int)(std::find(tabs.begin(), tabs.end(), menu_tab) - tabs.begin());
                if (i >= (int)tabs.size()) i = 0;
                i = std::max(0, std::min((int)tabs.size() - 1, i + (right ? 1 : -1)));
                ui_side_tab_[ui_side_] = menu_tab = tabs[i];
                menu_index = menu_first_selectable_row(menu_tab);
            }
            menu_refresh_options();
            done();
            return;
        }
        if (enter && menu_focus == 1) {
            std::vector<MenuItem> rows = menu_items(menu_tab);
            const int type = (menu_index >= 0 && menu_index < (int)rows.size()) ? rows[menu_index].type : 0;
            if (type == 1) {                          // a setting: start changing it
                menu_focus = 2;
                menu_edit_tab_ = menu_tab;
                menu_edit_label_ = rows[menu_index].label ? rows[menu_index].label : "";
                menu_refresh_options();
                done();
                return;
            }
            if (type != 3) { done(); return; }        // a reading or a group's name
            // An action: Enter below does it.
        } else if (enter && menu_focus == 2) {
            menu_focus = 1;                           // OK applies (below) and the row closes
            if (!menu_edit_row_intact()) {            // ...unless the row went: OK only closes it
                menu_refresh_options();
                done();
                return;
            }
        }
        // Up and Down go on below: the row with menu_focus 1, the value with 2.
    }

    if (key == 9) {
       menu_tab = (menu_tab + 1) % kMenuTabs;
       menu_index = -1;
       render_requested = true;
       pthread_cond_signal(&osd_cond);
       pthread_mutex_unlock(&osd_mutex);
       return;
    }

    // Derived, never hardcoded - see OSD::menu_items().
    int items_count = (int)menu_items(menu_tab).size();

    // Up and down move within the list (menu_focus 1) or change the value
    // being edited (2); left and right are the canopy's (above).
    const bool k_up    = (key == 'w' || key == 'W' || key == KEY_UP    || key == 0x101);
    const bool k_down  = (key == 's' || key == 'S' || key == KEY_DOWN  || key == 0x102);

    if (k_up || k_down) {
        const int dir = k_down ? -1 : 1;   // rows ascend, so UP increments
        if (menu_focus == 0) {
            bool connected = (osd_vars.artosyn.state == 2);
            int next = menu_tab - dir;     // the section column reads top-down
            while (next < 0) next += kMenuTabs;
            next %= kMenuTabs;
            if (!connected && next < 1) next = (dir < 0) ? 1 : kMenuTabs - 1;   // VIDEO needs a link; RF LINK holds Bind
            menu_tab = next;
            menu_index = menu_first_selectable_row(menu_tab);
        } else if (menu_focus == 1) {
            std::vector<MenuItem> rows = menu_items(menu_tab);
            int n = (int)rows.size();
            if (n > 0) {
                menu_index = (menu_index - dir + n) % n;
                // Section headers are not a row to land on - keep stepping
                // the same direction until a real setting turns up.
                for (int guard = 0; guard < n && rows[menu_index].type == 2; guard++)
                    menu_index = (menu_index - dir + n) % n;
            }
        } else if (!menu_edit_row_intact()) {
            menu_focus = 1;                           // the row being changed went: nothing to step
        } else {
            std::vector<MenuItem> rows = menu_items(menu_tab);
            const bool ro = (menu_index >= 0 && menu_index < (int)rows.size() &&
                             rows[menu_index].type == 0);
            const bool tracked = !ro && menu_tab >= 0 && menu_tab < kMenuTabs &&
                                 menu_index >= 0 && menu_index < kMenuRows &&
                                 !menu_row_applies_live(menu_tab, menu_index);
            // A row Enter has to commit: remember what it read before the
            // first nudge, so the value column can say what is pending and
            // so walking back to it counts as changing nothing.
            char* from = tracked ? menu_pending_from[menu_tab][menu_index] : nullptr;
            if (from && !menu_dirty[menu_tab][menu_index])
                menu_plain_value(menu_tab, menu_index, from, sizeof(menu_pending_from[0][0]));
            // The value column is a list read top to bottom, and the
            // highlight follows the key: UP is the entry above, which is
            // the one before it in the row's order.
            if (!ro) menu_apply_change(menu_tab, menu_index, -dir);
            if (from) {
                char now[32];
                menu_plain_value(menu_tab, menu_index, now, sizeof(now));
                const bool back = (strcmp(now, from) == 0);
                menu_dirty[menu_tab][menu_index] = !back;
                if (back) from[0] = '\0';
            }
        }
        // The selection or the value just moved, so the value column has to be
        // worked out again - here, where the menu state is ours to step.
        menu_refresh_options();
        render_requested = true;
        pthread_cond_signal(&osd_cond);
        pthread_mutex_unlock(&osd_mutex);
        return;
    }

    // The tab strip has no row of its own for Enter to act on. This used to
    // be keyed off menu_index == -1, a sentinel a fresh menu open left
    // behind - but left/right (above) and up/down (further above) already
    // move the SECTION column via menu_focus alone and never touched that
    // sentinel, so menu_index could just as easily be a real row while
    // menu_focus was still 0 (move to the row column, then back to the
    // tabs with a left press). menu_focus is what actually says which
    // column is lit, and now that a menu open always leaves menu_index on a
    // real, selectable row (see where it opens), it is the only thing this
    // needs to check.
    if (menu_focus == 0) {
        pthread_mutex_unlock(&osd_mutex);
        return;
    }
    
    if (key == 13 || key == '\n') { // Enter
        if (menu_open) {
            menu_mark_dirty(false);      // committing is what clears the marker
            if (menu_tab == 0) {
                // 0x01 = video mode (routed to SET_CHN_RES in main); 0x300+n are
                // the AR8030 camera settings, kept out of the kestrel-air id space.
                if (cmd_cb) {
                    switch (video_row(menu_index)) {
                        case 0: cmd_cb(0x01,  menu_video_mode); break;
                        case 3: cmd_cb(0x301, kEvSteps[menu_cam_ev]); break;
                        case 5: cmd_cb(0x302, menu_cam_sat); break;
                        case 6: cmd_cb(0x303, menu_cam_contrast); break;
                        case 7: cmd_cb(0x304, menu_cam_sharp); break;
                        case 2: cmd_cb(0x305, kSceneVals[menu_cam_scene]); break;
                        case 4: cmd_cb(0x307, wb_kelvin(menu_cam_wb)); break;
                        case 1:
                            // 0x30B = CAM_ANGLE -> sky cmd 0x06. The camera
                            // accepts this and reports the new angle back in
                            // its status frame, but the picture does not change
                            // on its own - so follow it with the current video
                            // mode, which is what makes the camera reconfigure
                            // the channel (section 21.7). Order matters: the
                            // angle has to be set before the mode is re-applied.
                            cmd_cb(0x30B, menu_cam_flip ? 1 : 0);
                            cmd_cb(0x01,  menu_video_mode);
                            break;
                        // 0x30D = CAM_3DNR. No sky command exists for it, so
                        // the source re-sends CMD_SET_CONFIG instead.
                        case 8: cmd_cb(0x30D, kDnrVals[menu_cam_3dnr]); break;
                        case 9: cmd_cb(0x306, menu_cam_focus ? 1 : 0); break;
                        // Max Bitrate: saved, and sent again at every link-up.
                        case 10: cmd_cb(0x30E, kArMaxBrVals[menu_ar_maxbr]); break;
                        default: break;
                    }
                }
            } else if (menu_tab == 1) {
                // AR8030: RF ioctls go straight to our own radio via
                // Ar8030Source (0x310 + field). No kestrel-air hop, and no
                // a second SDK client would fight the source
                // for the daemon's single owner slot.
                auto rfi = ar_rf_items();
                if (cmd_cb && menu_index >= 0 && menu_index < (int)rfi.size()) {
                    switch (rfi[menu_index].second) {
                        case 5:
                            chan_open();
                            break;
                        case 1: {   // saved per air unit there; nothing before it is known
                            const int prj = Ar8030Source::air_prj.load();
                            if (prj > 0) cmd_cb(0x311, ar_pwr_fit(prj, menu_ar_power));
                            break;
                        }
                        case 8:
                            // Force standby on/off (sky cmd 0x23 via the queue that
                            // owns the port-2 socket). Momentary: NOT persisted, and
                            // the air overrides it via fpv_sky_standby_mode_thread
                            // (auto-parks when idle, exits when flying). Stock offers
                            // the same temporary force from its menu.
                            cmd_cb(0x308, menu_ar_standby ? 1 : 0);
                            break;
                        case 11:
                            // Saved and sent to the air unit; again at every link-up.
                            cmd_cb(0x30F, kArMaxBwVals[menu_ar_maxbw]);
                            break;
                        case 12:
                            // Saved and sent to the radio now; again after every link-up.
                            cmd_cb(0x30A, kArFrameVals[menu_ar_frame]);
                            break;
                        case 14:
                            // Saved and sent to our radio now; again after every link-up.
                            cmd_cb(0x320, menu_ar_maxmcs - 1);
                            break;
                        case 15:
                            // Saved and sent to our radio now; again after every link-up.
                            cmd_cb(0x321, menu_ar_steady);
                            break;
                        case 13:
                            // Saved; a change restarts the radio with that firmware.
                            cmd_cb(0x309, menu_ar_fw);
                            break;
                        case 9:
                            // Bind. Parked for the RX thread - the menu runs on the
                            // OSD thread and must never call into the baseband SDK.
                            // This branch runs whenever the radio reports any
                            // capability, i.e. always in practice; the id-based
                            // dispatch below it is only the no-capabilities
                            // fallback, which is why a handler there alone left
                            // this row's Enter doing nothing.
                            Ar8030Source::request_bind();
                            break;
                        default: break;
                    }
                }
            } else if (menu_tab == 2) { // HUD - see menu_items() and hud_row()
                // Only Calib Distance has anything for Enter to do. Every
                // other row applies and saves as the arrows move it.
                if (hud_row(menu_index) == kHudRowCalib) {
                    // Zero the ranging: with the units side by side, Enter
                    // captures the current reading as the new base value. The
                    // arrows nudge the same offset by hand.
                    if (link_distance_raw >= 0) {
                        dist_offset = link_distance_raw;
                        save_dist_offset();
                        printf("[OSD] distance calibrated for air unit %s: offset = %d\n",
                               osd_peer_mac_[0] ? osd_peer_mac_ : "?", dist_offset);
                    }
                }
            } else if (menu_tab == 3) { // DISPLAY
                // Brightness, UI Scaling, Theme and Picture Size all apply and
                // save as they are nudged. Background Video is the one row
                // here Enter still owns, because switching it starts or stops
                // a player rather than changing a number.
                if (menu_index == kDispBgVideo) {
                    bg_video_enabled = menu_bg_video;
                    Settings::getInstance().set("bg_video_enabled", bg_video_enabled);
                    // Only start the player if we are still in idle (BACKGROUND) state -
                    // once FPV has been received the frozen last frame stays on screen.
                    do_start_player = bg_video_enabled && (video_state_ == VideoState::BACKGROUND);
                    do_stop_player  = !bg_video_enabled;
                }
            } else if (menu_tab == kTabDvr) {
                if (menu_index == 0) {
                    // Just a mode flag now. The REC button owns starting and
                    // stopping; this used to spin up a second DVR of its own the
                    // moment it was switched on, which recorded to a fixed
                    // dvr_screen.mp4 and had nothing to do with the REC button.
                    dvr_screen = menu_dvr_source > 0;
                    Settings::getInstance().set("dvr_screen", dvr_screen);
                    DvrRecorder::instance().set_screen_mode(dvr_screen);
                }
            } else if (menu_tab == kTabSystem) {
                if (menu_index == 3 && screen_mode_menu() != screen_mode_saved()) {
                    // Only a trial: main.cpp applies it at the restart and asks
                    // (begin_screen_mode_confirm); screen_key_ is written once
                    // it is kept.
                    const int m = screen_mode_menu();
                    char v[32] = "auto";
                    if (m > 0) {
                        const ScreenModeOpt& o = screen_modes()[m - 1];
                        snprintf(v, sizeof(v), "%dx%d@%d", o.w, o.h, o.hz);
                    }
                    Settings::getInstance().set("screen_mode_try",
                        (screen_id_.empty() ? std::string("-") : screen_id_) + " " + v);
                    printf("menu: screen mode %s on trial - restarting to apply it\n", v);
                    kestrel_request_restart();
                }
                if (menu_index == 4 && wifi_ap_available()) {
                    wifi_ap_set(menu_wifi_ap != 0);   // for this session only
                }
            }
            render_requested = true;
        }
    }
    
    // Set the flag, not just the signal. run()'s wait is
    //     while (!render_requested && ... ) pthread_cond_timedwait(...)
    // so a bare signal wakes the thread, the predicate re-checks, and it goes
    // straight back to sleep - the wakeup is swallowed. Several key paths
    // (menu up/down among them) signalled without setting the flag, so those
    // keys only redrew on the 1000 ms idle timeout. With video decoding the
    // per-frame redraw hid it; with no link and no video there is nothing else
    // driving the loop, which is why navigation felt laggy exactly then.
    render_requested = true;
    pthread_cond_signal(&osd_cond);
    pthread_mutex_unlock(&osd_mutex);

    // Player start/stop MUST happen outside osd_mutex: stop() calls
    // pthread_join() which blocks until the decode thread exits, and the
    // decode thread calls on_new_frame → signal_render → locks osd_mutex.
    if (do_stop_player && bg_player_)  bg_player_->stop();
    else if (do_start_player && bg_player_) bg_player_->start();
}

void OSD::update_menu_from_tx(int cmd, int val) {
    pthread_mutex_lock(&osd_mutex);
    // The legacy kestrel-air menu rows (bitrate, slice, exposure, ...) and the
    // no-capabilities RF list are gone; their acks are simply ignored here.
    if (cmd == 0x01) menu_video_mode = val;
    else if (cmd == 0x07) osd_vars.sky_exposure_us = val;
    else if (cmd == 0x08) osd_vars.sky_framerate = val;
    else if (cmd == 0x17) {
        // AIR-owned channel MODE (1=auto hop, 0=force). Arrives before 0x11.
        // (This used to skip a goggle-side "pin", read from a pinned_chan
        // setting nothing has written since pinning was removed.)
        air_chan_mode = val;
        if (val == 1) {
            if (!rf_chan_asserted && cmd_cb) {
                rf_chan_asserted = true;
                cmd_cb(0x107, 1);                 // re-assert AUTO on the AP
            }
        }
    }
    else if (cmd == 0x11) {
        // AIR-owned forced channel: re-assert on our AP radio once per link
        // session (channel is AP-arbitrated and resets to config on re-pair).
        if (air_chan_mode != 1 && !rf_chan_asserted && cmd_cb) {
            rf_chan_asserted = true;
            printf("[rf] re-asserting AIR-owned channel %d on the AP\n", val);
            cmd_cb(0x107, 0);
            cmd_cb(0x101, val);
        }
    }
    else if (cmd == 0x14) {
        // AIR-owned RF bandwidth. The AP arbitrates bandwidth and resets it to
        // config on every re-pair, so re-assert the persisted value ONCE per
        // link session (rf_bw_applied is re-armed on link drop).
        if (val != rf_bw_applied && cmd_cb) {
            rf_bw_applied = val;
            printf("[rf] re-asserting AIR-owned bandwidth enum %d on the AP\n", val);
            cmd_cb(0x104, val);
        }
    }
    
    menu_synced = true;
    render_requested = true;
    pthread_cond_signal(&osd_cond);
    pthread_mutex_unlock(&osd_mutex);
}

// True if a decoded FPV frame arrived within the last 1.5s — the "pipeline is
// actually producing pictures" signal for the reconnect retry in main.
bool OSD::video_decoding() {
    pthread_mutex_lock(&osd_mutex);
    uint64_t last = last_fpv_frame_us_;
    pthread_mutex_unlock(&osd_mutex);
    return last != 0 && (get_time_us() - last) < 1500000ULL;
}

void OSD::notify_video_frame(bool is_keyframe) {
    pthread_mutex_lock(&osd_mutex);
    uint64_t now = get_time_us();
    // The frames that change what the HUD shows: video coming back after a
    // stall (the lock screen, stale-picture dimming and the red rails all key
    // off frame freshness), and the one that confirms video on this link.
    const uint64_t prev_frame_us = last_fpv_frame_us_;
    bool edge = last_fpv_frame_us_ == 0 || now - last_fpv_frame_us_ > 400000;
    last_fpv_frame_us_ = now;
    if (frames_since_connect_ < ~0u) frames_since_connect_++;
    if (frames_since_connect_ == kFramesForVideo) edge = true;
    if (video_state_ == VideoState::BACKGROUND) {
        // The first connection's warp into live video waits for a decent
        // picture: a link comes up on 5 MHz and widens from there, and its
        // first second or two of video (a Mbit/s or so) is a smear that the
        // zoom would land on. Decoding goes on underneath the idle screen;
        // the warp starts once the video runs above kRevealMbps - or 2 below
        // the Max Bitrate cap when that is lower - or kRevealTimeoutUs after
        // the first clean picture if it never does.
        // Pictures stopping for a while (the link lost before the reveal)
        // start the wait over.
        if (reveal_wait_us_ && now - prev_frame_us > 1500000) reveal_wait_us_ = 0;
        const int cap_kbps = kArMaxBrVals[menu_ar_maxbr];   // 0 = AUTO, no cap
        const float reveal_mbps = cap_kbps > 0 ? std::min(kRevealMbps, cap_kbps / 1000.0f - 2.0f) : kRevealMbps;
        if (is_keyframe && !reveal_wait_us_) {
            reveal_wait_us_ = now;
            printf("[OSD] First clean keyframe decoded - live FPV once the video is above %.0f Mbit/s\n",
                   reveal_mbps);
        }
        const float mbps = osd_vars.video_bandwidth;
        const bool good = mbps > reveal_mbps;
        if (reveal_wait_us_ && (good || now - reveal_wait_us_ >= kRevealTimeoutUs)) {
            video_state_         = VideoState::TRANSITION;
            transition_start_us_ = now;
            printf("[OSD] Starting transition to live FPV after %.1f s: video %.1f Mbit/s%s\n",
                   (now - reveal_wait_us_) / 1e6, mbps, good ? "" : " (timed out)");
            edge = true;
        }
    }
    if (edge) {
        render_requested = true;
        pthread_cond_signal(&osd_cond);
    }
    pthread_mutex_unlock(&osd_mutex);
}

// Draw background.png full-screen with optional warp-zoom transition.
// warp_t == 0: static full-frame display.
// warp_t in (0,1]: same zoom + alpha-fade curves as the warp shader so the
// PNG participates in the BACKGROUND→LIVE transition identically to the video.
void OSD::draw_bg_png(float fw, float fh, const float* mvp, float warp_t) {
    if (!bg_tex) return;

    // Mirror the warp shader's curves exactly:
    //   zoom  = 1 + t² × 8   (geometry expansion ≡ UV zoom into centre)
    //   alpha = max(0, 1 − t² × 1.35)
    float t2   = warp_t * warp_t;
    float zoom  = 1.0f + t2 * 8.0f;
    float alpha = std::max(0.0f, 1.0f - t2 * 1.35f);

    float qw = fw * zoom;   // expanded quad half-extents
    float qh = fh * zoom;

    float verts[] = {
        -qw, -qh, 0.0f,  0.0f, 1.0f,
         qw, -qh, 0.0f,  1.0f, 1.0f,
        -qw,  qh, 0.0f,  0.0f, 0.0f,
         qw,  qh, 0.0f,  1.0f, 0.0f,
    };

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, bg_tex);
    glUseProgram(shader_program);
    glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp);
    glUniform1i(u_tex_, 0);
    glUniform1i(u_is_text_,    1);
    glUniform1i(u_use_shading_, 0);
    glUniform4f(u_color_, 1.0f, 1.0f, 1.0f, 1.0f);
    glUniform1f(u_alpha_, 1.0f);

    // Set fade via the v_alpha_factor constant vertex attribute BEFORE drawing.
    GLint alpha_loc = a_alpha_factor_;
    if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, alpha);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);

    GLint pos_loc = a_pos_;
    GLint uv_loc  = a_uv_;
    glEnableVertexAttribArray(pos_loc);
    glEnableVertexAttribArray(uv_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glVertexAttribPointer(uv_loc,  2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // Always reset to opaque so subsequent draws are unaffected.
    if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, 1.0f);
}

// Import a player's newest decoded picture (an NV12 DMA-BUF) as the external
// texture `tex`. The one place this is done, for the idle background and for
// a recording in the gallery.
bool OSD::import_drm_frame(BgVideoPlayer* player, GLuint tex) {
    if (!player || !pfn_eglCreateImageKHR || !pfn_glEGLImageTargetTexture2DOES) return false;
    BgDrmFrame drm;
    if (!player->get_latest_drm_frame(drm) || !drm.valid) return false;
    EGLint attribs[32]; int ai = 0;
    attribs[ai++] = EGL_WIDTH;                    attribs[ai++] = drm.width;
    attribs[ai++] = EGL_HEIGHT;                   attribs[ai++] = drm.height;
    attribs[ai++] = EGL_LINUX_DRM_FOURCC_EXT;    attribs[ai++] = (EGLint)drm.drm_fmt;
    attribs[ai++] = EGL_DMA_BUF_PLANE0_FD_EXT;  attribs[ai++] = drm.fd;
    attribs[ai++] = EGL_DMA_BUF_PLANE0_OFFSET_EXT; attribs[ai++] = 0;
    attribs[ai++] = EGL_DMA_BUF_PLANE0_PITCH_EXT;  attribs[ai++] = (EGLint)drm.stride_y;
    attribs[ai++] = EGL_DMA_BUF_PLANE1_FD_EXT;  attribs[ai++] = drm.fd;
    attribs[ai++] = EGL_DMA_BUF_PLANE1_OFFSET_EXT; attribs[ai++] = (EGLint)drm.offset_uv;
    attribs[ai++] = EGL_DMA_BUF_PLANE1_PITCH_EXT;  attribs[ai++] = (EGLint)drm.stride_uv;
    if (drm.modifier != DRM_FORMAT_MOD_INVALID && drm.modifier != DRM_FORMAT_MOD_LINEAR) {
        attribs[ai++] = EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT; attribs[ai++] = (EGLint)(drm.modifier & 0xFFFFFFFF);
        attribs[ai++] = EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT; attribs[ai++] = (EGLint)(drm.modifier >> 32);
        attribs[ai++] = EGL_DMA_BUF_PLANE1_MODIFIER_LO_EXT; attribs[ai++] = (EGLint)(drm.modifier & 0xFFFFFFFF);
        attribs[ai++] = EGL_DMA_BUF_PLANE1_MODIFIER_HI_EXT; attribs[ai++] = (EGLint)(drm.modifier >> 32);
    }
    attribs[ai++] = EGL_NONE;

    EGLImageKHR img = pfn_eglCreateImageKHR(
        display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, attribs);
    if (img == EGL_NO_IMAGE_KHR) {
        fprintf(stderr, "[OSD] eglCreateImageKHR failed: 0x%x\n", eglGetError());
        return false;
    }
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, tex);
    pfn_glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, img);
    pfn_eglDestroyImageKHR(display, img);
    // drm.ref drops here — EGL holds its own GEM import reference
    return true;
}

void OSD::draw_bg_video(float fw, float fh, const float* mvp, float warp_t) {
    bool use_ext = bg_player_ && bg_player_->is_drm() &&
                   bg_ext_tex_ && bg_ext_shader_;
    if (!use_ext && !bg_video_tex_) return;

    int screen_w = dev->output_list->mode.hdisplay;
    int screen_h = dev->output_list->mode.vdisplay;
    float screen_asp = (float)screen_w / (float)screen_h;
    float video_asp  = (bg_player_ && bg_player_->height() > 0)
                       ? (float)bg_player_->width() / (float)bg_player_->height()
                       : screen_asp;

    // Crop-to-fill UV so bg video covers the screen without black bars
    float u0 = 0.0f, u1 = 1.0f, v0 = 0.0f, v1 = 1.0f;
    if (video_asp > screen_asp) {
        float s = screen_asp / video_asp;
        u0 = 0.5f - s * 0.5f;
        u1 = 0.5f + s * 0.5f;
    } else if (video_asp < screen_asp) {
        float s = video_asp / screen_asp;
        v0 = 0.5f - s * 0.5f;
        v1 = 0.5f + s * 0.5f;
    }

    float verts[] = {
        -fw, -fh, 0.0f,  u0, v1,
         fw, -fh, 0.0f,  u1, v1,
        -fw,  fh, 0.0f,  u0, v0,
         fw,  fh, 0.0f,  u1, v0,
    };

    glActiveTexture(GL_TEXTURE0);

    GLuint prog;
    if (use_ext) {
        // DRM/EGL zero-copy path — use external texture shaders
        GLenum ext_target = GL_TEXTURE_EXTERNAL_OES;
        glBindTexture(ext_target, bg_ext_tex_);
        prog = (warp_t > 0.0f && warp_ext_shader_) ? warp_ext_shader_ : bg_ext_shader_;
        glUseProgram(prog);
        glUniformMatrix4fv(glGetUniformLocation(prog, "mvp"), 1, GL_FALSE, mvp);
        glUniform1i(glGetUniformLocation(prog, "tex"), 0);
        if (warp_t > 0.0f)
            glUniform1f(glGetUniformLocation(prog, "warp_t"), warp_t);
    } else {
        // CPU/BGRA path — existing shader flow
        glBindTexture(GL_TEXTURE_2D, bg_video_tex_);
        prog = (warp_t > 0.0f && warp_shader_prog_) ? warp_shader_prog_ : shader_program;
        glUseProgram(prog);
        glUniformMatrix4fv(glGetUniformLocation(prog, "mvp"), 1, GL_FALSE, mvp);
        glUniform1i(glGetUniformLocation(prog, "tex"), 0);
        if (warp_t > 0.0f && warp_shader_prog_) {
            glUniform1f(glGetUniformLocation(warp_shader_prog_, "warp_t"), warp_t);
        } else {
            glUniform1i(u_is_text_,    1);
            glUniform1i(u_use_shading_, 0);
            glUniform4f(u_color_, 1.0f, 1.0f, 1.0f, 1.0f);
            glUniform1f(u_alpha_, 1.0f);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);

    GLint pos_loc = glGetAttribLocation(prog, "pos");
    glEnableVertexAttribArray(pos_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);

    GLint uv_loc = glGetAttribLocation(prog, "uv");
    glEnableVertexAttribArray(uv_loc);
    glVertexAttribPointer(uv_loc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void*)(3 * sizeof(float)));

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glUseProgram(shader_program);
    GLint alpha_loc = a_alpha_factor_;
    if (alpha_loc != -1) glVertexAttrib1f(alpha_loc, 1.0f);
}

void OSD::update_msp_data(const uint8_t* data, size_t len) {
    {
    std::lock_guard<std::mutex> lk(msp_rx_mtx);
    // Bounded: the OSD thread wakes at most every refresh_frequency_ms, and a
    // stalled one must not grow this without limit. 64 KB is ~17 s of the
    // measured 3.8 KB/s telemetry stream.
    if (msp_rx_buf.size() + len > 64 * 1024) {
        msp_rx_dropped += len;
        return;
    }
    msp_rx_buf.insert(msp_rx_buf.end(), data, data + len);
    }
    // Let the OSD thread parse it - at most ~30 times a second. It redraws
    // only if the parsed data changes what the HUD shows (MspOsd::content_ver_).
    // Always note that data is waiting (run() picks it up within 33 ms even
    // if this call does not signal), so the last chunk of a burst is never
    // left sitting until the next heartbeat.
    uint64_t now = get_time_us();
    pthread_mutex_lock(&osd_mutex);
    msp_dirty_ = true;
    if (now - last_msp_signal_us_ >= 33000) {
        last_msp_signal_us_ = now;
        msp_wake_ = true;
        pthread_cond_signal(&osd_cond);
    }
    pthread_mutex_unlock(&osd_mutex);
}

void OSD::drain_msp() {
    std::vector<uint8_t> pending;
    {
        std::lock_guard<std::mutex> lk(msp_rx_mtx);
        if (msp_rx_buf.empty()) return;
        pending.swap(msp_rx_buf);
    }
    msp_osd.parse_bytes(pending.data(), pending.size());
}


void OSD::set_dvr(std::shared_ptr<DVR> dvr_, bool dvr_screen_) {
    pthread_mutex_lock(&osd_mutex);
    dvr = dvr_;
    dvr_screen = dvr_screen_;
    pthread_mutex_unlock(&osd_mutex);
}

