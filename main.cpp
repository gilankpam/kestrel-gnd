
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <execinfo.h>
#include <dlfcn.h>
#include <ucontext.h>
#include <sys/prctl.h>
#include <memory>
#include <linux/random.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <thread>

extern "C" {
    bool verbose_vid = false;
    bool verbose_rf  = false;
}

#include "utils/sd_log.hpp"
#include "frame_mode.hpp"
#include "webstream.hpp"
#include "main.hpp"
#include "drm.hpp"
#include "osd.hpp"
#include "vdec/vdec_helper.hpp"
#include "renderer.hpp"
#include "utils/ltrace.hpp"
#include "utils/vrx_buttons.hpp"
#include "artosyn/ar8030_source.hpp"
#include "artosyn/bb_watchdog.hpp"
#include "artosyn/bb_client.h"
#include "utils/term.h"
#include "utils/input_reader.hpp"
#ifdef MODULE_TAG
#undef MODULE_TAG
#endif
#define MODULE_TAG "kestrel-gnd"
#include "kestrel_gnd_config.h"
#include "settings.hpp"
#include "zones.hpp"
#include "utils/screen_id.h"

using namespace std;


// Threads
// No wireless link threads anymore

// signal
volatile bool signal_stop = false;

// Ctrl-C has to work even when a thread is wedged inside the AR8030 SDK.
// Ar8030Source::run() executes on the main thread, and bb_init()/bb_socket_open()
// block with no timeout whenever the baseband is not healthy (a previous client
// killed with SIGKILL leaves the daemon's single owner slot taken). In that state
// nothing is left alive to notice signal_stop, so a plain flag-setting handler
// leaves the process unkillable by anything short of SIGKILL.
//
// So: first signal asks for a graceful stop and arms a watchdog; a second signal,
// or the watchdog firing, exits immediately. Everything below is
// async-signal-safe (write/_exit/alarm), unlike printf.
static volatile sig_atomic_t stop_requested = 0;
#define SIG_MSG(m) do { ssize_t r_ = write(STDERR_FILENO, m, sizeof(m) - 1); (void)r_; } while (0)

static void force_exit(int signum)
{
	(void)signum;
	SIG_MSG("\nkestrel-gnd: forced exit (shutdown wedged, most likely in the BB SDK)\n");
	_exit(130);
}

// kill -USR1 <pid>: start/stop a recording. Same entry point as the REC button.
// The button was the only way in, which made the DVR impossible to exercise
// over ssh or from a script - and that is how several DVR bugs stayed hidden.
//
// This was documented here and never registered, so SIGUSR1 kept its default
// disposition and killed the process instead. Anyone following the comment to
// test the DVR remotely just watched the app die.
static volatile sig_atomic_t g_rec_toggle = 0;
static void rec_toggle_handler(int) { g_rec_toggle = 1; }
// A restart, for a setting that is only read at start - the screen mode: the
// display is set up once. The ordinary shutdown runs, so a recording is
// stopped and finalized and the radio's sockets are closed, and main() then
// execs the same binary where it would have returned. Should that shutdown
// hang, the alarm that would force an exit re-execs instead: a restart must
// never end with no app on the screen. Everything here is async-signal-safe.
static volatile sig_atomic_t g_restart = 0;
static char** g_main_argv = nullptr;
static void reexec_now(int)
{
	sd_log::restore_output();   // the new process starts its own
	for (int fd = 3; fd < 1024; fd++) close(fd);
	execv("/proc/self/exe", g_main_argv);
	_exit(91);
}
void kestrel_request_restart()
{
	if (stop_requested) return;
	// The setting that asked for the restart (a screen mode on trial) must be
	// on disk for the new process, and the forced re-exec below cannot write it.
	Settings::getInstance().flush();
	g_restart = 1;
	stop_requested = 1;
	signal_stop = true;
	signal(SIGALRM, reexec_now);
	alarm(5);
}

void sig_handler(int signum)
{
	if (stop_requested) force_exit(signum);   // second Ctrl-C: go now
	stop_requested = 1;
	signal_stop = true;
	SIG_MSG("\nkestrel-gnd: stopping (Ctrl-C again to force)\n");
	// If the graceful path has not finished in 5 s it is blocked in a call that
	// cannot be interrupted; take the process down rather than hang.
	signal(SIGALRM, force_exit);
	alarm(5);
}


// The VRX Pro's own front buttons are a SARADC resistor ladder, not an input
// device (see utils/vrx_buttons.hpp), so they need their own poller. Mapped onto
// the same keys the keyboard path already uses, so menu behaviour is identical:
//   up/down    'w'/'s'  move between rows
//   left/right 'a'/'d'  change the selected value
//   ok         Enter    apply; on the live picture, open the menu
//   esc        Back     on the live picture, open the gallery; in the menu, close it
//   rec        'r'      currently unbound in OSD::handle_key - the app has no
//                       runtime record toggle yet, so this is a hook, not a
//                       working record button.
void* vrx_button_thread_func(void* arg) {
	OSD* osd = (OSD*)arg;
	SchedulingHelper::set_thread_params_max_realtime("VRX_BTN", 10);
	VrxButtons buttons;
	// The signal handler only sets a flag: DvrRecorder::toggle() takes a lock
	// and opens files, none of which is safe to do inside a signal.
	auto service_rec_signal = [&]() {
		if (!g_rec_toggle) return;
		g_rec_toggle = 0;
		bool on = DvrRecorder::instance().toggle();
		printf("SIGUSR1: rec -> %s\n", on ? "RECORDING" : "stopped");
		fflush(stdout);
		if (osd) osd->signal_render(prof::kWakeInput);
	};
	if (!buttons.available()) {
		printf("VRX buttons: SARADC channel not readable, front panel disabled\n");
		return nullptr;
	}
	printf("VRX buttons: front panel active (SARADC ladder)\n");
	// The BIND button is a separate ADC channel, not part of the ladder above.
	// Polled on this same thread because it is the same kind of device and the
	// cadence suits it; see utils/vrx_buttons.hpp for why it is read directly
	// rather than through its input device.
	VrxBindButton bind_button;
	const bool have_bind = bind_button.available();
	printf("VRX buttons: bind button %s\n",
	       have_bind ? "active (SARADC ch0)" : "NOT readable - bind via the menu");
	while (!signal_stop) {
		service_rec_signal();
		if (have_bind && bind_button.pressed()) {
			// Binding is not a menu action - it has to work with the menu
			// shut, which is the state anyone reaching for this button is in.
			printf("VRX buttons: BIND pressed -> requesting bind\n");
			fflush(stdout);
			Ar8030Source::request_bind();
			if (osd) osd->signal_render(prof::kWakeInput);
		}
		VrxButtons::Key b = buttons.poll(get_time_ms());
		if (b != VrxButtons::VB_NONE) {
			int key = 0;
			switch (b) {
				case VrxButtons::VB_UP:    key = 'w'; break;
				case VrxButtons::VB_DOWN:  key = 's'; break;
				case VrxButtons::VB_LEFT:  key = 'a'; break;
				case VrxButtons::VB_RIGHT: key = 'd'; break;
				case VrxButtons::VB_OK:    key = 13;  break;
				case VrxButtons::VB_ESC:   key = OSD::kKeyBack; break;
				case VrxButtons::VB_REC: {
					// Toggle recording directly: the OSD has no runtime record
					// action, and routing this through handle_key would just be
					// indirection. The indicator reads DvrRecorder's state.
					bool on = DvrRecorder::instance().toggle();
					printf("VRX buttons: rec -> %s\n", on ? "RECORDING" : "stopped");
					if (osd) osd->signal_render(prof::kWakeInput);
					key = 0;
					break;
				}
				default: break;
			}
			if (key) osd->handle_key(key);
		}
		usleep(25000);   // 25 ms; stock polls at 20 ms
	}
	return nullptr;
}

void* kb_thread_func(void* arg) {
    OSD* osd = (OSD*)arg;
    SchedulingHelper::set_thread_params_max_realtime("KB_THREAD", 10); // Low RT priority
    // Only treat stdin as a key source when it's an interactive terminal.
    // Under systemd (StandardInput=null) stdin is /dev/null, which polls as
    // always-readable and reads as EOF — polling it would spin this thread at
    // 100% CPU. As a service we rely solely on the evdev devices below.
    bool stdin_is_tty = isatty(STDIN_FILENO);
    if (stdin_is_tty) set_terminal_mode(1);

    InputReader input_reader;
    std::vector<unsigned char> esc_buf;
    int rescan_tick = 0;

    while (!signal_stop) {
        // Pick up input devices that appear at runtime (SSH menu bridge, USB
        // keyboard hotplug) — the startup scan alone would miss them.
        if (++rescan_tick >= 20) {   // every ~2s at the 100ms poll cadence
            rescan_tick = 0;
            input_reader.rescan();
        }

        std::vector<struct pollfd> pfds;
        int stdin_idx = -1;
        if (stdin_is_tty) {
            stdin_idx = (int)pfds.size();
            pfds.push_back({ STDIN_FILENO, POLLIN, 0 });
        }
        size_t evdev_start = pfds.size();
        for (int fd : input_reader.get_fds()) {
            pfds.push_back({ fd, POLLIN, 0 });
        }

        if (pfds.empty()) {
            // No input sources (service with no evdev keyboard attached) —
            // sleep instead of spinning on a zero-fd poll.
            usleep(100000); // 100ms, matches the poll timeout below
            continue;
        }

        int ret = poll(pfds.data(), pfds.size(), 100); // 100ms timeout
        if (ret < 0 && errno == EINTR) continue;
        if (ret <= 0) {
            if (!esc_buf.empty()) {
                for (auto c : esc_buf) osd->handle_key(c);
                esc_buf.clear();
            }
            continue;
        }

        // 1. Check STDIN (Normal terminal)
        if (stdin_idx >= 0 && (pfds[stdin_idx].revents & POLLIN)) {
            unsigned char buf[16];
            int n = read(STDIN_FILENO, buf, sizeof(buf));
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    unsigned char c = buf[i];
                    if (!esc_buf.empty() || c == 27) {
                        esc_buf.push_back(c);
                        if (esc_buf.size() >= 3) {
                            if (esc_buf[0] == 27 && esc_buf[1] == 91) {
                                int code = esc_buf[2];
                                if (code == 65) osd->handle_key(0x101); // Up
                                else if (code == 66) osd->handle_key(0x102); // Down
                                else if (code == 67) osd->handle_key(0x103); // Right
                                else if (code == 68) osd->handle_key(0x104); // Left
                                else osd->handle_key(code);
                            } else {
                                // Not an arrow sequence we recognize, flush buffer
                                for (auto b : esc_buf) osd->handle_key(b);
                            }
                            esc_buf.clear();
                        } else if (esc_buf.size() == 2 && esc_buf[1] != 91) {
                            // Sequence like Alt+Key, flush
                            for (auto b : esc_buf) osd->handle_key(b);
                            esc_buf.clear();
                        }
                    } else {
                        osd->handle_key(c);
                    }
                }
            }
        }

        // 2. Check Evdev devices (Raw USB Keyboard/HID, even if service)
        std::vector<int> dead_fds;
        for (size_t i = evdev_start; i < pfds.size(); i++) {
            if (pfds[i].revents & (POLLERR | POLLHUP)) {
                dead_fds.push_back(pfds[i].fd);   // device node went away
                continue;
            }
            if (pfds[i].revents & POLLIN) {
                struct input_event ev;
                while (read(pfds[i].fd, &ev, sizeof(ev)) > 0) {
                    if (ev.type == EV_KEY) {
                        printf("[main-evdev] Event: code=%d, value=%d\n", ev.code, ev.value);
                        fflush(stdout);
                    }
                    if (ev.type == EV_KEY && ev.value == 1) { // Key Down
                        // The VRX Pro's BIND button. It is not on the front
                        // panel's SARADC ladder (see utils/vrx_buttons.hpp) but
                        // on the separate "adc-keys" input device, SARADC
                        // channel 0 - and the device tree labels that node's
                        // keys "volume up/down/menu/back", a stock RK3568 node
                        // Caddx reused rather than renamed. This goggle has no
                        // volume control, so KEY_VOLUMEUP is the bind button
                        // and nothing else. Verified by capture: press drives
                        // channel 0 from 1023 to ~10 and emits code 115.
                        //
                        // Handled here rather than through handle_key() because
                        // binding is not a menu action - it has to work with
                        // the menu shut, which is the state anyone reaching for
                        // this button is already in.
                        if (ev.code == KEY_VOLUMEUP) {
                            printf("[main-evdev] BIND button -> requesting bind\n");
                            fflush(stdout);
                            Ar8030Source::request_bind();
                            if (osd) osd->signal_render(prof::kWakeInput);
                            continue;
                        }
                        int key = 0;
                        switch (ev.code) {
                            case KEY_UP: key = 0x101; break;
                            case KEY_DOWN: key = 0x102; break;
                            case KEY_RIGHT: key = 0x103; break;
                            case KEY_LEFT: key = 0x104; break;
                            case KEY_ENTER: key = 13; break;
                            case KEY_ESC: key = OSD::kKeyBack; break; // Esc is the Back button
                            case KEY_M: key = 'm'; break;
                            case KEY_W: key = 'w'; break;
                            case KEY_S: key = 's'; break;
                            case KEY_A: key = 'a'; break;
                            case KEY_D: key = 'd'; break;
                            case KEY_G: key = 'g'; break;
                            case KEY_TAB: key = 9; break;
                            case KEY_BACKSPACE: key = 8; break;
                        }
                        if (key != 0) {
                            printf("[main-evdev] Handling key: 0x%x\n", key);
                            fflush(stdout);
                            osd->handle_key(key);
                        }
                    }
                }
            }
        }
        for (int fd : dead_fds) input_reader.remove(fd);
    }
    if (stdin_is_tty) set_terminal_mode(0);
    return NULL;
}


void printHelp() {
  printf(
    "\n\t\tkestrel-gnd (v%s)\n"
    "\n"
    "  Usage:\n"
    "    kestrel-gnd [Arguments]\n"
    "\n"
    "  Arguments:\n"
    "    --link ar8030            - Take video from the AR8030 baseband on this board\n"
    "                               (the only supported source; also settable as 'link:'\n"
    "                               in kestrel-gnd.yaml)\n"
    "\n"
    "    --menu                   - Open the settings menu at startup. For looking at the\n"
    "                               menu from a workstation: it is otherwise reachable\n"
    "                               only by pressing a button on the goggle, which makes\n"
    "                               it the one screen that cannot be captured.\n"
    "\n"
    "    --bbwatchdog=N           - Bring-up watchdog: 0 disables it, 1 uses the default\n"
    "                               deadline, any larger N is the deadline in seconds.\n"
    "                               bb_socket_open() has no timeout, so a lost reply from\n"
    "                               the baseband daemon parks the bring-up for good; the\n"
    "                               watchdog notices and re-execs. Disable it to see the\n"
    "                               parks rather than the recovery.\n"
    "\n"
    "    --osd          		- Enable OSD\n"
    "\n"
    "\n"
    "\n"
    "    --osd-refresh  		- Defines the delay between osd refresh (Default: 1000 ms)\n"
    "    --ui-scale N            - OSD scale factor (default 1.0); also in HUD > UI Scaling\n"
    "    --stats                 - Print link/video stats to the console\n"
    "\n"
    "    (recording is on the REC button; files land in dvr_dir, default /media/dvr)\n"
    "\n"
    "    --dvr-framerate        - Force the dvr framerate fro smoother dvr\n"
    "\n"
    "    --dvr-format           - Dvr format: raw, mp4, fmp4 (default mp4)\n"
    "\n"
    "    --screen-mode   		- Override default screen mode. ex:1920x1080@120\n"
    "\n"
    "    --vsync              		- Enable V-Sync (tear-free, adds latency)\n"
    "\n"
    "    --no-display         		- Disable rendering (decode-only benchmarking)\n"
    "\n"
    "	 --decoder-name 		- Decoder to use, ex: rkmpp, hevc_cuvid, hevc_qsv, vaapi...\n"
    "    --codec h264|h265       - Video codec to decode (default h265)\n"
    "    --record [File]         - Record raw to File (same as the REC button)\n"
    "    --disable-vrr           - Force disable VRR (Variable Refresh Rate)\n"
    "    --ar8030-chan-manual    - Pin the AR8030 to --ar8030-freq instead of scanning\n"
    "    --ar8030-ap-index N     - Use bb_mac_addr_N from user_cfg.json as the AP (default 0)\n"
    "    --ar8030-pair           - Enter AR8030 pairing mode for ~30s at startup\n"
    "    --ar8030-power DBM      - AR8030 TX power in dBm (default 11, i.e. 25mW)\n"
    "    --ar8030-mode [C:]WxH@F - Retune the camera, e.g. 1920x1080@60 (default: leave as-is)\n"
    "    --ar8030-freq KHZ       - AR8030 link frequency in kHz (default 5740000)\n"
    "    --ar8030-bw N           - AR8030 bandwidth index (default 0)\n"
    "    --ar8030-slot N         - AR8030 BB slot (default 0)\n"
    "    --ar8030-port N         - AR8030 video plane port (default 3)\n"
    "    --no-osd                - Disable the GL/EGL OSD entirely (video plane only, GPU idle)\n"
    "    --no-decode             - Receive but skip feeding frames to the decoder\n"
    "    --simulation            - Simulation mode: no graphics, reports packet loss\n"
    "    --verbose               - Enable verbose logging (video + RF)\n"
    "    --verbose-vid           - Enable verbose video/decoder logging only\n"
    "    --verbose-rf            - Enable verbose RF/link stats logging only\n"
    "    --version               - Print the version and exit\n"
    "\n"
    "    Development-only flags share a --debug-* prefix and are not listed here\n"
    "    (baseband dumps, RF/mode probes, handshake and standby overrides).\n"
    "\n", KESTREL_GND_VERSION
  );
}

// A code address as library+offset, which addr2line takes; or that it is in
// no library at all.
static void crash_addr(const char* what, void* a) {
    Dl_info di;
    if (a && dladdr(a, &di) && di.dli_fname)
        fprintf(stderr, "  %s %p = %s+0x%lx (%s)\n", what, a, di.dli_fname,
                (unsigned long)((uintptr_t)a - (uintptr_t)di.dli_fbase),
                di.dli_sname ? di.dli_sname : "?");
    else
        fprintf(stderr, "  %s %p (in no library)\n", what, a);
}

// The registers come first. A call through a corrupted pointer - heap
// corruption, as the renderer's stats overflow once caused - leaves the PC in
// no library, and the backtrace stops right there with nothing to go on; the
// link register still says who made the call, and the thread's name whose
// thread it was.
static void segv_bt(int sig, siginfo_t* si, void* ucv){
    char name[17] = {0};
    prctl(PR_GET_NAME, name);
    fprintf(stderr, "\n### %s in thread %s, fault address %p ###\n",
            sig == SIGSEGV ? "SIGSEGV" : sig == SIGBUS ? "SIGBUS" : "SIGABRT", name, si->si_addr);
#if defined(__aarch64__)
    const mcontext_t& mc = ((ucontext_t*)ucv)->uc_mcontext;
    crash_addr("pc", (void*)mc.pc);
    crash_addr("lr", (void*)mc.regs[30]);
    fprintf(stderr, "  sp %p\n", (void*)mc.sp);
#else
    (void)ucv;
#endif
    void* bt[48]; int n=backtrace(bt,48);
    fprintf(stderr,"### backtrace (%d frames) ###\n", n);
    backtrace_symbols_fd(bt,n,2);
    signal(sig,SIG_DFL); raise(sig);
}

int main(int argc, char **argv)
{
	g_main_argv = argv;   // for a restart (kestrel_request_restart)
	// Line-buffer stdout. It is a terminal only when someone is watching by
	// hand; in service it is redirected to a file, where glibc switches to full
	// buffering and the log then trails reality by up to a buffer. That is
	// merely annoying until you are reading it to find out where a hang is, at
	// which point the last line printed is not where the process stopped and
	// the log actively misleads.
	setvbuf(stdout, nullptr, _IOLBF, 0);
	ltrace::start();

	int ret;	
	int i, j;

	// Absolute, deliberately: a relative path resolves against the working
	// directory, so settings silently followed wherever the app was started from
	// (an init script or a systemd unit with no WorkingDirectory would start from
	// defaults and persist somewhere else). Override with KESTREL_CONFIG for
	// testing without touching the installed file.
	{
		const char *cfg = getenv("KESTREL_CONFIG");
		Settings::getInstance().load(cfg && *cfg ? cfg : "/etc/kestrel/kestrel-gnd.yaml");
		// SYSTEM > Time Zone: its TZ, before any thread reads the time.
		zones::load();
		// HUD > Debug Info on: the log on the recordings partition too, named
		// as the recordings are (fpvOS_YYYYMMDD_HHMMSS.log), so it survives a
		// power-off; and the air unit's flight logs (sent at link-up).
		sd_log::init(Settings::getInstance().getString("dvr_dir", "/media/dvr"));
		Ar8030Source::air_logs = Settings::getInstance().getBool("hud_debug", false);
		sd_log::set_enabled(Ar8030Source::air_logs);
		// The low-latency video features, each with a switch that survives a
		// reboot (the KESTREL_* variables and the /tmp files still work too):
		//   stream_decode: 0   pictures go to the decoder whole
		//   early_present: 0   no picture goes up before it is all decoded
		//   video_halves: 0    no picture in halves on two planes
		if (Settings::getInstance().getInt("stream_decode", 1) == 0) {
			setenv("KESTREL_STREAM_DECODE", "0", 1);
			Ar8030Source::stream_decode = false;
		}
		if (Settings::getInstance().getInt("early_present", 1) == 0) setenv("KESTREL_EARLY_PRESENT", "0", 1);
		if (Settings::getInstance().getInt("video_halves", 1) == 0) setenv("KESTREL_SPLIT", "0", 1);
	}

	uint16_t mode_width = 0;
	uint16_t mode_height = 0;
	uint32_t mode_vrefresh = 0;
	// The display mode belongs to the screen: screen_mode_<ID>, by the ID in
	// its EDID (utils/screen_id.h), and unset means Auto - the highest refresh
	// it offers. screen_mode is for a screen with no ID, and --screen-mode.
	// A mode picked in SYSTEM > Screen Mode is only on trial the first time:
	// screen_mode_try names the screen and the mode, and is read and removed
	// here, so a mode that leaves the screen black is gone at the next start
	// even if nobody could see the prompt to reject it (see OSD, confirm).
	char screen_id_buf[16] = "";
	const bool have_screen_id = screen_id(screen_id_buf, sizeof(screen_id_buf));
	const std::string screen_key = have_screen_id ? std::string("screen_mode_") + screen_id_buf
	                                              : std::string("screen_mode");
	std::string screen_mode_str = Settings::getInstance().getString(screen_key, "");
	// DISPLAY > Refresh Sync is the screen's too, and off unless tried and
	// kept on it: some screens blank on the one-line timing changes it makes.
	// A screen with no EDID ID cannot be told from another, so it starts off
	// every time (the menu still turns it on for the run).
	{
		const int v = have_screen_id
			? Settings::getInstance().getInt(std::string("display_lock_") + screen_id_buf, 0) : 0;
		g_display_lock = v >= 0 && v <= 2 ? v : 0;   // OFF, ON, FREE
		// SPIKE: camera steering (3) on every screen, whatever is saved.
		g_display_lock = 3;
		printf("display lock: SPIKE - Refresh Sync CAMERA forced on\n");
	}
	g_display_lock_slice = Settings::getInstance().getInt("display_lock_slice", 1) == 2 ? 2 : 1;
	{
		const int ms = Settings::getInstance().getInt("display_lock_margin_ms", 0);
		g_display_lock_margin_us = (ms >= 0 && ms <= 4 ? ms : 0) * 1000;
	}
	std::string screen_mode_trial;       // on trial this run, "" for none
	{
		const std::string t = Settings::getInstance().getString("screen_mode_try", "");
		if (!t.empty()) {
			Settings::getInstance().remove("screen_mode_try");
			const size_t sp = t.find(' ');
			const std::string id = sp == std::string::npos ? "" : t.substr(0, sp);
			const std::string mode = sp == std::string::npos ? "" : t.substr(sp + 1);
			if (!mode.empty() && id == (have_screen_id ? screen_id_buf : "-")) {
				screen_mode_trial = mode;
				screen_mode_str = (mode == "auto") ? "" : mode;
			}
		}
	}
	printf("display: screen %s, mode %s%s\n", have_screen_id ? screen_id_buf : "(no EDID ID)",
	       screen_mode_str.empty() ? "auto" : screen_mode_str.c_str(),
	       screen_mode_trial.empty() ? "" : " (on trial)");
	if (!screen_mode_str.empty()) {
		// Parsing: 1920x1080@60
		try {
			size_t x_pos = screen_mode_str.find('x');
			size_t at_pos = screen_mode_str.find('@');
			if (x_pos != std::string::npos && at_pos != std::string::npos) {
				mode_width = std::stoi(screen_mode_str.substr(0, x_pos));
				mode_height = std::stoi(screen_mode_str.substr(x_pos + 1, at_pos - x_pos - 1));
				mode_vrefresh = std::stoi(screen_mode_str.substr(at_pos + 1));
			}
		} catch(...) {}
	}
	
	int video_zpos = Settings::getInstance().getInt("video_zpos", 1);
	int enable_osd = Settings::getInstance().getInt("enable_osd", 1);
	int osd_zpos = Settings::getInstance().getInt("osd_zpos", 2);
	int osd_refresh_frequency_ms = Settings::getInstance().getInt("osd_refresh", 1000);
	bool console_stats = Settings::getInstance().getBool("stats", false);
	bool no_decode_flag = false;
	bool simulation_mode = false;

	// Default: FrontBuffer (lowest latency)
	RenderMode render_mode = FrontBuffer;
	bool vsync_flag = Settings::getInstance().getBool("vsync", false);
	bool no_display_flag = Settings::getInstance().getBool("no_display", false);
	
	if (no_display_flag) render_mode = Disable;
	else if (vsync_flag) render_mode = Atomic;
	else render_mode = FrontBuffer;

	std::string dvr_filename = Settings::getInstance().getString("dvr", "");
	int dvr_framerate = Settings::getInstance().getInt("dvr_framerate", 0);
	bool dvr_screen = Settings::getInstance().getBool("dvr_screen", kDvrScreenDefault);
	
	// Fragmented MP4 by default: every frame goes out as a self-contained
	// fragment behind a header written up front, so a recording cut short -
	// the battery pulled with REC on, which is how most of them end - still
	// plays up to its last second. A plain MP4 writes its index only when it
	// is closed, and without one no player opens the file. "mp4" is still
	// available for tools that insist on it.
	DvrFormat dvr_format = DvrFormat::FMP4;
	std::string dvr_format_str = Settings::getInstance().getString("dvr_format", "fmp4");
	if (dvr_format_str == "mp4") dvr_format = DvrFormat::MP4;
	else if (dvr_format_str == "raw") dvr_format = DvrFormat::RAW;
	
	// The channel is the air unit's to decide: the goggle always starts by
	// searching for it (chan_auto), and a channel picked in the menu lasts
	// for the session. Only --ar8030-chan-manual starts it pinned.
	// Decode was flag-gated because it was being isolated while chasing the
	// decode freeze; for normal use it belongs on. Settings-backed so a bare
	// "kestrel-gnd" works on hardware where the config says ar8030: 1.
	Ar8030Source::decode_enabled = Settings::getInstance().getInt("ar8030_decode", 1) != 0;
	int ar8030_slot = Settings::getInstance().getInt("ar8030_slot", 0);
	int ar8030_port = Settings::getInstance().getInt("ar8030_port", 3);
	// mW, in stock's encoding (N = hold, N+1 = auto capped at N). See
	// kArPwrLevels in common.hpp for the levels this board accepts.
	Ar8030Source::tx_power_mw = Settings::getInstance().getInt("tx_power_mw", kArPwrDefaultMw);
	Ar8030Source::tx_power_want_mw = Ar8030Source::tx_power_mw;   // the choice, until an air unit's own
	{
		int fm = Settings::getInstance().getInt("frame_mode", kFrameWhole);
		g_frame_mode = (fm >= 0 && fm < kFrameModeCount) ? fm : kFrameWhole;
		printf("frame mode: %s\n", frame_mode_label(g_frame_mode.load()));
	}
	Ar8030Source::tx_power_dbm = kArPwrLevels[ar_pwr_index(Ar8030Source::tx_power_mw)].dbm;
	// The fastest capture -> arrival the air delay is pinned to (see
	// air_delay_for). Measure it for a setup with both clocks synced.
	Ar8030Source::air_floor_us =
		(int)(Settings::getInstance().getFloat("air_floor_ms", 20.5f) * 1000.0f);
	// 0 = off. See drain_msp_socket(): an unserved port wedges bring-up.
	Ar8030Source::msp_bb_port = Settings::getInstance().getInt("msp_bb_port", 2);
	// Standby is dictated by the air unit, not the ground: fpv_sky_standby_mode_thread
	// auto-parks the air to low power when it is disarmed/idle and forces itself out
	// of standby while flying. So the ground does not command it - standby_mode stays
	// -1 (leave alone), connect_bb() skips CAM_STANDBY, and the menu just reflects the
	// air's reported state (osd air_standby). --debug-standby still overrides for
	// experiments.
	// bb_bandwidth_e gear: 0=1.25M 1=2.5M 2=5M 3=10M 4=20M 5=40M.
	Ar8030Source::bandwidth = Settings::getInstance().getInt("ar8030_bw", -1);
	// Stock's values; -1 on either disables the PRJ_DISPATCH call entirely.
	// Both default to -1 (off): see send_prj_rf_config(). Enabling the ground
	// side alone measurably hurts; air_bw is the other half and its units are
	// not yet verified.
	Ar8030Source::prj_rf_bw     = Settings::getInstance().getInt("prj_rf_bw", -1);
	Ar8030Source::prj_rf_pwr_mw = Settings::getInstance().getInt("prj_rf_pwr_mw", -1);
	Ar8030Source::air_bw        = Settings::getInstance().getInt("air_bw", -1);
	// The cap on kestrel-air's video bitrate (RF menu, Max Bitrate), kbps, 0 = none.
	Ar8030Source::max_kbps      = Settings::getInstance().getInt("ar8030_max_kbps", 0);
	// The cap on the video link's bandwidth (RF menu, Max Bandwidth): 20 or 40 MHz,
	// with Open8030 here also AUTO (0: nothing sent), 5 or 10.
	{
		const int bw = Settings::getInstance().getInt("ar8030_max_bw_mhz", 40);
		Ar8030Source::max_bw_mhz = bw == 0 || bw == 5 || bw == 10 || bw == 20 ? bw : 40;
		const int mcs = Settings::getInstance().getInt("ar8030_max_mcs", -1);
		Ar8030Source::max_mcs = mcs >= 0 && mcs <= 11 ? mcs : -1;   // the menu's range, AUTO + MCS 0..11
		Ar8030Source::steady = Settings::getInstance().getInt("ar8030_steady", 0) == 1 ? 1 : 0;
	}
	// RADIO > VRX Radio Firmware: Open8030 when its image is on the unit, unless
	// the menu said STOCK (0) - as bb_only.sh decides what it uploads.
	Ar8030Source::radio_fw      = access("/lib/firmware/open8030-vrxpro-usb.img", R_OK) == 0 &&
	                              Settings::getInstance().getInt("ar8030_firmware", 1) != 0 ? 1 : 0;
	// The radio's TDD frame mode (RADIO > Radio Frame): 2 short, 0 standard.
	Ar8030Source::frame_mode    = Settings::getInstance().getInt("ar8030_frame_mode", 2) == 0 ? 0 : 2;
	Ar8030Source::replay_stock_rf = Settings::getInstance().getInt("replay_stock_rf", 1);
	std::string decoder_name = Settings::getInstance().getString("decoder_name", "rkmpp");
	// VRR (FreeSync over HDMI): opt-in on Rockchip, where it needs fpvOS's
	// kernel support and a sink that declares FreeSync (or is forced to with
	// rockchipdrm.freesync_force). Used only when the connector is vrr_capable.
	#if defined(__aarch64__) || defined(__arm__)
	bool enable_vrr = Settings::getInstance().getBool("enable_vrr", false);
	#else
	bool enable_vrr = Settings::getInstance().getBool("enable_vrr", true);
	#endif
	printf("VRR %s by the settings\n", enable_vrr ? "allowed" : "off");
	
	float ui_scale = Settings::getInstance().getFloat("ui_scale", 1.0f);




	VideoCodec codec = VideoCodec::H265;

	// Load console arguments
	__BeginParseConsoleArguments__(printHelp) 

	__OnArgument("--codec") {
		char * codec_str = const_cast<char*>(__ArgValue);
		codec = video_codec(codec_str);
		if (codec == VideoCodec::UNKNOWN ) {
			printf("unsupported video codec");
			return -1;
		}
		Settings::getInstance().set("codec", codec_str);
		continue;
	}

	__OnArgument("--dvr-framerate") {
		dvr_framerate = atoi(__ArgValue);
		Settings::getInstance().set("dvr_framerate", dvr_framerate);
		continue;
	}

	__OnArgument("--dvr-format") {
		char* fmt = const_cast<char*>(__ArgValue);
		if (!strcmp(fmt, "fmp4")) {
         dvr_format = DvrFormat::FMP4;
    	} else if (!strcmp(fmt, "raw")) {
         dvr_format = DvrFormat::RAW;
    	}
    	Settings::getInstance().set("dvr_format", fmt);
		continue;
	}

	__OnArgument("--osd") {
		enable_osd = 1;
		osd_zpos = 2;
		Settings::getInstance().set("enable_osd", 1);
		Settings::getInstance().set("osd_zpos", 2);
		continue;
	}

	__OnArgument("--decoder-name") {
		char* name = const_cast<char*>(__ArgValue);
		decoder_name = std::string(name);
		Settings::getInstance().set("decoder_name", decoder_name);
		continue;
	}

	__OnArgument("--osd-refresh") {
		osd_refresh_frequency_ms = atoi(__ArgValue);
		Settings::getInstance().set("osd_refresh", osd_refresh_frequency_ms);
		continue;
	}
	
	__OnArgument("--screen-mode") {
		char* mode = const_cast<char*>(__ArgValue);
		std::string mode_copy = mode;
		char* pt = strtok(mode, "x");
		if (pt) mode_width = atoi(pt);
		
		pt = strtok(NULL, "@");
		if (pt) mode_height = atoi(pt);
		
		pt = strtok(NULL, "@");
		if (pt) mode_vrefresh = atoi(pt);

		Settings::getInstance().set("screen_mode", mode_copy);
		continue;
	}

	__OnArgument("--vsync") {
		vsync_flag = true;
		render_mode = Atomic;
		Settings::getInstance().set("vsync", true);
		continue;
	}
	
	__OnArgument("--no-display") {
		no_display_flag = true;
		render_mode = Disable;
		Settings::getInstance().set("no_display", true);
		continue;
	}



	if (!strncmp(Arg, "--menu=", 7) || !strcmp(Arg, "--menu")) {
		osd_set_menu_at_start(true);
		if (Arg[6] == '=') osd_set_menu_start_tab(atoi(Arg + 7));
		continue;
	}

	// --bbwatchdog=N - see bb_watchdog.cpp. 0 off, 1 default deadline, N>1 the
	// deadline in seconds. Kept as a flag rather than only an environment
	// variable because the thing it guards against happens at bring-up, which
	// is exactly when you are launching the binary by hand to watch it.
	if (!strncmp(Arg, "--bbwatchdog=", 13) || !strcmp(Arg, "--bbwatchdog")) {
		std::string spec = (Arg[12] == '=') ? std::string(Arg + 13) : std::string(__ArgValue);
		int n = atoi(spec.c_str());
		if (n < 0) {
			printf("--bbwatchdog: expected 0 (off), 1 (default), or seconds; got '%s'\n",
			       spec.c_str());
			return 1;
		}
		bb_watchdog_set_deadline(n);
		continue;
	}

	// --link selects the video source. Only "ar8030" exists; the flag is kept
	// so existing command lines and init scripts keep working. Both
	// "--link x" and "--link=x" parse.
	if (!strncmp(Arg, "--link=", 7) || !strcmp(Arg, "--link")) {
		std::string spec = (Arg[6] == '=') ? std::string(Arg + 7) : std::string(__ArgValue);
		if (spec != "ar8030") {
			printf("--link: unknown source '%s' (only 'ar8030' is supported)\n", spec.c_str());
			return 1;
		}
		Settings::getInstance().set("link", spec);
		continue;
	}





	__OnArgument("--stats") {
		console_stats = true;
		Settings::getInstance().set("stats", true);
		continue;
	}
	
	__OnArgument("--disable-vrr") {
		enable_vrr = false;
		Settings::getInstance().set("enable_vrr", false);
		continue;
	}

	__OnArgument("--ui-scale") {
		ui_scale = atof(__ArgValue);
		Settings::getInstance().set("ui_scale", ui_scale);
		continue;
	}

	__OnArgument("--ar8030-chan-manual") {
		Ar8030Source::chan_auto = false;
		Ar8030Source::chan_manual_cli = true;
		continue;
	}

	__OnArgument("--ar8030-ap-index") {
		Ar8030Source::ap_index = std::stoi(__ArgValue);
		continue;
	}

	__OnArgument("--ar8030-pair") {
		Ar8030Source::do_pair = true;
		continue;
	}

	__OnArgument("--debug-no-handshake") {
		Ar8030Source::skip_handshake = true;
		continue;
	}

	__OnArgument("--ar8030-mode") {
		// WxH@FPS, optionally prefixed with the channel: "1:1920x1080@60".
		// Channel 0 is the FPV stream we decode, 1 the air unit's recording.
		std::string v = __ArgValue;
		size_t colon = v.find(':');
		if (colon != std::string::npos) {
			Ar8030Source::mode_chn = std::stoi(v.substr(0, colon));
			v = v.substr(colon + 1);
		}
		size_t x = v.find('x'), at = v.find('@');
		if (x == std::string::npos || at == std::string::npos || at < x) {
			printf("--ar8030-mode: expected [CHN:]WxH@FPS, e.g. 1920x1080@60\n");
			return 1;
		}
		Ar8030Source::mode_w   = std::stoi(v.substr(0, x));
		Ar8030Source::mode_h   = std::stoi(v.substr(x + 1, at - x - 1));
		Ar8030Source::mode_fps = std::stoi(v.substr(at + 1));
		continue;
	}

	__OnArgument("--debug-probe-rf") {
		Ar8030Source::probe_rf = true;
		continue;
	}

	__OnArgument("--debug-probe-modes") {
		Ar8030Source::probe_modes = true;
		continue;
	}

	__OnArgument("--ar8030-power") {
		Ar8030Source::tx_power_dbm = std::stoi(__ArgValue);
		continue;
	}

	__OnArgument("--ar8030-freq") {
		Ar8030Source::freq_khz = (unsigned)std::stoul(__ArgValue);
		continue;
	}

	__OnArgument("--ar8030-bw") {
		Ar8030Source::bandwidth = std::stoi(__ArgValue);
		continue;
	}

	__OnArgument("--ar8030-slot") {
		ar8030_slot = std::stoi(__ArgValue);
		continue;
	}

	__OnArgument("--ar8030-port") {
		ar8030_port = std::stoi(__ArgValue);
		continue;
	}

	__OnArgument("--debug-msp-bb-port") {
		Ar8030Source::msp_bb_port = std::stoi(__ArgValue);
		continue;
	}

	__OnArgument("--debug-standby") {
		Ar8030Source::standby_mode = std::stoi(__ArgValue);
		continue;
	}

	__OnArgument("--debug-bb-dump") {
		Ar8030Source::dump_path = __ArgValue;
		continue;
	}

	__OnArgument("--debug-replay") {
		Ar8030Source::replay_path = __ArgValue;
		continue;
	}

	__OnArgument("--debug-video-dump") {
		Ar8030Source::video_dump_path = __ArgValue;
		continue;
	}

	__OnArgument("--no-osd") {
		enable_osd = 0;
		g_disable_gl = true;   // skip EGL/GLES entirely - GPU stays idle
		Settings::getInstance().set("enable_osd", 0);
		continue;
	}

	__OnArgument("--no-decode") {
		no_decode_flag = true;
		continue;
	}

	__OnArgument("--record") {
		dvr_filename = const_cast<char*>(__ArgValue);
		dvr_format = DvrFormat::RAW;
		continue;
	}

	__OnArgument("--verbose") {
		verbose_vid = true;
		verbose_rf  = true;
		continue;
	}

	__OnArgument("--verbose-vid") {
		verbose_vid = true;
		continue;
	}

	__OnArgument("--verbose-rf") {
		verbose_rf = true;
		continue;
	}

	__OnArgument("--simulation") {
		simulation_mode = true;
		console_stats = true;
		enable_osd = 0;
		no_decode_flag = true;
		render_mode = Disable;
		continue;
	}

	__OnArgument("--version") {
		printf("kestrel-gnd v%s\n", KESTREL_GND_VERSION);
		return 0;
	}

	__EndParseConsoleArguments__;

	if (!dvr_filename.empty() && dvr_format != DvrFormat::RAW && dvr_framerate < 0 ) {
		printf("--dvr-framerate must be provided when (f)mp4 dvr is enabled.\n");
		return 0;
	}

	printf("kestrel-gnd v%s (built %s, git:%s)\n", KESTREL_GND_VERSION, KESTREL_GND_BUILD_TIME, KESTREL_GND_GIT_HASH);


	if (enable_osd == 0 ) {
		video_zpos = 4;
	}

	// Start decoding the heavy idle-screen assets (background.png + title) on a
	// worker thread now, so it overlaps the DRM/EGL driver bring-up below
	// instead of running serially on the OSD thread afterwards. The OSD is
	// created whenever we're not in simulation mode, so mirror that condition.
	if (!simulation_mode) {
		osd_prefetch_assets_async();
	}

	////////////////////////////////// SIGNAL SETUP

	signal(SIGINT, sig_handler);

	// The DVR toggle the comment above promised. Without this, SIGUSR1 keeps

	// its default disposition and terminates the process.

	signal(SIGUSR1, rec_toggle_handler);
	signal(SIGTERM, sig_handler);   // systemctl stop / killall, same watchdog
	{
		struct sigaction sa = {};
		sa.sa_sigaction = segv_bt;
		sa.sa_flags = SA_SIGINFO;
		sigaction(SIGSEGV, &sa, nullptr);
		sigaction(SIGBUS, &sa, nullptr);
		sigaction(SIGABRT, &sa, nullptr);
	}
	// Ignore SIGPIPE — a broken socket (e.g. the RPC link when the gnd radio
	// reboots, as a channel pin does) must surface as an EPIPE the send/recv
	// paths handle + reconnect from, not tear the whole app down.
	signal(SIGPIPE, SIG_IGN);

	//////////////////////////////////  DRM SETUP
	std::shared_ptr<DrmDevice> dev;
	if (!simulation_mode) {
		dev = std::make_shared<DrmDevice>();
		dev->display_id = screen_id_buf;   // a different screen plugged in restarts us
		// Auto: the refresh follows the camera, from the rate last seen.
		dev->follow_camera = !(mode_width > 0 && mode_height > 0 && mode_vrefresh > 0);
		const int last_fps = Settings::getInstance().getInt("camera_fps", 0);
		dev->follow_fps = last_fps > 0 ? (uint32_t)last_fps : 0;
		const int last_lines = Settings::getInstance().getInt("camera_lines", 0);
		dev->follow_lines = last_lines > 0 ? (uint32_t)last_lines : 0;
		dev->init(mode_width, mode_height, mode_vrefresh, video_zpos, osd_zpos, enable_vrr);

	}

	////////////////////////////////// Pipeline setup
	std::shared_ptr<DVR> dvr;
	if (!dvr_filename.empty()) {
		int dvr_fps = dvr_framerate > 0 ? dvr_framerate : 30;
		dvr = std::make_shared<DVR>(dvr_filename, codec, dvr_fps, dvr_format, &signal_stop, dvr_screen);
	}

	std::shared_ptr<OSD> osd;
	std::shared_ptr<Renderer> renderer;
	std::shared_ptr<Vdec> vdec;

	if (!simulation_mode) {
		// With --no-osd there is no EGL display/context (init_gl() was skipped),
		// so constructing the OSD would segfault as soon as its thread calls
		// eglMakeCurrent(). The video plane is driven by the Renderer via DRM
		// and does not need GL.
		if (!g_disable_gl) {
			osd = std::make_shared<OSD>(dev, osd_refresh_frequency_ms, &signal_stop, console_stats);
			osd->set_ui_scale(ui_scale);
			osd->set_decoder_name(decoder_name);
			osd->set_screen(screen_id_buf, screen_key);
			if (!screen_mode_trial.empty())
				osd->begin_screen_mode_confirm(screen_mode_trial);
			if (dvr_screen && dvr) {
				osd->set_dvr(dvr, true);
			}
		}
		
		renderer = std::make_shared<Renderer>(render_mode, dev, dvr, osd, &signal_stop, console_stats);
		dev->flips_are_frames = (render_mode == Atomic);   // the stats screen reads flip timing as smoothness
		vdec = new_vdec(codec, decoder_name, renderer, dev, &signal_stop);
	}

    // 2. Start Video/UI threads IMMEDIATELY
    pthread_t tid_frame = 0;
    pthread_t tid_rend = 0;
    pthread_t tid_osd = 0;
    pthread_t tid_kb = 0;
    pthread_t tid_btn = 0;
    if (vdec) {
        ret = pthread_create(&tid_frame, NULL, Vdec::run_frame_thread, vdec.get());
        assert(!ret);
    }
	if (renderer) {
		ret = pthread_create(&tid_rend, NULL, Renderer::run_thread, renderer.get());
		assert(!ret);
	}
	if (osd) {
		ret = pthread_create(&tid_osd, NULL, OSD::run_thread, osd.get());
		assert(!ret);
		ret = pthread_create(&tid_kb, NULL, kb_thread_func, osd.get());
		assert(!ret);
		ret = pthread_create(&tid_btn, NULL, vrx_button_thread_func, osd.get());
		assert(!ret);
	}
    

    // Recording is button-driven now, so give the recorder everything it needs
    // to build a DVR on demand. dvr_dir defaults to the recordings partition
    // created on first boot (S35dvrpart).
    DvrRecorder::instance().set_display(dev.get());
    DvrRecorder::instance().set_screen_mode(
        Settings::getInstance().getBool("dvr_screen", kDvrScreenDefault));
    DvrRecorder::instance().configure(
        codec, dvr_framerate > 0 ? dvr_framerate : 60, dvr_format, &signal_stop,
        Settings::getInstance().getString("dvr_dir", "/media/dvr"));

    // Live video in a browser: http://<goggle>/ on the USB link or WiFi.
    if (Settings::getInstance().getBool("web_stream", true))
        WebStream::instance().start(Settings::getInstance().getInt("web_port", 80));


    if (osd) {
        // Show the camera's own modes in the VIDEO tab instead of the
        // ArtLynk air unit's, and start on whatever --ar8030-mode asked for
        // (falling back to the mode stock boots into).
        std::vector<std::string> names;
        std::vector<uint16_t> needs;
        int cur = sky::default_fpv_mode_index();
        for (int i = 0; i < sky::kFpvModeCount; i++) {
            names.push_back(sky::kFpvModes[i].name);
            needs.push_back(sky::kFpvModes[i].need);
            if (Ar8030Source::mode_w == sky::kFpvModes[i].w &&
                Ar8030Source::mode_h == sky::kFpvModes[i].h &&
                Ar8030Source::mode_fps == sky::kFpvModes[i].fps)
                cur = i;
        }
        osd->set_video_mode_names(names, cur, needs);
    }

    if (osd) {
        osd->set_command_callback([&](int cmd, int val) {
            switch(cmd) {
                case 0x200: // UI-local: channel-scan screen open/close → fast/slow chan_info polling
                    Ar8030Source::scan_active = (val != 0);
                    break;
                case 0x311: case 0x312: case 0x313: case 0x314: case 0x315:
                case 0x316:
                    // AR8030 RF controls from the RF LINK tab (cmd - 0x310).
                    Ar8030Source::request_rf(cmd - 0x310, val);
                    return;
                case 0x201: // UI-local: HUD > Debug Info - the logs on the card and the air unit's
                    sd_log::set_enabled(val != 0);
                    Ar8030Source::air_logs = val != 0;
                    Ar8030Source::request_setting(Ar8030Source::CAM_AIR_LOGS, val != 0);
                    return;
                case 0x30E:
                    // RF menu, Max Bitrate: kept here and sent at every link-up.
                    Ar8030Source::max_kbps = val;
                    Settings::getInstance().set("ar8030_max_kbps", val);
                    Ar8030Source::request_setting(Ar8030Source::CAM_MAX_KBPS, val);
                    return;
                case 0x30F: {
                    // RF menu, Max Bandwidth: kept here (and sent to a kestrel-air
                    // that takes it, at every link-up). Open8030 here: our radio
                    // takes it live (SET 22), 5 to 40 MHz. Stock: the goggle's
                    // radio config caps the width (bb_only.sh picks it from this
                    // setting), loaded with the radio: a change restarts the
                    // radio, and kestrel-gnd with it, as the driver cannot go
                    // while kestrel-gnd holds it - from a shell of its own.
                    if (Ar8030Source::radio_fw == 1) {
                        Ar8030Source::max_bw_mhz = val == 0 || val == 5 || val == 10 || val == 20 ? val : 40;
                        Settings::getInstance().set("ar8030_max_bw_mhz", Ar8030Source::max_bw_mhz);
                        Ar8030Source::request_setting(Ar8030Source::CAM_MAX_BW,
                            Ar8030Source::max_bw_mhz == 0 || Ar8030Source::max_bw_mhz >= 40 ? 40 : 20);
                        Ar8030Source::caps_live.store(true);
                        Ar8030Source::caps_due.store(true);
                        return;
                    }
                    const int was = Ar8030Source::max_bw_mhz;
                    Ar8030Source::max_bw_mhz = (val == 20) ? 20 : 40;
                    Settings::getInstance().set("ar8030_max_bw_mhz", Ar8030Source::max_bw_mhz);
                    Ar8030Source::request_setting(Ar8030Source::CAM_MAX_BW, Ar8030Source::max_bw_mhz);
                    if (Ar8030Source::max_bw_mhz != was) {
                        printf("menu: max bandwidth %d MHz - restarting the radio with it\n", Ar8030Source::max_bw_mhz);
                        fflush(stdout);
                        Settings::getInstance().flush();   // bb_only.sh reads it
                        if (system("setsid sh -c '/etc/init.d/S70kestrel stop; /etc/init.d/S60ar8030 restart; "
                                   "sleep 2; /etc/init.d/S70kestrel start' </dev/null >/dev/null 2>&1 &") != 0)
                            printf("menu: could not restart the radio\n");
                    }
                    return;
                }
                case 0x309: {
                    // RADIO > VRX Radio Firmware: which baseband firmware bb_only.sh
                    // uploads with the radio (0 stock, 1 Open8030, when its image
                    // is on the unit). Loaded with the radio, so a change restarts
                    // the radio and kestrel-gnd, as Max Bandwidth does.
                    const int was = Ar8030Source::radio_fw;
                    Ar8030Source::radio_fw = val == 1 ? 1 : 0;
                    Settings::getInstance().set("ar8030_firmware", Ar8030Source::radio_fw);
                    if (Ar8030Source::radio_fw != was) {
                        printf("menu: radio firmware %s - restarting the radio with it\n",
                               Ar8030Source::radio_fw ? "Open8030" : "stock");
                        fflush(stdout);
                        Settings::getInstance().flush();   // bb_only.sh reads it
                        if (system("setsid sh -c '/etc/init.d/S70kestrel stop; /etc/init.d/S60ar8030 restart; "
                                   "sleep 2; /etc/init.d/S70kestrel start' </dev/null >/dev/null 2>&1 &") != 0)
                            printf("menu: could not restart the radio\n");
                    }
                    return;
                }
                case 0x320:
                    // RADIO > Max MCS (Open8030 here): the highest MCS our radio's
                    // receive policy may use, -1 none. Live; again at every link-up.
                    Ar8030Source::max_mcs = val >= 0 && val <= 13 ? val : -1;
                    Settings::getInstance().set("ar8030_max_mcs", Ar8030Source::max_mcs);
                    Ar8030Source::caps_live.store(true);
                    Ar8030Source::caps_due.store(true);
                    return;
                case 0x321:
                    // RADIO > Steady Latency (Open8030 here): a block the air radio
                    // sent again steps our radio's MCS down. Live; again at link-up.
                    Ar8030Source::steady = val == 1 ? 1 : 0;
                    Settings::getInstance().set("ar8030_steady", Ar8030Source::steady);
                    Ar8030Source::caps_live.store(true);
                    Ar8030Source::caps_due.store(true);
                    return;
                case 0x30A:
                    // RF menu, Radio Frame: kept here, sent now and after every link-up.
                    Ar8030Source::frame_mode = (val == 0) ? 0 : 2;
                    Settings::getInstance().set("ar8030_frame_mode", Ar8030Source::frame_mode);
                    Ar8030Source::frame_mode_changed.store(true);
                    return;
                case 0x301: case 0x302: case 0x303: case 0x304:
                case 0x305: case 0x306: case 0x307: case 0x308:
                case 0x30B: case 0x30C: case 0x30D:
                    // AR8030 camera settings from the VIDEO tab. The field ids
                    // line up with Ar8030Source::CamField (cmd - 0x300).
                    Ar8030Source::request_setting(cmd - 0x300, val);
                    return;
                case 0x01:
                    // Video mode. With the AR8030 the camera is reached over the
                    // baseband's port-2 control socket (SET_CHN_RES), not the
                    // kestrel-air UDP link, and it has its own list of modes.
                    Ar8030Source::request_mode_index(val);
                    return;
                default:
                    printf("Unknown command 0x%x = %d\n", cmd, val);
                    return;
            }
        });

    }
    pthread_t tid_dvr;
	if (dvr) {
		if (dvr_screen && dev && dev->output_list) {
			dvr->init(dev->output_list->mode.hdisplay, dev->output_list->mode.vdisplay);
		}
		ret = pthread_create(&tid_dvr, NULL, DVR::run_dvr_thread, dvr.get());
		assert(!ret);
	}

	////////////////////////////////////////////// MAIN LOOP
	// The AR8030 baseband is the only supported link.
	//
	// Armed before the link is brought up, because the call that wedges is
	// inside that bring-up and cannot be interrupted once it has parked.
	// Settings supply the deadline unless --bbwatchdog said otherwise on the
	// command line, so the choice survives a reboot without a rebuild:
	//   bb_watchdog: 0   off
	//   bb_watchdog: 1   built-in default
	//   bb_watchdog: N   deadline in seconds
	// It is off at the moment. The ioctl timeouts (see ar_ioctl in
	// ar8030_source.cpp) removed the parks this was containing, so it is
	// carrying no weight that anyone has measured - and every version of it so
	// far has cost more than it saved. Turn it back on if the symptom returns.
	if (!bb_watchdog_deadline_set())
		bb_watchdog_set_deadline(Settings::getInstance().getInt("bb_watchdog", 0));
	bb_watchdog_start(argv);
	{
		auto ar_src = std::make_shared<Ar8030Source>(codec, vdec, dvr_screen ? nullptr : dvr,
		                                             osd, &signal_stop, "127.0.0.1", 50000,
		                                             ar8030_slot, ar8030_port);
		ar_src->run();
	}

	////////////////////////////////////////////// MPI CLEANUP
	bb_watchdog_stop();   // a deliberate exit is not a wedge
	printf("Shutting down threads...\n");
	DvrRecorder::instance().shutdown();
	WebStream::instance().stop();
	if (renderer) {
		ret = pthread_join(tid_rend, NULL);
		assert(!ret);
		printf("Renderer thread joined.\n");
		// The display lock's thread reads the DRM device every vblank; it has
		// to be gone before dev->cleanup() frees that (the renderer object
		// itself lives to the end of main).
		renderer->stop_display_lock();
	}
	
	if (dev) dev->cond_signal();
	
	if (osd) {
		//osd->stop();
		ret = pthread_join(tid_osd, NULL);
		assert(!ret);
		printf("OSD thread joined.\n");
	}
	if (dvr){
		dvr->stop();
		ret = pthread_join(tid_dvr, NULL);
		assert(!ret);
	}

	if (osd) {
		if (tid_btn) pthread_join(tid_btn, NULL);
		ret = pthread_join(tid_kb, NULL);
		assert(!ret);
		printf("Keyboard thread joined.\n");
	}
    
    if (vdec) {
        printf("Senging EOS to decoder...\n");
        vdec->cleanup();
        printf("Joining frame thread...\n");
        pthread_join(tid_frame, NULL);
        printf("Frame thread joined.\n");
    }
	
	// Settings are written from a background thread; whatever is still pending
	// goes now, before a restart or the next start reads the file.
	Settings::getInstance().flush();

	////////////////////////////////////////////// DRM CLEANUP;
	if (dev) dev->cleanup();

	if (g_restart) {
		alarm(0);                          // it would outlive the exec
		unsetenv("KESTREL_BB_ATTEMPT");    // a fresh start, not a watchdog retry
		printf("kestrel-gnd: restarting\n");
		fflush(nullptr);
		reexec_now(0);
	}
	printf("kestrel-gnd done.\n");
	return 0;
}
