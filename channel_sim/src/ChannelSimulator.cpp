// ChannelSimulator.cpp — ZMQ transparent pipe + ncurses control TUI
//
// Pipeline:
//   OFDMModulator --zmq  ->  tcp:5558  ->  [ChannelSimulator]  ->  tcp:5559  ->  OFDMModulator sensing RX
//
// Run:
//   ./ChannelSimulator Modulator.yaml

#include "ChannelParams.hpp"
#include "RadioInterface.hpp"
#include "ZmqInterface.hpp"
#include "Common.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <csignal>
#include <cstring>
#include <mutex>
#include <ncurses.h>
#include <pthread.h>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// ─── Global stop flag ────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running.store(false, std::memory_order_relaxed); }

// ─── TUI message line (any thread posts here) ─────────────────────────────────

static std::string g_tui_msg;
static std::mutex  g_tui_msg_mtx;
static void tui_post(std::string s) {
    std::lock_guard<std::mutex> lk(g_tui_msg_mtx);
    g_tui_msg = std::move(s);
}

// ─── Channel physics constants ────────────────────────────────────────────────

static constexpr float C_LIGHT    = 299792458.0f;
static constexpr float PIf        = 3.14159265358979f;
// Normalisation factor: at d=25m, rcs=1m², omni (G=1), fc=5.8GHz → echo amplitude ≈ 0.01
static constexpr float KSIM_NORM  = 120.0f;
// AWGN noise amplitude per I/Q component (≈ −60 dBFS floor before rx_gain_db)
static constexpr float KNOISE_SIG = 1e-3f;
// Pre-computed Gaussian table size (power-of-2 for cheap masking); 512 KB fits in L3 cache
static constexpr size_t NOISE_TBL_SZ = 1u << 17;  // 131072 floats

// ─── Signal pipe loop (high-priority RT thread) ───────────────────────────────
//
// Implements a monostatic radar channel for up to 3 independent targets:
//   - Round-trip range delay  (circular sample buffer)
//   - Doppler shift           (incremental complex phasor)
//   - Micro-Doppler AM-phase  (FM modulation by sinusoidal vibration)
//   - Radar equation amplitude (normalised for simulation visibility)
//   - AWGN                    (thermal noise floor, scalable via rx_gain_db)
//
static void signal_loop(RadioInterface& radio, size_t buffer_size,
                        ChannelParams& params, float fs, float fc) {
    const float lambda = C_LIGHT / fc;

    // Ring buffer: deep enough for the TUI's max range of 10 km (round trip = 20 km)
    const size_t max_delay_samps =
        static_cast<size_t>(std::ceil(2.0f * 20000.0f / C_LIGHT * fs))
        + buffer_size + 1u;
    std::vector<std::complex<float>> ring(max_delay_samps, {0.f, 0.f});
    size_t ring_write = 0;

    // Per-target Doppler phasor and micro-Doppler phase accumulator
    std::complex<float> dop_phasor[3] = {{1.f, 0.f}, {1.f, 0.f}, {1.f, 0.f}};
    float md_phase[3] = {0.f, 0.f, 0.f};

    // Pre-compute Gaussian noise table once — table lookups are ~8× faster than
    // per-sample std::normal_distribution in the real-time hot path.
    std::vector<float> noise_tbl(NOISE_TBL_SZ);
    {
        std::mt19937 rng0{42u};
        std::normal_distribution<float> g{0.f, 1.f};
        for (auto& x : noise_tbl) x = g(rng0);
    }
    size_t nidx = 0;

    std::vector<std::complex<float>> in_buf(buffer_size);
    std::vector<std::complex<float>> out_buf(buffer_size);
    int renorm_ctr = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        const ssize_t n = radio.recv(in_buf.data(), buffer_size);
        if (n <= 0) continue;

        // ── Read channel params once per frame ────────────────────────────────
        const float tx_power_db  = params.tx_power_db.load(std::memory_order_relaxed);
        const float rx_gain_db   = params.rx_gain_db.load(std::memory_order_relaxed);
        const float noise_pdb    = params.noise_power_db.load(std::memory_order_relaxed);
        const int   ant_type     = params.antenna_type.load(std::memory_order_relaxed);
        const float ant_bw_deg   = params.antenna_beamwidth_deg.load(std::memory_order_relaxed);
        const int   num_tgt      = std::clamp(
            params.num_targets.load(std::memory_order_relaxed), 0, 3);

        const float tx_scale = std::pow(10.0f, tx_power_db / 20.0f);
        const float rx_scale = std::pow(10.0f, rx_gain_db  / 20.0f);
        // Peak gain derived from beamwidth (Gaussian beam: G_max = 16·ln2 / θ²_3dB_rad)
        const float G = (ant_type == 0 || ant_bw_deg <= 0.0f) ? 1.0f : [&] {
            const float bw = ant_bw_deg * PIf / 180.0f;
            return 16.0f * 0.6931472f / (bw * bw);
        }();

        // ── Feed ring buffer ──────────────────────────────────────────────────
        for (ssize_t i = 0; i < n; ++i)
            ring[(ring_write + static_cast<size_t>(i)) % max_delay_samps] = in_buf[i];

        // ── Sum target echoes ─────────────────────────────────────────────────
        std::fill(out_buf.begin(), out_buf.begin() + n, std::complex<float>{0.f, 0.f});

        for (int ti = 0; ti < num_tgt; ++ti) {
            auto& t = params.targets[ti];
            const float dist     = std::max(t.distance.load(std::memory_order_relaxed), 0.1f);
            const float vel      = t.velocity.load(std::memory_order_relaxed);
            const float angle_d  = t.angle_deg.load(std::memory_order_relaxed);
            const float md_amp   = t.micro_doppler_amp.load(std::memory_order_relaxed);
            const float md_freq  = t.micro_doppler_freq.load(std::memory_order_relaxed);
            const float rcs      = std::max(t.rcs.load(std::memory_order_relaxed), 0.0f);

            const size_t delay_samps =
                static_cast<size_t>(std::round(2.0f * dist / C_LIGHT * fs));

            // Gaussian beam pattern: G(θ) = G_max · exp(−ln2 · (θ / θ_half)²)
            // θ_half = beamwidth_3dB / 2.  For omni, pattern = 1.
            float beam_factor = 1.0f;
            if (ant_type == 1 && ant_bw_deg > 0.0f) {
                const float theta_half = (ant_bw_deg * 0.5f) * PIf / 180.0f;
                const float theta      = std::abs(angle_d) * PIf / 180.0f;
                beam_factor = std::exp(-0.6931472f * (theta / theta_half) * (theta / theta_half));
            }

            // Normalised radar-equation amplitude (beam_factor replaces per-target G directivity)
            const float echo_amp = KSIM_NORM * G * beam_factor * lambda * std::sqrt(rcs)
                                   / (dist * dist) * tx_scale * rx_scale;

            // Incremental Doppler phasor (one complex multiply per sample, no trig in loop)
            const float   d_phi    = 2.0f * PIf * (2.0f * vel * fc / C_LIGHT) / fs;
            const std::complex<float> d_rot{std::cos(d_phi), std::sin(d_phi)};

            // Micro-Doppler: incremental sin/cos for the modulating frequency
            const float md_step   = 2.0f * PIf * md_freq / fs;
            float md_sin = std::sin(md_phase[ti]);
            float md_cos = std::cos(md_phase[ti]);
            const float md_sin_s  = std::sin(md_step);
            const float md_cos_s  = std::cos(md_step);

            for (ssize_t i = 0; i < n; ++i) {
                const size_t src =
                    (ring_write + static_cast<size_t>(i) + max_delay_samps - delay_samps)
                    % max_delay_samps;

                std::complex<float> phasor = dop_phasor[ti];
                if (md_amp > 0.0f) {
                    const float md_mod = md_amp * md_sin;
                    phasor *= std::complex<float>{std::cos(md_mod), std::sin(md_mod)};
                }
                out_buf[i] += echo_amp * ring[src] * phasor;

                // Advance Doppler phasor
                dop_phasor[ti] = {
                    dop_phasor[ti].real() * d_rot.real() - dop_phasor[ti].imag() * d_rot.imag(),
                    dop_phasor[ti].real() * d_rot.imag() + dop_phasor[ti].imag() * d_rot.real()
                };
                // Advance micro-Doppler accumulator (sum formula, no trig per sample)
                if (md_amp > 0.0f) {
                    const float ns = md_sin * md_cos_s + md_cos * md_sin_s;
                    const float nc = md_cos * md_cos_s - md_sin * md_sin_s;
                    md_sin = ns; md_cos = nc;
                }
            }
            // Save micro-Doppler phase for next frame
            md_phase[ti] = std::atan2(md_sin, md_cos);
        }

        // Renormalise Doppler phasors every 256 frames to prevent float drift
        if (++renorm_ctr >= 256) {
            renorm_ctr = 0;
            for (int ti = 0; ti < 3; ++ti) {
                const float mag = std::abs(dop_phasor[ti]);
                if (mag > 0.0f) dop_phasor[ti] /= mag;
            }
        }

        // ── Advance ring buffer write pointer ────────────────────────────────
        ring_write = (ring_write + static_cast<size_t>(n)) % max_delay_samps;

        // ── AWGN (table lookup — avoids per-sample Gaussian RNG in the hot path) ──
        // noise_pdb is in dB de potencia: P = 10^(x/10), sigma = sqrt(P) = 10^(x/20).
        // nsigma es independiente de rx_scale: rx_gain mejora el SNR del eco, no afecta al ruido.
        const float nsigma = std::sqrt(std::pow(10.0f, noise_pdb / 10.0f));
        for (ssize_t i = 0; i < n; ++i) {
            out_buf[i] += std::complex<float>{
                nsigma * noise_tbl[nidx        & (NOISE_TBL_SZ - 1u)],
                nsigma * noise_tbl[(nidx + 1u) & (NOISE_TBL_SZ - 1u)]
            };
            nidx += 2u;
        }

        radio.send(out_buf.data(), static_cast<size_t>(n));
    }
}

static void set_realtime_priority(std::thread& t, int prio = 80) {
    sched_param sp{prio};
    if (pthread_setschedparam(t.native_handle(), SCHED_FIFO, &sp) != 0)
        tui_post("[WARN] Sin prioridad RT — necesita CAP_SYS_NICE");
    else
        tui_post("[SIG] Thread RT activo SCHED_FIFO/" + std::to_string(prio));
}

// ─── YAML helper ─────────────────────────────────────────────────────────────

template<typename T>
static T yget(const YAML::Node& n, const char* key, T dflt) {
    return n[key] ? n[key].as<T>() : dflt;
}

// ─── Control TUI ─────────────────────────────────────────────────────────────

namespace {

static const char* ANTENNA_NAMES[]      = { "Omnidireccional", "Directiva" };
static const char* TARGET_COUNT_NAMES[] = { "0", "1", "2", "3" };
static const char* TARGET_HEADERS[]     = { " TARGET 1", " TARGET 2", " TARGET 3" };

// One displayable line in the table (either a section header or a parameter row).
struct Row {
    bool is_header = false;

    // Header fields
    const char* header_text = "";

    // Param fields
    const char*         label    = "";
    const char*         unit     = "";
    std::atomic<float>* fval     = nullptr;   // null for discrete
    std::atomic<int>*   ival     = nullptr;   // null for float
    float               step     = 1.0f;
    float               min_val  = -1e9f;
    float               max_val  =  1e9f;
    int                 n_choices = 0;
    const char* const*  choices  = nullptr;

    bool is_discrete()  const { return ival     != nullptr; }
    bool is_selectable() const { return !is_header; }
};

using StepMap = std::unordered_map<std::atomic<float>*, float>;

// Builds the current visible row list depending on antenna_type and num_targets.
static std::vector<Row> build_layout(ChannelParams& p, const StepMap& steps) {
    std::vector<Row> rows;
    rows.reserve(32);

    auto step_of = [&](std::atomic<float>* f, float def) -> float {
        auto it = steps.find(f);
        return it != steps.end() ? it->second : def;
    };

    auto hdr = [&](const char* text) {
        Row r; r.is_header = true; r.header_text = text;
        rows.push_back(r);
    };

    auto fp = [&](const char* lbl, const char* unit, std::atomic<float>* f,
                  float def_step, float mn, float mx) {
        Row r;
        r.label = lbl; r.unit = unit; r.fval = f;
        r.step = step_of(f, def_step);
        r.min_val = mn; r.max_val = mx;
        rows.push_back(r);
    };

    auto dp = [&](const char* lbl, std::atomic<int>* iv, int n, const char* const* ch) {
        Row r;
        r.label = lbl; r.ival = iv; r.n_choices = n; r.choices = ch;
        rows.push_back(r);
    };

    // ── System section ────────────────────────────────────────────────────────
    hdr(" SISTEMA");
    fp("Potencia TX",   "dB",    &p.tx_power_db,    1.0f,  -30.0f,   30.0f);
    fp("Ganancia RX",   "dB",    &p.rx_gain_db,     1.0f,    0.0f,   76.0f);
    fp("Ruido AWGN",    "dB",    &p.noise_power_db, 5.0f, -120.0f,    0.0f);
    dp("Tipo de antena",         &p.antenna_type, 2, ANTENNA_NAMES);
    const int ant_t = p.antenna_type.load(std::memory_order_relaxed);
    if (ant_t == 1) {
        fp("Ancho haz -3dB",   "deg", &p.antenna_beamwidth_deg,   5.0f,  1.0f, 180.0f);
    }
    dp("Targets", &p.num_targets, 4, TARGET_COUNT_NAMES);

    // ── Per-target sections ───────────────────────────────────────────────────
    int nt = std::clamp(p.num_targets.load(std::memory_order_relaxed), 0, 3);
    for (int i = 0; i < nt; ++i) {
        auto& t = p.targets[i];
        hdr(TARGET_HEADERS[i]);
        fp("Distancia",      "m",   &t.distance,          10.0f,   0.0f, 10000.0f);
        fp("Velocidad",      "m/s", &t.velocity,           5.0f, -999.0f,   999.0f);
        if (ant_t == 1) {
            fp("Angulo haz", "deg", &t.angle_deg,          5.0f,  -90.0f,    90.0f);
        }
        fp("MicroDop. Amp",  "rad", &t.micro_doppler_amp,  0.01f,   0.0f,    10.0f);
        fp("MicroDop. Freq", "Hz",  &t.micro_doppler_freq, 1.0f,    0.1f,  1000.0f);
        fp("RCS",            "m2",  &t.rcs,                0.5f,    0.0f,  1000.0f);
    }

    return rows;
}

// Clamp sel to the nearest selectable row (search forward, then backward).
static int clamp_sel(int sel, const std::vector<Row>& rows) {
    int n = static_cast<int>(rows.size());
    if (n == 0) return 0;
    sel = std::clamp(sel, 0, n - 1);
    for (int d : {0, 1, -1, 2, -2, 3, -3}) {
        int s = sel + d;
        if (s >= 0 && s < n && rows[s].is_selectable()) return s;
    }
    for (int s = 0; s < n; ++s)
        if (rows[s].is_selectable()) return s;
    return 0;
}

// Advance sel by delta, skipping headers.
static int move_sel(int sel, int delta, const std::vector<Row>& rows) {
    int n = static_cast<int>(rows.size());
    int s = sel + delta;
    while (s >= 0 && s < n && rows[s].is_header) s += delta;
    if (s < 0 || s >= n) return sel;
    return s;
}

static void draw_table(const std::vector<Row>& rows, int sel,
                       int vp_start, int vp_rows, double sr, double fc) {
    clear();

    // ── Header ───────────────────────────────────────────────────────────────
    attron(COLOR_PAIR(1) | A_BOLD);
    mvprintw(0, 0, " CHANNEL SIMULATOR — PANEL DE CONTROL"
             "     SR:%.0fMHz  Fc:%.3fGHz", sr / 1e6, fc / 1e9);
    attroff(COLOR_PAIR(1) | A_BOLD);

    attron(A_BOLD);
    mvprintw(1, 0, " %-22s  %-17s  %s", "Parametro", "Valor", "Paso");
    attroff(A_BOLD);
    mvhline(2, 0, ACS_HLINE, 72);

    // ── Visible rows ─────────────────────────────────────────────────────────
    int display_y = 3;
    int n = static_cast<int>(rows.size());
    for (int i = vp_start; i < n && (i - vp_start) < vp_rows; ++i) {
        const auto& r = rows[i];

        if (r.is_header) {
            attron(COLOR_PAIR(5) | A_BOLD);
            mvprintw(display_y, 0, "%s", r.header_text);
            attroff(COLOR_PAIR(5) | A_BOLD);
        } else {
            bool is_sel = (i == sel);

            // Value string
            char vbuf[22];
            if (r.is_discrete()) {
                int idx = std::clamp(r.ival->load(std::memory_order_relaxed),
                                     0, r.n_choices - 1);
                snprintf(vbuf, sizeof(vbuf), "%-17s", r.choices[idx]);
            } else {
                snprintf(vbuf, sizeof(vbuf), "%10.3f %-4s",
                         r.fval->load(std::memory_order_relaxed), r.unit);
            }

            // Step string
            char sbuf[12];
            if (r.is_discrete()) snprintf(sbuf, sizeof(sbuf), "   <  >  ");
            else                  snprintf(sbuf, sizeof(sbuf), "%-9.4g",  r.step);

            if (is_sel) attron(COLOR_PAIR(2) | A_BOLD);
            mvprintw(display_y, 0, " %s %-21s  %s  %s",
                     is_sel ? ">" : " ", r.label, vbuf, sbuf);
            if (is_sel) attroff(COLOR_PAIR(2) | A_BOLD);
        }
        ++display_y;
    }

    // ── Footer ────────────────────────────────────────────────────────────────
    int max_y, max_x; getmaxyx(stdscr, max_y, max_x); (void)max_x;
    mvhline(max_y - 2, 0, ACS_HLINE, 72);
    attron(COLOR_PAIR(4));
    mvprintw(max_y - 2, 0,
             " up/dn Navegar | +/- <- -> Valor | j/k Paso x1/2 x2 | Q Salir");
    attroff(COLOR_PAIR(4));

    {
        std::lock_guard<std::mutex> lk(g_tui_msg_mtx);
        if (!g_tui_msg.empty()) {
            attron(COLOR_PAIR(1));
            mvprintw(max_y - 1, 0, " > %s", g_tui_msg.c_str());
            attroff(COLOR_PAIR(1));
        }
    }

    refresh();
}

} // namespace

static void run_control_tui(ChannelParams& params, double sr, double fc) {
    initscr();
    start_color();
    use_default_colors();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    halfdelay(10);   // 1 second redraw interval
    curs_set(0);

    init_pair(1, COLOR_CYAN,  -1);           // header / message
    init_pair(2, COLOR_BLACK, COLOR_WHITE);  // selected row
    init_pair(3, COLOR_GREEN, -1);           // (reserved)
    init_pair(4, COLOR_YELLOW,-1);           // help bar
    init_pair(5, COLOR_CYAN,  -1);           // section headers

    StepMap step_map;
    std::vector<Row> layout = build_layout(params, step_map);

    // Track layout-affecting params so we know when to rebuild
    int prev_antenna_type = params.antenna_type.load();
    int prev_num_targets  = params.num_targets.load();

    int sel    = clamp_sel(0, layout);
    int vp_start = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        // Rebuild layout if antenna_type or num_targets changed
        int cur_ant = params.antenna_type.load(std::memory_order_relaxed);
        int cur_nt  = params.num_targets.load(std::memory_order_relaxed);
        if (cur_ant != prev_antenna_type || cur_nt != prev_num_targets) {
            // Save current steps before rebuilding
            for (auto& r : layout)
                if (!r.is_header && r.fval) step_map[r.fval] = r.step;
            layout = build_layout(params, step_map);
            sel = clamp_sel(sel, layout);
            prev_antenna_type = cur_ant;
            prev_num_targets  = cur_nt;
        }

        // Adjust viewport so sel is always visible
        int max_y, max_x; getmaxyx(stdscr, max_y, max_x); (void)max_x;
        int vp_rows = max_y - 5;  // 3 header lines + 2 footer lines
        if (vp_rows < 1) vp_rows = 1;
        int n = static_cast<int>(layout.size());
        if (sel < vp_start) vp_start = sel;
        if (sel >= vp_start + vp_rows) vp_start = sel - vp_rows + 1;
        vp_start = std::clamp(vp_start, 0, std::max(0, n - vp_rows));

        draw_table(layout, sel, vp_start, vp_rows, sr, fc);

        int ch = getch();
        if (ch == ERR) continue;  // 1 s timeout — just redraw

        if (sel < 0 || sel >= n) { sel = clamp_sel(0, layout); continue; }
        Row& r = layout[sel];
        char msg[96];

        switch (ch) {
        // ── Navigation ───────────────────────────────────────────────────────
        case KEY_UP:   sel = move_sel(sel, -1, layout); break;
        case KEY_DOWN: sel = move_sel(sel, +1, layout); break;

        // ── Increase ─────────────────────────────────────────────────────────
        case KEY_RIGHT: case '+': case '=':
            if (r.is_discrete()) {
                int v = (r.ival->load() + 1) % r.n_choices;
                r.ival->store(v, std::memory_order_relaxed);
                snprintf(msg, sizeof(msg), "%s -> %s", r.label, r.choices[v]);
            } else {
                float v = std::min(r.fval->load() + r.step, r.max_val);
                r.fval->store(v, std::memory_order_relaxed);
                snprintf(msg, sizeof(msg), "%s -> %.4g %s", r.label, v, r.unit);
            }
            tui_post(msg);
            break;

        // ── Decrease ─────────────────────────────────────────────────────────
        case KEY_LEFT: case '-':
            if (r.is_discrete()) {
                int v = (r.ival->load() - 1 + r.n_choices) % r.n_choices;
                r.ival->store(v, std::memory_order_relaxed);
                snprintf(msg, sizeof(msg), "%s -> %s", r.label, r.choices[v]);
            } else {
                float v = std::max(r.fval->load() - r.step, r.min_val);
                r.fval->store(v, std::memory_order_relaxed);
                snprintf(msg, sizeof(msg), "%s -> %.4g %s", r.label, v, r.unit);
            }
            tui_post(msg);
            break;

        // ── Step adjust ──────────────────────────────────────────────────────
        case 'j':
            if (!r.is_discrete()) {
                r.step /= 2.0f;
                step_map[r.fval] = r.step;
                snprintf(msg, sizeof(msg), "Paso %s -> %.4g", r.label, r.step);
                tui_post(msg);
            }
            break;
        case 'k':
            if (!r.is_discrete()) {
                r.step *= 2.0f;
                step_map[r.fval] = r.step;
                snprintf(msg, sizeof(msg), "Paso %s -> %.4g", r.label, r.step);
                tui_post(msg);
            }
            break;

        case 'q': case 'Q':
            g_running.store(false, std::memory_order_relaxed);
            break;
        }
    }

    endwin();
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string cfg_path =
        argc > 1 ? argv[1] : "config/Modulator_B210.yaml";

    Config cfg;
    if (!load_modulator_config_from_yaml(cfg, cfg_path)) {
        std::cerr << "[FATAL] Cannot load config: " << cfg_path << '\n';
        return 1;
    }
    const double sample_rate = cfg.sample_rate;
    const double center_freq = cfg.center_freq;

    YAML::Node yaml_root;
    try { yaml_root = YAML::LoadFile(cfg_path); }
    catch (const YAML::Exception& e) {
        std::cerr << "[FATAL] YAML: " << e.what() << '\n';
        return 1;
    }

    const YAML::Node sim_node   = yaml_root["channel_sim"];
    const int    ctrl_port      = sim_node ? yget<int>(sim_node, "control_port", 9998) : 9998;
    const size_t buf_size       = sim_node
        ? yget<size_t>(sim_node, "buffer_size", cfg.samples_per_frame())
        : cfg.samples_per_frame();

    ZmqChannelSimConfig zcfg;
    if (sim_node && sim_node["zmq"]) {
        const auto& z = sim_node["zmq"];
        zcfg.rx_bind_addr    = yget<std::string>(z, "rx_bind_addr",    zcfg.rx_bind_addr);
        zcfg.tx_bind_addr    = yget<std::string>(z, "tx_bind_addr",    zcfg.tx_bind_addr);
        zcfg.recv_timeout_ms = yget<int>(z, "recv_timeout_ms", 100);
    }

    ZmqInterface radio(zcfg);

    // ── Channel params + UDP control ──────────────────────────────────────────
    ChannelParams params;
    if (sim_node && sim_node["init"]) {
        const auto& init = sim_node["init"];
        params.tx_power_db.store(         yget<float>(init, "tx_power_db",         0.0f),  std::memory_order_relaxed);
        params.rx_gain_db.store(          yget<float>(init, "rx_gain_db",           0.0f),  std::memory_order_relaxed);
        params.num_targets.store(         yget<int>  (init, "num_targets",          0),     std::memory_order_relaxed);
        params.targets[0].distance.store( yget<float>(init, "target_distance",    100.0f),  std::memory_order_relaxed);
        params.targets[0].velocity.store( yget<float>(init, "target_velocity",      0.0f),  std::memory_order_relaxed);
        params.targets[0].micro_doppler_amp.store( yget<float>(init, "micro_doppler_amp",  0.0f), std::memory_order_relaxed);
        params.targets[0].micro_doppler_freq.store(yget<float>(init, "micro_doppler_freq",10.0f), std::memory_order_relaxed);
    }

    // UDP commands update target 0 and post to TUI
    ControlCommandHandler ctrl_handler(ctrl_port);
    ctrl_handler.register_command(ChanCmd::VEL, [&params](int32_t v) {
        float val = static_cast<float>(v) / 100.0f;
        params.targets[0].velocity.store(val, std::memory_order_relaxed);
        char buf[64]; snprintf(buf, sizeof(buf), "[UDP] T1 velocidad -> %.2f m/s", val);
        tui_post(buf);
    });
    ctrl_handler.register_command(ChanCmd::DIST, [&params](int32_t v) {
        float val = static_cast<float>(v) / 10.0f;
        params.targets[0].distance.store(val, std::memory_order_relaxed);
        char buf[64]; snprintf(buf, sizeof(buf), "[UDP] T1 distancia -> %.1f m", val);
        tui_post(buf);
    });
    ctrl_handler.register_command(ChanCmd::MDAF, [&params](int32_t v) {
        float val = static_cast<float>(v) / 10000.0f;
        params.targets[0].micro_doppler_amp.store(val, std::memory_order_relaxed);
        char buf[64]; snprintf(buf, sizeof(buf), "[UDP] T1 mdop amp -> %.4f rad", val);
        tui_post(buf);
    });
    ctrl_handler.register_command(ChanCmd::MDFF, [&params](int32_t v) {
        float val = static_cast<float>(v) / 100.0f;
        params.targets[0].micro_doppler_freq.store(val, std::memory_order_relaxed);
        char buf[64]; snprintf(buf, sizeof(buf), "[UDP] T1 mdop freq -> %.2f Hz", val);
        tui_post(buf);
    });

    // ── Start ────────────────────────────────────────────────────────────────
    radio.start();
    ctrl_handler.start();

    std::thread sig_thread([&] {
        signal_loop(radio, buf_size, params,
                    static_cast<float>(sample_rate),
                    static_cast<float>(center_freq));
    });
    set_realtime_priority(sig_thread);

    run_control_tui(params, sample_rate, center_freq);  // blocks until Q or Ctrl+C

    // ── Shutdown ─────────────────────────────────────────────────────────────
    sig_thread.join();
    ctrl_handler.stop();
    radio.stop();
    return 0;
}
