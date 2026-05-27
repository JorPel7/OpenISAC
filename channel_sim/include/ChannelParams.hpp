#pragma once
#include <atomic>

struct ChannelParams {
    // ── System parameters (always active) ────────────────────────────────────
    std::atomic<float> tx_power_db{0.0f};              // dB  — TX power offset
    std::atomic<float> rx_gain_db{0.0f};               // dB  — RX gain offset
    std::atomic<float> noise_power_db{-60.0f};         // dB potencia — AWGN (−60 dBP → sigma=1e-3, −40 dBP → sigma=1e-2)
    std::atomic<int>   antenna_type{0};                // 0=Omni  1=Directiva
    std::atomic<float> antenna_beamwidth_deg{30.0f};   // deg — 3dB beamwidth, Directiva only
    std::atomic<int>   num_targets{1};                 // 0..3

    // ── Per-target parameters ─────────────────────────────────────────────────
    // Active whenever num_targets >= (index + 1)
    struct Target {
        std::atomic<float> distance{25.0f};            // m
        std::atomic<float> velocity{10.0f};            // m/s  positive = approaching
        std::atomic<float> angle_deg{0.0f};            // deg  off-boresight (0=on-axis); only directive
        std::atomic<float> micro_doppler_amp{0.0f};    // rad  phase deviation depth
        std::atomic<float> micro_doppler_freq{10.0f};  // Hz   modulation rate
        std::atomic<float> rcs{1.0f};                  // m²
    };
    Target targets[3];   // targets[0]=T1, targets[1]=T2, targets[2]=T3
};

// UDP control command IDs
namespace ChanCmd {
    inline constexpr const char* VEL  = "VEL ";   // velocity  ×100  (int32)
    inline constexpr const char* DIST = "DIST";   // distance  ×10   (int32)
    inline constexpr const char* MDAF = "MDAF";   // md_amp    ×10000(int32)
    inline constexpr const char* MDFF = "MDFF";   // md_freq   ×100  (int32)
}
