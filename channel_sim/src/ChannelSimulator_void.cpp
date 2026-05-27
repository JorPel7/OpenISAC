// ChannelSimulator_void.cpp — Canal transparente (sin procesado)
//
// Pipeline:
//   OFDMModulator --zmq  ->  tcp:5558  ->  [ChannelSimulator_void]  ->  tcp:5559  ->  OFDMDemodulator --zmq
//
// Funcionamiento:
//   Recibe muestras I/Q complejas por ZMQ (PULL) y las retransmite
//   inmediatamente por ZMQ (PUSH) sin ningún tipo de procesado.
//
// Uso:
//   ./ChannelSimulator_void [config.yaml]

#include "RadioInterface.hpp"
#include "ZmqInterface.hpp"
#include "Common.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <string>
#include <vector>

// ─── Flag de parada global ────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running.store(false, std::memory_order_relaxed); }

// ─── Bucle de señal (pipe transparente) ──────────────────────────────────────

static void signal_loop(RadioInterface& radio, size_t buffer_size) {
    std::vector<std::complex<float>> buf(buffer_size);
    uint64_t frames = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        const ssize_t n = radio.recv(buf.data(), buffer_size);
        if (n <= 0) continue;   // timeout o error — volver a intentar

        radio.send(buf.data(), static_cast<size_t>(n));
        ++frames;

        // Mostrar actividad cada 1000 tramas (~1 s a 1 kHz de tramas)
        if (frames % 1000u == 0u)
            std::printf("[VOID] Tramas retransmitidas: %llu\n",
                        static_cast<unsigned long long>(frames));
    }

    std::printf("[VOID] Detenido. Total de tramas retransmitidas: %llu\n",
                static_cast<unsigned long long>(frames));
}

// ─── YAML helper ─────────────────────────────────────────────────────────────

template<typename T>
static T yget(const YAML::Node& n, const char* key, T dflt) {
    return n[key] ? n[key].as<T>() : dflt;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string cfg_path =
        argc > 1 ? argv[1] : "config/Modulator_B210.yaml";

    // Cargar configuración (solo necesitamos samples_per_frame y las
    // direcciones ZMQ opcionales del bloque channel_sim.zmq)
    Config cfg;
    if (!load_modulator_config_from_yaml(cfg, cfg_path)) {
        std::fprintf(stderr, "[FATAL] No se puede cargar la configuración: %s\n",
                     cfg_path.c_str());
        return 1;
    }

    YAML::Node yaml_root;
    try { yaml_root = YAML::LoadFile(cfg_path); }
    catch (const YAML::Exception& e) {
        std::fprintf(stderr, "[FATAL] YAML: %s\n", e.what());
        return 1;
    }

    const YAML::Node sim_node = yaml_root["channel_sim"];
    const size_t buf_size = sim_node
        ? yget<size_t>(sim_node, "buffer_size", cfg.samples_per_frame())
        : cfg.samples_per_frame();

    // Configuración ZMQ: mismas direcciones que el ChannelSimulator completo
    ZmqChannelSimConfig zcfg;
    if (sim_node && sim_node["zmq"]) {
        const auto& z = sim_node["zmq"];
        zcfg.rx_bind_addr    = yget<std::string>(z, "rx_bind_addr",    zcfg.rx_bind_addr);
        zcfg.tx_bind_addr    = yget<std::string>(z, "tx_bind_addr",    zcfg.tx_bind_addr);
        zcfg.recv_timeout_ms = yget<int>(z, "recv_timeout_ms", 100);
    }

    std::printf("[VOID] Canal transparente iniciado\n");
    std::printf("[VOID]   RX (PULL): %s\n", zcfg.rx_bind_addr.c_str());
    std::printf("[VOID]   TX (PUSH): %s\n", zcfg.tx_bind_addr.c_str());
    std::printf("[VOID]   Buffer:    %zu muestras\n", buf_size);
    std::printf("[VOID] Ctrl+C para salir.\n\n");
    std::fflush(stdout);

    ZmqInterface radio(zcfg);
    radio.start();

    signal_loop(radio, buf_size);   // bloquea hasta Ctrl+C / SIGTERM

    radio.stop();
    return 0;
}
