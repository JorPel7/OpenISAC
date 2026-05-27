#include "ZmqInterface.hpp"
#include <cstring>
#include <iostream>

ZmqInterface::ZmqInterface(const ZmqChannelSimConfig& cfg)
    : _rx_sock{_ctx, zmq::socket_type::pull}
    , _tx_sock{_ctx, zmq::socket_type::push}
{
    _tx_sock.set(zmq::sockopt::sndhwm, 16);
    _rx_sock.set(zmq::sockopt::rcvhwm, 16);

    // Blocking recv with timeout so the signal loop can observe g_running
    _rx_sock.set(zmq::sockopt::rcvtimeo, cfg.recv_timeout_ms);

    _rx_sock.bind(cfg.rx_bind_addr);
    _tx_sock.bind(cfg.tx_bind_addr);

    std::cout << "[ZMQ] Channel sim ready."
              << "  RX=" << cfg.rx_bind_addr
              << "  TX=" << cfg.tx_bind_addr << '\n';
}

void ZmqInterface::stop() {
    _rx_sock.close();
    _tx_sock.close();
}

ssize_t ZmqInterface::recv(std::complex<float>* buf, size_t max_samples) {
    zmq::message_t msg;
    auto result = _rx_sock.recv(msg);
    if (!result) return 0;  // timeout — let the caller check g_running

    size_t n = std::min(msg.size() / sizeof(std::complex<float>), max_samples);
    std::memcpy(buf, msg.data(), n * sizeof(std::complex<float>));
    return static_cast<ssize_t>(n);
}

ssize_t ZmqInterface::send(const std::complex<float>* buf, size_t num_samples) {
    zmq::message_t msg(num_samples * sizeof(std::complex<float>));
    std::memcpy(msg.data(), buf, num_samples * sizeof(std::complex<float>));
    auto result = _tx_sock.send(std::move(msg), zmq::send_flags::none);
    if (!result) return 0;
    return static_cast<ssize_t>(num_samples);
}
