#pragma once
#include "RadioInterface.hpp"
#include <zmq.hpp>
#include <string>

struct ZmqChannelSimConfig {
    // The channel sim BINDS these endpoints so the transmitter/receiver connect to it.
    std::string rx_bind_addr = "tcp://*:5558";  // PULL  — receives raw I/Q from transmitter
    std::string tx_bind_addr = "tcp://*:5559";  // PUSH  — sends processed I/Q to receiver
    // recv() blocks up to this many ms before returning 0 (lets the loop check g_running)
    int recv_timeout_ms = 100;
};

class ZmqInterface : public RadioInterface {
public:
    explicit ZmqInterface(const ZmqChannelSimConfig& cfg);
    ~ZmqInterface() override = default;

    ssize_t          recv(std::complex<float>* buf, size_t max_samples) override;
    ssize_t          send(const std::complex<float>* buf, size_t num_samples) override;
    void             start() override {}
    void             stop()  override;
    std::string_view name()  const override { return "ZMQ"; }

private:
    zmq::context_t _ctx{1};
    zmq::socket_t  _rx_sock;   // ZMQ_PULL — bound to rx_bind_addr
    zmq::socket_t  _tx_sock;   // ZMQ_PUSH — bound to tx_bind_addr
};
