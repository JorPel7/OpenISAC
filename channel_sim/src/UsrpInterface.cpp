#include "UsrpInterface.hpp"
#include <uhd/types/tune_request.hpp>
#include <iostream>
#include <stdexcept>

UsrpInterface::UsrpInterface(const UsrpChannelSimConfig& cfg) {
    _usrp = uhd::usrp::multi_usrp::make(cfg.device_args);

    _usrp->set_rx_rate(cfg.sample_rate, cfg.rx_channel);
    _usrp->set_rx_freq(uhd::tune_request_t(cfg.center_freq), cfg.rx_channel);
    _usrp->set_rx_gain(cfg.rx_gain, cfg.rx_channel);

    _usrp->set_tx_rate(cfg.sample_rate, cfg.tx_channel);
    _usrp->set_tx_freq(uhd::tune_request_t(cfg.center_freq), cfg.tx_channel);
    _usrp->set_tx_gain(cfg.tx_gain, cfg.tx_channel);

    uhd::stream_args_t args(cfg.cpu_format, cfg.wire_format);

    args.channels = {cfg.rx_channel};
    _rx_stream = _usrp->get_rx_stream(args);

    args.channels = {cfg.tx_channel};
    _tx_stream = _usrp->get_tx_stream(args);

    std::cout << "[USRP] Channel sim device ready."
              << "  rate=" << cfg.sample_rate / 1e6 << " MHz"
              << "  freq=" << cfg.center_freq / 1e9 << " GHz\n";
}

UsrpInterface::~UsrpInterface() { stop(); }

void UsrpInterface::start() {
    if (_streaming) return;
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    cmd.stream_now = true;
    _rx_stream->issue_stream_cmd(cmd);
    _streaming = true;
    std::cout << "[USRP] RX streaming started.\n";
}

void UsrpInterface::stop() {
    if (!_streaming) return;
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    _rx_stream->issue_stream_cmd(cmd);
    _streaming = false;
    std::cout << "[USRP] RX streaming stopped.\n";
}

ssize_t UsrpInterface::recv(std::complex<float>* buf, size_t max_samples) {
    size_t n = _rx_stream->recv(buf, max_samples, _rx_meta, 0.1);

    if (_rx_meta.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
        return 0;
    if (_rx_meta.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        std::cerr << "[USRP] RX error: " << _rx_meta.strerror() << '\n';
        return -1;
    }
    return static_cast<ssize_t>(n);
}

ssize_t UsrpInterface::send(const std::complex<float>* buf, size_t num_samples) {
    uhd::tx_metadata_t md;
    md.has_time_spec  = false;
    md.start_of_burst = false;
    md.end_of_burst   = false;
    size_t n = _tx_stream->send(buf, num_samples, md, 0.1);
    return static_cast<ssize_t>(n);
}
