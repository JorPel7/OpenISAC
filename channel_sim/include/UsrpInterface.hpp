#pragma once
#include "RadioInterface.hpp"
#include <uhd/usrp/multi_usrp.hpp>
#include <string>

// Hardware parameters that are the same as the rest of the system
// (sample_rate, center_freq) come from Config and are passed in here.
struct UsrpChannelSimConfig {
    std::string device_args;          // USRP address / args (separate device from modulator)
    double      sample_rate  = 50e6;  // Must match Config::sample_rate
    double      center_freq  = 2.4e9; // Must match Config::center_freq
    double      rx_gain      = 30.0;
    double      tx_gain      = 30.0;
    size_t      rx_channel   = 0;
    size_t      tx_channel   = 0;
    std::string cpu_format   = "fc32";
    std::string wire_format  = "sc16";
};

class UsrpInterface : public RadioInterface {
public:
    explicit UsrpInterface(const UsrpChannelSimConfig& cfg);
    ~UsrpInterface() override;

    ssize_t          recv(std::complex<float>* buf, size_t max_samples) override;
    ssize_t          send(const std::complex<float>* buf, size_t num_samples) override;
    void             start() override;
    void             stop()  override;
    std::string_view name()  const override { return "USRP"; }

private:
    uhd::usrp::multi_usrp::sptr _usrp;
    uhd::rx_streamer::sptr       _rx_stream;
    uhd::tx_streamer::sptr       _tx_stream;
    uhd::rx_metadata_t           _rx_meta;
    bool                         _streaming = false;
};
