#pragma once
#include <complex>
#include <cstddef>
#include <string_view>
#include <sys/types.h>  // ssize_t

// Abstract radio I/O interface.
// The signal-processing loop uses this exclusively — it never touches UHD or ZMQ directly.
class RadioInterface {
public:
    virtual ~RadioInterface() = default;

    // Blocking receive. Writes up to max_samples into buf.
    // Returns the number of samples written, 0 on timeout, -1 on hard error.
    virtual ssize_t recv(std::complex<float>* buf, size_t max_samples) = 0;

    // Non-blocking send. Sends num_samples from buf.
    // Returns the number accepted, 0 if the downstream is full (drop), -1 on error.
    virtual ssize_t send(const std::complex<float>* buf, size_t num_samples) = 0;

    virtual void start() = 0;
    virtual void stop()  = 0;

    virtual std::string_view name() const = 0;
};
