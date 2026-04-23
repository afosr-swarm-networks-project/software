#pragma once

#include <cmath>
#include <complex>
#include <functional>
#include <vector>

namespace rf_gz
{

/// Pure signal data flowing through the per-tick DSP pipeline.
///
/// cf_hz holds the TX carrier frequency while the signal travels through
/// the wrapped fn chain.  Combine() resets it to the RX CF on completion.
struct RfSignal
{
  std::vector<std::complex<double>> iq;
  double fs_hz{0.0};  ///< Sampling rate Hz          (set by SdfReceiver::Receive)
  double cf_hz{0.0};  ///< Carrier frequency Hz       (set by TX fn; reset to RX CF by Combine)
  double time {0.0};  ///< Sim time at end of window  (set by SdfReceiver::Receive)
};

using SignalFn = std::function<void(RfSignal&)>;

/// Rotate signal.iq by delta_f_hz relative to the window end time stored in signal.time.
inline void ApplyFreqShift(RfSignal& signal, double delta_f_hz)
{
  const double sample_dt = 1.0 / signal.fs_hz;
  const std::size_t n    = signal.iq.size();
  const double t0        = signal.time - static_cast<double>(n) * sample_dt;
  for (std::size_t i = 0; i < n; ++i)
  {
    const double phase = 2.0 * M_PI * delta_f_hz * (t0 + static_cast<double>(i) * sample_dt);
    signal.iq[i] *= std::complex<double>(std::cos(phase), std::sin(phase));
  }
}

}  // namespace rf_gz
