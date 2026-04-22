#pragma once

#include <complex>
#include <string>
#include <vector>
#include <gz/math/Pose3.hh>

namespace rf_gz
{

/// Transmitter-side deployment context for one TX→RX pair.
struct TxContext
{
  gz::math::Pose3d tx_pose;  ///< Transmitter world pose
  std::string      tx_name;  ///< Transmitter name
};

/// Receiver-side deployment context for one tick.
struct RxContext
{
  gz::math::Pose3d rx_pose;  ///< Receiver world pose
  std::string      rx_name;  ///< Receiver name
};

/// Pure signal data flowing through the per-tick DSP pipeline.
///
/// Lifetime within one tick (per receiver):
///   PreReceive  — allocates iq, sets fs_hz.
///   TransmitIQ  — writes iq, sets cf_hz.
///   Apply       — channel attenuates iq in-place.
///   Receive     — accumulates iq into internal buffer, zeros iq for next TX.
///   PostReceive — adds noise, forwards to sink.
struct RfSignal
{
  std::vector<std::complex<double>> iq;  ///< IQ buffer (allocated by PreReceive)
  double fs_hz{0.0};                     ///< Sampling rate Hz       (set by PreReceive)
  double cf_hz{0.0};                     ///< TX carrier frequency Hz (set by TransmitIQ)
};

}  // namespace rf_gz
