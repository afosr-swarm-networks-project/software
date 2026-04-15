#pragma once

#include <sdf/Element.hh>
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

/// Abstract base class for all signal sink (output) models.
///
/// A signal sink receives the fully-processed combined baseband IQ from a
/// receiver (after downconversion, antenna gain, and noise addition) and
/// decides what to do with it — e.g. publish to gz transport.
///
/// RfReceiverBase owns one instance and calls ConsumeSamples() each tick.
class RfSignalSinkBase
{
public:
  virtual ~RfSignalSinkBase() = default;

  /// Called once per Gazebo tick with the final processed baseband signal.
  /// signal.iq holds the combined IQ after downconversion, antenna gain,
  /// and noise. signal.fs_hz and signal.cf_hz reflect the receiver's config.
  /// rx.time is the simulation time at the END of the current window.
  /// rx.rx_name identifies the receiver (e.g. for default topic naming).
  /// Implementations must be non-blocking (runs on the physics thread).
  virtual void ConsumeSamples(const RfSignal& signal, const RxContext& rx) = 0;

  /// Load sink-specific parameters from SDF.
  /// Receives either the <sink> child element (if present) or the full
  /// <receiver> element as a fallback.
  /// Returns false if a required field is missing; factory returns nullptr.
  virtual bool LoadSdf(sdf::ElementPtr sdf) = 0;
};

}  // namespace rf_gz
