#pragma once

#include <string_view>
#include <sdf/Element.hh>
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

/// Abstract base class for all signal sink (output) models.
///
/// A signal sink receives the fully-processed combined baseband IQ from a
/// receiver (after downconversion, antenna gain, and noise addition) and
/// decides what to do with it — e.g. publish to a ROS 2 topic.
///
/// RfReceiverBase owns one instance and calls ConsumeSamples() each tick.
class RfSignalSinkBase
{
public:
  virtual ~RfSignalSinkBase() = default;

  /// Called once per Gazebo tick with the final processed baseband signal.
  /// signal.cf_hz holds the RX centre frequency.
  /// signal.time is the simulation time at the end of the current window.
  /// Implementations must be non-blocking (runs on the physics thread).
  virtual void ConsumeSamples(const RfSignal& signal) = 0;

  /// Called by RfReceiverBase::OnNameSet to forward the receiver name.
  /// Sinks may use it for default topic naming or logging.
  virtual void SetName(std::string_view /*name*/) {}

  /// Load sink-specific parameters from SDF.
  /// Returns false if a required field is missing; factory returns nullptr.
  virtual bool LoadSdf(sdf::ElementPtr sdf) = 0;
};

}  // namespace rf_gz
