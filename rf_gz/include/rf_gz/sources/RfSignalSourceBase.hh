#pragma once

#include <complex>
#include <vector>
#include <sdf/Element.hh>
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

/// Abstract base class for all signal source (modulation) models.
///
/// Advance() is called exactly once per tick by SdfTransmitter::Transmit,
/// stepping internal state forward by dt seconds.
/// Generate() is called once per receiver from inside the SignalFn; it fills
/// signal.iq from the state snapshot prepared by Advance without mutating state.
class RfSignalSourceBase
{
public:
  virtual ~RfSignalSourceBase() = default;

  /// Step internal state forward by dt seconds. Called once per tick.
  virtual void Advance(double dt) = 0;

  /// Fill signal.iq from the snapshot prepared by Advance. Called per receiver.
  virtual void Generate(RfSignal& signal) = 0;

  /// Returns false if a required field is missing; factory returns nullptr.
  virtual bool LoadSdf(sdf::ElementPtr sdf) = 0;
};

}  // namespace rf_gz
