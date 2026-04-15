#pragma once

#include <complex>
#include <vector>
#include <sdf/Element.hh>
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

/// Abstract base class for all signal source (modulation) models.
///
/// GenerateBaseband() writes into signal.iq in-place (already sized by PreReceive).
/// Reads signal.fs_hz for timing. SdfTransmitter normalises to power_dbm
/// and applies antenna TX gain after the call.
class RfSignalSourceBase
{
public:
  virtual ~RfSignalSourceBase() = default;

  virtual void GenerateBaseband(RfSignal& signal) = 0;

  /// Returns false if a required field is missing; factory returns nullptr.
  virtual bool LoadSdf(sdf::ElementPtr sdf) = 0;
};

}  // namespace rf_gz
