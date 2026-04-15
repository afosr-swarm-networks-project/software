#pragma once

#include <sdf/Element.hh>
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

class RfChannelModelBase
{
public:
  /// Returns false if a required field is missing; factory returns nullptr.
  virtual bool LoadSdf(sdf::ElementPtr /*sdf*/) { return true; }

  /// Applies channel effects to signal.iq in-place.
  virtual void Apply(RfSignal& signal, const TxContext& tx, const RxContext& rx) = 0;

  virtual ~RfChannelModelBase() = default;
};

}  // namespace rf_gz
