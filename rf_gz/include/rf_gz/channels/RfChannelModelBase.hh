#pragma once

#include <gz/math/Pose3.hh>
#include <gz/sim/Types.hh>
#include <sdf/Element.hh>
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

class RfChannelModelBase
{
public:
  /// Returns false if a required field is missing; factory returns nullptr.
  virtual bool LoadSdf(sdf::ElementPtr /*sdf*/) { return true; }

  /// Wraps inner with channel effects captured at build time for this TX/RX pair.
  virtual SignalFn Wrap(SignalFn inner,
                        const gz::math::Pose3d& tx_pose,
                        const gz::math::Pose3d& rx_pose,
                        const gz::sim::UpdateInfo& info) = 0;

  virtual ~RfChannelModelBase() = default;
};

}  // namespace rf_gz
