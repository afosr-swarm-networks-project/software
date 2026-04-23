#pragma once

#include <gz/math/Pose3.hh>
#include <gz/sim/Types.hh>
#include "rf_gz/RfDeviceBase.hh"
#include "rf_gz/RfSignal.hh"
#include "rf_gz/sources/RfSignalSourceBase.hh"
#include "rf_gz/sources/RfSignalSourceFactory.hh"

namespace rf_gz
{

class RfTransmitterBase : public RfDeviceBase
{
public:

  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (!RfDeviceBase::LoadSdf(sdf)) return false;
    if (sdf->HasElement("cf_hz"))
      cf_hz = sdf->Get<double>("cf_hz");
    source = CreateSignalSource(sdf);
    if (!source) return false;
    return true;
  }

  /// Returns a SignalFn that generates one frame of baseband IQ and sets signal.cf_hz.
  virtual SignalFn Transmit(const gz::sim::UpdateInfo& info) = 0;

  /// Wraps inner with TX antenna gain applied after inner runs.
  SignalFn WrapTxGain(SignalFn inner,
                      const gz::math::Pose3d& tx_pose,
                      const gz::math::Pose3d& rx_pose)
  {
    return [inner = std::move(inner), this, tx_pose, rx_pose](RfSignal& s) mutable {
      inner(s);
      antenna->ApplyTxGain(s, tx_pose, rx_pose);
    };
  }

protected:
  double cf_hz{0.0};
  std::shared_ptr<RfSignalSourceBase> source;
};

}  // namespace rf_gz
