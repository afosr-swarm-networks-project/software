#pragma once

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

  /// Generate baseband IQ into signal.iq and set signal.cf_hz.
  /// signal.fs_hz and signal.iq.size() are set by PreReceive before this call.
  /// Called once per transmitter per RX tick by RfWorldPlugin.
  virtual void Transmit(RfSignal& signal, const TxContext& tx, const RxContext& rx) = 0;

protected:
  double cf_hz{0.0};  ///< Carrier frequency Hz (for downconversion at the receiver)
  std::shared_ptr<RfSignalSourceBase> source;
};

}  // namespace rf_gz
