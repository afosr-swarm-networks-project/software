#pragma once

#include <memory>
#include "rf_gz/RfDeviceBase.hh"
#include "rf_gz/RfSignal.hh"
#include "rf_gz/sinks/RfSignalSinkFactory.hh"

namespace rf_gz
{

class RfReceiverBase : public RfDeviceBase
{
public:

  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (!RfDeviceBase::LoadSdf(sdf)) return false;
    if (sdf->HasElement("cf_hz"))
      cf_hz = sdf->Get<double>("cf_hz");
    if (sdf->HasElement("fs_hz"))
      fs_hz = sdf->Get<double>("fs_hz");

    sink = CreateSignalSink(sdf);
    if (!sink) return false;
    return true;
  }

  /// Allocate signal.iq (using rx.dt and the receiver's own fs_hz),
  /// set signal.fs_hz, and reset the internal accumulator.
  virtual void PreReceive(RfSignal& signal, const RxContext& rx) = 0;

  /// Accumulate signal.iq (one transmitter's contribution, after channel),
  /// then zero signal.iq so the buffer is ready for the next transmitter.
  virtual void Receive(RfSignal& signal, const TxContext& tx, const RxContext& rx) = 0;

  /// Finalise: add noise, forward accumulated IQ to the sink.
  virtual void PostReceive(RfSignal& signal, const RxContext& rx) = 0;

protected:
  double cf_hz{0.0};  ///< Receiver centre frequency Hz (for downconversion)
  double fs_hz{0.0};  ///< Sampling rate Hz
  std::unique_ptr<RfSignalSinkBase> sink;
};

}  // namespace rf_gz
