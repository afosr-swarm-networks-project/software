#pragma once

#include <memory>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/sim/Types.hh>
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

  void OnNameSet() override
  {
    if (sink) sink->SetName(name);
  }

  /// Combines per-TX wrapped fns into one fn.
  /// For each fn: allocates a tmp buffer, calls fn(tmp) which sets tmp.cf_hz
  /// to the TX CF, applies freq shift (delta_f = tmp.cf_hz - cf_hz), and
  /// accumulates into the result.  Sets signal.cf_hz = cf_hz on completion.
  SignalFn Combine(std::vector<SignalFn> fns)
  {
    return [this, fns = std::move(fns)](RfSignal& signal) mutable {
      const std::size_t n = signal.iq.size();
      std::vector<std::complex<double>> acc(n, {0.0, 0.0});

      for (auto& fn : fns)
      {
        RfSignal tmp;
        tmp.iq.assign(n, {0.0, 0.0});
        tmp.fs_hz = signal.fs_hz;
        tmp.time  = signal.time;
        fn(tmp);

        const double delta_f = tmp.cf_hz - cf_hz;
        if (std::abs(delta_f) >= 1.0)
          ApplyFreqShift(tmp, delta_f);

        for (std::size_t i = 0; i < n; ++i)
          acc[i] += tmp.iq[i];
      }

      signal.iq    = std::move(acc);
      signal.cf_hz = cf_hz;
    };
  }

  /// Wraps inner with RX antenna gain applied after inner runs.
  SignalFn WrapRxGain(SignalFn inner,
                      const gz::math::Pose3d& tx_pose,
                      const gz::math::Pose3d& rx_pose)
  {
    return [inner = std::move(inner), this, tx_pose, rx_pose](RfSignal& s) mutable {
      inner(s);
      antenna->ApplyRxGain(s, tx_pose, rx_pose);
    };
  }

  /// Allocates signal, executes fn (combined contributions + AWGN),
  /// adds thermal noise, and forwards to the sink.
  virtual void Receive(SignalFn fn, const gz::sim::UpdateInfo& info) = 0;

protected:
  double cf_hz{0.0};  ///< RX centre frequency Hz
  double fs_hz{0.0};  ///< Sampling rate Hz
  std::unique_ptr<RfSignalSinkBase> sink;
};

}  // namespace rf_gz
