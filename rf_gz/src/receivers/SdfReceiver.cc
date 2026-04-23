#include <chrono>
#include <cmath>
#include <complex>
#include <random>
#include <vector>

#include <sdf/Element.hh>

#include "rf_gz/receivers/RfReceiverFactory.hh"

namespace rf_gz
{

/// SDF-configured receiver.
///
/// Signal model (link budget):
///   pr = pt + gt - fspl + gr(Δθ)    [dBm]
///
/// pt + gt  — baked into signal.iq by SdfTransmitter + WrapTxGain.
/// fspl     — applied by FreeSpaceChannel::Wrap.
/// gr(Δθ)  — applied by WrapRxGain.
/// Freq shift — applied in Combine() (delta_f = tx_cf - rx_cf).
/// Thermal noise: -174 dBm/Hz + 10·log10(fs) + NF applied here.
///
/// SDF parameters:
///   <cf_hz>     Centre frequency Hz  (parsed by RfReceiverBase)
///   <fs_hz>     Sampling rate Hz     (parsed by RfReceiverBase)
///   <nf_db>     Noise figure dB      (default 6.0)
///   <antenna type="omni|yagi" ...>...</antenna>
///   <sink type="iq_pub"><topic>...</topic></sink>
class SdfReceiver : public RfReceiverBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (!RfReceiverBase::LoadSdf(sdf)) return false;
    if (sdf->HasElement("nf_db"))
      nf_db_ = sdf->Get<double>("nf_db");
    return true;
  }

  void Receive(SignalFn fn, const gz::sim::UpdateInfo& info) override
  {
    const std::size_t n = static_cast<std::size_t>(
      std::round(fs_hz * std::chrono::duration<double>(info.dt).count()));
    if (n == 0) return;

    RfSignal signal;
    signal.fs_hz = fs_hz;
    signal.time  = std::chrono::duration<double>(info.simTime).count();
    signal.iq.assign(n, {0.0, 0.0});

    fn(signal);  // Combine + AWGN; sets signal.cf_hz = cf_hz

    const double pn      = std::pow(10.0,
      (-174.0 + 10.0 * std::log10(fs_hz) + nf_db_) / 10.0);
    const double std_dev = std::sqrt(pn / 2.0);
    for (auto& s : signal.iq)
      s += std::complex<double>(gauss_(rng_) * std_dev, gauss_(rng_) * std_dev);

    sink->ConsumeSamples(signal);
  }

private:
  double nf_db_{6.0};

  std::mt19937_64                  rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace rf_gz

REGISTER_RF_RECEIVER("sdf", rf_gz::SdfReceiver);
