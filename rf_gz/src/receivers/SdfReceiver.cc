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
/// Signal model (matches hitl_gui signal_engine.py):
///   pr = pt + gt - fspl + gr(Δθ)        [dBm link budget]
///
/// pt + gt is baked into signal.iq by SdfTransmitter.
/// fspl    is applied by FreeSpaceChannel.
/// gr(Δθ)  is applied per-contribution in Receive() via the antenna.
///
/// Three-phase pipeline per tick:
///   PreReceive  — allocate signal.iq and accumulator_, set fs_hz.
///   Receive     — downconvert, apply RX antenna gain, accumulate into buffer,
///                 then zero signal.iq for the next transmitter.
///   PostReceive — accumulate world AWGN (signal.iq), add thermal noise, sink.
///
/// SDF parameters:
///   <cf_hz>     Centre frequency Hz  (parsed by RfReceiverBase)
///   <fs_hz>     Sampling rate Hz     (parsed by RfReceiverBase)
///   <nf_db>     Noise figure dB      (default 6.0)
///   <antenna type="omni|yagi" model="..." link="...">...</antenna>
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

  void PreReceive(RfSignal& signal, const RxContext& rx,
                  const gz::sim::UpdateInfo& info) override
  {
    signal.fs_hz = fs_hz;
    const double dt = std::chrono::duration<double>(info.dt).count();
    const auto n = static_cast<std::size_t>(std::round(fs_hz * dt));
    signal.iq.assign(n, {0.0, 0.0});
    accumulator_.assign(n, {0.0, 0.0});
  }

  void Receive(RfSignal& signal, const TxContext& tx, const RxContext& rx,
               const gz::sim::UpdateInfo& info) override
  {
    const uint32_t n     = static_cast<uint32_t>(signal.iq.size());
    const double delta_f = signal.cf_hz - cf_hz;
    antenna->ApplyRxGain(signal, tx, rx);

    if (std::abs(delta_f) < 1.0)
    {
      for (uint32_t i = 0; i < n; ++i)
        accumulator_[i] += signal.iq[i];
    }
    else
    {
      const double sim_time  = std::chrono::duration<double>(info.simTime).count();
      const double sample_dt = 1.0 / signal.fs_hz;
      const double t0        = sim_time - static_cast<double>(n) * sample_dt;
      for (uint32_t i = 0; i < n; ++i)
      {
        const double phase = 2.0 * M_PI * delta_f * (t0 + i * sample_dt);
        accumulator_[i] += signal.iq[i]
                           * std::complex<double>(std::cos(phase), std::sin(phase));
      }
    }

    std::fill(signal.iq.begin(), signal.iq.end(), std::complex<double>{0.0, 0.0});
  }

  void PostReceive(RfSignal& signal, const RxContext& rx,
                   const gz::sim::UpdateInfo& info) override
  {
    // ── Environmental AWGN (filled into signal.iq by RfWorldPlugin) ───────
    for (std::size_t i = 0; i < accumulator_.size(); ++i)
      accumulator_[i] += signal.iq[i];

    // ── Thermal noise: -174 dBm/Hz + 10·log10(fs) + NF ─────────────────
    const double pn_thermal  = std::pow(10.0,
      (-174.0 + 10.0 * std::log10(signal.fs_hz) + nf_db_) / 10.0);
    const double thermal_std = std::sqrt(pn_thermal / 2.0);
    for (auto& s : accumulator_)
      s += std::complex<double>(gauss_(rng_) * thermal_std,
                                gauss_(rng_) * thermal_std);

    RfSignal out;
    out.iq    = std::move(accumulator_);
    out.fs_hz = signal.fs_hz;
    out.cf_hz = cf_hz;
    sink->ConsumeSamples(out, rx, info);
  }

private:
  double nf_db_{6.0};

  std::vector<std::complex<double>> accumulator_;

  std::mt19937_64                  rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace rf_gz

REGISTER_RF_RECEIVER("sdf",  rf_gz::SdfReceiver);
