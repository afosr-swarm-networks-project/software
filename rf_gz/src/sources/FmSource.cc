#include <cmath>
#include <complex>
#include <vector>

#include <sdf/Element.hh>

#include "rf_gz/RfSignal.hh"
#include "rf_gz/sources/RfSignalSourceFactory.hh"

namespace rf_gz
{

/// FM signal source: sinusoidal audio tone with frequency modulation.
///
/// Matches hitl_gui fm_emitter().
/// Instantaneous frequency: deviation_hz * sin(2π * audio_hz * t)
/// Phase is accumulated continuously across Gazebo ticks.
///
/// SDF parameters (inside <source type="fm"> or directly on <transmitter>):
///   <deviation_hz>  Max frequency deviation Hz  (default 75e3)
///   <audio_hz>      Modulating tone Hz           (default 3000)
class FmSource : public RfSignalSourceBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("deviation_hz"))
      deviation_hz_ = sdf->Get<double>("deviation_hz");
    if (sdf && sdf->HasElement("audio_hz"))
      audio_hz_ = sdf->Get<double>("audio_hz");
    return true;
  }

  void GenerateBaseband(RfSignal& signal) override
  {
    const uint32_t N = static_cast<uint32_t>(signal.iq.size());

    for (uint32_t i = 0; i < N; ++i)
    {
      double m = std::sin(audio_phase_);
      audio_phase_ += 2.0 * M_PI * audio_hz_ / signal.fs_hz;
      fm_phase_    += 2.0 * M_PI * (deviation_hz_ * m) / signal.fs_hz;
      signal.iq[i] = std::complex<double>(std::cos(fm_phase_), std::sin(fm_phase_));
    }
  }

private:
  double deviation_hz_{75e3};
  double audio_hz_    {3000.0};

  double fm_phase_   {0.0};
  double audio_phase_{0.0};
};

}  // namespace rf_gz

REGISTER_RF_SOURCE("fm", rf_gz::FmSource);
