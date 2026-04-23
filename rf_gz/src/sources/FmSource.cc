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

  /// Advance phases analytically by dt seconds; snapshot start-of-tick phases
  /// for Generate so multiple receivers get identical output.
  void Advance(double dt) override
  {
    gen_audio_phase_ = audio_phase_;
    gen_fm_phase_    = fm_phase_;

    // audio phase advances linearly
    const double new_audio = audio_phase_ + 2.0 * M_PI * audio_hz_ * dt;

    // FM phase: integral of 2π·deviation·sin(audio_phase_0 + 2π·audio_hz·τ) dτ
    // = (deviation/audio_hz)·(cos(audio_phase_0) - cos(audio_phase_0 + 2π·audio_hz·dt))
    fm_phase_ += (deviation_hz_ / audio_hz_) *
                 (std::cos(audio_phase_) - std::cos(new_audio));
    audio_phase_ = new_audio;
  }

  /// Generate N samples from the snapshotted start-of-tick phases.
  /// Does not mutate member state; idempotent across multiple receivers.
  void Generate(RfSignal& signal) override
  {
    const std::size_t N = signal.iq.size();
    double ap = gen_audio_phase_;
    double fp = gen_fm_phase_;
    for (std::size_t i = 0; i < N; ++i)
    {
      const double m = std::sin(ap);
      ap += 2.0 * M_PI * audio_hz_    / signal.fs_hz;
      fp += 2.0 * M_PI * deviation_hz_ * m / signal.fs_hz;
      signal.iq[i] = std::complex<double>(std::cos(fp), std::sin(fp));
    }
  }

private:
  double deviation_hz_{75e3};
  double audio_hz_    {3000.0};

  double audio_phase_    {0.0};
  double fm_phase_       {0.0};
  double gen_audio_phase_{0.0};
  double gen_fm_phase_   {0.0};
};

}  // namespace rf_gz

REGISTER_RF_SOURCE("fm", rf_gz::FmSource);
