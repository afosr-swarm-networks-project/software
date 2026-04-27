#include <cmath>
#include <complex>
#include <random>
#include <vector>

#include <sdf/Element.hh>

#include "rf_gz/BandlimitedNoise.hh"
#include "rf_gz/RfSignal.hh"
#include "rf_gz/sources/RfSignalSourceFactory.hh"

namespace rf_gz
{

/// FHSS signal source: bandlimited noise hopping across channels.
///
/// Matches hitl_gui fhss_emitter().  Each hop: pick a random channel,
/// generate bandlimited Gaussian noise, mix to that channel's centre.
/// Phase resets at each hop boundary (realistic).
///
/// SDF parameters (inside <source type="fhss"> or directly on <transmitter>):
///   <bandwidth_hz>  Total hopping span Hz      (default 80e6)
///   <num_channels>  Number of channels         (default 10)
///   <hop_rate_hz>   Hops per second            (default 1000)
class FhssSource : public RfSignalSourceBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("bandwidth_hz"))
      bandwidth_hz_ = sdf->Get<double>("bandwidth_hz");
    if (sdf && sdf->HasElement("num_channels"))
      num_channels_ = sdf->Get<int>("num_channels");
    if (sdf && sdf->HasElement("hop_rate_hz"))
      hop_rate_hz_  = sdf->Get<double>("hop_rate_hz");

    num_channels_ = std::max(1, num_channels_);
    chan_dist_ = std::uniform_int_distribution<int>(0, num_channels_ - 1);
    return true;
  }

  void Advance(double /*dt*/) override {}

  void Generate(RfSignal& signal) override
  {
    if (hop_samples_ == 0)
    {
      hop_samples_ = static_cast<uint32_t>(std::round(signal.fs_hz / hop_rate_hz_));
      hop_samples_ = std::max(hop_samples_, 1u);
      GenerateNextHop(signal.fs_hz);
    }

    const uint32_t N = static_cast<uint32_t>(signal.iq.size());
    uint32_t written = 0;

    while (written < N)
    {
      if (hop_pos_ >= hop_samples_)
        GenerateNextHop(signal.fs_hz);

      uint32_t take = std::min(hop_samples_ - hop_pos_, N - written);
      for (uint32_t i = 0; i < take; ++i)
        signal.iq[written + i] = hop_buffer_[hop_pos_ + i];
      written  += take;
      hop_pos_ += take;
    }
  }

private:
  void GenerateNextHop(double fs_hz)
  {
    const double channel_bw = bandwidth_hz_ / num_channels_;
    const int    channel    = chan_dist_(rng_);
    const double f_center   = -bandwidth_hz_ / 2.0
                              + channel * channel_bw + channel_bw / 2.0;

    hop_buffer_ = bandlimited_complex_noise(
      hop_samples_, channel_bw / 2.0 / fs_hz, rng_, gauss_);

    for (uint32_t i = 0; i < hop_samples_; ++i)
    {
      double phase = 2.0 * M_PI * f_center * static_cast<double>(i) / fs_hz;
      hop_buffer_[i] *= std::complex<double>(std::cos(phase), std::sin(phase));
    }

    hop_pos_ = 0;
  }

  double bandwidth_hz_{80e6};
  int    num_channels_{10};
  double hop_rate_hz_ {1000.0};

  uint32_t                          hop_samples_{0};
  uint32_t                          hop_pos_    {0};
  std::vector<std::complex<double>> hop_buffer_;

  std::mt19937_64                    rng_{std::random_device{}()};
  std::normal_distribution<double>   gauss_{0.0, 1.0};
  std::uniform_int_distribution<int> chan_dist_{0, 0};
};

}  // namespace rf_gz

REGISTER_RF_SOURCE("fhss", rf_gz::FhssSource);
