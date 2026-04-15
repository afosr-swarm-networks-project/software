#include <cmath>
#include <complex>
#include <random>
#include <vector>

#include <sdf/Element.hh>

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

  void GenerateBaseband(RfSignal& signal) override
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
  static constexpr uint32_t kFilterTaps_{64};

  std::vector<std::complex<double>>
  BandlimitedNoise(uint32_t n, double f_high_hz, double fs_hz)
  {
    const double   fc = std::min(f_high_hz / fs_hz, 0.499);
    const uint32_t M  = kFilterTaps_;

    std::vector<double> h(M + 1);
    for (uint32_t k = 0; k <= M; ++k)
    {
      double x    = static_cast<double>(k) - M * 0.5;
      double sinc = (std::abs(x) < 1e-12)
                    ? 2.0 * fc
                    : std::sin(2.0 * M_PI * fc * x) / (M_PI * x);
      double win  = 0.5 * (1.0 - std::cos(2.0 * M_PI * k / M));
      h[k] = sinc * win;
    }

    std::vector<std::complex<double>> noise(n + M);
    for (auto& s : noise)
      s = std::complex<double>(gauss_(rng_), gauss_(rng_)) * M_SQRT1_2;

    std::vector<std::complex<double>> out(n, {0.0, 0.0});
    for (uint32_t i = 0; i < n; ++i)
      for (uint32_t j = 0; j <= M; ++j)
        out[i] += h[j] * noise[M + i - j];

    return out;
  }

  void GenerateNextHop(double fs_hz)
  {
    const double channel_bw = bandwidth_hz_ / num_channels_;
    const int    channel    = chan_dist_(rng_);
    const double f_center   = -bandwidth_hz_ / 2.0
                              + channel * channel_bw + channel_bw / 2.0;

    hop_buffer_ = BandlimitedNoise(hop_samples_, channel_bw / 2.0, fs_hz);

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
