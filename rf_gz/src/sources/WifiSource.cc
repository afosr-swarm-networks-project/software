#include <cmath>
#include <complex>
#include <random>
#include <unordered_map>
#include <vector>

#include <sdf/Element.hh>

#include "rf_gz/RfSignal.hh"
#include "rf_gz/sources/RfSignalSourceFactory.hh"

namespace rf_gz
{

/// WiFi signal source: bandlimited complex Gaussian noise.
///
/// Matches hitl_gui wifi_device(): the noise burst is generated ONCE on the
/// first tick and then cycled repeatedly (same realization, not new draws).
/// Circular convolution ensures the buffer tiles with no spectral seam.
///
/// SDF parameters (inside <source type="wifi"> or directly on <transmitter>):
///   <bandwidth_hz>  One-sided baseband bandwidth Hz  (default 1e6)
///   <noise_buf_s>   Pre-generated burst duration s   (default 0.001)
class WifiSource : public RfSignalSourceBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("bandwidth_hz"))
      bandwidth_hz_ = sdf->Get<double>("bandwidth_hz");
    if (sdf && sdf->HasElement("noise_buf_s"))
      noise_buf_s_ = sdf->Get<double>("noise_buf_s");
    return true;
  }

  void GenerateBaseband(RfSignal& signal) override
  {
    auto& entry = bursts_[signal.fs_hz];
    if (entry.burst.empty())
      entry.burst = GenerateBurst(signal.fs_hz);

    const uint32_t N = static_cast<uint32_t>(signal.iq.size());
    const uint32_t L = static_cast<uint32_t>(entry.burst.size());

    for (uint32_t i = 0; i < N; ++i)
    {
      signal.iq[i] = entry.burst[entry.pos];
      if (++entry.pos >= L)
        entry.pos = 0;
    }
  }

private:
  std::vector<std::complex<double>> GenerateBurst(double fs_hz)
  {
    const uint32_t M = kFilterTaps_;
    const uint32_t L = std::max(
      static_cast<uint32_t>(std::round(noise_buf_s_ * fs_hz)), M + 1u);

    std::vector<std::complex<double>> noise(L);
    for (auto& s : noise)
      s = std::complex<double>(gauss_(rng_), gauss_(rng_)) * M_SQRT1_2;

    const double fc = std::min(bandwidth_hz_ / fs_hz, 0.499);
    std::vector<double> h(M + 1);
    for (uint32_t n = 0; n <= M; ++n)
    {
      double x    = static_cast<double>(n) - M * 0.5;
      double sinc = (std::abs(x) < 1e-12)
                    ? 2.0 * fc
                    : std::sin(2.0 * M_PI * fc * x) / (M_PI * x);
      double win  = 0.5 * (1.0 - std::cos(2.0 * M_PI * n / M));
      h[n] = sinc * win;
    }

    std::vector<std::complex<double>> burst(L, {0.0, 0.0});
    for (uint32_t i = 0; i < L; ++i)
      for (uint32_t j = 0; j <= M; ++j)
        burst[i] += h[j] * noise[(i - j + L) % L];

    return burst;
  }

  double bandwidth_hz_{1e6};
  double noise_buf_s_ {0.001};

  static constexpr uint32_t kFilterTaps_{64};

  struct BurstEntry {
    std::vector<std::complex<double>> burst;
    uint32_t                          pos{0};
  };
  std::unordered_map<double, BurstEntry> bursts_;

  std::mt19937_64                  rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace rf_gz

REGISTER_RF_SOURCE("wifi", rf_gz::WifiSource);
