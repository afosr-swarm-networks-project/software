#pragma once

#include <cmath>
#include <complex>
#include <random>
#include <vector>

namespace rf_gz
{

/// Generate n samples of bandlimited complex Gaussian noise.
///
/// @param n        Number of output samples.
/// @param fc_norm  Normalised one-sided cutoff (bandwidth_hz / fs_hz).
///                 Clamped internally to 0.499.
/// @param rng      Caller's Mersenne-Twister engine (state advanced in place).
/// @param gauss    Caller's N(0,1) distribution.
/// @param circular If true, use circular convolution — burst tiles with no
///                 spectral seam at the loop boundary (needed by WifiSource).
///                 If false, use linear convolution with M warm-up samples
///                 discarded (suitable for slot/hop sources).
/// @param M        FIR filter half-length; taps = M+1.
inline std::vector<std::complex<double>>
bandlimited_complex_noise(
  uint32_t n,
  double   fc_norm,
  std::mt19937_64&                  rng,
  std::normal_distribution<double>& gauss,
  bool     circular = false,
  uint32_t M        = 64u)
{
  const double fc = std::min(fc_norm, 0.499);

  // Design Hann-windowed sinc lowpass filter
  std::vector<double> h(M + 1);
  for (uint32_t k = 0; k <= M; ++k)
  {
    const double x    = static_cast<double>(k) - M * 0.5;
    const double sinc = (std::abs(x) < 1e-12)
                        ? 2.0 * fc
                        : std::sin(2.0 * M_PI * fc * x) / (M_PI * x);
    const double win  = 0.5 * (1.0 - std::cos(2.0 * M_PI * k / M));
    h[k] = sinc * win;
  }

  if (circular)
  {
    // Circular convolution: noise wraps at boundary so the burst tiles cleanly
    std::vector<std::complex<double>> noise(n);
    for (auto& s : noise)
      s = std::complex<double>(gauss(rng), gauss(rng)) * M_SQRT1_2;

    std::vector<std::complex<double>> out(n, {0.0, 0.0});
    for (uint32_t i = 0; i < n; ++i)
      for (uint32_t j = 0; j <= M; ++j)
        out[i] += h[j] * noise[(i - j + n) % n];

    return out;
  }
  else
  {
    // Linear convolution: generate n+M noise samples, discard first M outputs
    const uint32_t total = n + M;
    std::vector<std::complex<double>> noise(total);
    for (auto& s : noise)
      s = std::complex<double>(gauss(rng), gauss(rng)) * M_SQRT1_2;

    std::vector<std::complex<double>> out(n, {0.0, 0.0});
    for (uint32_t i = 0; i < n; ++i)
      for (uint32_t j = 0; j <= M; ++j)
        out[i] += h[j] * noise[M + i - j];

    return out;
  }
}

}  // namespace rf_gz
