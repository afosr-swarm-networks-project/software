#include <cmath>
#include <complex>
#include <random>
#include <vector>

#include <sdf/Element.hh>

#include "rf_gz/RfSignal.hh"
#include "rf_gz/sources/RfSignalSourceFactory.hh"

namespace rf_gz
{

/// WiFi signal source: bursty bandlimited complex Gaussian noise.
///
/// Burst/silence duty cycle matches hitl_gui wifi_device():
///   burst(burst_time_s_) → silence(uniform) → repeat
///
/// State is tracked in time (seconds) so it is sample-rate-agnostic.
/// Advance() steps the burst/silence machine once per tick and records a
/// segment list.  Generate() replays those segments into signal.iq without
/// touching any state — safe to call once per receiver.
///
/// SDF parameters:
///   <bandwidth_hz>    One-sided baseband bandwidth Hz  (default 1e6)
///   <burst_time_s>    Burst duration s                 (default 0.001)
///   <downtime_min_s>  Min silence gap s                (default 0.0001)
///   <downtime_max_s>  Max silence gap s                (default 0.001)
class WifiSource : public RfSignalSourceBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("bandwidth_hz"))
      bandwidth_hz_  = sdf->Get<double>("bandwidth_hz");
    if (sdf && sdf->HasElement("burst_time_s"))
      burst_time_s_  = sdf->Get<double>("burst_time_s");
    if (sdf && sdf->HasElement("downtime_min_s"))
      downtime_min_s_ = sdf->Get<double>("downtime_min_s");
    if (sdf && sdf->HasElement("downtime_max_s"))
      downtime_max_s_ = sdf->Get<double>("downtime_max_s");
    return true;
  }

  /// Step burst/silence machine by dt seconds.
  /// Builds tick_segs_: a list of (t_start, in_burst, burst_t) segments
  /// covering [0, dt).  Each segment starts at t_start seconds into the tick;
  /// burst_t is the time offset into the burst array at that segment's start.
  void Advance(double dt) override
  {
    gen_dt_s_ = dt;
    tick_segs_.clear();

    double t    = 0.0;
    double bt   = burst_t_elapsed_s_;
    double silt = silence_t_remaining_s_;

    while (t < dt - 1e-12)
    {
      if (silt > 1e-12)
      {
        tick_segs_.push_back({t, false, 0.0});
        const double advance = std::min(silt, dt - t);
        t    += advance;
        silt -= advance;
      }
      else
      {
        silt = 0.0;
        tick_segs_.push_back({t, true, bt});
        const double remaining = burst_time_s_ - bt;
        const double advance   = std::min(remaining, dt - t);
        t  += advance;
        bt += advance;
        if (bt >= burst_time_s_ - 1e-12)
        {
          bt   = 0.0;
          silt = NextSilenceTime();
        }
      }
    }

    burst_t_elapsed_s_     = bt;
    silence_t_remaining_s_ = silt;
  }

  /// Fill signal.iq by replaying tick_segs_.  Reads burst_ read-only;
  /// does not mutate any state — idempotent across multiple receivers.
  void Generate(RfSignal& signal) override
  {
    if (burst_.empty())
    {
      burst_fs_hz_ = signal.fs_hz;
      burst_       = GenerateBurst(signal.fs_hz);
    }

    if (tick_segs_.empty())
    {
      std::fill(signal.iq.begin(), signal.iq.end(), std::complex<double>{0.0, 0.0});
      return;
    }

    const std::size_t N          = signal.iq.size();
    const double      sample_dt  = 1.0 / signal.fs_hz;
    const uint32_t    L          = static_cast<uint32_t>(burst_.size());

    std::size_t seg_idx = 0;
    for (std::size_t i = 0; i < N; ++i)
    {
      const double t = static_cast<double>(i) * sample_dt;

      while (seg_idx + 1 < tick_segs_.size() &&
             tick_segs_[seg_idx + 1].t_start <= t)
        ++seg_idx;

      const auto& seg = tick_segs_[seg_idx];
      if (!seg.in_burst)
      {
        signal.iq[i] = {0.0, 0.0};
      }
      else
      {
        const double   t_into  = seg.burst_t + (t - seg.t_start);
        const uint32_t idx     = static_cast<uint32_t>(
          std::floor(t_into * burst_fs_hz_)) % L;
        signal.iq[i] = burst_[idx];
      }
    }
  }

private:
  std::vector<std::complex<double>> GenerateBurst(double fs_hz)
  {
    const uint32_t M = kFilterTaps_;
    const uint32_t L = std::max(
      static_cast<uint32_t>(std::round(burst_time_s_ * fs_hz)), M + 1u);

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

  double NextSilenceTime()
  {
    std::uniform_real_distribution<double> dist{downtime_min_s_, downtime_max_s_};
    return dist(rng_);
  }

  // Config
  double bandwidth_hz_  {1e6};
  double burst_time_s_  {0.01};
  double downtime_min_s_{0.0001};
  double downtime_max_s_{0.001};

  static constexpr uint32_t kFilterTaps_{64};

  // Burst array (generated lazily at first fs_hz seen)
  std::vector<std::complex<double>> burst_;
  double burst_fs_hz_{0.0};

  // Live state — advanced only in Advance()
  double burst_t_elapsed_s_   {0.0};
  double silence_t_remaining_s_{0.0};

  // Tick snapshot — written by Advance(), read by Generate()
  struct TickSegment { double t_start; bool in_burst; double burst_t; };
  std::vector<TickSegment> tick_segs_;
  double gen_dt_s_{0.0};

  std::mt19937_64                  rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace rf_gz

REGISTER_RF_SOURCE("wifi", rf_gz::WifiSource);
