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

/// Bluetooth Classic FHSS signal source.
///
/// Matches hitl_gui bluetooth_classic_fhss_emitter().
/// Time divided into slots (1/hop_rate_hz each).  Each slot: with probability
/// active_hop_prob a burst of bandlimited noise is placed at a random offset
/// within the slot at a random frequency within hop_span_hz around 0 Hz.
/// Burst duration is uniform in [300 µs, 580 µs] (clamped to slot length).
///
/// SDF parameters (inside <source type="bluetooth"> or directly on <transmitter>):
///   <hop_rate_hz>     Slots per second             (default 1600)
///   <hop_bw_hz>       Per-burst bandwidth Hz       (default 1e6)
///   <hop_span_hz>     Total frequency span Hz      (default 8e6)
///   <active_hop_prob> Probability a slot is active (default 0.8)
class BluetoothSource : public RfSignalSourceBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("hop_rate_hz"))
      hop_rate_hz_     = sdf->Get<double>("hop_rate_hz");
    if (sdf && sdf->HasElement("hop_bw_hz"))
      hop_bw_hz_       = sdf->Get<double>("hop_bw_hz");
    if (sdf && sdf->HasElement("hop_span_hz"))
      hop_span_hz_     = sdf->Get<double>("hop_span_hz");
    if (sdf && sdf->HasElement("active_hop_prob"))
      active_hop_prob_ = sdf->Get<double>("active_hop_prob");
    return true;
  }

  void Advance(double /*dt*/) override {}

  void Generate(RfSignal& signal) override
  {
    if (slot_samples_ == 0)
    {
      slot_samples_ = static_cast<uint32_t>(std::round(signal.fs_hz / hop_rate_hz_));
      slot_samples_ = std::max(slot_samples_, 1u);
      GenerateNextSlot(signal.fs_hz);
    }

    const uint32_t N = static_cast<uint32_t>(signal.iq.size());

    for (uint32_t i = 0; i < N; ++i)
    {
      if (slot_pos_ >= slot_samples_)
        GenerateNextSlot(signal.fs_hz);

      if (slot_pos_ >= burst_start_ && slot_pos_ < burst_end_)
        signal.iq[i] = burst_buffer_[slot_pos_ - burst_start_];

      ++slot_pos_;
    }
  }

private:
  void GenerateNextSlot(double fs_hz)
  {
    slot_pos_    = 0;
    burst_start_ = 0;
    burst_end_   = 0;
    burst_buffer_.clear();

    if (prob_dist_(rng_) > active_hop_prob_)
      return;

    const double slot_dur_s = static_cast<double>(slot_samples_) / fs_hz;
    const double min_dur    = std::min(0.00030, slot_dur_s);
    const double max_dur    = std::min(0.00058, slot_dur_s);
    const double dur_s      = (max_dur > min_dur)
                              ? std::uniform_real_distribution<double>{min_dur, max_dur}(rng_)
                              : min_dur;

    uint32_t burst_n = static_cast<uint32_t>(std::round(dur_s * fs_hz));
    burst_n = std::max(burst_n, 1u);
    burst_n = std::min(burst_n, slot_samples_);

    const uint32_t max_start = slot_samples_ - burst_n;
    burst_start_ = (max_start > 0)
                   ? std::uniform_int_distribution<uint32_t>{0u, max_start}(rng_)
                   : 0u;
    burst_end_ = burst_start_ + burst_n;

    const double hop_offset = std::uniform_real_distribution<double>{
                                -hop_span_hz_ / 2.0, hop_span_hz_ / 2.0}(rng_);
    burst_buffer_ = bandlimited_complex_noise(
      burst_n, hop_bw_hz_ / 2.0 / fs_hz, rng_, gauss_);

    for (uint32_t i = 0; i < burst_n; ++i)
    {
      double phase = 2.0 * M_PI * hop_offset * static_cast<double>(i) / fs_hz;
      burst_buffer_[i] *= std::complex<double>(std::cos(phase), std::sin(phase));
    }
  }

  double hop_rate_hz_    {1600.0};
  double hop_bw_hz_      {1.0e6};
  double hop_span_hz_    {8.0e6};
  double active_hop_prob_{0.8};

  uint32_t slot_samples_{0};
  uint32_t slot_pos_    {0};
  uint32_t burst_start_ {0};
  uint32_t burst_end_   {0};
  std::vector<std::complex<double>> burst_buffer_;

  std::mt19937_64                        rng_      {std::random_device{}()};
  std::normal_distribution<double>       gauss_    {0.0, 1.0};
  std::uniform_real_distribution<double> prob_dist_{0.0, 1.0};
};

}  // namespace rf_gz

REGISTER_RF_SOURCE("bluetooth", rf_gz::BluetoothSource);
