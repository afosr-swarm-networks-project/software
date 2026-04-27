#include <cmath>
#include <complex>
#include <random>
#include <vector>

#include <sdf/Element.hh>

#include "rf_gz/RfSignal.hh"
#include "rf_gz/sources/RfSignalSourceFactory.hh"

namespace rf_gz
{

/// LoRa signal source: continuous cyclic chirp sequence.
///
/// Matches hitl_gui complex_chirp() + lora_mod() + lora_emitter().
/// Phase resets to 0 at each symbol boundary (matches Python complex_chirp()).
///
/// SDF parameters (inside <source type="lora"> or directly on <transmitter>):
///   <bandwidth_hz>  Chirp bandwidth Hz       (default 125e3)
///   <sf>            Spreading factor 6–12    (default 7)
///   <num_chirps>    Symbols in sequence      (default 48)
class LoraSource : public RfSignalSourceBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("bandwidth_hz"))
      bandwidth_hz_ = sdf->Get<double>("bandwidth_hz");
    if (sdf && sdf->HasElement("sf"))
      sf_ = sdf->Get<int>("sf");
    if (sdf && sdf->HasElement("num_chirps"))
      num_chirps_ = sdf->Get<int>("num_chirps");

    sf_ = std::max(6, std::min(12, sf_));
    GenerateSymbolSequence();
    return true;
  }

  void Advance(double /*dt*/) override {}

  void Generate(RfSignal& signal) override
  {
    if (N_sym_ == 0)
    {
      double Ts   = std::pow(2.0, sf_) / bandwidth_hz_;
      N_sym_      = static_cast<uint32_t>(std::round(signal.fs_hz * Ts));
      chirp_rate_ = bandwidth_hz_ * bandwidth_hz_ / std::pow(2.0, sf_);
    }

    const uint32_t N = static_cast<uint32_t>(signal.iq.size());

    for (uint32_t i = 0; i < N; ++i)
    {
      double t_sym  = static_cast<double>(sym_pos_) / signal.fs_hz;
      double f_base = chirp_rate_ * t_sym;
      double freq;
      if (up_seq_[sym_idx_])
        freq = std::fmod(k0_seq_[sym_idx_] + f_base, bandwidth_hz_);
      else
        freq = std::fmod(bandwidth_hz_ - std::fmod(k0_seq_[sym_idx_] + f_base, bandwidth_hz_),
                         bandwidth_hz_);

      double f_inst = freq - bandwidth_hz_ / 2.0;
      phase_acc_ += 2.0 * M_PI * f_inst / signal.fs_hz;
      signal.iq[i] = std::complex<double>(std::cos(phase_acc_), std::sin(phase_acc_));

      if (++sym_pos_ >= N_sym_)
      {
        sym_pos_ = 0;
        sym_idx_ = (sym_idx_ + 1) % static_cast<uint32_t>(num_chirps_);
        phase_acc_ = 0.0;
      }
    }
  }

private:
  void GenerateSymbolSequence()
  {
    std::uniform_real_distribution<double> k0_dist(0.0, bandwidth_hz_);
    std::uniform_int_distribution<int>     pol_dist(0, 1);

    k0_seq_.resize(num_chirps_);
    up_seq_.resize(num_chirps_);
    for (int i = 0; i < num_chirps_; ++i)
    {
      k0_seq_[i] = k0_dist(rng_);
      up_seq_[i] = pol_dist(rng_) == 1;
    }
  }

  double   bandwidth_hz_{125e3};
  int      sf_          {7};
  int      num_chirps_  {48};

  uint32_t N_sym_      {0};
  double   chirp_rate_ {0.0};
  double   phase_acc_  {0.0};
  uint32_t sym_pos_    {0};
  uint32_t sym_idx_    {0};

  std::vector<double> k0_seq_;
  std::vector<bool>   up_seq_;

  std::mt19937_64 rng_{std::random_device{}()};
};

}  // namespace rf_gz

REGISTER_RF_SOURCE("lora", rf_gz::LoraSource);
