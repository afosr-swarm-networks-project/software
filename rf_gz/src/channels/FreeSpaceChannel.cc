#include <cmath>
#include <complex>
#include <vector>

#include "rf_gz/channels/RfChannelFactory.hh"

namespace rf_gz
{

/// Free-space path loss (Friis).
/// Amplitude scale = c / (4π · d · cf_hz)
class FreeSpaceChannel : public RfChannelModelBase
{
public:
  void Apply(RfSignal& signal, const TxContext& tx, const RxContext& rx) override
  {
    constexpr double c = 2.998e8;

    const double dx = rx.rx_pose.Pos().X() - tx.tx_pose.Pos().X();
    const double dy = rx.rx_pose.Pos().Y() - tx.tx_pose.Pos().Y();
    const double dz = rx.rx_pose.Pos().Z() - tx.tx_pose.Pos().Z();
    double d = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (d < 1e-3) d = 1e-3;

    const double cf    = signal.cf_hz > 0.0 ? signal.cf_hz : 1.0;
    const double scale = c / (4.0 * M_PI * d * cf);
    for (auto& s : signal.iq)
      s *= scale;
  }
};

}  // namespace rf_gz

REGISTER_RF_CHANNEL("free_space", rf_gz::FreeSpaceChannel);
