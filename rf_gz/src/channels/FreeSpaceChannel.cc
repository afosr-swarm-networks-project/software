#include <cmath>
#include <complex>

#include "rf_gz/channels/RfChannelFactory.hh"

namespace rf_gz
{

/// Free-space path loss (Friis).
/// Distance is captured at wrap time; FSPL scale uses signal.cf_hz (TX CF).
class FreeSpaceChannel : public RfChannelModelBase
{
public:
  SignalFn Wrap(SignalFn inner,
                const gz::math::Pose3d& tx_pose,
                const gz::math::Pose3d& rx_pose,
                const gz::sim::UpdateInfo&) override
  {
    const double dx = rx_pose.Pos().X() - tx_pose.Pos().X();
    const double dy = rx_pose.Pos().Y() - tx_pose.Pos().Y();
    const double dz = rx_pose.Pos().Z() - tx_pose.Pos().Z();
    double d = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (d < 1e-3) d = 1e-3;

    return [inner = std::move(inner), d](RfSignal& s) mutable {
      inner(s);
      constexpr double c = 2.998e8;
      const double cf    = s.cf_hz > 0.0 ? s.cf_hz : 1.0;
      const double scale = c / (4.0 * M_PI * d * cf);
      for (auto& sample : s.iq) sample *= scale;
    };
  }
};

}  // namespace rf_gz

REGISTER_RF_CHANNEL("free_space", rf_gz::FreeSpaceChannel);
