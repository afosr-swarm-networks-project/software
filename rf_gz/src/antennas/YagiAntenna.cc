#include <cmath>
#include <sdf/Element.hh>
#include "rf_gz/antennas/RfAntennaBase.hh"
#include "rf_gz/antennas/RfAntennaFactory.hh"
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

/// Yagi directional antenna pattern.
///
/// Pattern model (matches hitl_gui yagi_gain_db()):
///   gain(Δθ) = g_peak + 10·log10(max(cos(Δθ), 0)^n)
///   n        = log(0.5) / log(cos(hpbw/2))
///   floored at sidelobe_floor_db
///
/// For TX: Δθ is the angle from the TX antenna's pointing direction (yaw)
///         to the direction of the receiver.
/// For RX: Δθ is the angle from the RX antenna's pointing direction (yaw)
///         to the direction of the transmitter.
///
/// SDF parameters (inside <antenna type="yagi">):
///   <g_peak_dbi>          Peak gain, dBi             (default 13.0)
///   <hpbw_deg>            Half-power beamwidth, deg   (default 40.0)
///   <sidelobe_floor_db>   Sidelobe floor, dB          (default -20.0)
class YagiAntenna : public RfAntennaBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (!sdf) return true;
    if (sdf->HasElement("g_peak_dbi"))
      g_peak_dbi_ = sdf->Get<double>("g_peak_dbi");
    if (sdf->HasElement("hpbw_deg"))
      hpbw_deg_ = sdf->Get<double>("hpbw_deg");
    if (sdf->HasElement("sidelobe_floor_db"))
      sidelobe_floor_db_ = sdf->Get<double>("sidelobe_floor_db");
    return true;
  }

protected:
  double TxGainDbi(const TxContext& tx, const RxContext& rx) const override
  {
    const double dx      = rx.rx_pose.Pos().X() - tx.tx_pose.Pos().X();
    const double dy      = rx.rx_pose.Pos().Y() - tx.tx_pose.Pos().Y();
    const double bearing = std::atan2(dy, dx);
    const double pointing = tx.tx_pose.Rot().Yaw();
    const double delta   = std::remainder(bearing - pointing, 2.0 * M_PI);
    return PatternDbi(delta);
  }

  double RxGainDbi(const TxContext& tx, const RxContext& rx) const override
  {
    const double dx      = tx.tx_pose.Pos().X() - rx.rx_pose.Pos().X();
    const double dy      = tx.tx_pose.Pos().Y() - rx.rx_pose.Pos().Y();
    const double bearing = std::atan2(dy, dx);
    const double pointing = rx.rx_pose.Rot().Yaw();
    const double delta   = std::remainder(bearing - pointing, 2.0 * M_PI);
    return PatternDbi(delta);
  }


private:
  double PatternDbi(double delta_theta_rad) const
  {
    const double hpbw_rad = hpbw_deg_ * M_PI / 180.0;
    const double n        = std::log(0.5) / std::log(std::cos(hpbw_rad / 2.0));
    const double cos_val  = std::max(std::cos(delta_theta_rad), 0.0);
    const double gain     = g_peak_dbi_
                            + 10.0 * std::log10(std::max(std::pow(cos_val, n), 1e-12));
    return std::max(gain, sidelobe_floor_db_);
  }

  double g_peak_dbi_       {13.0};
  double hpbw_deg_         {40.0};
  double sidelobe_floor_db_{-20.0};
};

}  // namespace rf_gz

REGISTER_RF_ANTENNA("yagi", rf_gz::YagiAntenna);
