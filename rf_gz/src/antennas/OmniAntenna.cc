#include <sdf/Element.hh>
#include "rf_gz/antennas/RfAntennaBase.hh"
#include "rf_gz/antennas/RfAntennaFactory.hh"
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

/// Omnidirectional antenna: uniform gain in every direction.
///
/// SDF parameters (inside <antenna type="omni">):
///   <gain_dbi>   Isotropic gain, dBi   (default 0.0)
class OmniAntenna : public RfAntennaBase
{
public:

  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("gain_dbi"))
      gain_dbi_ = sdf->Get<double>("gain_dbi");
    return true;
  }

protected:
  double TxGainDbi(const TxContext&, const RxContext&) const override
  {
    return gain_dbi_;
  }

  double RxGainDbi(const TxContext&, const RxContext&) const override
  {
    return gain_dbi_;
  }

private:
  double gain_dbi_{0.0};
};

}  // namespace rf_gz

REGISTER_RF_ANTENNA("omni", rf_gz::OmniAntenna);
