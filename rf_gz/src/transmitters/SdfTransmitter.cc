#include <cmath>
#include <complex>
#include <memory>
#include <vector>

#include <gz/common/Console.hh>
#include <sdf/Element.hh>

#include "rf_gz/transmitters/RfTransmitterFactory.hh"
#include "rf_gz/sources/RfSignalSourceFactory.hh"

namespace rf_gz
{

/// SDF-configured transmitter.
///
/// Delegates baseband generation to a RfSignalSourceBase selected by the
/// required <source type="..."> child element.  Output is normalised to
/// power_dbm and antenna TX gain is applied before returning.
///
/// SDF parameters (on <transmitter type="sdf">):
///   <cf_hz>       Carrier frequency Hz        (parsed by RfTransmitterBase)
///   <power_dbm>   Transmit power dBm          (default 0.0)
///   <source type="wifi|lora|fhss|fm|bluetooth">
///     ...source-specific parameters...
///   </source>
///   <antenna type="omni|yagi" model="..." link="...">
///     ...antenna-specific parameters...
///   </antenna>
class SdfTransmitter : public RfTransmitterBase
{
public:
  double power_dbm{0.0};

  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (!RfTransmitterBase::LoadSdf(sdf)) return false;
    if (!source)
    {
      gzerr << "[rf_gz] SdfTransmitter: missing required <source> element\n";
      return false;
    }
    if (sdf->HasElement("power_dbm"))
      power_dbm = sdf->Get<double>("power_dbm");
    return true;
  }

  void Transmit(RfSignal& signal, const TxContext& tx, const RxContext& rx,
                const gz::sim::UpdateInfo& /*info*/) override
  {
    signal.cf_hz = cf_hz;
    source->GenerateBaseband(signal);
    Normalize(signal.iq);
    antenna->ApplyTxGain(signal, tx, rx);
  }

private:

  void Normalize(std::vector<std::complex<double>>& iq) const
  {
    if (iq.empty()) return;

    double rms = 0.0;
    for (const auto& s : iq) rms += std::norm(s);
    rms = std::sqrt(rms / static_cast<double>(iq.size()));
    if (rms <= 1e-12) return;

    const double power_amp = std::pow(10.0, power_dbm / 20.0);
    const double scale     = power_amp / rms;
    for (auto& s : iq) s *= scale;

    
  }
};

}  // namespace rf_gz

REGISTER_RF_TRANSMITTER("sdf", rf_gz::SdfTransmitter);
