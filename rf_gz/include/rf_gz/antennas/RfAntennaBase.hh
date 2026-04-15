#pragma once

#include <cmath>
#include <complex>
#include <vector>
#include <sdf/Element.hh>
#include "rf_gz/RfSignal.hh"

namespace rf_gz
{

/// Abstract base class for all antenna models.
class RfAntennaBase
{
public:
  virtual ~RfAntennaBase() = default;

  /// Load antenna parameters from the <antenna> SDF element.
  /// Returns false if a required field is missing; factory returns nullptr.
  virtual bool LoadSdf(sdf::ElementPtr sdf) = 0;

  void ApplyTxGain(RfSignal& signal, const TxContext& tx, const RxContext& rx) const
  {
    const double amp = std::pow(10.0, TxGainDbi(tx, rx) / 20.0);
    for (auto& s : signal.iq) s *= amp;
  }

  void ApplyRxGain(RfSignal& signal, const TxContext& tx, const RxContext& rx) const
  {
    const double amp = std::pow(10.0, RxGainDbi(tx, rx) / 20.0);
    for (auto& s : signal.iq) s *= amp;
  }

protected:
  virtual double TxGainDbi(const TxContext& tx, const RxContext& rx) const = 0;
  virtual double RxGainDbi(const TxContext& tx, const RxContext& rx) const = 0;
};

}  // namespace rf_gz
