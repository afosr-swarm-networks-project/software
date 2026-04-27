#pragma once

#include <cmath>
#include <gz/math/Pose3.hh>
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

  void ApplyTxGain(RfSignal& signal,
                   const gz::math::Pose3d& tx_pose,
                   const gz::math::Pose3d& rx_pose) const
  {
    const double amp = std::pow(10.0, TxGainDbi(tx_pose, rx_pose) / 20.0);
    for (auto& s : signal.iq) s *= amp;
  }

  void ApplyRxGain(RfSignal& signal,
                   const gz::math::Pose3d& tx_pose,
                   const gz::math::Pose3d& rx_pose) const
  {
    const double amp = std::pow(10.0, RxGainDbi(tx_pose, rx_pose) / 20.0);
    for (auto& s : signal.iq) s *= amp;
  }

protected:
  virtual double TxGainDbi(const gz::math::Pose3d& tx_pose,
                            const gz::math::Pose3d& rx_pose) const = 0;
  virtual double RxGainDbi(const gz::math::Pose3d& tx_pose,
                            const gz::math::Pose3d& rx_pose) const = 0;
};

}  // namespace rf_gz
