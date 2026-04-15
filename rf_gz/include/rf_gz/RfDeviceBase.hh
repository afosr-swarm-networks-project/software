#pragma once

#include <memory>
#include <string>
#include <gz/common/Console.hh>
#include <sdf/Element.hh>
#include "rf_gz/antennas/RfAntennaFactory.hh"

namespace rf_gz
{

class RfDeviceBase
{
public:

  /// Parses name and type from element attributes and instantiates the antenna.
  /// Returns false if a required field is missing; factory returns nullptr.
  /// Subclass implementations must call this first.
  virtual bool LoadSdf(sdf::ElementPtr sdf)
  {
    antenna = CreateAntenna(sdf);
    if (!antenna) return false;
    return true;
  }

  virtual ~RfDeviceBase() = default;

protected:
  std::shared_ptr<RfAntennaBase> antenna;
};

}  // namespace rf_gz
