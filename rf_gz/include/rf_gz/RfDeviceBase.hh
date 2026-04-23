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
  std::string name;  ///< Set by RfWorldPlugin immediately after registration

  /// Parses name and type from element attributes and instantiates the antenna.
  /// Returns false if a required field is missing; factory returns nullptr.
  /// Subclass implementations must call this first.
  virtual bool LoadSdf(sdf::ElementPtr sdf)
  {
    antenna = CreateAntenna(sdf);
    if (!antenna) return false;
    return true;
  }

  /// Called by RfWorldPlugin after setting name.  Subclasses may override to
  /// forward the name to owned objects (e.g. sink).
  virtual void OnNameSet() {}

  virtual ~RfDeviceBase() = default;

protected:
  std::shared_ptr<RfAntennaBase> antenna;
};

}  // namespace rf_gz
