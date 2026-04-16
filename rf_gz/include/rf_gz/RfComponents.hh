#pragma once

#include <ostream>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <sdf/Element.hh>

namespace rf_gz
{

/// Serializer for a single sdf::ElementPtr.
/// Serialization is write-only (logging/debug); deserialization is not needed
/// because these components are always populated in-process by RfModelPlugin.
struct RfSdfElemSerializer
{
  static bool Serialize(std::ostream& out, const sdf::ElementPtr& data)
  {
    if (data) out << data->ToString("");
    return true;
  }

  static bool Deserialize(std::istream& /*in*/, sdf::ElementPtr& /*data*/)
  {
    return false;
  }
};

/// Component attached to a link entity by RfModelPlugin.
/// A link may carry at most one transmitter.
using RfTransmitterSdf = gz::sim::components::Component<
  sdf::ElementPtr, class RfTransmitterSdfTag, RfSdfElemSerializer>;

/// Component attached to a link entity by RfModelPlugin.
/// A link may carry at most one receiver.
using RfReceiverSdf = gz::sim::components::Component<
  sdf::ElementPtr, class RfReceiverSdfTag, RfSdfElemSerializer>;

GZ_SIM_REGISTER_COMPONENT("rf_gz.RfTransmitterSdf", RfTransmitterSdf)
GZ_SIM_REGISTER_COMPONENT("rf_gz.RfReceiverSdf",    RfReceiverSdf)

}  // namespace rf_gz
