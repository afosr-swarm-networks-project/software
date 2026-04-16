#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include "rf_gz/RfComponents.hh"

namespace rf_gz
{

/// Model-level plugin that reads <transmitter> and <receiver> elements from
/// its own SDF element and attaches them as ECM components on the target link.
/// Each link may carry at most one transmitter and at most one receiver.
/// The link is identified by the required link= attribute on each device tag.
/// RfWorldPlugin discovers those components via EachNew<>.
class RfModelPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure
{
public:
  void Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& /*eventMgr*/) override
  {
    // Clone to get a mutable ElementPtr — GetElement() is non-const.
    auto elem = sdf->Clone();
    gz::sim::Model model(entity);

    AttachDevices<RfTransmitterSdf>(elem, "transmitter", model, ecm);
    AttachDevices<RfReceiverSdf>   (elem, "receiver",    model, ecm);
  }

private:
  template<typename Component>
  static void AttachDevices(
    sdf::ElementPtr pluginElem,
    const std::string& tag,
    gz::sim::Model& model,
    gz::sim::EntityComponentManager& ecm)
  {
    if (!pluginElem->HasElement(tag)) return;

    for (auto devElem = pluginElem->GetElement(tag);
         devElem;
         devElem = devElem->GetNextElement(tag))
    {
      const std::string devName = devElem->HasAttribute("name")
        ? devElem->GetAttribute("name")->GetAsString() : "<unnamed>";

      if (!devElem->HasAttribute("link"))
      {
        gzerr << "[rf_gz] RfModelPlugin: " << tag << " '" << devName
              << "' missing required 'link' attribute — skipping\n";
        continue;
      }

      const std::string linkName =
        devElem->GetAttribute("link")->GetAsString();

      const gz::sim::Entity linkEnt = model.LinkByName(ecm, linkName);
      if (linkEnt == gz::sim::kNullEntity)
      {
        gzerr << "[rf_gz] RfModelPlugin: link '" << linkName
              << "' not found in ECM — skipping\n";
        continue;
      }

      if (ecm.Component<Component>(linkEnt))
      {
        gzerr << "[rf_gz] RfModelPlugin: link '" << linkName
              << "' already has a " << tag
              << " — only one per link is allowed, skipping '" << devName << "'\n";
        continue;
      }

      ecm.CreateComponent(linkEnt, Component(devElem));
      gzmsg << "[rf_gz] Attached " << tag << " '" << devName
            << "' to link '" << linkName << "'\n";
    }
  }
};

}  // namespace rf_gz

GZ_ADD_PLUGIN(
  rf_gz::RfModelPlugin,
  gz::sim::System,
  rf_gz::RfModelPlugin::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(rf_gz::RfModelPlugin, "rf_gz::RfModelPlugin")
