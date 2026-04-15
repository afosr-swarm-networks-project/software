#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include "rf_gz/RfSignal.hh"
#include "rf_gz/transmitters/RfTransmitterFactory.hh"
#include "rf_gz/receivers/RfReceiverFactory.hh"
#include "rf_gz/channels/RfChannelFactory.hh"

namespace rf_gz
{

/// Gazebo model/link attachment for an RF device.
/// Caches the link entity on first WorldPose() call to avoid repeated ECM scans.
struct Placement
{
  std::string model;
  std::string link;
  gz::sim::Entity link_entity{gz::sim::kNullEntity};

  /// Resolves and caches the link entity on first call.
  /// Returns the world pose, or std::nullopt if model/link cannot be found.
  std::optional<gz::math::Pose3d> WorldPose(
    const gz::sim::EntityComponentManager& ecm)
  {
    if (link_entity == gz::sim::kNullEntity)
    {
      gz::sim::Entity model_ent = ecm.EntityByComponents(
        gz::sim::components::Name(model),
        gz::sim::components::Model());
      if (model_ent == gz::sim::kNullEntity)
        return std::nullopt;

      link_entity = gz::sim::Model(model_ent).LinkByName(ecm, link);
      if (link_entity == gz::sim::kNullEntity)
        return std::nullopt;
    }
    return gz::sim::Link(link_entity).WorldPose(ecm);
  }
};

struct TxEntry { std::string name; Placement placement; std::unique_ptr<RfTransmitterBase> device; };
struct RxEntry { std::string name; Placement placement; std::unique_ptr<RfReceiverBase>    device; };

class RfWorldPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  // ── ISystemConfigure ───────────────────────────────────────────────────────
  void Configure(
    const gz::sim::Entity& /*entity*/,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& /*ecm*/,
    gz::sim::EventManager& /*eventMgr*/) override
  {
    auto elem = sdf->Clone();

    if (elem->HasElement("awgn_dbm"))
    {
      awgn_dbm_ = elem->Get<double>("awgn_dbm");
      gzmsg << "[rf_gz] Environmental AWGN " << awgn_dbm_ << " dBm\n";
    }

    if (elem->HasElement("channel")) {
      auto ch_elem = elem->GetElement("channel");
      std::string type = ch_elem->Get<std::string>("type");
      channel_ = RfChannelFactory::Instance().Create(type, ch_elem);
      if (!channel_)
        gzerr << "[rf_gz] Unknown channel type: " << type << "\n";
      else
        gzmsg << "[rf_gz] Using channel model '" << type << "'\n";
    }

    ParseDevices(elem, "transmitter", transmitters_,
                 RfTransmitterFactory::Instance());
    ParseDevices(elem, "receiver", receivers_,
                 RfReceiverFactory::Instance());
  }

  // ── ISystemPostUpdate ─────────────────────────────────────────────────────
  void PostUpdate(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm) override
  {
    if (info.paused) return;

    const double sim_time = std::chrono::duration<double>(info.simTime).count();
    const double dt       = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0.0) return;

    for (auto rx_it = receivers_.begin(); rx_it != receivers_.end(); )
    {
      auto rx_pose = rx_it->placement.WorldPose(ecm);
      if (!rx_pose)
      {
        gzwarn << "[rf_gz] Receiver '" << rx_it->name
               << "': model '" << rx_it->placement.model
               << "' or link '" << rx_it->placement.link
               << "' not found — removing\n";
        rx_it = receivers_.erase(rx_it);
        continue;
      }

      const RxContext rx_ctx{*rx_pose, rx_it->name, sim_time, dt};

      RfSignal signal;
      rx_it->device->PreReceive(signal, rx_ctx);
      if (signal.iq.empty()) { ++rx_it; continue; }

      for (auto tx_it = transmitters_.begin(); tx_it != transmitters_.end(); )
      {
        auto tx_pose = tx_it->placement.WorldPose(ecm);
        if (!tx_pose)
        {
          gzwarn << "[rf_gz] Transmitter '" << tx_it->name
                 << "': model '" << tx_it->placement.model
                 << "' or link '" << tx_it->placement.link
                 << "' not found — removing\n";
          tx_it = transmitters_.erase(tx_it);
          continue;
        }

        const TxContext tx_ctx{*tx_pose, tx_it->name};
        tx_it->device->Transmit(signal, tx_ctx, rx_ctx);

        if (channel_)
          channel_->Apply(signal, tx_ctx, rx_ctx);

        rx_it->device->Receive(signal, tx_ctx, rx_ctx);
        ++tx_it;
      }

      // ── Fill signal.iq with environmental AWGN before PostReceive ────────
      FillAwgn(signal);

      rx_it->device->PostReceive(signal, rx_ctx);
      ++rx_it;
    }
  }

private:
  template<typename Entry, typename Factory>
  void ParseDevices(
    sdf::ElementPtr rootElem,
    const std::string& tag,
    std::vector<Entry>& list,
    Factory& factory)
  {
    if (!rootElem->HasElement(tag)) return;

    for (auto devElem = rootElem->GetElement(tag);
         devElem;
         devElem = devElem->GetNextElement(tag))
    {
      const std::string devName = devElem->HasAttribute("name")
        ? devElem->GetAttribute("name")->GetAsString() : "<unnamed>";

      if (!devElem->HasAttribute("type"))
      {
        gzerr << "[rf_gz] " << tag << " '" << devName
              << "': missing required 'type' attribute — skipping\n";
        continue;
      }

      const std::string rfType = devElem->GetAttribute("type")->GetAsString();
      auto dev = factory.Create(rfType, devElem);
      if (!dev)
      {
        gzerr << "[rf_gz] " << tag << " '" << devName
              << "': failed to create type '" << rfType
              << "' (unknown type or invalid SDF) — skipping\n";
        continue;
      }

      if (!devElem->HasAttribute("model") || !devElem->HasAttribute("link"))
      {
        gzerr << "[rf_gz] " << tag << " '" << devName
              << "': missing required 'model' or 'link' attribute — skipping\n";
        continue;
      }

      Entry e;
      e.name            = devName;
      e.placement.model = devElem->GetAttribute("model")->GetAsString();
      e.placement.link  = devElem->GetAttribute("link")->GetAsString();
      e.device          = std::move(dev);

      gzmsg << "[rf_gz] Registered " << tag << " '" << devName
            << "' at model='" << e.placement.model
            << "' link='" << e.placement.link << "'\n";

      list.push_back(std::move(e));
    }
  }

  /// Fills signal.iq with white Gaussian noise at awgn_dbm_ power.
  /// signal.iq is already sized and zeroed; left as-is if AWGN is off.
  void FillAwgn(RfSignal& signal)
  {
    if (awgn_dbm_ <= -190.0) return;  // effectively off
    const double std = std::sqrt(std::pow(10.0, awgn_dbm_ / 10.0) / 2.0);
    for (auto& s : signal.iq)
      s = std::complex<double>(gauss_(rng_) * std, gauss_(rng_) * std);
  }

  std::vector<TxEntry>                transmitters_;
  std::vector<RxEntry>                receivers_;
  std::unique_ptr<RfChannelModelBase> channel_;

  double awgn_dbm_{-200.0};  ///< Environmental noise floor dBm (default: off)
  std::mt19937_64                  rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace rf_gz

GZ_ADD_PLUGIN(
  rf_gz::RfWorldPlugin,
  gz::sim::System,
  rf_gz::RfWorldPlugin::ISystemConfigure,
  rf_gz::RfWorldPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(rf_gz::RfWorldPlugin, "rf_gz::RfWorldPlugin")
