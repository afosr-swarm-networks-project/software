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
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include "rf_gz/RfComponents.hh"
#include "rf_gz/RfSignal.hh"
#include "rf_gz/transmitters/RfTransmitterFactory.hh"
#include "rf_gz/receivers/RfReceiverFactory.hh"
#include "rf_gz/channels/RfChannelFactory.hh"

namespace rf_gz
{

struct TxEntry { std::string name; gz::sim::Entity link_entity{gz::sim::kNullEntity}; std::unique_ptr<RfTransmitterBase> device; };
struct RxEntry { std::string name; gz::sim::Entity link_entity{gz::sim::kNullEntity}; std::unique_ptr<RfReceiverBase>    device; };

class RfWorldPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
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

    if (elem->HasElement("channel"))
    {
      auto ch_elem = elem->GetElement("channel");
      std::string type = ch_elem->Get<std::string>("type");
      channel_ = RfChannelFactory::Instance().Create(type, ch_elem);
      if (!channel_)
        gzerr << "[rf_gz] Unknown channel type: " << type << "\n";
      else
        gzmsg << "[rf_gz] Using channel model '" << type << "'\n";
    }
  }

  // ── ISystemPreUpdate ──────────────────────────────────────────────────────
  void PreUpdate(
    const gz::sim::UpdateInfo& /*info*/,
    gz::sim::EntityComponentManager& ecm) override
  {
    // RfTransmitterSdf / RfReceiverSdf components are created by RfModelPlugin
    // when a model first enters the ECM (static world tick 0, dynamic spawn
    // on arrival tick). EachNew fires exactly once per link per simulation run.
    ecm.EachNew<gz::sim::components::Link, RfTransmitterSdf>(
      [&](gz::sim::Entity linkEnt,
          gz::sim::components::Link*,
          RfTransmitterSdf* comp) -> bool
      {
        RegisterDevice<TxEntry>(linkEnt, comp->Data(), ecm,
                                transmitters_, RfTransmitterFactory::Instance());
        return true;
      });

    ecm.EachNew<gz::sim::components::Link, RfReceiverSdf>(
      [&](gz::sim::Entity linkEnt,
          gz::sim::components::Link*,
          RfReceiverSdf* comp) -> bool
      {
        RegisterDevice<RxEntry>(linkEnt, comp->Data(), ecm,
                                receivers_, RfReceiverFactory::Instance());
        return true;
      });
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
      auto rx_pose = gz::sim::Link(rx_it->link_entity).WorldPose(ecm);
      if (!rx_pose)
      {
        gzwarn << "[rf_gz] Receiver '" << rx_it->name
               << "' (entity " << rx_it->link_entity
               << ") not found — removing\n";
        rx_it = receivers_.erase(rx_it);
        continue;
      }

      const RxContext rx_ctx{*rx_pose, rx_it->name, sim_time, dt};

      RfSignal signal;
      rx_it->device->PreReceive(signal, rx_ctx);
      if (signal.iq.empty()) { ++rx_it; continue; }

      for (auto tx_it = transmitters_.begin(); tx_it != transmitters_.end(); )
      {
        auto tx_pose = gz::sim::Link(tx_it->link_entity).WorldPose(ecm);
        if (!tx_pose)
        {
          gzwarn << "[rf_gz] Transmitter '" << tx_it->name
                 << "' (entity " << tx_it->link_entity
                 << ") not found — removing\n";
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
  void RegisterDevice(
    gz::sim::Entity linkEnt,
    sdf::ElementPtr devElem,
    const gz::sim::EntityComponentManager& ecm,
    std::vector<Entry>& list,
    Factory& factory)
  {
    const std::string devName = devElem->HasAttribute("name")
      ? devElem->GetAttribute("name")->GetAsString() : "<unnamed>";

    if (!devElem->HasAttribute("type"))
    {
      gzerr << "[rf_gz] Device '" << devName
            << "': missing required 'type' attribute — skipping\n";
      return;
    }

    const std::string rfType = devElem->GetAttribute("type")->GetAsString();
    auto dev = factory.Create(rfType, devElem);
    if (!dev)
    {
      gzerr << "[rf_gz] Device '" << devName
            << "': failed to create type '" << rfType
            << "' (unknown type or invalid SDF) — skipping\n";
      return;
    }

    const auto linkName = ecm.ComponentData<gz::sim::components::Name>(linkEnt);

    Entry e;
    e.name        = devName;
    e.link_entity = linkEnt;
    e.device      = std::move(dev);

    gzmsg << "[rf_gz] Registered '" << devName
          << "' on link '" << linkName.value_or(std::to_string(linkEnt)) << "'\n";

    list.push_back(std::move(e));
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
  rf_gz::RfWorldPlugin::ISystemPreUpdate,
  rf_gz::RfWorldPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(rf_gz::RfWorldPlugin, "rf_gz::RfWorldPlugin")
