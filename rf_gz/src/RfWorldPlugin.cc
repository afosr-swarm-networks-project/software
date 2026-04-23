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
    if (std::chrono::duration<double>(info.dt).count() <= 0.0) return;

    // ── Phase 1: build per-TX signal fns (one copy per TX, shared across RXs) ──
    struct TxData { gz::math::Pose3d pose; SignalFn fn; };
    std::vector<TxData> txd;
    txd.reserve(transmitters_.size());

    for (auto tx_it = transmitters_.begin(); tx_it != transmitters_.end(); )
    {
      auto tx_pose = gz::sim::Link(tx_it->link_entity).WorldPose(ecm);
      if (!tx_pose)
      {
        gzwarn << "[rf_gz] Transmitter '" << tx_it->name
               << "' (entity " << tx_it->link_entity << ") not found — removing\n";
        tx_it = transmitters_.erase(tx_it);
        continue;
      }
      txd.push_back({*tx_pose, tx_it->device->Transmit(info)});
      ++tx_it;
    }

    // ── Phase 2 + 3: per-RX wrapping and receive ──────────────────────────
    for (auto rx_it = receivers_.begin(); rx_it != receivers_.end(); )
    {
      auto rx_pose = gz::sim::Link(rx_it->link_entity).WorldPose(ecm);
      if (!rx_pose)
      {
        gzwarn << "[rf_gz] Receiver '" << rx_it->name
               << "' (entity " << rx_it->link_entity << ") not found — removing\n";
        rx_it = receivers_.erase(rx_it);
        continue;
      }

      std::vector<SignalFn> per_rx;
      per_rx.reserve(txd.size());

      // transmitters_ and txd are kept in the same order after Phase 1 erases
      std::size_t idx = 0;
      for (auto& tx : transmitters_)
      {
        SignalFn fn = txd[idx].fn;  // copy — each RX gets its own fn instance
        fn = tx.device->WrapTxGain(std::move(fn), txd[idx].pose, *rx_pose);
        if (channel_)
          fn = channel_->Wrap(std::move(fn), txd[idx].pose, *rx_pose, info);
        fn = rx_it->device->WrapRxGain(std::move(fn), txd[idx].pose, *rx_pose);
        per_rx.push_back(std::move(fn));
        ++idx;
      }

      SignalFn combined = rx_it->device->Combine(std::move(per_rx));
      rx_it->device->Receive(WrapAwgn(std::move(combined)), info);
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

    dev->name = devName;
    dev->OnNameSet();

    Entry e;
    e.name        = devName;
    e.link_entity = linkEnt;
    e.device      = std::move(dev);

    gzmsg << "[rf_gz] Registered '" << devName
          << "' on link '" << linkName.value_or(std::to_string(linkEnt)) << "'\n";

    list.push_back(std::move(e));
  }

  /// Wraps inner to add environmental AWGN after inner completes.
  SignalFn WrapAwgn(SignalFn inner)
  {
    return [this, inner = std::move(inner)](RfSignal& s) mutable {
      inner(s);
      if (awgn_dbm_ <= -190.0) return;
      const double std = std::sqrt(std::pow(10.0, awgn_dbm_ / 10.0) / 2.0);
      for (auto& x : s.iq)
        x += std::complex<double>(gauss_(rng_) * std, gauss_(rng_) * std);
    };
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
