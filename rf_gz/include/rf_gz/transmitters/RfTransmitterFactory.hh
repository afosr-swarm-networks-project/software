#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <sdf/Element.hh>
#include "rf_gz/transmitters/RfTransmitterBase.hh"

namespace rf_gz
{

class RfTransmitterFactory
{
public:
  using CreatorFn =
    std::function<std::unique_ptr<RfTransmitterBase>(sdf::ElementPtr)>;

  static RfTransmitterFactory& Instance()
  {
    static RfTransmitterFactory factory;
    return factory;
  }

  void Register(const std::string& type, CreatorFn fn)
  {
    creators_[type] = std::move(fn);
  }

  std::unique_ptr<RfTransmitterBase>
  Create(const std::string& type, sdf::ElementPtr sdf) const
  {
    auto it = creators_.find(type);
    return (it == creators_.end()) ? nullptr : it->second(sdf);
  }

private:
  std::map<std::string, CreatorFn> creators_;
};

}  // namespace rf_gz

// ── Registration macro ────────────────────────────────────────────────────────

#define _RF_TX_CONCAT_(a, b) a##b
#define _RF_TX_CONCAT(a, b)  _RF_TX_CONCAT_(a, b)

#define REGISTER_RF_TRANSMITTER(type_str, ClassName)                               \
  static const bool _RF_TX_CONCAT(_rf_reg_tx_, __COUNTER__) = []() {              \
    ::rf_gz::RfTransmitterFactory::Instance().Register(                            \
      type_str,                                                                    \
      [](::sdf::ElementPtr sdf)                                                    \
        -> std::unique_ptr<::rf_gz::RfTransmitterBase> {                           \
        auto dev = std::make_unique<ClassName>();                                  \
        if (!dev->LoadSdf(sdf)) return nullptr;                                    \
        return dev;                                                                \
      });                                                                          \
    return true;                                                                   \
  }()
