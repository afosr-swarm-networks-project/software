#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <sdf/Element.hh>
#include "rf_gz/channels/RfChannelModelBase.hh"

namespace rf_gz
{

class RfChannelFactory
{
public:
  using CreatorFn =
    std::function<std::unique_ptr<RfChannelModelBase>(sdf::ElementPtr)>;

  static RfChannelFactory& Instance()
  {
    static RfChannelFactory factory;
    return factory;
  }

  void Register(const std::string& type, CreatorFn fn)
  {
    creators_[type] = std::move(fn);
  }

  std::unique_ptr<RfChannelModelBase>
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

#define _RF_CH_CONCAT_(a, b) a##b
#define _RF_CH_CONCAT(a, b)  _RF_CH_CONCAT_(a, b)

#define REGISTER_RF_CHANNEL(type_str, ClassName)                                   \
  static const bool _RF_CH_CONCAT(_rf_reg_ch_, __COUNTER__) = []() {              \
    ::rf_gz::RfChannelFactory::Instance().Register(                                \
      type_str,                                                                    \
      [](::sdf::ElementPtr sdf)                                                    \
        -> std::unique_ptr<::rf_gz::RfChannelModelBase> {                          \
        auto ch = std::make_unique<ClassName>();                                   \
        if (!ch->LoadSdf(sdf)) return nullptr;                                     \
        return ch;                                                                 \
      });                                                                          \
    return true;                                                                   \
  }()
