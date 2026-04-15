#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <sdf/Element.hh>
#include "rf_gz/antennas/RfAntennaBase.hh"

namespace rf_gz
{

class RfAntennaFactory
{
public:
  using CreatorFn =
    std::function<std::unique_ptr<RfAntennaBase>(sdf::ElementPtr)>;

  static RfAntennaFactory& Instance()
  {
    static RfAntennaFactory factory;
    return factory;
  }

  void Register(const std::string& type, CreatorFn fn)
  {
    creators_[type] = std::move(fn);
  }

  std::unique_ptr<RfAntennaBase>
  Create(const std::string& type, sdf::ElementPtr sdf) const
  {
    auto it = creators_.find(type);
    return (it == creators_.end()) ? nullptr : it->second(sdf);
  }

private:
  std::map<std::string, CreatorFn> creators_;
};

/// Instantiate the correct antenna from an <antenna type="..."> child of
/// parent_sdf.  Falls back to type "omni" when the element is absent or the
/// type is unrecognised.
inline std::shared_ptr<RfAntennaBase>
CreateAntenna(sdf::ElementPtr parent_sdf)
{
  sdf::ElementPtr ant_sdf;
  std::string     type = "omni";

  if (parent_sdf && parent_sdf->HasElement("antenna"))
  {
    ant_sdf = parent_sdf->GetElement("antenna");
    if (ant_sdf->HasAttribute("type"))
      type = ant_sdf->GetAttribute("type")->GetAsString();
  }

  auto ant = RfAntennaFactory::Instance().Create(type, ant_sdf);
  if (!ant)
    ant = RfAntennaFactory::Instance().Create("omni", ant_sdf);  // unknown type → omni
  return ant;
}

}  // namespace rf_gz

// ── Registration macro ────────────────────────────────────────────────────────

#define _RF_ANT_CONCAT_(a, b) a##b
#define _RF_ANT_CONCAT(a, b)  _RF_ANT_CONCAT_(a, b)

#define REGISTER_RF_ANTENNA(type_str, AntennaClass)                                \
  static const bool _RF_ANT_CONCAT(_rf_reg_ant_, __COUNTER__) = []() {            \
    ::rf_gz::RfAntennaFactory::Instance().Register(                                \
      type_str,                                                                    \
      [](::sdf::ElementPtr sdf)                                                    \
        -> std::unique_ptr<::rf_gz::RfAntennaBase> {                               \
        auto ant = std::make_unique<AntennaClass>();                               \
        if (!ant->LoadSdf(sdf)) return nullptr;                                    \
        return ant;                                                                \
      });                                                                          \
    return true;                                                                   \
  }()
