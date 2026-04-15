#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <sdf/Element.hh>
#include "rf_gz/sources/RfSignalSourceBase.hh"

namespace rf_gz
{

class RfSignalSourceFactory
{
public:
  using CreatorFn =
    std::function<std::unique_ptr<RfSignalSourceBase>(sdf::ElementPtr)>;

  static RfSignalSourceFactory& Instance()
  {
    static RfSignalSourceFactory factory;
    return factory;
  }

  void Register(const std::string& type, CreatorFn fn)
  {
    creators_[type] = std::move(fn);
  }

  std::unique_ptr<RfSignalSourceBase>
  Create(const std::string& type, sdf::ElementPtr sdf) const
  {
    auto it = creators_.find(type);
    return (it == creators_.end()) ? nullptr : it->second(sdf);
  }

private:
  std::map<std::string, CreatorFn> creators_;
};

/// Instantiate the correct signal source from SDF.
///
/// Reads type from a <source type="..."> child element of parent_sdf.
/// Falls back to the "type" attribute of parent_sdf itself so that SDF files
/// without a <source> element continue to work (backward compatible).
inline std::unique_ptr<RfSignalSourceBase>
CreateSignalSource(sdf::ElementPtr parent_sdf)
{
  sdf::ElementPtr src_sdf;
  std::string     type;

  if (parent_sdf && parent_sdf->HasElement("source"))
  {
    src_sdf = parent_sdf->GetElement("source");
    if (src_sdf->HasAttribute("type"))
      type = src_sdf->GetAttribute("type")->GetAsString();
  }

  if (type.empty() && parent_sdf && parent_sdf->HasAttribute("type"))
    type = parent_sdf->GetAttribute("type")->GetAsString();

  return RfSignalSourceFactory::Instance().Create(
    type, src_sdf ? src_sdf : parent_sdf);
}

}  // namespace rf_gz

// ── Registration macro ────────────────────────────────────────────────────────
// Registers SourceClass with RfSignalSourceFactory under type_str.
// Each source .cc also separately calls REGISTER_RF_TRANSMITTER so the world
// plugin's transmitter factory can instantiate RfTransmitterBase for that type.

#define _RF_SRC_CONCAT_(a, b) a##b
#define _RF_SRC_CONCAT(a, b)  _RF_SRC_CONCAT_(a, b)

#define REGISTER_RF_SOURCE(type_str, SourceClass)                                  \
  static const bool _RF_SRC_CONCAT(_rf_reg_src_, __COUNTER__) = []() {            \
    ::rf_gz::RfSignalSourceFactory::Instance().Register(                           \
      type_str,                                                                     \
      [](::sdf::ElementPtr sdf)                                                    \
        -> std::unique_ptr<::rf_gz::RfSignalSourceBase> {                          \
        auto src = std::make_unique<SourceClass>();                                \
        if (!src->LoadSdf(sdf)) return nullptr;                                    \
        return src;                                                                \
      });                                                                          \
    return true;                                                                   \
  }()
