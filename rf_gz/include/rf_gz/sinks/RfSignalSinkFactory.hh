#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <sdf/Element.hh>
#include "rf_gz/sinks/RfSignalSinkBase.hh"

namespace rf_gz
{

class RfSignalSinkFactory
{
public:
  using CreatorFn =
    std::function<std::unique_ptr<RfSignalSinkBase>(sdf::ElementPtr)>;

  static RfSignalSinkFactory& Instance()
  {
    static RfSignalSinkFactory factory;
    return factory;
  }

  void Register(const std::string& type, CreatorFn fn)
  {
    creators_[type] = std::move(fn);
  }

  std::unique_ptr<RfSignalSinkBase>
  Create(const std::string& type, sdf::ElementPtr sdf) const
  {
    auto it = creators_.find(type);
    return (it == creators_.end()) ? nullptr : it->second(sdf);
  }

private:
  std::map<std::string, CreatorFn> creators_;
};

/// Instantiate the correct signal sink from SDF.
///
/// Reads type from a <sink type="..."> child element of parent_sdf.
/// Defaults to type "iq_pub" (IqPubSink) when no <sink> element is present
/// so that existing receiver SDF configurations work without modification.
inline std::unique_ptr<RfSignalSinkBase>
CreateSignalSink(sdf::ElementPtr parent_sdf)
{
  sdf::ElementPtr sink_sdf;
  std::string     type = "iq_pub";  // default

  if (parent_sdf && parent_sdf->HasElement("sink"))
  {
    sink_sdf = parent_sdf->GetElement("sink");
    if (sink_sdf->HasAttribute("type"))
      type = sink_sdf->GetAttribute("type")->GetAsString();
  }

  return RfSignalSinkFactory::Instance().Create(
    type, sink_sdf ? sink_sdf : parent_sdf);
}

}  // namespace rf_gz

// ── Registration macro ────────────────────────────────────────────────────────

#define _RF_SINK_CONCAT_(a, b) a##b
#define _RF_SINK_CONCAT(a, b)  _RF_SINK_CONCAT_(a, b)

#define REGISTER_RF_SINK(type_str, SinkClass)                                      \
  static const bool _RF_SINK_CONCAT(_rf_reg_sink_, __COUNTER__) = []() {          \
    ::rf_gz::RfSignalSinkFactory::Instance().Register(                             \
      type_str,                                                                    \
      [](::sdf::ElementPtr sdf)                                                    \
        -> std::unique_ptr<::rf_gz::RfSignalSinkBase> {                            \
        auto sink = std::make_unique<SinkClass>();                                 \
        if (!sink->LoadSdf(sdf)) return nullptr;                                   \
        return sink;                                                               \
      });                                                                          \
    return true;                                                                   \
  }()
