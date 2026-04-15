#include <cmath>
#include <complex>
#include <string>
#include <vector>

#include <gz/msgs/float_v.pb.h>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "rf_gz/sinks/RfSignalSinkFactory.hh"

namespace rf_gz
{

/// IQ-publish sink: forwards the combined baseband IQ to a gz transport topic.
///
/// SDF parameters (inside <sink type="iq_pub"> or directly on <receiver>):
///   <topic>   gz transport topic  (default: /rf/<rx_name>/iq)
class IqPubSink : public RfSignalSinkBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("topic"))
      topic_ = sdf->Get<std::string>("topic");
    return true;
  }

  void ConsumeSamples(const RfSignal& signal, const RxContext& rx) override
  {
    if (!publisher_.Valid())
    {
      const std::string topic =
        topic_.empty() ? "/rf/" + rx.rx_name + "/iq" : topic_;
      publisher_ = node_.Advertise<gz::msgs::Float_V>(topic);
    }

    gz::msgs::Float_V msg;
    auto* stamp = msg.mutable_header()->mutable_stamp();
    stamp->set_sec(static_cast<int64_t>(rx.time));
    stamp->set_nsec(static_cast<int32_t>(
      (rx.time - std::floor(rx.time)) * 1e9));

    const auto n = static_cast<uint32_t>(signal.iq.size());
    msg.mutable_data()->Reserve(2 * n);
    for (const auto& s : signal.iq)
    {
      msg.add_data(static_cast<float>(s.real()));
      msg.add_data(static_cast<float>(s.imag()));
    }
    publisher_.Publish(msg);
  }

private:
  std::string topic_;

  gz::transport::Node            node_;
  gz::transport::Node::Publisher publisher_;
};

}  // namespace rf_gz

REGISTER_RF_SINK("iq_pub", rf_gz::IqPubSink);
