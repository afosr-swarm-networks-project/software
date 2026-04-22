#include <chrono>
#include <cmath>
#include <complex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rf_msgs/msg/iq_frame.hpp>
#include <sdf/Element.hh>

#include "rf_gz/sinks/RfSignalSinkFactory.hh"

namespace rf_gz
{

/// IQ-publish sink: publishes baseband IQ directly to a ROS 2 topic.
///
/// SDF parameters (inside <sink type="iq_pub"> or directly on <receiver>):
///   <topic>   ROS 2 topic name  (default: /rf/<rx_name>/iq)
class IqPubSink : public RfSignalSinkBase
{
public:
  bool LoadSdf(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("topic"))
      topic_ = sdf->Get<std::string>("topic");
    return true;
  }

  void ConsumeSamples(const RfSignal& signal, const RxContext& rx,
                      const gz::sim::UpdateInfo& info) override
  {
    if (!publisher_)
    {
      const std::string topic =
        topic_.empty() ? "/rf/" + rx.rx_name + "/iq" : topic_;
      if (!rclcpp::ok())
        rclcpp::init(0, nullptr);
      ros_node_ = std::make_shared<rclcpp::Node>("iq_pub_sink_" + rx.rx_name);
      publisher_ = ros_node_->create_publisher<rf_msgs::msg::IqFrame>(topic, 10);
    }

    const double sim_time = std::chrono::duration<double>(info.simTime).count();
    rf_msgs::msg::IqFrame msg;
    msg.stamp.sec     = static_cast<int32_t>(sim_time);
    msg.stamp.nanosec = static_cast<uint32_t>(
      (sim_time - std::floor(sim_time)) * 1e9);
    msg.fs_hz = signal.fs_hz;
    msg.fc_hz = signal.cf_hz;

    const auto n = signal.iq.size();
    msg.data.reserve(2 * n);
    for (const auto& s : signal.iq)
    {
      msg.data.push_back(static_cast<float>(s.real()));
      msg.data.push_back(static_cast<float>(s.imag()));
    }
    publisher_->publish(msg);
  }

private:
  std::string topic_;

  rclcpp::Node::SharedPtr                                    ros_node_;
  rclcpp::Publisher<rf_msgs::msg::IqFrame>::SharedPtr        publisher_;
};

}  // namespace rf_gz

REGISTER_RF_SINK("iq_pub", rf_gz::IqPubSink);
