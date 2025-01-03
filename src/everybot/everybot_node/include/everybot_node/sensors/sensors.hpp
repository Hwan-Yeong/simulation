#ifndef EVERYBOT_NODE__SENSORS__SENSORS_HPP_
#define EVERYBOT_NODE__SENSORS__SENSORS_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "everybot_node/control_table.hpp"
#include "everybot_node/dynamixel_sdk_wrapper.hpp"

namespace everybot
{
namespace everybot
{
extern const ControlTable extern_control_table;
namespace sensors
{
class Sensors
{
public:
  explicit Sensors(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & frame_id = "")
  : nh_(nh),
    frame_id_(frame_id)
  {
  }

  virtual void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) = 0;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::string frame_id_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
};
}  // namespace sensors
}  // namespace everybot
}  // namespace everybot
#endif  // EVERYBOT_NODE__SENSORS__SENSORS_HPP_
