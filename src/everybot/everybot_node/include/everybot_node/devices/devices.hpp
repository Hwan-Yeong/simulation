#ifndef EVERYBOT_NODE__DEVICES__DEVICES_HPP_
#define EVERYBOT_NODE__DEVICES__DEVICES_HPP_

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
namespace devices
{
class Devices
{
public:
  explicit Devices(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
  : nh_(nh),
    dxl_sdk_wrapper_(dxl_sdk_wrapper)
  {
  }

  virtual void command(const void * request, void * response) = 0;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::ServicesQoS());
};
}  // namespace devices
}  // namespace everybot
}  // namespace everybot
#endif  // EVERYBOT_NODE__DEVICES__DEVICES_HPP_
