#ifndef EVERYBOT_NODE__SENSORS__SENSOR_STATE_HPP_
#define EVERYBOT_NODE__SENSORS__SENSOR_STATE_HPP_

#include <everybot_msgs/msg/sensor_state.hpp>

#include <memory>
#include <string>

#include "everybot_node/sensors/sensors.hpp"

namespace everybot
{
namespace everybot
{
namespace sensors
{
class SensorState : public Sensors
{
public:
  explicit SensorState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "sensor_state",
    const uint8_t & bumper_forward = 0,
    const uint8_t & bumper_backward = 0,
    const uint8_t & illumination = 0,
    const uint8_t & cliff = 0,
    const uint8_t & sonar = 0);

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<everybot_msgs::msg::SensorState>::SharedPtr pub_;

  uint8_t bumper_forward_;
  uint8_t bumper_backward_;
  uint8_t illumination_;
  uint8_t cliff_;
  uint8_t sonar_;
};
}  // namespace sensors
}  // namespace everybot
}  // namespace everybot
#endif  // EVERYBOT_NODE__SENSORS__SENSOR_STATE_HPP_
