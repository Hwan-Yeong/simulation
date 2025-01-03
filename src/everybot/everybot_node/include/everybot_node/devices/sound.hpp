#ifndef EVERYBOT_NODE__DEVICES__SOUND_HPP_
#define EVERYBOT_NODE__DEVICES__SOUND_HPP_

#include <everybot_msgs/srv/sound.hpp>
#include <memory>
#include <string>
#include "everybot_node/devices/devices.hpp"

namespace everybot
{
namespace everybot
{
namespace devices
{
class Sound : public Devices
{
public:
  static void request(
    rclcpp::Client<everybot_msgs::srv::Sound>::SharedPtr client,
    everybot_msgs::srv::Sound::Request req);

  explicit Sound(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "sound");

  void command(const void * request, void * response) override;

private:
  rclcpp::Service<everybot_msgs::srv::Sound>::SharedPtr srv_;
};
}  // namespace devices
}  // namespace everybot
}  // namespace everybot
#endif  // EVERYBOT_NODE__DEVICES__SOUND_HPP_
