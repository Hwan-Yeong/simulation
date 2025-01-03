#ifndef EVERYBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define EVERYBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "everybot_node/odometry.hpp"

namespace everybot
{
namespace everybot
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace everybot
}  // namespace everybot
#endif  // EVERYBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_
