#include "fuse_optimizers/fixed_lag_smoother_component.hpp"

namespace fuse_optimizers
{
  FixedLagSmootherComponent::FixedLagSmootherComponent(const rclcpp::NodeOptions& options)
    : Node("fixed_lag_smoother", options),
      fixed_lag_smoother_(std::make_shared<FixedLagSmoother>(*this))
  {
    RCLCPP_INFO(get_logger(), "FixedLagSmootherComponent constructed");
  }
}  // namespace fuse_optimizers

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fuse_optimizers::FixedLagSmootherComponent)