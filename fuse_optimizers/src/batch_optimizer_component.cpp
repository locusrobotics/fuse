#include "fuse_optimizers/batch_optimizer_component.hpp"
namespace fuse_optimizers
{
  BatchOptimizerComponent::BatchOptimizerComponent(const rclcpp::NodeOptions& options)
    : Node("batch_optimizer", options),
      batch_optimizer_(std::make_shared<BatchOptimizer>(*this))
  {
    RCLCPP_INFO(get_logger(), "BatchOptimizerComponent constructed");
  }
}  // namespace fuse_optimizers

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fuse_optimizers::BatchOptimizerComponent)