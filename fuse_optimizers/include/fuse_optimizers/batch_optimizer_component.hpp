#ifndef FUSE_OPTIMIZERS__BATCH_OPTIMIZER_COMPONENT_HPP_
#define FUSE_OPTIMIZERS__BATCH_OPTIMIZER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "fuse_optimizers/batch_optimizer.hpp"

namespace fuse_optimizers
{
  class BatchOptimizerComponent : public rclcpp::Node
  {
  public:
    explicit BatchOptimizerComponent(const rclcpp::NodeOptions& options);


  private:
    std::shared_ptr<BatchOptimizer> batch_optimizer_;
  };
}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS__BATCH_OPTIMIZER_COMPONENT_HPP_