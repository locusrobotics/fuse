#ifndef FUSE_OPTIMIZERS__FIXED_LAG_SMOOTHER_COMPONENT_HPP_
#define FUSE_OPTIMIZERS__FIXED_LAG_SMOOTHER_COMPONENT_HPP_

#include <fuse_optimizers/fixed_lag_smoother.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fuse_optimizers
{
  class FixedLagSmootherComponent : public rclcpp::Node
  {
  public:
    explicit FixedLagSmootherComponent(const rclcpp::NodeOptions& options);

  private:  
    std::shared_ptr<FixedLagSmoother> fixed_lag_smoother_;
  };
}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS__FIXED_LAG_SMOOTHER_COMPONENT_HPP_