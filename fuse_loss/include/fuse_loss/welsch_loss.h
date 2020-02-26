/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FUSE_LOSS_WELSCH_LOSS_H
#define FUSE_LOSS_WELSCH_LOSS_H

#include <fuse_core/loss.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

#include <ostream>
#include <string>


namespace fuse_loss
{

/**
 * @brief The Welsch loss function.
 *
 * This class encapsulates the ceres::WelschLoss class, adding the ability to serialize it and load it
 * dynamically.
 *
 * The Welsch loss is not provided by the Ceres solver, so it is implemented here, based on table #1 from:
 * http://www2.informatik.uni-freiburg.de/~spinello/agarwalICRA13.pdf (p. 3)
 *
 * See the Ceres documentation for more details. http://ceres-solver.org/nnls_modeling.html#lossfunction
 */
class WelschLoss : public fuse_core::Loss
{
public:
  FUSE_LOSS_DEFINITIONS(WelschLoss);

  /**
   * @brief Constructor
   *
   * @param[in] a WelschLoss parameter 'a'
   */
  explicit WelschLoss(const double a = 1.0);

  /**
   * @brief Destructor
   */
  ~WelschLoss() override = default;

  /**
   * @brief Perform any required post-construction initialization, such as reading from the parameter server.
   *
   * This will be called on each plugin after construction.
   *
   * @param[in] name A unique name to initialize this plugin instance, such as from the parameter server.
   */
  void initialize(const std::string& name) override;

  /**
   * @brief Print a human-readable description of the loss function to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Return a raw pointer to a ceres::LossFunction that implements the loss function
   *
   * The Ceres interface requires a raw pointer. Ceres will take ownership of the pointer and promises to properly
   * delete the loss function when it is done. Additionally, Fuse promises that the Loss object will outlive any
   * generated loss functions (i.e. the Ceres objects will be destroyed before the Loss Function objects). This
   * guarantee may allow optimizations for the creation of the loss function objects.
   *
   * @return A base pointer to an instance of a derived ceres::LossFunction.
   */
  ceres::LossFunction* lossFunction() const override;

  /**
   * @brief Parameter 'a' accessor.
   *
   * @return Parameter 'a'.
   */
  double a() const
  {
    return a_;
  }

  /**
   * @brief Parameter 'a' mutator.
   *
   * @param[in] a Parameter 'a'.
   */
  void a(const double a)
  {
    a_ = a;
  }

private:
  double a_{ 1.0 };  //<! WelschLoss parameter 'a'

  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Loss>(*this);
    archive & a_;
  }
};

}  // namespace fuse_loss

BOOST_CLASS_EXPORT_KEY(fuse_loss::WelschLoss);

#endif  // FUSE_LOSS_WELSCH_LOSS_H
