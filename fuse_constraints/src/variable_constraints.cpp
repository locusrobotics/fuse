/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#include <initializer_list>
#include <numeric>

#include <fuse_constraints/variable_constraints.hpp>

namespace fuse_constraints
{

void VariableConstraints::reserve(const size_t variable_count)
{
  variable_constraints_.reserve(variable_count);
}

bool VariableConstraints::empty() const
{
  return variable_constraints_.empty();
}

size_t VariableConstraints::size() const
{
  auto sum_edges = [](const size_t input, const ConstraintCollection & edges)
    {
      return input + edges.size();
    };
  return std::accumulate(variable_constraints_.begin(), variable_constraints_.end(), 0u, sum_edges);
}

unsigned int VariableConstraints::nextVariableIndex() const
{
  return variable_constraints_.size();
}

void VariableConstraints::insert(const unsigned int constraint, const unsigned int variable)
{
  if (variable >= variable_constraints_.size()) {
    variable_constraints_.resize(variable + 1);
  }
  variable_constraints_[variable].insert(constraint);
}

void VariableConstraints::insert(
  const unsigned int constraint,
  std::initializer_list<unsigned int> variable_list)
{
  return insert(constraint, variable_list.begin(), variable_list.end());
}

void VariableConstraints::insert(const unsigned int variable)
{
  if (variable >= variable_constraints_.size()) {
    // This automatically create a new variable entry with an empty ConstraintCollection
    variable_constraints_.resize(variable + 1);
  }
}

void VariableConstraints::print(std::ostream & stream) const
{
  for (size_t variable = 0; variable < variable_constraints_.size(); ++variable) {
    stream << variable << ": [";

    for (const auto & constraint : variable_constraints_[variable]) {
      stream << constraint << ", ";
    }

    stream << "]\n";
  }
}

std::ostream & operator<<(std::ostream & stream, const VariableConstraints & variable_constraints)
{
  variable_constraints.print(stream);
  return stream;
}

}  // namespace fuse_constraints
