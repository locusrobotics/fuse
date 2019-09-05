/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Clearpath Robotics
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

#ifndef FUSE_VIZ_SERIALIZED_GRAPH_DOT_WRITER_H
#define FUSE_VIZ_SERIALIZED_GRAPH_DOT_WRITER_H

#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_msgs/SerializedGraph.h>

#include <boost/graph/adjacency_list.hpp>

#include <string>

namespace fuse_viz
{

/**
 * @brief A fuse_msgs::SerializedGraph to Graphviz DOT format writer.
 *
 * This converts a fuse_msgs::SerializedGraph message into a BGL (Boost Graph Library) graph that can be easily
 * converted/serialized into a Graphviz DOT format. The DOT format is serialized into an std::ostream.
 */
class SerializedGraphDOTWriter
{
public:
  /**
   * @brief A vertex property bundle to store inside the BGL graph.
   *
   * This allows to store the fields from the variables and constraints in a fuse_msgs::SerializedGraph message. This
   * information is stored as part of the BGL graph as a property bundle, so later we can easily access that
   * information.
   */
  class VertexProperty
  {
  public:
    /**
     * @brief Vertex property type.
     *
     * A vertex property can either be a VARIABLE or a CONSTRAINT. The default constructor uses the UNKOWN type though.
     */
    enum Type
    {
      VARIABLE,    //< Variable type
      CONSTRAINT,  //< Constraint type
      UNKNOWN      //< Unknown type
    };

    /**
     * @brief Vertex property constructor.
     */
    VertexProperty(const std::string& uuid = fuse_core::uuid::to_string(fuse_core::uuid::NIL),
                   const Type type = UNKNOWN, const std::string& subtype = "NO_SUBTYPE")
      : uuid(uuid), type(type), subtype(subtype)
    {
    }

    /**
     * @brief Vertex property constructor from a fuse_core::Variable.
     * The type is set to VARIABLE. The variable type is used to set the subtype.
     *
     * @param[in] variable A fuse_core::Variable.
     */
    explicit VertexProperty(const fuse_core::Variable& variable)
      : uuid(fuse_core::uuid::to_string(variable.uuid())), type(VARIABLE), subtype(variable.type())
    {
    }

    /**
     * @brief Vertex property constructor from a fuse_core::Constraint.
     * The type is set to CONSTRAINT. The constraint type is used to set the subtype.
     *
     * @param[in] constraint A fuse_core::Constraint.
     */
    explicit VertexProperty(const fuse_core::Constraint& constraint)
      : uuid(fuse_core::uuid::to_string(constraint.uuid())), type(CONSTRAINT), subtype(constraint.type())
    {
    }

    std::string uuid;     //< Variable/Constraint UUID
    Type type;            //< VARIABLE or CONTRAINT type
    std::string subtype;  //< Variable/Constraint type
  };

  // Graph typedef
  using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty>;

  // Disable constructor and destructor, since this class only provides static methods
  SerializedGraphDOTWriter() = delete;
  ~SerializedGraphDOTWriter() = delete;

  /**
   * @brief Converts a fuse_msgs::SerializedGraph message into a BGL graph.
   *
   * @param[in] msg A fuse_msgs::SerializedGraph message.
   * @return A BGL graph.
   */
  static Graph toBoost(const fuse_msgs::SerializedGraphConstPtr& msg);

  /**
   * @brief Write a fuse_msgs::SerializedGraph message into an std::ostream.
   * The message is first converted into a BGL graph and then serialized as a Graphviz DOT format.
   *
   * @param[in, out] os  Output stream.
   * @param[in]      msg A fuse_msgs::SerializedGraph message.
   * @return Output stream with the fuse_msgs::SerializedGraph serialized into it.
   */
  static std::ostream& write(std::ostream& os, const fuse_msgs::SerializedGraphConstPtr& msg);
};

}  // namespace fuse_viz

#endif  // FUSE_VIZ_SERIALIZED_GRAPH_DOT_WRITER_H
