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

#include <fuse_publishers/graph_dot_writer.h>

#include <boost/graph/graphviz.hpp>

#include <map>
#include <string>

namespace fuse_publishers
{

/**
 * @brief Vertex writer.
 *
 * This tells BGL how to write the vertex properites into the Graphviz DOT format.
 * This class implements a functor operator() that it's called for each vertex in the graph.
 *
 * @tparam UUIDMap    Vertex property UUID mapping.
 * @tparam TypeMap    Vertex property type mapping.
 * @tparam SubTypeMap Vertex property subtype mapping.
 */
template <class UUIDMap, class TypeMap, class SubTypeMap>
class vertex_writer
{
public:
  /**
   * @brief Vertex writer constructor.
   *
   * @param[in] uuid_map    Vertex property UUID mapping.
   * @param[in] type_map    Vertex property type mapping.
   * @param[in] subtype_map Vertex property subtype mapping.
   */
  vertex_writer(UUIDMap uuid_map, TypeMap type_map, SubTypeMap subtype_map)
    : uuid_map_(uuid_map), type_map_(type_map), subtype_map_(subtype_map)
  {
  }

  /**
   * @brief Functor operator() the writes the vertex properties in DOT format.
   * This writes the vertex properties as field-value pairs for each node/vertex into the DOT format.
   *
   * @param[in, out] os Output stream.
   * @param[in]      v  Vertex.
   *
   * @tparam Vertex Vertex descriptor in the BGL graph.
   */
  template <class Vertex>
  void operator()(std::ostream& os, const Vertex& v) const
  {
    os << "["
       << "shape=" << TYPE_SHAPES[type_map_[v]] << ", label=\"" << uuid_map_[v] << "\""
       << ", taillabel=\"" << subtype_map_[v] << "\""
       << "]";
  }

private:
  UUIDMap uuid_map_;        //< Vertex property UUID mapping.
  TypeMap type_map_;        //< Vertex property type mapping.
  SubTypeMap subtype_map_;  //< Vertex property subtype mapping.

  static const char* const TYPE_SHAPES[];  //< Graphviz DOT shape strings for each vertex property type supported.
};

// The shapes for each type supported:
// * VARIABLE   -> circle
// * CONSTRAINT -> diamond
template <class UUIDMap, class TypeMap, class SubTypeMap>
const char* const vertex_writer<UUIDMap, TypeMap, SubTypeMap>::TYPE_SHAPES[] = { "circle", "diamond", "none" };

/**
 * @brief A helper function to make/create a vertex writer.
 *
 * @param[in] uuid_map    Vertex property UUID mapping.
 * @param[in] type_map    Vertex property type mapping.
 * @param[in] subtype_map Vertex property subtype mapping.
 *
 * @tparam UUIDMap    Vertex property UUID mapping.
 * @tparam TypeMap    Vertex property type mapping.
 * @tparam SubTypeMap Vertex property subtype mapping.
 */
template <class UUIDMap, class TypeMap, class SubTypeMap>
inline vertex_writer<UUIDMap, TypeMap, SubTypeMap> make_vertex_writer(UUIDMap uuid_map, TypeMap type_map,
                                                                      SubTypeMap subtype_map)
{
  return vertex_writer<UUIDMap, TypeMap, SubTypeMap>(uuid_map, type_map, subtype_map);
}

GraphDOTWriter::Graph GraphDOTWriter::toBoost(const fuse_msgs::GraphConstPtr& msg)
{
  using VertexMap = std::map<std::string, Graph::vertex_descriptor>;

  Graph graph;

  VertexMap variable_vertex_by_uuid;
  for (const auto& variable : msg->variables)
  {
    variable_vertex_by_uuid[variable.uuid] = boost::add_vertex(variable, graph);
  }

  for (const auto& constraint : msg->constraints)
  {
    const auto constraint_vertex = boost::add_vertex(constraint, graph);

    for (const auto& variable : constraint.variables)
    {
      boost::add_edge(constraint_vertex, variable_vertex_by_uuid[variable], graph);
    }
  }

  return graph;
}

std::ostream& GraphDOTWriter::write(std::ostream& os, const fuse_msgs::GraphConstPtr& msg)
{
  const auto graph = toBoost(msg);
  boost::write_graphviz(os, graph,
                        make_vertex_writer(boost::get(&VertexProperty::uuid, graph),
                                           boost::get(&VertexProperty::type, graph),
                                           boost::get(&VertexProperty::subtype, graph)));
  return os;
}

}  // namespace fuse_publishers
