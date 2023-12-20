// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAP_FRAGMENT__ROAD_SEGMENTS__GENERATE_LANELET_MAP__HPP_
#define MAP_FRAGMENT__ROAD_SEGMENTS__GENERATE_LANELET_MAP__HPP_

#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <functional>
#include <map_fragment/road_segments/road_segment.hpp>
#include <map_fragment/road_segments/road_segment_connection.hpp>
#include <rcpputils/asserts.hpp>

namespace map_fragment::road_segments
{
using LineStrings3dByRoadSegment =
  std::map<RoadSegment::ConstSharedPointer, lanelet::LineStrings3d>;

using GroupsOfPoints3d = std::vector<lanelet::Points3d>;

/**
 * Graph representation of point equivalence.
 *
 * Each vertex represents a point primitive and each edge means the two connected
 * vertices represent points which are equivalent. This graph can be used to extract
 * groups of equivalent points so that they can be merged together.
 */
class PointEquivalenceGraph
{
  // cspell: ignore Bimap
  // cspell: ignore bimap

  using UndirectedGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS>;

  using PointIdentifierToVertexBimap =
    boost::bimap<lanelet::Id, UndirectedGraph::vertex_descriptor>;

  using PointLookupTableByIdentifier = std::map<lanelet::Id, lanelet::Point3d>;

  UndirectedGraph graph;

  PointIdentifierToVertexBimap mappings;

  PointLookupTableByIdentifier point_lookup_table;

public:
  /**
   * Register equivalence between two points
   */
  auto registerEquivalence(
    const lanelet::Point3d & first_point, const lanelet::Point3d & second_point)
  {
    const auto vertex_for_first_point = findOrInsertVertexForPoint(first_point);
    const auto vertex_for_second_point = findOrInsertVertexForPoint(second_point);
    boost::add_edge(vertex_for_first_point, vertex_for_second_point, graph);
  }

  /**
   * Extract groups of equivalent points by computing connected components of the graph
   */
  auto extractGroupsOfEquivalentPoints() const -> GroupsOfPoints3d
  {
    if (boost::num_vertices(graph) == 0) {
      return GroupsOfPoints3d();
    } else {
      std::vector<int> component_map(boost::num_vertices(graph));
      boost::connected_components(graph, &component_map[0]);

      const auto number_of_groups =
        *std::max_element(component_map.begin(), component_map.end()) + 1;

      GroupsOfPoints3d groups_of_equivalent_points(number_of_groups);
      for (const auto vertex : boost::make_iterator_range(boost::vertices(graph))) {
        const auto point = getPointForVertex(vertex);
        const auto group_index = component_map[vertex];
        groups_of_equivalent_points[group_index].push_back(point);
      }

      return groups_of_equivalent_points;
    }
  }

private:
  /**
   * Find vertex representing the given point in the graph, or insert a new one if it doesn't exist yet
   */
  auto findOrInsertVertexForPoint(const lanelet::Point3d & point)
    -> UndirectedGraph::vertex_descriptor
  {
    const auto iterator = mappings.left.find(point.id());

    if (iterator != mappings.left.end()) {
      return iterator->second;
    } else {
      const auto vertex = boost::add_vertex(graph);
      mappings.insert({point.id(), vertex});
      point_lookup_table[point.id()] = point;
      return vertex;
    }
  }

  /**
   * Get point represented by the given vertex
   */
  auto getPointForVertex(const UndirectedGraph::vertex_descriptor vertex) const -> lanelet::Point3d
  {
    return point_lookup_table.at(mappings.right.at(vertex));
  }
};

/**
 * Generate points constituting a cross section at given position,
 * ordered left to right w.r.t. the tangent vector
 */
auto generatePointsLeftToRightForCrossSection(
  const RoadCrossSectionDescription & description, const Point & origin,
  const Vector & tangent_vector) -> lanelet::Points3d
{
  lanelet::Points3d cross_section_points;

  const auto normal_vector = rotateInLocalZAxisAssumingZeroRoll(tangent_vector, M_PI_2);

  for (auto i = 0; i < description.number_of_lanes + 1; i++) {
    const auto lateral_position = (description.number_of_lanes / 2. - i) * description.lane_width;
    const auto p = origin + normal_vector * lateral_position;

    cross_section_points.push_back(makePoint3d(p));
  }

  return cross_section_points;
}

/**
 * Generate lanelet boundaries for given road segment,
 * ordered left to right w.r.t. guide curve direction
 */
auto generateLaneletBoundariesLeftToRightForSegment(
  const RoadSegment::ConstSharedPointer road_segment, const int resolution)
  -> lanelet::LineStrings3d
{
  rcpputils::require_true(resolution, "Resolution must not be smaller than 2");

  const auto number_of_lanelet_boundaries =
    road_segment->cross_section_description.number_of_lanes + 1;

  lanelet::LineStrings3d lanelet_boundaries(number_of_lanelet_boundaries);
  for (auto boundary : lanelet_boundaries) {
    boundary.setId(generateNextLineStringId());
  }

  for (auto i = 0; i < resolution; i++) {
    const auto tangent = i / (resolution - 1.);
    const auto position = road_segment->guide_curve->getPosition(tangent);
    const auto tangent_vector = road_segment->guide_curve->getUnitTangentVector(tangent);

    const auto cross_section_points = generatePointsLeftToRightForCrossSection(
      road_segment->cross_section_description, position, tangent_vector);

    for (auto j = 0; j < number_of_lanelet_boundaries; j++) {
      lanelet_boundaries[j].push_back(cross_section_points[j]);
    }
  }

  return lanelet_boundaries;
}

/**
 * Build a point equivalence graph to identify points shared by multiple line strings
 */
auto createPointEquivalenceGraph(
  const RoadSegmentConnections & road_segment_connections,
  LineStrings3dByRoadSegment & lanelet_boundaries_by_road_segment) -> PointEquivalenceGraph
{
  PointEquivalenceGraph point_equivalence_graph;

  for (const auto & road_segment_connection : road_segment_connections) {
    const auto first_segment = road_segment_connection.first_segment;
    const auto second_segment = road_segment_connection.second_segment;

    for (const auto & boundary_indices : road_segment_connection.getBoundaryWiseConnections()) {
      auto & first_boundary =
        lanelet_boundaries_by_road_segment.at(first_segment)[boundary_indices.first];

      auto & second_boundary =
        lanelet_boundaries_by_road_segment.at(second_segment)[boundary_indices.second];

      rcpputils::assert_true(
        first_boundary.size() >= 2 && second_boundary.size() >= 2,
        "The lanelet boundaries must have at least 2 points each");

      const auto & first_point = road_segment_connection.is_first_segment_inverted
                                   ? first_boundary[0]
                                   : first_boundary[first_boundary.size() - 1];

      auto & second_point = road_segment_connection.is_second_segment_inverted
                              ? second_boundary[first_boundary.size() - 1]
                              : second_boundary[0];

      point_equivalence_graph.registerEquivalence(first_point, second_point);
    }
  }

  return point_equivalence_graph;
}

/**
 * Iterate through all groups of interconnected points within the
 * equivalence graph and merge them into a single point
 */
auto mergeEquivalentPoints(const PointEquivalenceGraph & point_equivalence_graph)
{
  for (const auto & group : point_equivalence_graph.extractGroupsOfEquivalentPoints()) {
    const auto first_point_in_group = group.front();
    for (auto iterator = group.begin() + 1; iterator != group.end(); iterator++) {
      auto point = *iterator;
      point.setId(first_point_in_group.id());
    }
  }
}

/**
 * Generate adjacent lanelets from boundaries ordered left to right
 */
auto generateAdjacentLaneletsFromBoundariesOrderedLeftToRight(
  const lanelet::LineStrings3d & lanelet_boundaries, const int number_of_reversed_lanes)
{
  rcpputils::assert_true(
    lanelet_boundaries.size() >= 2, "There must be at least 2 lanelet boundaries");

  const auto number_of_lanes = static_cast<int>(lanelet_boundaries.size() - 1);
  const auto index_of_first_reversed_lane = number_of_lanes - number_of_reversed_lanes;

  lanelet::Lanelets lanelets;
  for (auto i = 0; i < number_of_lanes; i++) {
    lanelets.push_back(
      i >= index_of_first_reversed_lane
        ? makeLanelet(lanelet_boundaries[i + 1], lanelet_boundaries[i])
        : makeLanelet(lanelet_boundaries[i], lanelet_boundaries[i + 1]));
  }
  return lanelets;
}

/**
 * Generate a lanelet map from given RoadSegment representation
 */
auto generateLaneletMap(
  const RoadSegments & road_segments, int resolution,
  const RoadSegmentConnections & road_segment_connections = RoadSegmentConnections())
{
  /*
   * 1) Generate lanelet boundaries for each road segment
   */
  LineStrings3dByRoadSegment lanelet_boundaries_by_road_segment;
  for (const auto & road_segment : road_segments) {
    lanelet_boundaries_by_road_segment[road_segment] =
      generateLaneletBoundariesLeftToRightForSegment(road_segment, resolution);
  }

  /*
   * 2) Analyze road segment connections to merge points shared between different lanelet boundaries
   */
  const auto point_equivalence_graph =
    createPointEquivalenceGraph(road_segment_connections, lanelet_boundaries_by_road_segment);
  mergeEquivalentPoints(point_equivalence_graph);

  /*
   * 3) Generate lanelets from modified lanelet boundaries
   */
  lanelet::Lanelets lanelets;
  for (const auto & road_segment : road_segments) {
    const auto road_segment_lanelet_boundaries = lanelet_boundaries_by_road_segment[road_segment];
    const auto road_segment_lanelets = generateAdjacentLaneletsFromBoundariesOrderedLeftToRight(
      road_segment_lanelet_boundaries,
      road_segment->cross_section_description.number_of_reversed_lanes);
    lanelets.insert(lanelets.end(), road_segment_lanelets.begin(), road_segment_lanelets.end());
  }

  return lanelet::utils::createMap(lanelets);
}
}  // namespace map_fragment::road_segments

#endif  // MAP_FRAGMENT__ROAD_SEGMENTS__GENERATE_LANELET_MAP_HPP_
