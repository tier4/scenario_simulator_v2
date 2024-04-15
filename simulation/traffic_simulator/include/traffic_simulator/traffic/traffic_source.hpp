// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_

#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <random>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic/traffic_module_base.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace traffic_simulator
{
namespace traffic
{

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;

class SpawnPoseValidator
{
public:
  struct LaneletArea
  {
    lanelet::Id first_id = -1, last_id = -1;
    lanelet::Ids ids{};
    std::vector<geometry_msgs::msg::Point> left_bound{}, right_bound{};
    /**
     * @note polygon and polygon_b are declared mutable for lazy calculation.
     * They may be not up to date, to get the most up to date polygon, call
     * `getPolygon()` and `getBoostPolygon()` respectively
     */
    mutable std::vector<geometry_msgs::msg::Point> polygon{};
    mutable boost_polygon polygon_b;

    LaneletArea() = default;
    LaneletArea(const lanelet::Id & id, std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils);

    /// @brief combine two lanelet areas in the order they are passed (second is inserted after first)
    auto operator+(const LaneletArea & other) const -> LaneletArea;

    /// @brief combine two lanelet areas in the order they are passed (second is inserted after first)
    static auto combine(const LaneletArea & before, const LaneletArea & after) -> LaneletArea;

    /// @brief whether this area contains the other area
    auto contains(const LaneletArea & other) const -> bool;

    /// @brief whether this area contains the lanelet id
    auto contains(const lanelet::Id & id) const -> bool;

    /// @note Closes the polygon (adds first point to the end)
    static auto toBoostPolygon(const std::vector<geometry_msgs::msg::Point> & points)
      -> boost_polygon;

    /// @note Returns closed polygon the polygon (first point == last point)
    auto getBoostPolygon() const -> const boost_polygon &;
    auto getPolygon() const -> const std::vector<geometry_msgs::msg::Point> &;
    void createPolygon() const;
    template <typename OSTREAM>
    friend OSTREAM & operator<<(OSTREAM & os, const LaneletArea & area)
    {
      os << "first_id: " << area.first_id << ", last_id: " << area.last_id << ", ids:";
      for (const auto & id : area.ids) {
        os << " " << id;
      }
      return os;
    }
  };

  SpawnPoseValidator(
    std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
    const geometry_msgs::msg::Pose & source_pose, const double source_radius, const bool disabled,
    const bool include_crosswalk);

  /**
   * @brief whether the 2D polygon does fit inside the lanelet with the given id
   * @note The polygon is checked against any lanelet combinations in the Validators area
   */
  auto isValid(const std::vector<geometry_msgs::msg::Point> & polygon, const lanelet::Id & id) const
    -> bool;

private:
  /**
    * @brief recursively find all spawning areas
    * @param previous_area used for recursion, do not use this parameter
    * @param in_front used for recursion, do not use this parameter
    */
  void findAllSpawningAreas(
    const lanelet::Id id, const std::set<lanelet::Id> & ids, const bool in_front,
    const std::optional<LaneletArea> & previous_area = std::nullopt);
  /**
   * @brief as after `findAllSpawningAreas` there will be many redundant areas (meaning one will
   * be a subset of the other), remove all subsets for faster search
   */
  void removeRedundantAreas();

  const bool disabled_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;
  std::vector<LaneletArea> areas_;
};

class TrafficSource : public TrafficModuleBase
{
public:
  using VehicleParams = traffic_simulator_msgs::msg::VehicleParameters;
  using PedestrianParams = traffic_simulator_msgs::msg::PedestrianParameters;
  using ParamsVariant = std::variant<VehicleParams, PedestrianParams>;

  struct Configuration
  {
    Configuration() = default;
    Configuration(const bool allow_spawn_outside_lane, const bool require_footprint_fitting)
    : allow_spawn_outside_lane(allow_spawn_outside_lane),
      require_footprint_fitting(require_footprint_fitting){};
    bool allow_spawn_outside_lane = false;
    bool require_footprint_fitting = false;
    bool use_random_orientation = false;
    double start_delay = 0.0;
  };

  explicit TrafficSource(
    const double radius, const double rate, const double speed,
    const geometry_msgs::msg::Pose & position,
    const std::vector<std::tuple<ParamsVariant, std::string, std::string, double>> & params,
    const std::optional<int> random_seed, const double current_time,
#define MAKE_FUNCTION_REF_TYPE(PARAMS, POSE)                                                     \
  const std::function<void(                                                                      \
    const std::string &, const POSE &, const PARAMS &, const std::string &, const std::string &, \
    double)> &
    // clang-format off
    MAKE_FUNCTION_REF_TYPE(VehicleParams,    CanonicalizedLaneletPose) vehicle_ll_spawn_function,
    MAKE_FUNCTION_REF_TYPE(PedestrianParams, CanonicalizedLaneletPose) pedestrian_ll_spawn_function,
    MAKE_FUNCTION_REF_TYPE(VehicleParams,    geometry_msgs::msg::Pose) vehicle_spawn_function,
    MAKE_FUNCTION_REF_TYPE(PedestrianParams, geometry_msgs::msg::Pose) pedestrian_spawn_function,
  // clang-format on
#undef MAKE_FUNCTION_REF_TYPE
    const Configuration & configuration, std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils);

  void execute(const double current_time, const double step_time) override;

  const double radius_;
  const double rate_;
  const double speed_;
  const geometry_msgs::msg::Pose source_pose_;

private:
  template <typename T, std::size_t N>
  static auto obtainParams(
    const std::vector<std::tuple<ParamsVariant, std::string, std::string, double>> & params)
    -> std::vector<T>
  {
    std::vector<T> ret;
    ret.reserve(params.size());
    for (const auto & param : params) {
      ret.push_back(std::get<N>(param));
    }
    return ret;
  }
  static auto isPedestrian(const ParamsVariant & params) -> bool;
  auto getRandomPose(const bool random_orientation = false) -> geometry_msgs::msg::Pose;
  auto getNewEntityName() -> std::string;
  auto isCurrentPedestrian() -> bool;
  auto getCurrentBoundingBox() -> traffic_simulator_msgs::msg::BoundingBox;
  auto isPoseValid(
    geometry_msgs::msg::Pose & pose, std::optional<CanonicalizedLaneletPose> & out_pose) -> bool;
  auto getValidRandomPose()
    -> std::pair<geometry_msgs::msg::Pose, std::optional<CanonicalizedLaneletPose>>;
  auto getRandomLaneletId() -> lanelet::Id;
  auto getRandomSValue(const lanelet::Id lanelet_id) -> double;
  auto randomizeCurrentParams() -> void;

  /// @note IDs
  inline static unsigned int next_source_id_ = 0u;
  const unsigned int source_id_;
  unsigned int entity_id_ = 0u;

/// @note Functions
#define MAKE_FUNCTION_TYPE(PARAMS, POSE)                                                         \
  const std::function<void(                                                                      \
    const std::string &, const POSE &, const PARAMS &, const std::string &, const std::string &, \
    const double)>
  // clang-format off
  MAKE_FUNCTION_TYPE(VehicleParams,    CanonicalizedLaneletPose) vehicle_ll_spawn_function_;
  MAKE_FUNCTION_TYPE(PedestrianParams, CanonicalizedLaneletPose) pedestrian_ll_spawn_function_;
  MAKE_FUNCTION_TYPE(VehicleParams,    geometry_msgs::msg::Pose) vehicle_spawn_function_;
  MAKE_FUNCTION_TYPE(PedestrianParams, geometry_msgs::msg::Pose) pedestrian_spawn_function_;
  // clang-format on
#undef MAKE_FUNCTION_TYPE

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;
  std::mt19937 engine_;
  std::uniform_real_distribution<double> angle_distribution_;
  std::uniform_real_distribution<double> radius_distribution_;
  std::discrete_distribution<> params_distribution_;
  const double start_execution_time_;
  unsigned int spawn_count_ = 0u;
  const Configuration config_;
  const std::vector<ParamsVariant> params_;
  const std::vector<std::string> behaviors_;
  const std::vector<std::string> models3d_;
  std::vector<ParamsVariant>::const_iterator current_params_;
  std::vector<std::string>::const_iterator current_behavior_;
  std::vector<std::string>::const_iterator current_model3d_;

  /// @note Validators, one does not allow positions on crosswalk, the other does allow positions on crosswalk
  const SpawnPoseValidator validator_;
  const SpawnPoseValidator validator_crosswalk_;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_
