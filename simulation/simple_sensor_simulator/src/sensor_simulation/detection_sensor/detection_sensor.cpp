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

#include <algorithm>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <geometry/quaternion/get_angle_difference.hpp>
#include <geometry/quaternion/get_normal_vector.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/vector3/hypot.hpp>
#include <memory>
#include <random>
#include <scenario_simulator_exception/exception.hpp>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace simple_sensor_simulator
{
auto distance(const geometry_msgs::Pose & pose1, const geometry_msgs::Pose & pose2)
{
  return std::hypot(
    pose1.position().x() - pose2.position().x(),  //
    pose1.position().y() - pose2.position().y(),  //
    pose1.position().z() - pose2.position().z());
}

auto DetectionSensorBase::isEgoEntityStatusToWhichThisSensorIsAttached(
  const traffic_simulator_msgs::EntityStatus & status) const -> bool
{
  return status.name() == configuration_.entity() and
         status.type().type() == traffic_simulator_msgs::EntityType::EGO;
}

auto DetectionSensorBase::findEgoEntityStatusToWhichThisSensorIsAttached(
  const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const
  -> std::vector<traffic_simulator_msgs::EntityStatus>::const_iterator
{
  if (auto iter = std::find_if(
        statuses.begin(), statuses.end(),
        [this](const auto & status) {
          return isEgoEntityStatusToWhichThisSensorIsAttached(status);
        });
      iter != statuses.end()) {
    return iter;
  } else {
    throw SimulationRuntimeError("Detection sensor can be attached only ego entity.");
  }
}

auto DetectionSensorBase::isOnOrAboveEgoPlane(
  const geometry_msgs::Pose & entity_pose, const geometry_msgs::Pose & ego_pose) -> bool
{
  /*
      The threshold for detecting significant changes in ego vehicle's orientation (unit: radian).
      The value determines the minimum angular difference required to consider the ego orientation
      as "changed".

      There is no technical basis for this value, it was determined based on experiments.
  */
  constexpr static double rotation_threshold_ = 0.04;
  /*
      Maximum downward offset in Z-axis relative to the ego position (unit: meter).
      If the NPC is lower than this offset relative to the ego position,
      the NPC will be excluded from detection

      There is no technical basis for this value, it was determined based on experiments.
  */
  constexpr static double max_downward_z_offset_ = 1.0;

  const auto hasEgoOrientationChanged = [this](const geometry_msgs::msg::Pose & ego_pose_ros) {
    return math::geometry::getAngleDifference(
             ego_pose_ros.orientation, ego_plane_pose_opt_->orientation) > rotation_threshold_;
  };

  // if other entity is at the same altitude as Ego or within max_downward_z_offset_ below Ego
  if (entity_pose.position().z() >= (ego_pose.position().z() - max_downward_z_offset_)) {
    return true;
    // otherwise check if other entity is above ego plane
  } else {
    // update ego plane if needed
    geometry_msgs::msg::Pose ego_pose_ros;
    simulation_interface::toMsg(ego_pose, ego_pose_ros);
    if (!ego_plane_opt_ || !ego_plane_pose_opt_ || hasEgoOrientationChanged(ego_pose_ros)) {
      ego_plane_opt_.emplace(
        ego_pose_ros.position, math::geometry::getNormalVector(ego_pose_ros.orientation));
      ego_plane_pose_opt_ = ego_pose_ros;
    }

    geometry_msgs::msg::Pose entity_pose_ros;
    simulation_interface::toMsg(entity_pose, entity_pose_ros);
    return ego_plane_opt_->offset(entity_pose_ros.position) >= 0.0;
  }
}

template <typename To, typename... From>
auto make(From &&...) -> To;

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status) -> unique_identifier_msgs::msg::UUID
{
  static auto generate_uuid = boost::uuids::name_generator(boost::uuids::random_generator()());
  const auto uuid = generate_uuid(status.name());
  unique_identifier_msgs::msg::UUID message;
  std::copy(uuid.begin(), uuid.end(), message.uuid.begin());
  return message;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::ObjectClassification
{
  auto object_classification = autoware_perception_msgs::msg::ObjectClassification();

  object_classification.label = [&]() {
    switch (status.subtype().value()) {
      case traffic_simulator_msgs::EntitySubtype::CAR:
        return autoware_perception_msgs::msg::ObjectClassification::CAR;
      case traffic_simulator_msgs::EntitySubtype::TRUCK:
        return autoware_perception_msgs::msg::ObjectClassification::TRUCK;
      case traffic_simulator_msgs::EntitySubtype::BUS:
        return autoware_perception_msgs::msg::ObjectClassification::BUS;
      case traffic_simulator_msgs::EntitySubtype::TRAILER:
        return autoware_perception_msgs::msg::ObjectClassification::TRAILER;
      case traffic_simulator_msgs::EntitySubtype::MOTORCYCLE:
        return autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
      case traffic_simulator_msgs::EntitySubtype::BICYCLE:
        return autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
      case traffic_simulator_msgs::EntitySubtype::PEDESTRIAN:
        return autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      default:
        return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    }
  }();

  object_classification.probability = 1;

  return object_classification;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status) -> geometry_msgs::msg::Pose
{
  auto pose = geometry_msgs::msg::Pose();
  simulation_interface::toMsg(status.pose(), pose);

  auto center_point = geometry_msgs::msg::Point();
  simulation_interface::toMsg(status.bounding_box().center(), center_point);

  Eigen::Vector3d center = math::geometry::getRotationMatrix(pose.orientation) *
                           Eigen::Vector3d(center_point.x, center_point.y, center_point.z);

  pose.position.x = pose.position.x + center.x();
  pose.position.y = pose.position.y + center.y();
  pose.position.z = pose.position.z + center.z();

  return pose;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status) -> geometry_msgs::msg::Twist
{
  auto twist = geometry_msgs::msg::Twist();
  simulation_interface::toMsg(status.action_status().twist(), twist);
  return twist;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::DetectedObjectKinematics
{
  auto kinematics = autoware_perception_msgs::msg::DetectedObjectKinematics();

  kinematics.pose_with_covariance.pose = make<geometry_msgs::msg::Pose>(status);

  // clang-format off
  kinematics.pose_with_covariance.covariance = {
    /*
       Row-major representation of the 6x6 covariance matrix. The orientation
       parameters use a fixed-axis representation. In order, the parameters
       are: (x, y, z, rotation about X axis, rotation about Y axis, rotation
       about Z axis)
    */
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1,
  };
  // clang-format on

  kinematics.twist_with_covariance.twist = make<geometry_msgs::msg::Twist>(status);

  kinematics.orientation_availability = [&]() {
    switch (status.subtype().value()) {
      case traffic_simulator_msgs::EntitySubtype::BICYCLE:
      case traffic_simulator_msgs::EntitySubtype::MOTORCYCLE:
        return autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
      default:
        return autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
    }
  }();

  return kinematics;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::Shape
{
  auto shape = autoware_perception_msgs::msg::Shape();
  simulation_interface::toMsg(status.bounding_box().dimensions(), shape.dimensions);
  shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  return shape;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::DetectedObject
{
  auto detected_object = autoware_perception_msgs::msg::DetectedObject();
  detected_object.classification.push_back(
    make<autoware_perception_msgs::msg::ObjectClassification>(status));
  detected_object.kinematics =
    make<autoware_perception_msgs::msg::DetectedObjectKinematics>(status);
  detected_object.shape = make<autoware_perception_msgs::msg::Shape>(status);
  return detected_object;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::TrackedObject
{
  // ref: https://github.com/autowarefoundation/autoware.universe/blob/main/common/perception_utils/src/conversion.cpp
  auto tracked_object = autoware_perception_msgs::msg::TrackedObject();
  const auto detected_object = make<autoware_perception_msgs::msg::DetectedObject>(status);
  // clang-format off
  tracked_object.object_id                           = make<unique_identifier_msgs::msg::UUID>(status);
  tracked_object.existence_probability               = detected_object.existence_probability;
  tracked_object.classification                      = detected_object.classification;
  tracked_object.kinematics.orientation_availability = detected_object.kinematics.orientation_availability;
  tracked_object.kinematics.pose_with_covariance     = detected_object.kinematics.pose_with_covariance;
  tracked_object.kinematics.twist_with_covariance    = detected_object.kinematics.twist_with_covariance;
  tracked_object.shape                               = detected_object.shape;
  // clang-format on
  return tracked_object;
};

template <>
auto DetectionSensor<autoware_perception_msgs::msg::DetectedObjects>::update(
  const double current_simulation_time,
  const std::vector<traffic_simulator_msgs::EntityStatus> & statuses,
  const rclcpp::Time & current_ros_time, const std::vector<std::string> & lidar_detected_entities)
  -> void
{
  if (
    current_simulation_time - previous_simulation_time_ - configuration_.update_duration() >=
    -0.002) {
    previous_simulation_time_ = current_simulation_time;

    const auto ego_entity_status = findEgoEntityStatusToWhichThisSensorIsAttached(statuses);

    auto is_in_range = [&](const auto & status) {
      return not isEgoEntityStatusToWhichThisSensorIsAttached(status) and
             distance(status.pose(), ego_entity_status->pose()) <= configuration_.range() and
             isOnOrAboveEgoPlane(status.pose(), ego_entity_status->pose()) and
             (configuration_.detect_all_objects_in_range() or
              std::find(
                lidar_detected_entities.begin(), lidar_detected_entities.end(), status.name()) !=
                lidar_detected_entities.end());
    };

    /*
       NOTE: for Autoware developers

       If you need to apply experimental noise to the DetectedObjects that the
       simulator publishes, comment out the following function and implement
       new one.
    */
    auto noise_v1 = [&](auto detected_entities, [[maybe_unused]] auto simulation_time) {
      auto position_noise_distribution =
        std::normal_distribution<>(0.0, configuration_.pos_noise_stddev());

      for (auto && detected_entity : detected_entities) {
        detected_entity.mutable_pose()->mutable_position()->set_x(
          detected_entity.pose().position().x() + position_noise_distribution(random_engine_));
        detected_entity.mutable_pose()->mutable_position()->set_y(
          detected_entity.pose().position().y() + position_noise_distribution(random_engine_));
      }

      detected_entities.erase(
        std::remove_if(
          detected_entities.begin(), detected_entities.end(),
          [this](auto &&) {
            return std::uniform_real_distribution()(random_engine_) <
                   configuration_.probability_of_lost();
          }),
        detected_entities.end());

      return detected_entities;
    };

    auto noise_v2 = [&](const auto & detected_entities, auto simulation_time) {
      auto noised_detected_entities = std::decay_t<decltype(detected_entities)>();

      for (auto detected_entity : detected_entities) {
        auto [noise_output, success] =
          noise_outputs.emplace(detected_entity.name(), simulation_time);

        const auto x =
          detected_entity.pose().position().x() - ego_entity_status->pose().position().x();
        const auto y =
          detected_entity.pose().position().y() - ego_entity_status->pose().position().y();
        const auto velocity = std::hypot(
          detected_entity.action_status().twist().linear().x(),
          detected_entity.action_status().twist().linear().y());
        const auto interval =
          simulation_time - std::exchange(noise_output->second.simulation_time, simulation_time);

        auto parameter = [this](const auto & name) {
          return concealer::getParameter<double>(
            detected_objects_publisher->get_topic_name() + std::string(".noise.v2.") + name);
        };

        auto parameters = [this](const auto & name) {
          const auto full_name =
            detected_objects_publisher->get_topic_name() + std::string(".noise.v2.") + name;
          const auto parameters = concealer::getParameter<std::vector<double>>(full_name);
          static const auto size = parameters.size();
          if (parameters.size() != size) {
            throw common::Error(
              "The sizes of the arrays given to the parameters of noise model version 2 must be "
              "the same. The parameter ",
              std::quoted(full_name), " is an array of size ", parameters.size(),
              ", and the other arrays are of size ", size, ".");
          } else {
            return parameters;
          }
        };

        /*
           We use AR(1) model to model the autocorrelation coefficients `phi` for 
           noises(distance_noise, yaw_noise) with Gaussian distribution, by the 
           following formula:
            noise(prev_noise) = mean + phi * (prev_noise - mean) + N(0, 1 - phi^2) * standard_deviation
  
           We use Markov process to model the autocorrelation coefficients `rho` 
           for noises(flip, tp) with Bernoulli distribution, by the transition matrix:
            | p_00 p_01 |   ==   | p0 + rho * p1         p1(1 - rho)   |
            | p_10 p_11 |   ==   | p0(1 - rho)           p1 - rho * p0 |

           In the code, we use `phi` for the above autocorrelation coefficients `phi` or `rho`,
           Which is calculated from the time_interval `dt` by the following formula:
            phi(dt) = amplitude * exp(-decay * dt) + offset
        */

        auto ar1_noise = [this](auto previous_noise, auto mean, auto standard_deviation, auto phi) {
          return mean + phi * (previous_noise - mean) +
                 std::normal_distribution<double>(
                   0, standard_deviation * std::sqrt(1 - phi * phi))(random_engine_);
        };

        auto mp_noise = [this](bool is_previous_noise_1, auto p1, auto phi) {
          auto rate = (is_previous_noise_1 ? 1.0 : 0.0) * phi + (1 - phi) * p1;
          return std::uniform_real_distribution<double>()(random_engine_) < rate;
        };

        auto get_phi = [&](const auto & interval, const std::string & name) {
          static const auto amplitude = parameter(name + ".phi_amplitude");
          static const auto decay = parameter(name + ".phi_decay");
          static const auto offset = parameter(name + ".phi_offset");
          return std::clamp(amplitude * std::exp(-interval * decay) + offset, 0.0, 1.0);
        };

        auto selector = [&](auto ellipse_normalized_x_radius, const auto & targets) {
          static const auto ellipse_y_radii = parameters("ellipse_y_radii");
          return [&, ellipse_normalized_x_radius, targets]() {
            /*
               If the parameter `<topic-name>.noise.v2.ellipse_y_radii`
               contains the value 0.0, division by zero will occur here.
               However, in that case, the distance will be NaN, which correctly
               expresses the meaning that "the distance cannot be defined", and
               this function will work without any problems (zero will be
               returned).
            */
            const auto distance = std::hypot(x / ellipse_normalized_x_radius, y);
            for (auto i = std::size_t(0); i < ellipse_y_radii.size(); ++i) {
              if (distance < ellipse_y_radii[i]) {
                return targets[i];
              }
            }
            return 0.0;
          };
        };

        noise_output->second.distance_noise = [&]() {
          static const auto mean = selector(
            parameter("distance.ellipse_normalized_x_radius.mean"), parameters("distance.means"));
          static const auto standard_deviation = selector(
            parameter("distance.ellipse_normalized_x_radius.standard_deviation"),
            parameters("distance.standard_deviations"));
          const auto phi = get_phi(interval, "distance");
          return ar1_noise(noise_output->second.distance_noise, mean(), standard_deviation(), phi);
        }();

        noise_output->second.yaw_noise = [&]() {
          static const auto mean =
            selector(parameter("yaw.ellipse_normalized_x_radius.mean"), parameters("yaw.means"));
          static const auto standard_deviation = selector(
            parameter("yaw.ellipse_normalized_x_radius.standard_deviation"),
            parameters("yaw.standard_deviations"));
          const auto phi = get_phi(interval, "yaw");
          return ar1_noise(noise_output->second.yaw_noise, mean(), standard_deviation(), phi);
        }();

        noise_output->second.flip = [&]() {
          static const auto velocity_threshold = parameter("yaw_flip.velocity_threshold");
          static const auto flip_rate = parameter("yaw_flip.flip_rate");
          const auto phi = get_phi(interval, "yaw_flip");
          return velocity < velocity_threshold and
                  mp_noise(noise_output->second.flip, flip_rate, phi);
        }();

        noise_output->second.tp = [&]() {
          static const auto tp_rate = selector(
            parameter("tp.ellipse_normalized_x_radius"), parameters("tp.tp_rates"));
          const auto phi = get_phi(interval, "tp");
          return mp_noise(noise_output->second.tp, tp_rate(), phi);
        }();

        if (noise_output->second.tp) {
          const auto angle = std::atan2(y, x);

          const auto yaw_rotated_orientation =
            tf2::Quaternion(
              detected_entity.pose().orientation().x(), detected_entity.pose().orientation().y(),
              detected_entity.pose().orientation().z(), detected_entity.pose().orientation().w()) *
            tf2::Quaternion(
              tf2::Vector3(0, 0, 1),
              noise_output->second.yaw_noise + (noise_output->second.flip ? M_PI : 0.0));

          detected_entity.mutable_pose()->mutable_position()->set_x(
            detected_entity.pose().position().x() +
            noise_output->second.distance_noise * std::cos(angle));
          detected_entity.mutable_pose()->mutable_position()->set_y(
            detected_entity.pose().position().y() +
            noise_output->second.distance_noise * std::sin(angle));
          detected_entity.mutable_pose()->mutable_orientation()->set_x(
            yaw_rotated_orientation.getX());
          detected_entity.mutable_pose()->mutable_orientation()->set_y(
            yaw_rotated_orientation.getY());
          detected_entity.mutable_pose()->mutable_orientation()->set_z(
            yaw_rotated_orientation.getZ());
          detected_entity.mutable_pose()->mutable_orientation()->set_w(
            yaw_rotated_orientation.getW());

          noised_detected_entities.push_back(detected_entity);
        }
      }

      return noised_detected_entities;
    };

    auto noise = [&](auto &&... xs) {
      switch (noise_model_version) {
        default:
          [[fallthrough]];
        case 1:
          return noise_v1(std::forward<decltype(xs)>(xs)...);
        case 2:
          return noise_v2(std::forward<decltype(xs)>(xs)...);
      }
    };

    auto make_detected_objects = [&](const auto & detected_entities) {
      auto detected_objects = autoware_perception_msgs::msg::DetectedObjects();
      detected_objects.header.stamp = current_ros_time;
      detected_objects.header.frame_id = "map";
      for (const auto & detected_entity : detected_entities) {
        detected_objects.objects.push_back(
          make<autoware_perception_msgs::msg::DetectedObject>(detected_entity));
      }
      return detected_objects;
    };

    auto make_ground_truth_objects = [&](const auto & detected_entities) {
      auto ground_truth_objects = autoware_perception_msgs::msg::TrackedObjects();
      ground_truth_objects.header.stamp = current_ros_time;
      ground_truth_objects.header.frame_id = "map";
      for (const auto & detected_entity : detected_entities) {
        ground_truth_objects.objects.push_back(
          make<autoware_perception_msgs::msg::TrackedObject>(detected_entity));
      }
      return ground_truth_objects;
    };

    auto detected_entities = std::vector<traffic_simulator_msgs::EntityStatus>();

    std::copy_if(
      statuses.begin(), statuses.end(), std::back_inserter(detected_entities), is_in_range);

    unpublished_detected_entities.emplace(detected_entities, current_simulation_time);

    if (
      current_simulation_time - unpublished_detected_entities.front().second >=
      configuration_.object_recognition_delay()) {
      const auto modified_detected_entities =
        std::apply(noise, unpublished_detected_entities.front());
      detected_objects_publisher->publish(make_detected_objects(modified_detected_entities));
      unpublished_detected_entities.pop();
    }

    unpublished_ground_truth_entities.emplace(detected_entities, current_simulation_time);

    if (
      current_simulation_time - unpublished_ground_truth_entities.front().second >=
      configuration_.object_recognition_ground_truth_delay()) {
      ground_truth_objects_publisher->publish(
        make_ground_truth_objects(unpublished_ground_truth_entities.front().first));
      unpublished_ground_truth_entities.pop();
    }
  }
}
}  // namespace simple_sensor_simulator
