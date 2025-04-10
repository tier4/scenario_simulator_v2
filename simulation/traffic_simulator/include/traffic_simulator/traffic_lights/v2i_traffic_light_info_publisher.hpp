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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_INFO_PUBLISHER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_INFO_PUBLISHER_HPP_

#include <jpn_signal_v2i_msgs/msg/traffic_light_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>

namespace traffic_simulator
{
using Message = jpn_signal_v2i_msgs::msg::TrafficLightInfo;
class V2ITrafficLightInfoPublisher
{
  const typename rclcpp::Publisher<Message>::SharedPtr publisher_;

  lanelet::Ids traffic_light_ids_;

  struct TrafficLightExtraInfo
  {
    // way id or regular element id
    lanelet::Id way_id;
    double current_phase_rest_time;
    double rest_time_to_red;
  };

  std::unordered_map<lanelet::Id, TrafficLightExtraInfo> traffic_light_extra_info_;

  TrafficLightsBase * traffic_lights_base_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;

public:
  template <typename NodePointer>
  explicit V2ITrafficLightInfoPublisher(
    const std::string & topic_name, const NodePointer & node,
    TrafficLightsBase * traffic_lights_base,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
  : publisher_(
      rclcpp::create_publisher<Message>(node, topic_name, rclcpp::QoS(10).transient_local())),
    traffic_lights_base_(traffic_lights_base),
    hdmap_utils_(hdmap_utils)
  {
    // TODO: get vehicle ID from ros parameter and set
  }

  auto publish() -> void
  {
    Message msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "map";

    for (const auto & extra_info : traffic_light_extra_info_) {
      auto traffic_light = traffic_lights_base_->getTrafficLight(extra_info.second.way_id);

      jpn_signal_v2i_msgs::msg::ExtraTrafficSignal extra_traffic_signal;
      extra_traffic_signal.header = msg.header;
      extra_traffic_signal.has_min_rest_time =
        (not std::isinf(static_cast<float>(extra_info.second.current_phase_rest_time)));
      extra_traffic_signal.min_rest_time =
        extra_traffic_signal.has_min_rest_time ? extra_info.second.current_phase_rest_time : 0.0;
      extra_traffic_signal.has_max_rest_time = extra_traffic_signal.has_min_rest_time;
      // copy min_rest_time to max_rest_time
      extra_traffic_signal.max_rest_time = extra_traffic_signal.min_rest_time;
      extra_traffic_signal.min_rest_time_to_red = extra_info.second.rest_time_to_red;

      auto relation_ids =
        hdmap_utils_->getTrafficLightRegulatoryElementIDsFromTrafficLight(extra_info.second.way_id);
      autoware_perception_msgs::msg::TrafficLightGroup traffic_signal_msg;
      {
        auto traffic_light_proto = static_cast<simulation_api_schema::TrafficSignal>(traffic_light);
        for (auto bulb_status : traffic_light_proto.traffic_light_status()) {
          using TrafficLightBulbType =
            autoware_perception_msgs::msg::TrafficLightGroup::_elements_type::value_type;
          TrafficLightBulbType light_bulb_message;
          simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
          traffic_signal_msg.elements.push_back(light_bulb_message);
        }
      }

      for (const auto & relation_id : relation_ids) {
        traffic_signal_msg.traffic_light_group_id = relation_id;
        extra_traffic_signal.states.push_back(traffic_signal_msg);
      }
      msg.car_lights.push_back(extra_traffic_signal);
    }

    publisher_->publish(msg);
  }

  void setTrafficLightExtraInfo(
    lanelet::Id id, double current_phase_rest_time, double rest_time_to_red)
  {
    TrafficLightExtraInfo info;
    info.current_phase_rest_time = current_phase_rest_time;
    info.rest_time_to_red = rest_time_to_red;

    if (hdmap_utils_->isTrafficLightRegulatoryElement(id)) {
      for (const auto & traffic_light : traffic_lights_base_->getTrafficLights(id)) {
        info.way_id = traffic_light.get().way_id;
        traffic_light_extra_info_.insert_or_assign(info.way_id, info);
      }
    } else {
      info.way_id = id;
      traffic_light_extra_info_.insert_or_assign(info.way_id, info);
    }
  }
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_INFO_PUBLISHER_HPP_
