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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_PERFORMANCE_MONITOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_PERFORMANCE_MONITOR_HPP_

#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <vector>

namespace simple_sensor_simulator
{
class LidarPerformanceMonitor
{
public:
  enum class MetricType {
    RAYCAST,
    NOISE,
    RAYCAST_ADD_ENTITIES,
    RAYCAST_COMMIT_SCENE,
    RAYCAST_INTERSECT,
    RAYCAST_CONVERT_IDS,
    RAYCAST_REMOVE_ENTITIES
  };

  class MeasurementScope
  {
  private:
    LidarPerformanceMonitor & monitor_;
    MetricType type_;
    std::chrono::high_resolution_clock::time_point start_time_;

  public:
    MeasurementScope(LidarPerformanceMonitor & monitor, MetricType type)
    : monitor_(monitor), type_(type), start_time_(std::chrono::high_resolution_clock::now())
    {
    }

    ~MeasurementScope()
    {
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);
      monitor_.addSample(type_, static_cast<double>(duration.count()));
    }
  };

private:
  struct ProcessingTimeStats
  {
    std::vector<double> samples;
    static constexpr size_t MAX_SAMPLES = 1000;

    void addSample(double time_us)
    {
      samples.push_back(time_us);
      if (samples.size() > MAX_SAMPLES) {
        samples.erase(samples.begin(), samples.begin() + (samples.size() - MAX_SAMPLES));
      }
    }

    double getMean() const
    {
      if (samples.empty()) return 0.0;
      return std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    }

    double getStdDev() const
    {
      if (samples.size() < 2) return 0.0;
      double mean = getMean();
      double variance = 0.0;
      for (double sample : samples) {
        variance += (sample - mean) * (sample - mean);
      }
      variance /= samples.size();
      return std::sqrt(variance);
    }

    size_t getSampleCount() const { return samples.size(); }
  };

  ProcessingTimeStats raycast_stats_;
  ProcessingTimeStats noise_stats_;
  ProcessingTimeStats raycast_add_entities_stats_;
  ProcessingTimeStats raycast_commit_scene_stats_;
  ProcessingTimeStats raycast_intersect_stats_;
  ProcessingTimeStats raycast_convert_ids_stats_;
  ProcessingTimeStats raycast_remove_entities_stats_;

  // Additional statistics
  std::vector<size_t> beam_counts_;
  std::vector<size_t> entity_counts_;
  std::vector<size_t> hit_point_counts_;

  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

public:
  explicit LidarPerformanceMonitor()
  : logger_(rclcpp::get_logger("lidar_sensor")), clock_(RCL_ROS_TIME)
  {
  }

  ~LidarPerformanceMonitor()
  {
    // Generate timestamp-based filename
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
    localtime_r(&now_time_t, &tm_buf);

    std::ostringstream filename;
    filename << "lidar_performance_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".txt";

    // Write summary to file
    std::ofstream file(filename.str());
    if (!file.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open performance summary file: %s", filename.str().c_str());
      return;
    }

    file << "LiDAR Performance Summary\n";
    file << "=========================\n";
    file << "Generated at: " << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "\n\n";

    // Calculate average counts
    auto avg = [](const std::vector<size_t> & vec) -> double {
      if (vec.empty()) return 0.0;
      return std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
    };

    file << "Configuration:\n";
    file << "  Avg Beam Count:      " << std::fixed << std::setprecision(0) << avg(beam_counts_)
         << "\n";
    file << "  Avg Entity Count:    " << std::fixed << std::setprecision(1) << avg(entity_counts_)
         << "\n";
    file << "  Avg Hit Point Count: " << std::fixed << std::setprecision(0)
         << avg(hit_point_counts_) << "\n\n";

    file << "Raycast Statistics (Total):\n";
    file << "  Mean:    " << std::fixed << std::setprecision(1) << raycast_stats_.getMean()
         << " μs\n";
    file << "  Std Dev: " << std::fixed << std::setprecision(1) << raycast_stats_.getStdDev()
         << " μs\n";
    file << "  Samples: " << raycast_stats_.getSampleCount() << "\n\n";

    if (raycast_add_entities_stats_.getSampleCount() > 0) {
      file << "Raycast Breakdown:\n";
      file << "  Add Entities:    " << std::fixed << std::setprecision(1)
           << raycast_add_entities_stats_.getMean() << " μs (±"
           << raycast_add_entities_stats_.getStdDev() << ")\n";
      file << "  Commit Scene:    " << std::fixed << std::setprecision(1)
           << raycast_commit_scene_stats_.getMean() << " μs (±"
           << raycast_commit_scene_stats_.getStdDev() << ")\n";
      file << "  Intersect:       " << std::fixed << std::setprecision(1)
           << raycast_intersect_stats_.getMean() << " μs (±"
           << raycast_intersect_stats_.getStdDev() << ")\n";
      file << "  Convert IDs:     " << std::fixed << std::setprecision(1)
           << raycast_convert_ids_stats_.getMean() << " μs (±"
           << raycast_convert_ids_stats_.getStdDev() << ")\n";
      file << "  Remove Entities: " << std::fixed << std::setprecision(1)
           << raycast_remove_entities_stats_.getMean() << " μs (±"
           << raycast_remove_entities_stats_.getStdDev() << ")\n\n";

      double total_breakdown = raycast_add_entities_stats_.getMean() +
                               raycast_commit_scene_stats_.getMean() +
                               raycast_intersect_stats_.getMean() +
                               raycast_convert_ids_stats_.getMean() +
                               raycast_remove_entities_stats_.getMean();
      file << "  Total Breakdown: " << std::fixed << std::setprecision(1) << total_breakdown
           << " μs\n";
      file << "  Overhead:        " << std::fixed << std::setprecision(1)
           << (raycast_stats_.getMean() - total_breakdown) << " μs\n\n";
    }


    if (noise_stats_.getSampleCount() > 0) {
      file << "Noise Statistics:\n";
      file << "  Mean:    " << std::fixed << std::setprecision(1) << noise_stats_.getMean()
           << " μs\n";
      file << "  Std Dev: " << std::fixed << std::setprecision(1) << noise_stats_.getStdDev()
           << " μs\n";
      file << "  Samples: " << noise_stats_.getSampleCount() << "\n";
    }

    file.close();
    RCLCPP_INFO(logger_, "Performance summary written to: %s", filename.str().c_str());
  }

  MeasurementScope startMeasurement(MetricType type) { return MeasurementScope(*this, type); }

  void addSample(MetricType type, double time_us)
  {
    switch (type) {
      case MetricType::RAYCAST:
        raycast_stats_.addSample(time_us);
        break;
      case MetricType::NOISE:
        noise_stats_.addSample(time_us);
        break;
      case MetricType::RAYCAST_ADD_ENTITIES:
        raycast_add_entities_stats_.addSample(time_us);
        break;
      case MetricType::RAYCAST_COMMIT_SCENE:
        raycast_commit_scene_stats_.addSample(time_us);
        break;
      case MetricType::RAYCAST_INTERSECT:
        raycast_intersect_stats_.addSample(time_us);
        break;
      case MetricType::RAYCAST_CONVERT_IDS:
        raycast_convert_ids_stats_.addSample(time_us);
        break;
      case MetricType::RAYCAST_REMOVE_ENTITIES:
        raycast_remove_entities_stats_.addSample(time_us);
        break;
    }
  }

  void recordRaycastInfo(size_t beam_count, size_t entity_count, size_t hit_point_count)
  {
    beam_counts_.push_back(beam_count);
    entity_counts_.push_back(entity_count);
    hit_point_counts_.push_back(hit_point_count);

    // Keep only the last 1000 samples
    if (beam_counts_.size() > 1000) {
      beam_counts_.erase(beam_counts_.begin());
      entity_counts_.erase(entity_counts_.begin());
      hit_point_counts_.erase(hit_point_counts_.begin());
    }
  }

  void outputStats(bool has_noise_processor)
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_, 5000, "Raycast stats: avg=%.1f μs, std=%.1f μs, samples=%zu",
      raycast_stats_.getMean(), raycast_stats_.getStdDev(), raycast_stats_.getSampleCount());

    if (has_noise_processor) {
      RCLCPP_INFO_THROTTLE(
        logger_, clock_, 5000, "Noise stats: avg=%.1f μs, std=%.1f μs, samples=%zu",
        noise_stats_.getMean(), noise_stats_.getStdDev(), noise_stats_.getSampleCount());
    }
  }
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_PERFORMANCE_MONITOR_HPP_
