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
  enum class MetricType { RAYCAST, NOISE };

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

public:
  explicit LidarPerformanceMonitor() = default;

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
      return;
    }

    file << "LiDAR Performance Summary\n";
    file << "=========================\n";
    file << "Generated at: " << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "\n\n";

    file << "Raycast Statistics:\n";
    file << "  Mean:    " << std::fixed << std::setprecision(1) << raycast_stats_.getMean()
         << " μs\n";
    file << "  Std Dev: " << std::fixed << std::setprecision(1) << raycast_stats_.getStdDev()
         << " μs\n";
    file << "  Samples: " << raycast_stats_.getSampleCount() << "\n\n";

    if (noise_stats_.getSampleCount() > 0) {
      file << "Noise Statistics:\n";
      file << "  Mean:    " << std::fixed << std::setprecision(1) << noise_stats_.getMean()
           << " μs\n";
      file << "  Std Dev: " << std::fixed << std::setprecision(1) << noise_stats_.getStdDev()
           << " μs\n";
      file << "  Samples: " << noise_stats_.getSampleCount() << "\n";
    }

    file.close();
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
    }
  }
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_PERFORMANCE_MONITOR_HPP_
