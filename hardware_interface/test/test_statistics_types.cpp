// Copyright 2026 ros2_control Development Team
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
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "hardware_interface/types/statistics_types.hpp"

TEST(TestStatisticsTypes, default_results_are_empty)
{
  const ros2_control::StatisticData data;
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(data.sample_count, 0u);
}

TEST(TestStatisticsTypes, measurements_and_reset_preserve_results)
{
  auto statistics = std::make_shared<ros2_control::MovingAverageStatistics>();
  statistics->reset();
  statistics->add_measurement(1.0);
  statistics->add_measurement(3.0);
  statistics->add_measurement(5.0);
  statistics->add_measurement(std::numeric_limits<double>::quiet_NaN());
  const auto data = statistics->get_statistics();
  EXPECT_DOUBLE_EQ(data.average, 3.0);
  EXPECT_DOUBLE_EQ(data.min, 1.0);
  EXPECT_DOUBLE_EQ(data.max, 5.0);
  EXPECT_DOUBLE_EQ(data.standard_deviation, std::sqrt(8.0 / 3.0));
  EXPECT_EQ(data.sample_count, 3u);

  ros2_control::MovingAverageStatisticsData snapshot;
  EXPECT_TRUE(std::isnan(snapshot.get_statistics().average));
  snapshot.update_statistics(statistics);
  EXPECT_DOUBLE_EQ(snapshot.get_statistics().average, data.average);
  EXPECT_EQ(snapshot.get_statistics().sample_count, data.sample_count);
  snapshot.reset();
  EXPECT_TRUE(std::isnan(snapshot.get_statistics().average));
  EXPECT_EQ(snapshot.get_statistics().sample_count, 0u);
  statistics->reset();
  EXPECT_EQ(statistics->get_count(), 0u);
  EXPECT_TRUE(std::isnan(statistics->get_current_measurement()));
}

TEST(TestStatisticsTypes, introspection_registers_and_reads_all_fields)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>(
      "test_statistics_types", rclcpp::NodeOptions().context(context));
    ros2_control::StatisticData data;
    data.average = 3.0;
    data.min = 1.0;
    data.max = 5.0;
    data.standard_deviation = 2.0;
    data.sample_count = 3;
    pal_statistics::StatisticsRegistry registry(node, "~/statistics");
    pal_statistics::customRegister(registry, "timing", &data, nullptr, true);
    const auto message = registry.createMsg();
    const auto expect_field = [&message](const std::string & name, double value)
    {
      const auto field = std::find_if(
        message.statistics.begin(), message.statistics.end(),
        [&name](const auto & statistic) { return statistic.name == name; });
      ASSERT_NE(field, message.statistics.end());
      EXPECT_DOUBLE_EQ(field->value, value);
    };
    expect_field("timing/average", 3.0);
    expect_field("timing/min", 1.0);
    expect_field("timing/max", 5.0);
    expect_field("timing/standard_deviation", 2.0);
    expect_field("timing/sample_count", 3.0);
    data.sample_count = 7;
    const auto updated = registry.createMsg();
    const auto count = std::find_if(
      updated.statistics.begin(), updated.statistics.end(),
      [](const auto & statistic) { return statistic.name == "timing/sample_count"; });
    ASSERT_NE(count, updated.statistics.end());
    EXPECT_DOUBLE_EQ(count->value, 7.0);
  }
  context->shutdown("test complete");
}
