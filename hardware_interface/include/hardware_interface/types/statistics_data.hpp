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

#ifndef HARDWARE_INTERFACE__TYPES__STATISTICS_DATA_HPP_
#define HARDWARE_INTERFACE__TYPES__STATISTICS_DATA_HPP_

#include <cstdint>
#include <limits>

namespace ros2_control
{
/// Results for a set of recorded observations; values are NaN until observations are available.
struct StatisticData
{
  double average = std::numeric_limits<double>::quiet_NaN();
  double min = std::numeric_limits<double>::quiet_NaN();
  double max = std::numeric_limits<double>::quiet_NaN();
  double standard_deviation = std::numeric_limits<double>::quiet_NaN();
  uint64_t sample_count = 0;
};
}  // namespace ros2_control

#endif  // HARDWARE_INTERFACE__TYPES__STATISTICS_DATA_HPP_
