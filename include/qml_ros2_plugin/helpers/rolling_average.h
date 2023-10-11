// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_ROLLING_AVERAGE_H
#define QML_ROS2_PLUGIN_ROLLING_AVERAGE_H

#include <array>

namespace qml_ros2_plugin
{

template<typename T, int COUNT>
class RollingAverage
{
public:
  RollingAverage() { values_.fill( 0 ); }

  void add( const T &value )
  {
    if ( count_values_ == COUNT )
      sum_ -= values_[index_];
    values_[index_] = value;
    sum_ += value;
    if ( ++index_ == COUNT )
      index_ = 0;
    if ( count_values_ != COUNT )
      ++count_values_;
  }

  T value() const { return count_values_ == 0 ? 0 : sum_ / count_values_; }

  operator T() const { return value(); }

private:
  std::array<T, COUNT> values_;
  T sum_ = 0;
  size_t count_values_ = 0;
  size_t index_ = 0;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_ROLLING_AVERAGE_H
