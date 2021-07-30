/*
 * Copyright (C) 2021  Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
