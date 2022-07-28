/*
* Copyright (C) 2022  Stefan Fabian
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

#include "qml_ros2_plugin/qos.hpp"

namespace qml_ros2_plugin
{

QoS::QoS( quint32 history_depth ) : qos_( history_depth ) { }

qml_ros2_plugin::QoS QoS::avoid_ros_namespace_conventions( bool value )
{
  qos_.avoid_ros_namespace_conventions( value );
  return *this;
}

QoS QoS::history( qos_policy::HistoryPolicy history )
{
  qos_.history( static_cast<rmw_qos_history_policy_t>( history ) );
  return *this;
}

qml_ros2_plugin::QoS QoS::transient_local()
{
  qos_.transient_local();
  return *this;
}

qml_ros2_plugin::QoS QoS::durability_volatile()
{
  qos_.durability_volatile();
  return *this;
}

qml_ros2_plugin::QoS QoS::best_effort()
{
  qos_.best_effort();
  return *this;
}

qml_ros2_plugin::QoS QoS::reliable()
{
  qos_.reliable();
  return *this;
}

qml_ros2_plugin::QoS QoS::keep_all()
{
  qos_.keep_all();
  return *this;
}
qml_ros2_plugin::QoS QoS::keep_last( quint32 depth )
{
  qos_.keep_last( depth );
  return *this;
}

const rclcpp::QoS &QoS::getQoS() const { return qos_; }
} // namespace qml_ros2_plugin
