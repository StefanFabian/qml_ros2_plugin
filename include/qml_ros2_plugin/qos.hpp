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

#ifndef QML_ROS2_PLUGIN_QOS_HPP
#define QML_ROS2_PLUGIN_QOS_HPP

#include <QtCore>
#include <rclcpp/qos.hpp>
#include <rmw/types.h>

namespace qml_ros2_plugin
{

namespace qos_policy
{
Q_NAMESPACE

enum HistoryPolicy {
  HistorySystemDefault = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
  HistoryKeepLast = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  HistoryKeepAll = RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  HistoryUnknown = RMW_QOS_POLICY_HISTORY_UNKNOWN
};
Q_ENUM_NS(HistoryPolicy)
}

class QoS
{
  Q_GADGET
public:

  explicit QoS( quint32 history_depth = 1 );

  Q_INVOKABLE qml_ros2_plugin::QoS avoid_ros_namespace_conventions( bool value );

  Q_INVOKABLE qml_ros2_plugin::QoS history( qos_policy::HistoryPolicy history );

  Q_INVOKABLE qml_ros2_plugin::QoS transient_local();

  Q_INVOKABLE qml_ros2_plugin::QoS durability_volatile();

  Q_INVOKABLE qml_ros2_plugin::QoS best_effort();

  Q_INVOKABLE qml_ros2_plugin::QoS reliable();

  Q_INVOKABLE qml_ros2_plugin::QoS keep_all();

  Q_INVOKABLE qml_ros2_plugin::QoS keep_last( quint32 depth );

  const rclcpp::QoS &getQoS() const;

private:
  rclcpp::QoS qos_;
};
} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( qml_ros2_plugin::QoS )

#endif // QML_ROS2_PLUGIN_QOS_HPP
