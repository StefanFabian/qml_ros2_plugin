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

#ifndef QML_ROS2_PLUGIN_MESSAGE_CONVERSIONS_HPP
#define QML_ROS2_PLUGIN_MESSAGE_CONVERSIONS_HPP

#include <QVariantMap>
#include <action_msgs/msg/goal_info.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_action/types.hpp>

namespace ros2_babel_fish
{
class BabelFish;
class Message;
} // namespace ros2_babel_fish

namespace qml_ros2_plugin
{

/*!
 * @brief Converts between QVariant and ROS2 messages.
 *
 * Most types have a clear correspondence, e.g., all int fields smaller or equal to 32bit are mapped to int, all uint
 * smaller or equal to 32bit are mapped to uint. Larger are mapped to qlonglong and qulonglong (64bit).
 *
 */
namespace conversion
{
template<typename T>
T &obtainValueAsReference( QVariant &value )
{
  return *reinterpret_cast<T *>( value.data() );
}

template<typename T>
const T &obtainValueAsConstReference( const QVariant &value )
{
  return *reinterpret_cast<const T *>( value.data() );
}

QVariantMap msgToMap( const std_msgs::msg::Header &msg );

QVariantMap msgToMap( const geometry_msgs::msg::Vector3 &msg );

QVariantMap msgToMap( const geometry_msgs::msg::Quaternion &msg );

QVariantMap msgToMap( const geometry_msgs::msg::Transform &msg );

QVariantMap msgToMap( const geometry_msgs::msg::TransformStamped &msg );

QString uuidToString( const rclcpp_action::GoalUUID &uuid );

QVariantMap msgToMap( const unique_identifier_msgs::msg::UUID &msg );

QVariantMap msgToMap( const action_msgs::msg::GoalInfo &msg );

QVariantMap msgToMap( const action_msgs::msg::GoalStatus &msg );

QVariant msgToMap( const std::shared_ptr<const ros2_babel_fish::Message> &msg );

QVariant msgToMap( const ros2_babel_fish::Message &msg );

bool fillMessage( ros2_babel_fish::Message &msg, const QVariant &value );

bool fillMessage( ros2_babel_fish::BabelFish &fish, ros2_babel_fish::Message &msg,
                  const QVariant &value );
} // namespace conversion
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_MESSAGE_CONVERSIONS_HPP
