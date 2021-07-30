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

#ifndef QML_ROS2_PLUGIN_GOAL_STATUS_HPP
#define QML_ROS2_PLUGIN_GOAL_STATUS_HPP

#include <QMetaEnum>
#include <rclcpp_action/types.hpp>

namespace qml_ros2_plugin
{

namespace action_goal_status
{
Q_NAMESPACE
enum GoalStatus {
  Aborted = rclcpp_action::GoalStatus::STATUS_ABORTED,
  Accepted = rclcpp_action::GoalStatus::STATUS_ACCEPTED,
  Canceled = rclcpp_action::GoalStatus::STATUS_CANCELED,
  Canceling = rclcpp_action::GoalStatus::STATUS_CANCELING,
  Executing = rclcpp_action::GoalStatus::STATUS_EXECUTING,
  Succeeded = rclcpp_action::GoalStatus::STATUS_SUCCEEDED,
  Unknown = rclcpp_action::GoalStatus::STATUS_UNKNOWN
};

Q_ENUM_NS( GoalStatus )
} // namespace action_goal_status
} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( qml_ros2_plugin::action_goal_status::GoalStatus )

#endif // QML_ROS2_PLUGIN_GOAL_STATUS_HPP
