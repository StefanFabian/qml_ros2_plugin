// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

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
