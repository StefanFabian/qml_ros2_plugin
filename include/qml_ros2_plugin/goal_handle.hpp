// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_GOAL_HANDLE_HPP
#define QML_ROS2_PLUGIN_GOAL_HANDLE_HPP

#include "qml_ros2_plugin/goal_status.hpp"
#include "qml_ros2_plugin/qobject_ros2.hpp"
#include "qml_ros2_plugin/time.hpp"

#include <ros_babel_fish/babel_fish.hpp>

namespace qml_ros2_plugin
{

class GoalHandle : public QObjectRos2
{
  Q_OBJECT
  //! The goal status in form of an action_goal_status enum value:
  //! Aborted, Accepted, Canceled, Canceling, Executing, Succeeded, Unknown
  Q_PROPERTY( qml_ros2_plugin::action_goal_status::GoalStatus status READ status )
  Q_PROPERTY( QString goalId READ goalId )
  Q_PROPERTY( qml_ros2_plugin::Time goalStamp READ goalStamp )
public:
  explicit GoalHandle( ros_babel_fish::BabelFishActionClient::SharedPtr client,
                       ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr handle );

  qml_ros2_plugin::action_goal_status::GoalStatus status() const;

  QString goalId() const;

  qml_ros2_plugin::Time goalStamp() const;

  //! Sends a cancellation request to the ActionServer.
  Q_INVOKABLE void cancel();

protected:
  void onRos2Shutdown() override;

private:
  ros_babel_fish::BabelFish babel_fish_;
  // Store the client to make sure its destructed after the goal handles
  ros_babel_fish::BabelFishActionClient::SharedPtr client_;
  ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr goal_handle_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_GOAL_HANDLE_HPP
