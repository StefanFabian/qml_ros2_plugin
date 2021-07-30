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

#ifndef QML_ROS2_PLUGIN_GOAL_HANDLE_HPP
#define QML_ROS2_PLUGIN_GOAL_HANDLE_HPP

#include "qml_ros2_plugin/goal_status.hpp"
#include "qml_ros2_plugin/qobject_ros2.hpp"
#include "qml_ros2_plugin/time.hpp"

#include <ros2_babel_fish/babel_fish.hpp>

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
  explicit GoalHandle( ros2_babel_fish::BabelFishActionClient::SharedPtr client,
                       ros2_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr handle );

  qml_ros2_plugin::action_goal_status::GoalStatus status() const;

  QString goalId() const;

  qml_ros2_plugin::Time goalStamp() const;

  //! Sends a cancellation request to the ActionServer.
  Q_INVOKABLE void cancel();

protected:
  void onRos2Shutdown() override;

private:
  ros2_babel_fish::BabelFish babel_fish_;
  // Store the client to make sure its destructed after the goal handles
  ros2_babel_fish::BabelFishActionClient::SharedPtr client_;
  ros2_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr goal_handle_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_GOAL_HANDLE_HPP
