// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_ACTION_CLIENT_HPP
#define QML_ROS2_PLUGIN_ACTION_CLIENT_HPP

#include "qml_ros2_plugin/goal_status.hpp"
#include "qml_ros2_plugin/qobject_ros2.hpp"
#include "qml_ros2_plugin/time.hpp"

#include <ros_babel_fish/babel_fish.hpp>

#include <QJSValue>
#include <QTimer>

namespace qml_ros2_plugin
{
class NodeHandle;

class GoalHandle;

class ActionClient : public QObjectRos2
{
  Q_OBJECT
  //! True if the ActionClient is connected to the ActionServer, false otherwise.
  Q_PROPERTY( bool ready READ isServerReady NOTIFY serverReadyChanged )
  //! The type of the action. Example: action_tutorials_interfaces/action/Fibonacci
  Q_PROPERTY( QString actionType READ actionType CONSTANT )
public:
  ActionClient( const QString &name, const QString &action_type );

  Q_INVOKABLE bool isServerReady() const;

  QString actionType() const;

  /*!
   * Sends a goal to the action server if it is connected.
   *
   * @param goal The goal that is sent to the action server.
   * @param options An object with optional callback members for:
   *   * goal_response: onGoalResponse(goal_handle)
   *       goal_handle will be null if goal was rejected
   *   * feedback: onFeedback(goal_handle, feedback_message)
   *   * result: onResult(wrapped_result)
   *       where wrapped_result has a *goalId*, result *code* and *result* message.
   * @return null if the action server is not connected, otherwise a GoalHandle keeping track of the state of the goal.
   */
  Q_INVOKABLE QObject *sendGoalAsync( const QVariantMap &goal, QJSValue options = QJSValue() );

  //! Cancels all goals that are currently tracked by this client.
  Q_INVOKABLE void cancelAllGoals();

  //! Cancels all goals that were sent at and before the given ROS time by this client.
  //! Use Time.now() to obtain the current ROS time which can differ from the actual time.
  Q_INVOKABLE void cancelGoalsBefore( const qml_ros2_plugin::Time &time );

  //! @copydoc cancelGoalsBefore(const Time&)
  Q_INVOKABLE void cancelGoalsBefore( const QDateTime &time );

signals:

  //! Emitted when the connected status changes, e.g., when the client connected to the server.
  void serverReadyChanged();

private slots:

  void checkServerReady();

  void
  invokeGoalResponseCallback( QJSValue callback,
                              ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr handle );

  void invokeFeedbackCallback( QJSValue callback,
                               ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr handle,
                               ros_babel_fish::CompoundMessage::ConstSharedPtr feedback );

  void invokeResultCallback( QJSValue callback, QString goal_id,
                             qml_ros2_plugin::action_goal_status::GoalStatus result_code,
                             ros_babel_fish::CompoundMessage::ConstSharedPtr result );

private:
  void onRos2Initialized() override;

  void onRos2Shutdown() override;

  ros_babel_fish::BabelFish babel_fish_;
  QString action_type_;
  QString name_;
  ros_babel_fish::BabelFishActionClient::SharedPtr client_;
  QTimer connect_timer_;
};
} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr )

Q_DECLARE_METATYPE( ros_babel_fish::CompoundMessage::ConstSharedPtr )

#endif // QML_ROS2_PLUGIN_ACTION_CLIENT_HPP
