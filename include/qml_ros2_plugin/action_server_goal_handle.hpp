// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_ACTION_SERVER_GOAL_HANDLE_HPP
#define QML_ROS2_PLUGIN_ACTION_SERVER_GOAL_HANDLE_HPP

#include "qml_ros2_plugin/qobject_ros2.hpp"

#include <QVariant>
#include <chrono>
#include <ros_babel_fish/babel_fish.hpp>

namespace qml_ros2_plugin
{

/*!
 * Server-side handle for a single accepted action goal.
 *
 * Instances are created by an ActionServer and passed to the onGoalAccepted handler.
 * They can not be created in QML.
 *
 * Use this handle to publish feedback and to terminate the goal using succeed, abort or canceled.
 * After a terminal call (succeed/abort/canceled) the handle is invalid and further calls are no-ops.
 */
class ActionServerGoalHandle : public QObjectRos2
{
  Q_OBJECT
  //! The goal value for this handle.
  Q_PROPERTY( QVariant goal READ goal CONSTANT )
  //! The uuid of the goal.
  Q_PROPERTY( QString goalId READ goalId CONSTANT )
  //! True if the goal is currently in the canceling state.
  Q_PROPERTY( bool isCanceling READ isCanceling NOTIFY statusChanged )
  //! True if the goal is in an active state (accepted or executing).
  Q_PROPERTY( bool isActive READ isActive NOTIFY statusChanged )
  //! True if the goal is currently executing.
  Q_PROPERTY( bool isExecuting READ isExecuting NOTIFY statusChanged )

public:
  explicit ActionServerGoalHandle(
      const std::shared_ptr<ros_babel_fish::BabelFishActionServerGoalHandle> &handle,
      QObject *parent = nullptr );

  const QVariant &goal() const;

  QString goalId() const;

  bool isCanceling() const;

  bool isActive() const;

  bool isExecuting() const;

  //! Marks the goal as succeeded and sends the given result to the client.
  Q_INVOKABLE void succeed( QVariantMap result = {} );

  //! Marks the goal as aborted and sends the given result to the client.
  Q_INVOKABLE void abort( QVariantMap result = {} );

  //! Marks the goal as canceled and sends the given result to the client.
  //! Only valid once the goal has entered the canceling state, i.e., call this from the
  //! cancelRequested handler, not synchronously from the handleCancel callback (where the goal is
  //! still executing and the transition would be rejected).
  Q_INVOKABLE void canceled( QVariantMap result = {} );

  //! Publishes a feedback message for this goal.
  Q_INVOKABLE void publishFeedback( QVariantMap feedback );

  //! Called by the ActionServer when a cancel request for this goal was accepted.
  void notifyCancelRequested();

signals:

  //! Emitted whenever the goal state changes.
  void statusChanged();

  //! Emitted when a cancellation of this goal was requested and accepted.
  void cancelRequested();

  //! Emitted when the goal reached a terminal state (succeeded, aborted or canceled).
  void terminated();

private:
  //! Emits cancelRequested once the goal has actually entered the canceling state.
  void deliverCancelRequested();

  //! Shared implementation of succeed/abort/canceled: fills the result and invokes the matching
  //! terminal transition (terminal), then flips the handle to the terminated state.
  void terminate( const char *verb, const QVariantMap &result,
                  void ( ros_babel_fish::BabelFishActionServerGoalHandle::*terminal )(
                      const ros_babel_fish::CompoundMessage & ) );

  bool checkActive( const char *method );

  std::shared_ptr<ros_babel_fish::BabelFishActionServerGoalHandle> handle_;
  QVariant goal_;
  QString goal_id_;
  bool terminated_ = false;
  bool cancel_pending_ = false;
  std::chrono::steady_clock::time_point cancel_request_deadline_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_ACTION_SERVER_GOAL_HANDLE_HPP
