// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_ACTION_SERVER_HPP
#define QML_ROS2_PLUGIN_ACTION_SERVER_HPP

#include "qml_ros2_plugin/helpers/alive_token.hpp"
#include "qml_ros2_plugin/qobject_ros2.hpp"

#include <QJSValue>
#include <QPointer>
#include <QVariant>
#include <atomic>
#include <ros_babel_fish/babel_fish.hpp>
#include <unordered_map>

namespace qml_ros2_plugin
{
class ActionServerGoalHandle;

/*!
 * A directly-creatable QML element that advertises a ROS action server and handles goals using
 * QML/JavaScript callbacks.
 *
 * Example:
 * @code
 * ActionServer {
 *   name: "/fibonacci"
 *   type: "example_interfaces/action/Fibonacci"
 *   handleGoal: function (goal, goalId) { return goal.order <= 100 } // optional
 *   handleCancel: function (handle) { return true }                  // optional
 *   onGoalAccepted: function (handle) {
 *     // drive execution, e.g., using a QML Timer; handle.publishFeedback({...});
 *     // handle.succeed({ sequence: [...] }) / handle.abort({...})
 *   }
 * }
 * @endcode
 *
 * The handleGoal and handleCancel callbacks are invoked synchronously on the executor and must
 * return quickly. They are optional; without handleGoal all goals are accepted. Without handleCancel
 * all cancel requests are rejected, so an action server must set handleCancel to support cancelling.
 */
class ActionServer : public QObjectRos2
{
  Q_OBJECT
  //! The action name. Setting this (re-)creates the action server.
  Q_PROPERTY( QString name READ name WRITE setName NOTIFY nameChanged )
  //! The type of the action, e.g., "example_interfaces/action/Fibonacci". Setting this (re-)creates
  //! the action server.
  Q_PROPERTY( QString type READ type WRITE setType NOTIFY typeChanged )
  //! Optional callback to decide whether a goal is accepted. Signature: ``function (goal, goalId)``
  //! returning a boolean. If not set, all goals are accepted.
  Q_PROPERTY( QJSValue handleGoal READ handleGoal WRITE setHandleGoal NOTIFY handleGoalChanged )
  //! Optional callback to decide whether a cancel request is accepted. Signature:
  //! ``function (handle)`` returning a boolean. If not set, all cancel requests are rejected.
  Q_PROPERTY( QJSValue handleCancel READ handleCancel WRITE setHandleCancel NOTIFY handleCancelChanged )
  //! Whether the action server is currently advertised.
  Q_PROPERTY( bool advertised READ isAdvertised NOTIFY advertisedChanged )

public:
  ActionServer();

  ~ActionServer() override;

  const QString &name() const;

  void setName( const QString &value );

  const QString &type() const;

  void setType( const QString &value );

  QJSValue handleGoal() const;

  void setHandleGoal( const QJSValue &value );

  QJSValue handleCancel() const;

  void setHandleCancel( const QJSValue &value );

  bool isAdvertised() const;

signals:

  void nameChanged();

  void typeChanged();

  void handleGoalChanged();

  void handleCancelChanged();

  void advertisedChanged();

  /*!
   * Emitted when a goal was accepted and should be executed.
   * @param handle The goal handle used to publish feedback and to terminate the goal.
   */
  void goalAccepted( qml_ros2_plugin::ActionServerGoalHandle *handle );

protected:
  void onRos2Initialized() override;

  void onRos2Shutdown() override;

private slots:

  bool onHandleGoal( QString uuid, QVariant goal );

  bool onHandleCancel( QString uuid );

  void onGoalAccepted( std::shared_ptr<ros_babel_fish::BabelFishActionServerGoalHandle> handle,
                       int generation );

private:
  //! (Re-)creates the action server if name, type and ROS are available. Drops any prior server.
  void tryCreate();

  void clearGoalHandles();

  ros_babel_fish::BabelFish babel_fish_;
  QString name_;
  QString type_;
  QJSValue handle_goal_;
  QJSValue handle_cancel_;
  std::atomic<bool> has_handle_goal_{ false };
  std::atomic<bool> has_handle_cancel_{ false };
  std::atomic<bool> shutting_down_{ false };
  ros_babel_fish::BabelFishActionServer::SharedPtr server_;
  // Bumped on every (re-)create so a goal-accept still queued from a previous server instance can
  // be recognized and dropped instead of producing a wrapper backed by the destroyed server.
  int server_generation_ = 0;
  // Maps a goal uuid to its wrapper. GUI-thread confined.
  std::unordered_map<QString, ActionServerGoalHandle *> goal_handles_;
  std::shared_ptr<AliveToken> alive_ = makeAliveToken();
};
} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( std::shared_ptr<ros_babel_fish::BabelFishActionServerGoalHandle> )

#endif // QML_ROS2_PLUGIN_ACTION_SERVER_HPP
