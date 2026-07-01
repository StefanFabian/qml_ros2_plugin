// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/action_server.hpp"
#include "logging.hpp"
#include "qml_ros2_plugin/action_server_goal_handle.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QJSEngine>

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

ActionServer::ActionServer() { babel_fish_ = BabelFishDispenser::getBabelFish(); }

ActionServer::~ActionServer()
{
  // Set first so any in-flight executor callback degrades to the safe default and never blocks on
  // this (tearing-down) GUI thread.
  shutting_down_ = true;
  // Wait for an in-flight handle_accepted callback to finish and stop future ones from touching us
  // before tearing down members. (handle_goal/handle_cancel are gated by shutting_down_ above.)
  retire( alive_ );
  server_.reset();
}

const QString &ActionServer::name() const { return name_; }

void ActionServer::setName( const QString &value )
{
  if ( name_ == value )
    return;
  name_ = value;
  tryCreate();
  emit nameChanged();
}

const QString &ActionServer::type() const { return type_; }

void ActionServer::setType( const QString &value )
{
  if ( type_ == value )
    return;
  type_ = value;
  tryCreate();
  emit typeChanged();
}

QJSValue ActionServer::handleGoal() const { return handle_goal_; }

void ActionServer::setHandleGoal( const QJSValue &value )
{
  handle_goal_ = value;
  has_handle_goal_ = value.isCallable();
  emit handleGoalChanged();
}

QJSValue ActionServer::handleCancel() const { return handle_cancel_; }

void ActionServer::setHandleCancel( const QJSValue &value )
{
  handle_cancel_ = value;
  has_handle_cancel_ = value.isCallable();
  emit handleCancelChanged();
}

bool ActionServer::isAdvertised() const { return server_ != nullptr; }

void ActionServer::onRos2Initialized()
{
  // A previous shutdown (or the destructor of a sibling cycle) latched this; clear it so an object
  // that outlived a shutdown/re-init admits goals again instead of rejecting every one forever.
  shutting_down_ = false;
  // Only create here if a setter did not already create it (avoids a needless drop/recreate that
  // would force re-discovery).
  if ( server_ == nullptr )
    tryCreate();
}

void ActionServer::onRos2Shutdown()
{
  shutting_down_ = true;
  bool was_advertised = server_ != nullptr;
  server_.reset();
  clearGoalHandles();
  if ( was_advertised )
    emit advertisedChanged();
}

void ActionServer::clearGoalHandles()
{
  for ( auto &[uuid, wrapper] : goal_handles_ ) {
    if ( wrapper )
      wrapper->deleteLater();
  }
  goal_handles_.clear();
}

void ActionServer::tryCreate()
{
  bool was_advertised = server_ != nullptr;
  server_.reset();
  clearGoalHandles();
  // Invalidate any goal-accept still queued from the previous server instance (its handle is backed
  // by the server we just destroyed) so onGoalAccepted drops it instead of emitting a zombie handle.
  ++server_generation_;
  if ( was_advertised )
    emit advertisedChanged();

  if ( name_.isEmpty() || type_.isEmpty() )
    return;
  Ros2Qml &ros2 = Ros2Qml::getInstance();
  std::shared_ptr<rclcpp::Node> node = ros2.node();
  if ( node == nullptr || !ros2.ok() )
    return;
  const std::string name = name_.toStdString();
  const std::string type = type_.toStdString();

  ActionServer *instance = this;
  std::shared_ptr<AliveToken> token = alive_;
  auto handle_goal =
      [token, instance]( const rclcpp_action::GoalUUID &uuid,
                         std::shared_ptr<const CompoundMessage> goal ) -> rclcpp_action::GoalResponse {
    std::unique_lock lock( token->mutex );
    if ( !token->alive || instance->shutting_down_.load() )
      return rclcpp_action::GoalResponse::REJECT;
    if ( !instance->has_handle_goal_.load() )
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    bool accept = false;
    lock.unlock();
    QMetaObject::invokeMethod( instance, "onHandleGoal", Qt::BlockingQueuedConnection,
                               Q_RETURN_ARG( bool, accept ), Q_ARG( QString, uuidToString( uuid ) ),
                               Q_ARG( QVariant, msgToMap( goal ) ) );
    return accept ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                  : rclcpp_action::GoalResponse::REJECT;
  };
  auto handle_cancel =
      [token, instance]( const std::shared_ptr<BabelFishActionServerGoalHandle> goal_handle )
      -> rclcpp_action::CancelResponse {
    // Cancels are rejected unless the user explicitly accepts them via handleCancel.
    std::unique_lock lock( token->mutex );
    if ( !token->alive || instance->shutting_down_.load() )
      return rclcpp_action::CancelResponse::REJECT;
    if ( !instance->has_handle_cancel_.load() )
      return rclcpp_action::CancelResponse::REJECT;
    bool accept = false;
    lock.unlock();
    QMetaObject::invokeMethod( instance, "onHandleCancel", Qt::BlockingQueuedConnection,
                               Q_RETURN_ARG( bool, accept ),
                               Q_ARG( QString, uuidToString( goal_handle->get_goal_id() ) ) );
    return accept ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
  };
  const int generation = server_generation_;
  ActionServer *self = this;
  auto handle_accepted = [self, token, generation](
                             std::shared_ptr<BabelFishActionServerGoalHandle> goal_handle ) {
    // Runs on the executor thread; the token keeps `self` valid while we post the (non-blocking)
    // queued call, or drops the goal if the server was destroyed.
    runIfAlive( token, [&] {
      QMetaObject::invokeMethod(
          self, "onGoalAccepted", Qt::QueuedConnection,
          Q_ARG( std::shared_ptr<ros_babel_fish::BabelFishActionServerGoalHandle>, goal_handle ),
          Q_ARG( int, generation ) );
    } );
  };

  // An action server's goal-expiry thread sleeps on the global default context; ensure it is valid
  // before the server is added to the executor. Done here so only apps that use an action server
  // touch the global context.
  ros2.ensureGlobalDefaultContextInitialized();
  try {
    server_ = babel_fish_.create_action_server( *node, name, type, handle_goal, handle_cancel,
                                                handle_accepted );
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to create ActionServer for action '%s': %s", name.c_str(),
                           ex.what() );
    return;
  }
  QML_ROS2_PLUGIN_DEBUG( "Advertised action '%s' of type '%s'.", name.c_str(), type.c_str() );
  emit advertisedChanged();
}

bool ActionServer::onHandleGoal( QString uuid, QVariant goal )
{
  if ( !handle_goal_.isCallable() )
    return true;
  QJSEngine *engine = jsEngine();
  if ( !engine ) {
    QML_ROS2_PLUGIN_ERROR(
        "ActionServer '%s': Failed to get QJSEngine for handleGoal. Accepting goal.",
        name_.toStdString().c_str() );
    return true;
  }
  QJSValue result = handle_goal_.call( { engine->toScriptValue( goal ), QJSValue( uuid ) } );
  if ( result.isError() ) {
    QML_ROS2_PLUGIN_ERROR( "ActionServer '%s': handleGoal threw an exception: %s. Rejecting goal.",
                           name_.toStdString().c_str(), result.toString().toStdString().c_str() );
    return false;
  }
  return result.toBool();
}

bool ActionServer::onHandleCancel( QString uuid )
{
  auto it = goal_handles_.find( uuid );
  if ( it == goal_handles_.end() ) {
    // Cancel arrived before the queued onGoalAccepted slot created the wrapper. Without a handle we
    // cannot consult handleCancel, so reject (consistent with the unhandled-cancel default).
    return false;
  }
  ActionServerGoalHandle *wrapper = it->second;
  bool accept = false;
  if ( handle_cancel_.isCallable() ) {
    if ( QJSEngine *engine = jsEngine() ) {
      QJSValue result = handle_cancel_.call( { engine->newQObject( wrapper ) } );
      if ( result.isError() ) {
        QML_ROS2_PLUGIN_ERROR(
            "ActionServer '%s': handleCancel threw an exception: %s. Rejecting cancel.",
            name_.toStdString().c_str(), result.toString().toStdString().c_str() );
      } else {
        accept = result.toBool();
      }
    } else {
      QML_ROS2_PLUGIN_ERROR(
          "ActionServer '%s': Failed to get QJSEngine for handleCancel. Rejecting cancel.",
          name_.toStdString().c_str() );
    }
  }
  if ( accept )
    wrapper->notifyCancelRequested();
  return accept;
}

void ActionServer::onGoalAccepted( std::shared_ptr<BabelFishActionServerGoalHandle> handle,
                                   int generation )
{
  // This is delivered via a queued connection and may arrive after a shutdown or a retarget reset
  // the server and cleared the handle map. Drop it rather than resurrect a wrapper backed by a dead
  // server (a stale generation means the goal belongs to a server instance that no longer exists).
  if ( shutting_down_.load() || generation != server_generation_ )
    return;
  auto *wrapper = new ActionServerGoalHandle( handle, this );
  const QString uuid = wrapper->goalId();
  goal_handles_[uuid] = wrapper;
  connect( wrapper, &ActionServerGoalHandle::terminated, this, [this, uuid]() {
    auto it = goal_handles_.find( uuid );
    if ( it == goal_handles_.end() )
      return;
    it->second->deleteLater();
    goal_handles_.erase( it );
  } );
  emit goalAccepted( wrapper );
}
} // namespace qml_ros2_plugin
