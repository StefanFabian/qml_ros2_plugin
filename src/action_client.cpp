// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/action_client.hpp"
#include "logging.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml_ros2_plugin/goal_handle.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QJSEngine>
#include <utility>

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

ActionClient::ActionClient( const QString &name, const QString &action_type )
    : action_type_( action_type ), name_( name )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

void ActionClient::onRos2Initialized()
{
  try {
    rclcpp::Node &node = *Ros2Qml::getInstance().node();
    client_ =
        babel_fish_.create_action_client( node, name_.toStdString(), action_type_.toStdString() );
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Could not create ActionClient: %s", ex.what() );
    client_ = nullptr;
    return;
  } catch ( std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Could not create ActionClient: %s", ex.what() );
    client_ = nullptr;
    return;
  } catch ( ... ) {
    QML_ROS2_PLUGIN_ERROR( "Could not create ActionClient: Unknown error." );
    client_ = nullptr;
    return;
  }
  connect_timer_.setInterval( 16 );
  connect_timer_.setSingleShot( false );
  connect( &connect_timer_, &QTimer::timeout, this, &ActionClient::checkServerReady );
  connect_timer_.start();
}

void ActionClient::onRos2Shutdown() { client_.reset(); }

bool ActionClient::isServerReady() const
{
  return client_ != nullptr && client_->action_server_is_ready();
}

QString ActionClient::name() const { return name_; }

QString ActionClient::actionType() const { return action_type_; }

int ActionClient::connectionTimeout() const { return connection_timeout_; }

void ActionClient::setConnectionTimeout( int timeout )
{
  connection_timeout_ = timeout;
  emit connectionTimeoutChanged();
}

void ActionClient::checkServerReady()
{
  if ( !isServerReady() )
    return;
  connect_timer_.stop();
  disconnect( &connect_timer_, &QTimer::timeout, this, &ActionClient::checkServerReady );

  std::vector<int> to_remove;
  std::unique_lock lock( pending_goals_mutex_ );
  for ( auto &[id, goal] : pending_goals_ ) {
    if ( clock::now() - goal.start > std::chrono::milliseconds( connection_timeout_ ) ) {
      QML_ROS2_PLUGIN_DEBUG(
          "Goal for action '%s' timeouted while waiting for server to become ready.",
          name_.toStdString().c_str() );
      goal.goal_handle_promise.set_value( nullptr );
      invokeGoalResponseCallback( id, nullptr );
      to_remove.push_back( id );
      continue;
    }
    if ( goal.sent )
      continue;
    QML_ROS2_PLUGIN_DEBUG( "ActionClient: Sending pending goal to action server '%s'.",
                           name_.toStdString().c_str() );
    auto future = internalSendGoal( id );
    goal.sent = true;
    if ( !future.valid() ) {
      goal.goal_handle_promise.set_value( nullptr );
      invokeGoalResponseCallback( id, nullptr );
      to_remove.push_back( id );
      continue;
    }
    // Forward future result to existing promise/future pair
    goal.forward_goal_future =
        std::async( std::launch::async,
                    [p = std::move( goal.goal_handle_promise ), f = std::move( future )]() mutable {
                      try {
                        QML_ROS2_PLUGIN_DEBUG(
                            "ActionClient: Received goal handle future. Forwarding to promise." );
                        // Wait for future to complete and set the value in the promise
                        p.set_value( f.get() );
                      } catch ( ... ) {
                        // Forward any exceptions that occurred in the second future
                        p.set_exception( std::current_exception() );
                      }
                    } );
  }

  // Remove pending goals that timed out or failed to send
  for ( const auto &id : to_remove ) { pending_goals_.erase( id ); }

  emit serverReadyChanged();
}

void ActionClient::invokeGoalResponseCallback(
    int internal_goal_id, ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr handle )
{
  QJSEngine *engine = engine_ ? engine_.data() : qjsEngine( this );
  if ( !engine_ ) {
    QML_ROS2_PLUGIN_ERROR( "ActionClient: Failed to get QJSEngine in invokeGoalResponseCallback. "
                           "Can not invoke callback." );
    return;
  }
  std::unique_lock lock( pending_goals_mutex_ );
  auto it = pending_goals_.find( internal_goal_id );
  if ( it == pending_goals_.end() ) {
    QML_ROS2_PLUGIN_ERROR( "ActionClient: Could not find pending goal with internal ID %d in "
                           "invokeGoalResponseCallback. Can not invoke callback.",
                           internal_goal_id );
    return;
  }
  PendingGoal &pending_goal = it->second;
  QJSValue &callback = pending_goal.goal_callback;
  if ( !callback.isCallable() ) {
    return;
  }
  if ( handle != nullptr ) {
    callback.call( { pending_goal.js_goal_handle } );
  } else {
    callback.call( { engine->newQObject( nullptr ) } );
  }
}

void ActionClient::invokeFeedbackCallback( int internal_goal_id,
                                           CompoundMessage::ConstSharedPtr feedback )
{
  QJSEngine *engine = engine_ ? engine_.data() : qjsEngine( this );
  if ( !engine_ ) {
    QML_ROS2_PLUGIN_ERROR( "ActionClient: Failed to get QJSEngine in invokeFeedbackCallback. "
                           "Can not invoke callback." );
    return;
  }
  std::unique_lock lock( pending_goals_mutex_ );
  auto it = pending_goals_.find( internal_goal_id );
  if ( it == pending_goals_.end() ) {
    QML_ROS2_PLUGIN_DEBUG( "ActionClient: Could not find pending goal with internal ID %d in "
                           "invokeFeedbackCallback. Can not invoke callback.",
                           internal_goal_id );
    return;
  }
  PendingGoal &pending_goal = it->second;
  QJSValue &callback = pending_goal.feedback_callback;
  if ( !callback.isCallable() ) {
    return;
  }
  try {
    QJSValue js_feedback = engine->toScriptValue<QVariant>( msgToMap( feedback ) );
    callback.call( { pending_goal.js_goal_handle, js_feedback } );
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to translate Action feedback: %s", ex.what() );
  }
}

void ActionClient::invokeResultCallback( int internal_goal_id, QString goal_id,
                                         qml_ros2_plugin::action_goal_status::GoalStatus result_code,
                                         ros_babel_fish::CompoundMessage::ConstSharedPtr result )
{
  QJSEngine *engine = engine_ ? engine_.data() : qjsEngine( this );
  if ( !engine_ ) {
    QML_ROS2_PLUGIN_ERROR( "ActionClient: Failed to get QJSEngine in invokeResultCallback. "
                           "Can not invoke callback." );
    return;
  }
  std::unique_lock lock( pending_goals_mutex_ );
  auto it = pending_goals_.find( internal_goal_id );
  if ( it == pending_goals_.end() ) {
    QML_ROS2_PLUGIN_ERROR( "ActionClient: Could not find pending goal with internal ID %d in "
                           "invokeResultCallback. Can not invoke callback.",
                           internal_goal_id );
    return;
  }
  PendingGoal &pending_goal = it->second;
  QJSValue &callback = pending_goal.result_callback;
  if ( callback.isCallable() ) {
    try {
      QVariantMap wrapped_result;
      wrapped_result["goalId"] = goal_id;
      wrapped_result["code"] = result_code;
      wrapped_result["result"] = msgToMap( result );
      callback.call( { engine->toScriptValue<QVariant>( wrapped_result ) } );
    } catch ( BabelFishException &ex ) {
      QML_ROS2_PLUGIN_ERROR( "Failed to translate Action result: %s", ex.what() );
    }
  }
  pending_goals_.erase( internal_goal_id );
}

QJSValue ActionClient::sendGoalAsync( const QVariantMap &goal, const QJSValue &options )
{
  if ( !engine_ ) {
    engine_ = qjsEngine( this );
  }
  std::unique_lock lock( pending_goals_mutex_ );
  int id = generateInternalGoalId();
  while ( pending_goals_.find( id ) != pending_goals_.end() ) { id = generateInternalGoalId(); }
  pending_goals_[id] = {};
  PendingGoal &pending_goal = pending_goals_[id];
  pending_goal.start = clock::now();
  pending_goal.goal = goal;
  pending_goal.goal_callback = options.property( "onGoalResponse" );
  pending_goal.feedback_callback = options.property( "onFeedback" );
  pending_goal.result_callback = options.property( "onResult" );

  if ( client_ == nullptr ) {
    QML_ROS2_PLUGIN_ERROR( "Tried to send goal when ActionClient was not connected!" );
    pending_goal.goal_handle_future = pending_goal.goal_handle_promise.get_future();
    pending_goal.js_goal_handle =
        engine_->newQObject( new GoalHandle( client_, pending_goal.goal_handle_future ) );
    return pending_goal.js_goal_handle;
  }

  pending_goal.goal_handle_future = internalSendGoal( id );
  if ( !pending_goal.goal_handle_future.valid() ) {
    pending_goals_.erase( id );
    return {};
  }
  pending_goal.sent = true;

  pending_goal.js_goal_handle =
      engine_->newQObject( new GoalHandle( client_, pending_goal.goal_handle_future ) );
  return pending_goal.js_goal_handle;
}

void ActionClient::cancelAllGoals()
{
  if ( client_ == nullptr )
    return;
  client_->async_cancel_all_goals(); // TODO this will block since the future will wait during destruction
}

void ActionClient::cancelGoalsBefore( const Time &time )
{
  if ( client_ == nullptr )
    return;
  client_->async_cancel_goals_before( time.getTime() );
}

void ActionClient::cancelGoalsBefore( const QDateTime &time )
{
  if ( client_ == nullptr )
    return;
  client_->async_cancel_goals_before( qmlToRos2Time( time ) );
}

int ActionClient::generateInternalGoalId()
{
  // Create a unique incrementing internal goal ID
  static std::atomic<int> current_id = 0;
  return current_id.fetch_add( 1 );
}

std::shared_future<ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr>
ActionClient::internalSendGoal( int internal_goal_id )
{
  std::unique_lock lock( pending_goals_mutex_ );
  auto it = pending_goals_.find( internal_goal_id );
  if ( it == pending_goals_.end() ) {
    QML_ROS2_PLUGIN_ERROR( "ActionClient: Could not find pending goal with internal ID %d in "
                           "internalSendGoal. Can not send goal.",
                           internal_goal_id );
    return {};
  }
  PendingGoal &pending_goal = it->second;
  try {
    auto message = client_->create_goal();
    if ( !fillMessage( message, pending_goal.goal ) )
      return {};
    QPointer instance = this;
    BabelFishActionClient::SendGoalOptions goal_options;
    if ( pending_goal.goal_callback.isCallable() ) {
      goal_options.goal_response_callback =
          [instance, internal_goal_id]( const BabelFishActionClient::GoalHandle::SharedPtr &gh ) {
            if ( !instance )
              return;
            QMetaObject::invokeMethod(
                instance, "invokeGoalResponseCallback", Qt::AutoConnection,
                Q_ARG( int, internal_goal_id ),
                Q_ARG( ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr, gh ) );
          };
    }
    if ( pending_goal.feedback_callback.isCallable() ) {
      goal_options.feedback_callback = [instance, internal_goal_id](
                                           const BabelFishActionClient::GoalHandle::SharedPtr &,
                                           const CompoundMessage::ConstSharedPtr &feedback ) mutable {
        if ( !instance )
          return;
        QMetaObject::invokeMethod(
            instance, "invokeFeedbackCallback", Qt::AutoConnection, Q_ARG( int, internal_goal_id ),
            Q_ARG( ros_babel_fish::CompoundMessage::ConstSharedPtr, feedback ) );
      };
    }
    if ( pending_goal.result_callback.isCallable() ) {
      goal_options.result_callback =
          [instance,
           internal_goal_id]( const BabelFishActionClient::GoalHandle::WrappedResult &result ) {
            if ( !instance )
              return;
            QMetaObject::invokeMethod(
                instance, "invokeResultCallback", Qt::AutoConnection,
                Q_ARG( int, internal_goal_id ), Q_ARG( QString, uuidToString( result.goal_id ) ),
                Q_ARG( qml_ros2_plugin::action_goal_status::GoalStatus,
                       static_cast<qml_ros2_plugin::action_goal_status::GoalStatus>( result.code ) ),
                Q_ARG( ros_babel_fish::CompoundMessage::ConstSharedPtr, result.result ) );
          };
    }
    return client_->async_send_goal( message, goal_options );
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to send Action goal: %s", ex.what() );
  }
  return {};
}
} // namespace qml_ros2_plugin
