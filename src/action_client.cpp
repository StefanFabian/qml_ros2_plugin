// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/action_client.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml_ros2_plugin/goal_handle.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QJSEngine>
#include <utility>

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

ActionClient::ActionClient( const QString &name, const QString &action_type )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  action_type_ = action_type;
  name_ = name;
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

QString ActionClient::actionType() const { return action_type_; }

void ActionClient::checkServerReady()
{
  if ( !isServerReady() )
    return;
  connect_timer_.stop();
  disconnect( &connect_timer_, &QTimer::timeout, this, &ActionClient::checkServerReady );
  emit serverReadyChanged();
}

void ActionClient::invokeGoalResponseCallback(
    QJSValue callback, ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr handle )
{
  QJSEngine *engine = qjsEngine( this );
  if ( handle != nullptr )
    callback.call( { engine->newQObject( new GoalHandle( client_, std::move( handle ) ) ) } );
  else
    callback.call( { engine->newQObject( nullptr ) } );
}

void ActionClient::invokeFeedbackCallback( QJSValue callback,
                                           BabelFishActionClient::GoalHandle::SharedPtr handle,
                                           CompoundMessage::ConstSharedPtr feedback )
{
  QJSEngine *engine = qjsEngine( this );
  QJSValue js_goal_handle = engine->newQObject( new GoalHandle( client_, std::move( handle ) ) );
  try {
    QJSValue js_feedback = engine->toScriptValue<QVariant>( msgToMap( feedback ) );
    callback.call( { js_goal_handle, js_feedback } );
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to translate Action feedback: %s", ex.what() );
  }
}

void ActionClient::invokeResultCallback( QJSValue callback, QString goal_id,
                                         qml_ros2_plugin::action_goal_status::GoalStatus result_code,
                                         ros_babel_fish::CompoundMessage::ConstSharedPtr result )
{
  QJSEngine *engine = qjsEngine( this );
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

QObject *ActionClient::sendGoalAsync( const QVariantMap &goal, QJSValue options )
{
  if ( client_ == nullptr ) {
    QML_ROS2_PLUGIN_ERROR( "Tried to send goal when ActionClient was not connected!" );
    return nullptr;
  }

  std::string goal_type = action_type_.toStdString();
  goal_type = goal_type.substr( 0, goal_type.length() - strlen( "Action" ) ) + "Goal";
  try {
    auto message = client_->create_goal();
    if ( !fillMessage( message, goal ) )
      return nullptr;
    BabelFishActionClient::SendGoalOptions goal_options;
    goal_options.goal_response_callback =
        [options, this]( const BabelFishActionClient::GoalHandle::SharedPtr &gh ) {
          if ( !options.hasProperty( "onGoalResponse" ) )
            return;
          QJSValue goal_response_cb = options.property( "onGoalResponse" );
          if ( !goal_response_cb.isCallable() )
            return;
          QMetaObject::invokeMethod(
              this, "invokeGoalResponseCallback", Qt::AutoConnection,
              Q_ARG( QJSValue, goal_response_cb ),
              Q_ARG( ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr, gh ) );
        };
    goal_options.feedback_callback =
        [options, this]( const BabelFishActionClient::GoalHandle::SharedPtr &goal_handle,
                         const CompoundMessage::ConstSharedPtr &feedback ) mutable {
          if ( !options.hasProperty( "onFeedback" ) )
            return;
          QJSValue feedback_cb = options.property( "onFeedback" );
          if ( !feedback_cb.isCallable() )
            return;
          QMetaObject::invokeMethod(
              this, "invokeFeedbackCallback", Qt::AutoConnection, Q_ARG( QJSValue, feedback_cb ),
              Q_ARG( ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr, goal_handle ),
              Q_ARG( ros_babel_fish::CompoundMessage::ConstSharedPtr, feedback ) );
        };
    goal_options.result_callback =
        [options, this]( const BabelFishActionClient::GoalHandle::WrappedResult &result ) {
          if ( !options.hasProperty( "onResult" ) )
            return;
          QJSValue result_cb = options.property( "onResult" );
          if ( !result_cb.isCallable() )
            return;
          QMetaObject::invokeMethod(
              this, "invokeResultCallback", Qt::AutoConnection, Q_ARG( QJSValue, result_cb ),
              Q_ARG( QString, uuidToString( result.goal_id ) ),
              Q_ARG( qml_ros2_plugin::action_goal_status::GoalStatus,
                     static_cast<qml_ros2_plugin::action_goal_status::GoalStatus>( result.code ) ),
              Q_ARG( ros_babel_fish::CompoundMessage::ConstSharedPtr, result.result ) );
        };
    auto goal_handle = client_->async_send_goal( message, goal_options );
    return nullptr; // TODO add promise or something like that
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to send Action goal: %s", ex.what() );
  }
  return nullptr;
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
} // namespace qml_ros2_plugin
