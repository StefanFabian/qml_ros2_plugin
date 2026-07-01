// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/action_server_goal_handle.hpp"
#include "logging.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"

#include <QTimer>

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

ActionServerGoalHandle::ActionServerGoalHandle(
    const std::shared_ptr<BabelFishActionServerGoalHandle> &handle, QObject *parent )
    : QObjectRos2( parent ), handle_( handle )
{
  assert( handle_ != nullptr );
  goal_ = msgToMap( handle_->get_goal() );
  goal_id_ = uuidToString( handle_->get_goal_id() );
}

const QVariant &ActionServerGoalHandle::goal() const { return goal_; }

QString ActionServerGoalHandle::goalId() const { return goal_id_; }

bool ActionServerGoalHandle::isCanceling() const { return handle_->is_canceling(); }

bool ActionServerGoalHandle::isActive() const { return handle_->is_active(); }

bool ActionServerGoalHandle::isExecuting() const { return handle_->is_executing(); }

bool ActionServerGoalHandle::checkActive( const char *method )
{
  if ( !terminated_ )
    return true;
  QML_ROS2_PLUGIN_WARN( "ActionServerGoalHandle: '%s' called on a goal that already reached a "
                        "terminal state. Ignoring.",
                        method );
  return false;
}

void ActionServerGoalHandle::terminate(
    const char *verb, const QVariantMap &result,
    void ( BabelFishActionServerGoalHandle::*terminal )( const CompoundMessage & ) )
{
  if ( !checkActive( verb ) )
    return;
  try {
    auto msg = handle_->create_result_message();
    if ( !fillMessage( msg, result ) ) {
      QML_ROS2_PLUGIN_WARN(
          "ActionServerGoalHandle: The result for '%s' could not be fully "
          "populated from the given value (see warnings above); sending it anyway.",
          verb );
    }
    ( handle_.get()->*terminal )( msg );
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "ActionServerGoalHandle: Failed to %s goal: %s", verb, ex.what() );
    return;
  }
  terminated_ = true;
  emit statusChanged();
  emit terminated();
}

void ActionServerGoalHandle::succeed( QVariantMap result )
{
  terminate( "succeed", result, &BabelFishActionServerGoalHandle::succeed );
}

void ActionServerGoalHandle::abort( QVariantMap result )
{
  terminate( "abort", result, &BabelFishActionServerGoalHandle::abort );
}

void ActionServerGoalHandle::canceled( QVariantMap result )
{
  terminate( "canceled", result, &BabelFishActionServerGoalHandle::canceled );
}

void ActionServerGoalHandle::publishFeedback( QVariantMap feedback )
{
  if ( !checkActive( "publishFeedback" ) )
    return;
  try {
    auto msg = handle_->create_feedback_message();
    if ( !fillMessage( msg, feedback ) ) {
      QML_ROS2_PLUGIN_WARN(
          "ActionServerGoalHandle: The feedback could not be fully populated from "
          "the given value (see warnings above); publishing it anyway." );
    }
    handle_->publish_feedback( msg );
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "ActionServerGoalHandle: Failed to publish feedback: %s", ex.what() );
  }
}

void ActionServerGoalHandle::notifyCancelRequested()
{
  if ( cancel_pending_ || terminated_ )
    return;
  cancel_pending_ = true;
  // The action server transitions the goal to the canceling state on the executor thread only AFTER
  // the cancel-accept callback returns (ros_babel_fish call_handle_cancel_callback calls the user
  // callback first, then _cancel_goal()). is_canceling() may therefore still be false here. Wait
  // for the transition before notifying so a canceled() issued in response is a valid
  // CANCELING->CANCELED transition rather than an invalid EXECUTING->CANCELED one.
  cancel_request_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds( 2 );
  deliverCancelRequested();
}

void ActionServerGoalHandle::deliverCancelRequested()
{
  if ( terminated_ )
    return;
  if ( !handle_->is_canceling() ) {
    // Give up if the cancel never took effect at the rcl layer (e.g. the transition failed); this
    // also bounds the polling so it cannot spin forever.
    if ( std::chrono::steady_clock::now() >= cancel_request_deadline_ ) {
      cancel_pending_ = false;
      QML_ROS2_PLUGIN_WARN(
          "ActionServerGoalHandle: Cancel of goal '%s' was accepted but the goal never entered the "
          "canceling state in time; cancelRequested will not be emitted. The goal must be "
          "terminated "
          "by other means (e.g. succeed/abort).",
          goal_id_.toStdString().c_str() );
      return;
    }
    QTimer::singleShot( 1, this, &ActionServerGoalHandle::deliverCancelRequested );
    return;
  }
  // The request has been delivered; clear the guard so a (theoretical) later cancel is not swallowed.
  cancel_pending_ = false;
  emit cancelRequested();
  emit statusChanged();
}
} // namespace qml_ros2_plugin
