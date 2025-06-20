// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/goal_handle.hpp"

#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;
using namespace std::chrono_literals;

namespace qml_ros2_plugin
{

GoalHandle::GoalHandle( BabelFishActionClient::SharedPtr client,
                        BabelFishActionClient::GoalHandle::SharedPtr handle )
    : client_( std::move( client ) ), goal_handle_( std::move( handle ) )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

GoalHandle::GoalHandle(
    ros_babel_fish::BabelFishActionClient::SharedPtr client,
    std::shared_future<ros_babel_fish::BabelFishActionClient::GoalHandle::SharedPtr> handle )
    : client_( std::move( client ) ), goal_handle_future_( std::move( handle ) )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

void GoalHandle::cancel()
{
  if ( client_ == nullptr )
    return;
  if ( goal_handle_ == nullptr ) {
    goal_handle_future_.wait_for( 100ms );
    checkFuture();
    if ( goal_handle_ == nullptr ) {
      QML_ROS2_PLUGIN_ERROR( "GoalHandle::cancel() called but goal_handle is not ready." );
      return;
    }
  }
  client_->async_cancel_goal( goal_handle_ );
}

qml_ros2_plugin::action_goal_status::GoalStatus GoalHandle::status() const
{
  checkFuture();
  if ( goal_handle_ == nullptr )
    return action_goal_status::Unknown;
  return static_cast<action_goal_status::GoalStatus>( goal_handle_->get_status() );
}

QString GoalHandle::goalId() const
{
  checkFuture();
  if ( goal_handle_ == nullptr )
    return {};
  return conversion::uuidToString( goal_handle_->get_goal_id() );
}

qml_ros2_plugin::Time GoalHandle::goalStamp() const
{
  checkFuture();
  if ( goal_handle_ == nullptr )
    return Time();
  return Time( goal_handle_->get_goal_stamp() );
}

void GoalHandle::onRos2Shutdown()
{
  client_ = nullptr;
  goal_handle_ = nullptr;
}

void GoalHandle::checkFuture() const
{
  if ( goal_handle_ != nullptr )
    return;
  if ( !goal_handle_future_.valid() )
    return;
  auto status = goal_handle_future_.wait_for( std::chrono::milliseconds( 0 ) );
  if ( status == std::future_status::ready ) {
    goal_handle_ = goal_handle_future_.get();
  }
}

} // namespace qml_ros2_plugin
