// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/goal_handle.hpp"

#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

GoalHandle::GoalHandle( BabelFishActionClient::SharedPtr client,
                        BabelFishActionClient::GoalHandle::SharedPtr handle )
    : client_( std::move( client ) ), goal_handle_( std::move( handle ) )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

void GoalHandle::cancel()
{
  if ( client_ == nullptr )
    return;
  client_->async_cancel_goal( goal_handle_ );
}

qml_ros2_plugin::action_goal_status::GoalStatus GoalHandle::status() const
{
  if ( goal_handle_ == nullptr )
    return action_goal_status::Unknown;
  return static_cast<action_goal_status::GoalStatus>( goal_handle_->get_status() );
}

QString GoalHandle::goalId() const
{
  if ( goal_handle_ == nullptr )
    return {};
  return conversion::uuidToString( goal_handle_->get_goal_id() );
}

qml_ros2_plugin::Time GoalHandle::goalStamp() const
{
  if ( goal_handle_ == nullptr )
    return Time();
  return Time( goal_handle_->get_goal_stamp() );
}

void GoalHandle::onRos2Shutdown()
{
  client_ = nullptr;
  goal_handle_ = nullptr;
}
} // namespace qml_ros2_plugin
