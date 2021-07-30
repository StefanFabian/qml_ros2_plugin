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

#include "qml_ros2_plugin/goal_handle.hpp"

#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"

using namespace ros2_babel_fish;
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
