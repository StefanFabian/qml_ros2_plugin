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

#ifndef QML_ROS2_PLUGIN_BABEL_FISH_DISPENSER_HPP
#define QML_ROS2_PLUGIN_BABEL_FISH_DISPENSER_HPP

#include <ros2_babel_fish/babel_fish.hpp>

namespace qml_ros2_plugin
{

/*!
 * @brief Can be used to obtain a BabelFish that is linked to a type support provider that is reused for all BabelFish.
 *
 * The ros2_babel_fish::TypeSupportProvider is responsible for looking up message type supports and caching them, hence,
 * sharing an instance significantly improves performance.
 */
class BabelFishDispenser
{
  BabelFishDispenser();

public:
  BabelFishDispenser( const BabelFishDispenser & ) = delete;

  void operator=( const BabelFishDispenser & ) = delete;

  //! @return A BabelFish instance using the shared type support providers.
  static ros2_babel_fish::BabelFish getBabelFish();

private:
  ros2_babel_fish::BabelFish createBabelFish();

  std::vector<ros2_babel_fish::TypeSupportProvider::SharedPtr> type_support_providers_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_BABEL_FISH_DISPENSER_HPP
