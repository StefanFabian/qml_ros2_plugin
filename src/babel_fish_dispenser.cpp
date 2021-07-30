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

#include "qml_ros2_plugin/babel_fish_dispenser.hpp"

namespace qml_ros2_plugin
{

BabelFishDispenser::BabelFishDispenser() = default;

ros2_babel_fish::BabelFish BabelFishDispenser::getBabelFish()
{
  static BabelFishDispenser dispenser;
  return dispenser.createBabelFish();
}

ros2_babel_fish::BabelFish BabelFishDispenser::createBabelFish()
{
  if ( !type_support_providers_.empty() )
    return ros2_babel_fish::BabelFish( type_support_providers_ );
  ros2_babel_fish::BabelFish babel_fish;
  type_support_providers_ = babel_fish.type_support_providers();
  return babel_fish;
}
} // namespace qml_ros2_plugin
