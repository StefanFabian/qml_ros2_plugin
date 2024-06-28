// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/babel_fish_dispenser.hpp"

namespace qml_ros2_plugin
{

BabelFishDispenser::BabelFishDispenser() = default;

ros_babel_fish::BabelFish BabelFishDispenser::getBabelFish()
{
  static BabelFishDispenser dispenser;
  return dispenser.createBabelFish();
}

ros_babel_fish::BabelFish BabelFishDispenser::createBabelFish()
{
  if ( !type_support_providers_.empty() )
    return ros_babel_fish::BabelFish( type_support_providers_ );
  ros_babel_fish::BabelFish babel_fish;
  type_support_providers_ = babel_fish.type_support_providers();
  return babel_fish;
}
} // namespace qml_ros2_plugin
