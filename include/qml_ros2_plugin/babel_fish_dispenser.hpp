// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_BABEL_FISH_DISPENSER_HPP
#define QML_ROS2_PLUGIN_BABEL_FISH_DISPENSER_HPP

#include <ros_babel_fish/babel_fish.hpp>

namespace qml_ros2_plugin
{

/*!
 * @brief Can be used to obtain a BabelFish that is linked to a type support provider that is reused for all BabelFish.
 *
 * The ros_babel_fish::TypeSupportProvider is responsible for looking up message type supports and caching them, hence,
 * sharing an instance significantly improves performance.
 */
class BabelFishDispenser
{
  BabelFishDispenser();

public:
  BabelFishDispenser( const BabelFishDispenser & ) = delete;

  void operator=( const BabelFishDispenser & ) = delete;

  //! @return A BabelFish instance using the shared type support providers.
  static ros_babel_fish::BabelFish getBabelFish();

private:
  ros_babel_fish::BabelFish createBabelFish();

  std::vector<ros_babel_fish::TypeSupportProvider::SharedPtr> type_support_providers_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_BABEL_FISH_DISPENSER_HPP
