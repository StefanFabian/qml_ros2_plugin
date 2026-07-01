// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_HELPERS_CALLBACK_REGISTRY_HPP
#define QML_ROS2_PLUGIN_HELPERS_CALLBACK_REGISTRY_HPP

#include <atomic>

namespace qml_ros2_plugin
{

//! Returns a process-wide unique, monotonically increasing internal id.
inline int nextInternalId()
{
  static std::atomic<int> current_id = 0;
  return current_id.fetch_add( 1 );
}

//! Returns an id that is not currently a key of \p map.
template<typename Map>
int freshId( const Map &map )
{
  int id = nextInternalId();
  while ( map.find( id ) != map.end() ) id = nextInternalId();
  return id;
}

//! Returns a pointer to the value mapped to \p id, or nullptr if \p id is not present. The caller
//! is responsible for any required locking.
template<typename Map>
typename Map::mapped_type *findPending( Map &map, int id )
{
  auto it = map.find( id );
  return it == map.end() ? nullptr : &it->second;
}
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_HELPERS_CALLBACK_REGISTRY_HPP
