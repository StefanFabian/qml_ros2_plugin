// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_ALIVE_TOKEN_HPP
#define QML_ROS2_PLUGIN_ALIVE_TOKEN_HPP

#include <memory>
#include <mutex>
#include <utility>

namespace qml_ros2_plugin
{

/*!
 * A small helper to safely run queued callbacks that could outlive the object.
 */
struct AliveToken {
  std::mutex mutex;
  bool alive = true;
};

inline std::shared_ptr<AliveToken> makeAliveToken() { return std::make_shared<AliveToken>(); }

//! Runs \p fn while holding the token's mutex, but only if the token has not been retired.
template<typename Fn>
void runIfAlive( const std::shared_ptr<AliveToken> &token, Fn &&fn )
{
  std::lock_guard<std::mutex> lock( token->mutex );
  if ( !token->alive )
    return;
  std::forward<Fn>( fn )();
}

//! Marks the token dead. Blocks until any concurrent runIfAlive() has finished. Call this from the
//! owner's destructor before destroying members the callbacks touch.
inline void retire( const std::shared_ptr<AliveToken> &token )
{
  std::lock_guard<std::mutex> lock( token->mutex );
  token->alive = false;
}
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_ALIVE_TOKEN_HPP
