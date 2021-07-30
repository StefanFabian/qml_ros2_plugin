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

#include "qml_ros2_plugin/qobject_ros2.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QCoreApplication>

namespace qml_ros2_plugin
{

QObjectRos2::QObjectRos2( QObject *parent ) : QObject( parent ), is_initialized_( false )
{
  if ( Ros2Qml::getInstance().isInitialized() ) {
    // Invoke initialize after object was constructed
    QMetaObject::invokeMethod( this, "_initialize", Qt::QueuedConnection );
  } else {
    QObject::connect( &Ros2Qml::getInstance(), &Ros2Qml::initialized, this,
                      &QObjectRos2::_initialize );
  }
  // These allow for safe clean-up if the application exits since the order of the singleton
  // destructors is undefined and this might lead to dependency issues.
  QObject::connect( &Ros2Qml::getInstance(), &Ros2Qml::shutdown, this, &QObjectRos2::_shutdown );
  QObject::connect( QCoreApplication::instance(), &QCoreApplication::aboutToQuit, this,
                    &QObjectRos2::_shutdown );
  // This object needs ROS communication, so it's safe to assume it wants the spinner to be running during its lifetime.
  Ros2Qml::getInstance().registerDependant();
}

QObjectRos2::~QObjectRos2() { Ros2Qml::getInstance().unregisterDependant(); }

bool QObjectRos2::isRosInitialized() const { return is_initialized_; }

void QObjectRos2::_initialize()
{
  if ( is_initialized_ )
    return;
  onRos2Initialized();
  is_initialized_ = true;
}

void QObjectRos2::_shutdown()
{
  if ( !is_initialized_ )
    return;
  onRos2Shutdown();
  is_initialized_ = false;
}
} // namespace qml_ros2_plugin
