// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

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
