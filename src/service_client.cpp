// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/service_client.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QJSEngine>
#include <thread>

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{
ServiceClient::ServiceClient( QString name, QString type )
    : name_( std::move( name ) ), service_type_( std::move( type ) )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

void ServiceClient::onRos2Initialized()
{
  try {
    rclcpp::Node &node = *Ros2Qml::getInstance().node();
    client_ =
        babel_fish_.create_service_client( node, name_.toStdString(), service_type_.toStdString() );
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Could not create ServiceClient: %s", ex.what() );
    client_ = nullptr;
    return;
  }
  connect_timer_.setInterval( 16 );
  connect_timer_.setSingleShot( false );
  connect( &connect_timer_, &QTimer::timeout, this, &ServiceClient::checkServiceReady );
  connect_timer_.start();
}

void ServiceClient::onRos2Shutdown() { client_.reset(); }

void ServiceClient::checkServiceReady()
{
  if ( !isServiceReady() )
    return;
  connect_timer_.stop();
  disconnect( &connect_timer_, &QTimer::timeout, this, &ServiceClient::checkServiceReady );
  emit serviceReadyChanged();
}

bool ServiceClient::isServiceReady() const
{
  return client_ != nullptr && client_->service_is_ready();
}

const QString &ServiceClient::name() const { return name_; }

const QString &ServiceClient::type() const { return service_type_; }

void ServiceClient::sendRequestAsync( const QVariantMap &req, const QJSValue &callback )
{
  if ( client_ == nullptr ) {
    QML_ROS2_PLUGIN_ERROR( "Tried to send goal when ServiceClient was not connected!" );
    if ( !callback.isCallable() )
      return;
    QMetaObject::invokeMethod( this, "invokeCallback", Qt::AutoConnection,
                               Q_ARG( QJSValue, callback ), Q_ARG( QVariant, QVariant( false ) ) );
    return;
  }
  CompoundMessage::SharedPtr message;
  try {
    message = babel_fish_.create_service_request_shared( service_type_.toStdString() );
    fillMessage( *message, req );
    client_->async_send_request(
        message, [callback, this]( BabelFishServiceClient::SharedFuture response ) {
          QMetaObject::invokeMethod( this, "invokeCallback", Qt::AutoConnection,
                                     Q_ARG( QJSValue, callback ),
                                     Q_ARG( QVariant, msgToMap( response.get() ) ) );
        } );
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to call service: %s", ex.what() );
    QMetaObject::invokeMethod( this, "invokeCallback", Qt::AutoConnection,
                               Q_ARG( QJSValue, callback ), Q_ARG( QVariant, QVariant( false ) ) );
    return;
  }
}

void ServiceClient::invokeCallback( QJSValue value, QVariant result )
{
  QJSEngine *engine = qjsEngine( this );
  value.call( { engine->toScriptValue( result ) } );
}
} // namespace qml_ros2_plugin
