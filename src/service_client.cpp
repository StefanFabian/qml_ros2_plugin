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

#include "qml_ros2_plugin/service_client.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QJSEngine>
#include <thread>

using namespace ros2_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{
ServiceClient::ServiceClient( QString name, QString type, QoS qos )
    : name_( std::move( name ) ), service_type_( std::move( type ) ), qos_( qos )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

void ServiceClient::onRos2Initialized()
{
  try {
    rclcpp::Node &node = *Ros2Qml::getInstance().node();
    client_ =
        babel_fish_.create_service_client( node, name_.toStdString(), service_type_.toStdString(), qos_.getQoS().get_rmw_qos_profile() );
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

qml_ros2_plugin::QoS ServiceClient::qos() { return qos_; }

void ServiceClient::invokeCallback( QJSValue value, QVariant result )
{
  QJSEngine *engine = qjsEngine( this );
  value.call( { engine->toScriptValue( result ) } );
}
} // namespace qml_ros2_plugin
