// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/service_client.hpp"
#include "logging.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QJSEngine>
#include <thread>

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{
ServiceClient::ServiceClient( QString name, QString type, const QoSWrapper &qos )
    : qos_( qos ), name_( std::move( name ) ), service_type_( std::move( type ) )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  connect_timer_.setInterval( 16 );
  connect_timer_.setSingleShot( false );
  connect( &connect_timer_, &QTimer::timeout, this, &ServiceClient::checkServiceReady );
}

ServiceClient::~ServiceClient() = default;

void ServiceClient::onRos2Initialized()
{
  try {
    rclcpp::Node &node = *Ros2Qml::getInstance().node();
    client_ =
        babel_fish_.create_service_client( node, name_.toStdString(), service_type_.toStdString(),
                                           qos_.rclcppQoS().get_rmw_qos_profile() );
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Could not create ServiceClient: %s", ex.what() );
    client_ = nullptr;
    return;
  } catch ( std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Could not create ServiceClient: %s", ex.what() );
    client_ = nullptr;
    return;
  } catch ( ... ) {
    QML_ROS2_PLUGIN_ERROR( "Could not create ServiceClient: Unknown error." );
    client_ = nullptr;
    return;
  }
  connect_timer_.start();
}

void ServiceClient::onRos2Shutdown() { client_.reset(); }

void ServiceClient::checkServiceReady()
{
  if ( !isServiceReady() ) {
    int timeouted = 0;
    for ( size_t i = 0; i < waiting_service_calls_.size(); ++i, ++timeouted ) {
      if ( clock::now() - waiting_service_calls_[i].start <
           std::chrono::milliseconds( connection_timeout_ ) )
        break;
      QML_ROS2_PLUGIN_DEBUG(
          "Request for service '%s' timeouted while waiting for it to become ready.",
          name_.toStdString().c_str() );
      pending_requests_--;
      QMetaObject::invokeMethod( this, "invokeCallback", Qt::AutoConnection,
                                 Q_ARG( QJSValue, waiting_service_calls_[i].callback ),
                                 Q_ARG( QVariant, false ) );
    }
    if ( timeouted > 0 ) {
      waiting_service_calls_.erase( waiting_service_calls_.begin(),
                                    waiting_service_calls_.begin() + timeouted );
      emit pendingRequestsChanged();
    }
    return;
  }
  QML_ROS2_PLUGIN_DEBUG( "Service '%s' is ready.", name_.toStdString().c_str() );
  connect_timer_.stop();
  emit serviceReadyChanged();

  if ( waiting_service_calls_.empty() )
    return;

  blockSignals( true );
  for ( const auto &call : waiting_service_calls_ ) {
    pending_requests_--;
    sendRequestAsync( call.request, call.callback );
  }
  waiting_service_calls_.clear();
  blockSignals( false );
  emit pendingRequestsChanged();
}

bool ServiceClient::isServiceReady() const
{
  return client_ != nullptr && client_->service_is_ready();
}

const QString &ServiceClient::name() const { return name_; }

const QString &ServiceClient::type() const { return service_type_; }

int ServiceClient::connectionTimeout() const { return connection_timeout_; }

void ServiceClient::setConnectionTimeout( int timeout )
{
  if ( connection_timeout_ == timeout )
    return;
  connection_timeout_ = timeout;
  emit connectionTimeoutChanged();
}

int ServiceClient::pendingRequests() const { return pending_requests_; }

void ServiceClient::sendRequestAsync( const QVariantMap &req, const QJSValue &callback )
{
  pending_requests_++;
  if ( !isServiceReady() ) {
    QML_ROS2_PLUGIN_DEBUG( "Service '%s' not ready, waiting up to %d ms.",
                           name_.toStdString().c_str(), connection_timeout_ );
    waiting_service_calls_.push_back( { req, callback, clock::now() } );
    emit pendingRequestsChanged();
    if ( !connect_timer_.isActive() ) {
      // If the time is not active, the service was ready at some point and is not ready anymore.
      emit serviceReadyChanged();
      connect_timer_.start();
    }
    return;
  }
  QML_ROS2_PLUGIN_DEBUG( "Service '%s' is ready. Sending request.", name_.toStdString().c_str() );
  CompoundMessage::SharedPtr message;
  try {
    message = babel_fish_.create_service_request_shared( service_type_.toStdString() );
    fillMessage( *message, req );
    client_->async_send_request( message, [callback,
                                           this]( BabelFishServiceClient::SharedFuture response ) {
      QML_ROS2_PLUGIN_DEBUG( "Received response from service %s.", name_.toStdString().c_str() );
      pending_requests_--;
      emit pendingRequestsChanged();
      QMetaObject::invokeMethod( this, "invokeCallback", Qt::AutoConnection,
                                 Q_ARG( QJSValue, callback ),
                                 Q_ARG( QVariant, msgToMap( response.get() ) ) );
    } );
    emit pendingRequestsChanged();
    return;
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to call service: %s", ex.what() );
  } catch ( std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to call service: %s", ex.what() );
  } catch ( ... ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to call service: Unknown error." );
  }
  pending_requests_--;
  QMetaObject::invokeMethod( this, "invokeCallback", Qt::AutoConnection,
                             Q_ARG( QJSValue, callback ), Q_ARG( QVariant, QVariant( false ) ) );
}

void ServiceClient::invokeCallback( QJSValue value, const QVariant &result )
{
  QJSEngine *engine = qjsEngine( this );
  value.call( { engine->toScriptValue( result ) } );
}
} // namespace qml_ros2_plugin
