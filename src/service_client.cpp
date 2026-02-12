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
ServiceClient::ServiceClient( QString name, QString type, const QoSWrapper &qos )
    : qos_( qos ), name_( std::move( name ) ), service_type_( std::move( type ) )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

ServiceClient::~ServiceClient()
{
  std::unique_lock lock( check_ready_mutex_ );
  connect_timer_.stop();
  disconnect( &connect_timer_, &QTimer::timeout, this, &ServiceClient::checkServiceReady );
}

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
  }
  connect_timer_.setInterval( 16 );
  connect_timer_.setSingleShot( false );
  connect( &connect_timer_, &QTimer::timeout, this, &ServiceClient::checkServiceReady );
  connect_timer_.start();
}

void ServiceClient::onRos2Shutdown() { client_.reset(); }

void ServiceClient::checkServiceReady()
{
  std::unique_lock lock( check_ready_mutex_ );
  if ( !isServiceReady() ) {
    int timeouted = 0;
    for ( size_t i = 0; i < waiting_service_calls_.size(); ++i, ++timeouted ) {
      if ( clock::now() - waiting_service_calls_[i].start <
           std::chrono::milliseconds( connection_timeout_ ) )
        break;
      QML_ROS2_PLUGIN_DEBUG(
          "Request for service '%s' timeouted while waiting for it to become ready.",
          name_.toStdString().c_str() );
      invokeCallback( waiting_service_calls_[i].callback, QVariant( false ) );
      pending_requests_--;
      invokeCallback( waiting_service_calls_[i].id, false );
    }
    if ( timeouted > 0 ) {
      waiting_service_calls_.erase( waiting_service_calls_.begin(),
                                    waiting_service_calls_.begin() + timeouted );
      emit pendingRequestsChanged();
    }
    return;
  }
  connect_timer_.stop();
  disconnect( &connect_timer_, &QTimer::timeout, this, &ServiceClient::checkServiceReady );
  emit serviceReadyChanged();

  if ( waiting_service_calls_.empty() )
    return;

  blockSignals( true );
  for ( const auto &call : waiting_service_calls_ ) {
    pending_requests_--;
    auto it = pending_callbacks_.find( call.id );
    if ( it == pending_callbacks_.end() ) {
      QML_ROS2_PLUGIN_ERROR(
          "ServiceClient: Could not find pending callback with ID %d in checkServiceReady. Can not "
          "send request after connection was established.",
          call.id );
      continue;
    }
    sendRequestAsync( call.request, it->second );
    pending_callbacks_.erase( it );
  }
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
  if ( !engine_ ) {
    engine_ = qjsEngine( this );
  }
  pending_requests_++;
  int callback_id = generateInternalCallbackId();
  while ( pending_callbacks_.find( callback_id ) != pending_callbacks_.end() ) {
    callback_id = generateInternalCallbackId();
  }
  pending_callbacks_[callback_id] = callback;
  if ( !isServiceReady() ) {
    QML_ROS2_PLUGIN_DEBUG( "Service '%s' not ready, waiting up to %d ms.",
                           name_.toStdString().c_str(), connection_timeout_ );

    waiting_service_calls_.push_back( { req, callback_id, clock::now() } );
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
    QPointer instance = this;
    client_->async_send_request(
        message, [callback_id, instance]( BabelFishServiceClient::SharedFuture response ) {
          if ( !instance )
            return;
          QML_ROS2_PLUGIN_DEBUG( "Received response from service %s.",
                                 instance->name_.toStdString().c_str() );
          instance->pending_requests_--;
          emit instance->pendingRequestsChanged();
          QMetaObject::invokeMethod( instance, "invokeCallback", Qt::AutoConnection,
                                     Q_ARG( int, callback_id ),
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
  invokeCallback( callback_id, false );
}

void ServiceClient::invokeCallback( int id, const QVariant &result )
{
  QJSEngine *engine = engine_ ? engine_.data() : qjsEngine( this );
  if ( !engine ) {
    QML_ROS2_PLUGIN_ERROR(
        "ServiceClient: Failed to get QJSEngine in invokeCallback. Can not invoke callback." );
    return;
  }
  auto it = pending_callbacks_.find( id );
  if ( it == pending_callbacks_.end() ) {
    QML_ROS2_PLUGIN_ERROR(
        "ServiceClient: Could not find pending callback with ID %d in invokeCallback. Can not "
        "invoke callback.",
        id );
    return;
  }
  QJSValue &callback = it->second;
  callback.call( { engine->toScriptValue( result ) } );
  pending_callbacks_.erase( it );
}

int ServiceClient::generateInternalCallbackId()
{
  // Create a unique incrementing internal goal ID
  static std::atomic<int> current_id = 0;
  return current_id.fetch_add( 1 );
}
} // namespace qml_ros2_plugin
