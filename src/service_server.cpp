// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/service_server.hpp"
#include "logging.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QJSEngine>

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

ServiceServer::ServiceServer() { babel_fish_ = BabelFishDispenser::getBabelFish(); }

ServiceServer::~ServiceServer()
{
  std::unique_lock lock( mutex_ );
  service_.reset();
  pending_requests_.clear();
}

const QString &ServiceServer::name() const { return name_; }

void ServiceServer::setName( const QString &value )
{
  if ( name_ == value )
    return;
  name_ = value;
  tryCreate();
  emit nameChanged();
}

const QString &ServiceServer::type() const { return type_; }

void ServiceServer::setType( const QString &value )
{
  if ( type_ == value )
    return;
  type_ = value;
  tryCreate();
  emit typeChanged();
}

const QoSWrapper &ServiceServer::qos() const { return qos_; }

void ServiceServer::setQoS( const QoSWrapper &value )
{
  if ( qos_ == value )
    return;
  qos_ = value;
  tryCreate();
  emit qosChanged();
}

QJSValue ServiceServer::processRequest() const { return process_request_; }

void ServiceServer::setProcessRequest( const QJSValue &value )
{
  process_request_ = value;
  emit processRequestChanged();
}

bool ServiceServer::isAdvertised() const { return service_ != nullptr; }

void ServiceServer::onRos2Initialized()
{
  // Only create here if a setter did not already create it (avoids a needless drop/recreate that
  // would force re-discovery).
  if ( service_ == nullptr )
    tryCreate();
}

void ServiceServer::onRos2Shutdown()
{
  std::unique_lock lock( mutex_ );
  bool was_advertised = service_ != nullptr;
  service_.reset();
  pending_requests_.clear();
  lock.unlock();
  if ( was_advertised )
    emit advertisedChanged();
}

void ServiceServer::tryCreate()
{
  bool was_advertised;
  {
    std::unique_lock lock( mutex_ );
    was_advertised = service_ != nullptr;
    // Drop any prior server so the same name is not double-registered and abandon stranded ids
    // (their request headers belong to the dropped entity).
    service_.reset();
    pending_requests_.clear();
  }
  if ( was_advertised )
    emit advertisedChanged();

  if ( name_.isEmpty() || type_.isEmpty() )
    return;
  Ros2Qml &ros2 = Ros2Qml::getInstance();
  std::shared_ptr<rclcpp::Node> node = ros2.node();
  if ( node == nullptr || !ros2.ok() )
    return;
  const std::string name = name_.toStdString();
  const std::string type = type_.toStdString();

  try {
    service_type_support_ = babel_fish_.get_service_type_support( type );
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to create ServiceServer for service '%s': %s", name.c_str(),
                           ex.what() );
    return;
  }

  QPointer<ServiceServer> instance = this;
  std::function<void( std::shared_ptr<rmw_request_id_t>, std::shared_ptr<CompoundMessage> )> callback =
      [instance]( std::shared_ptr<rmw_request_id_t> request_header,
                  std::shared_ptr<CompoundMessage> request ) {
        if ( !instance )
          return;
        instance->onRequestReceived( request_header, request );
      };
  try {
    BabelFishService::SharedPtr service = babel_fish_.create_service(
        *node, name, type, callback, qos_.rclcppQoS().get_rmw_qos_profile() );
    std::unique_lock lock( mutex_ );
    service_ = service;
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to create ServiceServer for service '%s': %s", name.c_str(),
                           ex.what() );
    return;
  }
  QML_ROS2_PLUGIN_DEBUG( "Advertised service '%s' of type '%s'.", name.c_str(), type.c_str() );
  emit advertisedChanged();
}

void ServiceServer::onRequestReceived( const std::shared_ptr<rmw_request_id_t> &request_header,
                                       const std::shared_ptr<CompoundMessage> &request )
{
  int id;
  {
    std::unique_lock lock( mutex_ );
    id = next_request_id_++;
    pending_requests_[id] = *request_header;
  }
  QMetaObject::invokeMethod( this, "handleRequest", Qt::QueuedConnection, Q_ARG( int, id ),
                             Q_ARG( QVariant, msgToMap( request ) ) );
}

void ServiceServer::handleRequest( int id, QVariant request )
{
  {
    // This slot may be dispatched by the shutdown event pump after the service was already torn
    // down (shutdown or retarget). Drop the request instead of running user JS against a
    // tearing-down engine; the client future is abandoned together with the service.
    std::unique_lock lock( mutex_ );
    if ( service_ == nullptr ) {
      pending_requests_.erase( id );
      return;
    }
  }
  if ( !process_request_.isCallable() ) {
    QML_ROS2_PLUGIN_WARN( "ServiceServer '%s': No processRequest callback set. Sending default "
                          "response.",
                          name_.toStdString().c_str() );
    sendResponse( id, {} );
    return;
  }
  QJSEngine *engine = jsEngine();
  if ( !engine ) {
    QML_ROS2_PLUGIN_ERROR( "ServiceServer '%s': Failed to get QJSEngine. Sending default response.",
                           name_.toStdString().c_str() );
    sendResponse( id, {} );
    return;
  }
  QJSValue result = process_request_.call( { engine->toScriptValue( request ), QJSValue( id ) } );
  if ( result.isError() ) {
    QML_ROS2_PLUGIN_ERROR( "ServiceServer '%s': processRequest threw an exception: %s. Sending "
                           "default response.",
                           name_.toStdString().c_str(), result.toString().toStdString().c_str() );
    sendResponse( id, {} );
    return;
  }
  if ( result.isUndefined() ) {
    // Deferred: the user will call sendResponse(id, ...) later.
    return;
  }
  // isObject() is also true for arrays; require a plain object so a stray array return is reported
  // instead of silently turning into an empty (default) response.
  if ( result.isObject() && !result.isArray() ) {
    sendResponse( id, result.toVariant().toMap() );
    return;
  }
  QML_ROS2_PLUGIN_WARN( "ServiceServer '%s': processRequest returned an unexpected value (expected "
                        "an object). Sending default response.",
                        name_.toStdString().c_str() );
  sendResponse( id, {} );
}

void ServiceServer::sendResponse( int id, QVariantMap response )
{
  std::unique_lock lock( mutex_ );
  BabelFishService::SharedPtr local = service_;
  auto it = pending_requests_.find( id );
  const bool found = it != pending_requests_.end();
  rmw_request_id_t header{};
  if ( found ) {
    header = it->second;
    pending_requests_.erase( it );
  }
  lock.unlock();

  if ( local == nullptr ) {
    QML_ROS2_PLUGIN_WARN( "ServiceServer '%s': Tried to send response but the service is no longer "
                          "advertised. Dropping response.",
                          name_.toStdString().c_str() );
    return;
  }
  if ( !found ) {
    QML_ROS2_PLUGIN_WARN(
        "ServiceServer '%s': Tried to send response for unknown or expired request id %d.",
        name_.toStdString().c_str(), id );
    return;
  }
  try {
    auto msg = CompoundMessage::make_shared( service_type_support_->response() );
    if ( !fillMessage( *msg, response ) ) {
      QML_ROS2_PLUGIN_WARN(
          "ServiceServer '%s': The response could not be fully populated from the "
          "returned value (see warnings above); sending it anyway.",
          name_.toStdString().c_str() );
    }
    local->send_response( header, *msg );
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "ServiceServer '%s': Failed to send response: %s",
                           name_.toStdString().c_str(), ex.what() );
  }
}
} // namespace qml_ros2_plugin
