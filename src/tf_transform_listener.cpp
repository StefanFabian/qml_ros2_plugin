// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/tf_transform_listener.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/ros2.hpp"

#include <QVariantMap>
#include <memory>
#include <tf2_ros/transform_listener.h>

using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

struct TfTransformListener::State {
  explicit State( rclcpp::Node::SharedPtr node )
      : buffer( node->get_clock() ), listener( buffer, node, false )
  {
    buffer.setUsingDedicatedThread( true );
  }

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
};

TfTransformListener &TfTransformListener::getInstance()
{
  static TfTransformListener instance;
  return instance;
}

TfTransformListener::TfTransformListener() : wrapper_count_( 0 ) { state_.reset(); }

TfTransformListener::~TfTransformListener() = default;

bool TfTransformListener::isInitialized() const { return state_ != nullptr; }

bool TfTransformListener::initialize()
{
  if ( state_ != nullptr )
    return true;
  if ( wrapper_count_ == 0 )
    return false;
  if ( !Ros2Qml::getInstance().isInitialized() ) {
    connect( &Ros2Qml::getInstance(), &Ros2Qml::initialized, this, &TfTransformListener::initialize );
    return true;
  }
  Ros2Qml::getInstance().registerDependant();
  state_ = std::make_unique<State>( Ros2Qml::getInstance().node() );
  return true;
}

QVariant TfTransformListener::canTransform( const QString &target_frame, const QString &source_frame,
                                            const rclcpp::Time &time, double timeout ) const
{
  if ( !isInitialized() )
    return QString( "Uninitialized" );
  if ( state_ == nullptr )
    return QString( "Invalid state" );
  std::string error;
  bool result;
  if ( timeout <= 0.0000001 ) {
    using namespace std::chrono_literals;
    result = state_->buffer.canTransform( target_frame.toStdString(), source_frame.toStdString(),
                                          time, rclcpp::Duration( 0ns ), &error );
  } else {
    result = state_->buffer.canTransform( target_frame.toStdString(), source_frame.toStdString(),
                                          time, qmlToRos2Duration( timeout ), &error );
  }
  if ( result )
    return true;
  if ( error.empty() )
    return false;
  return QString::fromStdString( error );
}

QVariant TfTransformListener::canTransform( const QString &target_frame,
                                            const rclcpp::Time &target_time,
                                            const QString &source_frame,
                                            const rclcpp::Time &source_time,
                                            const QString &fixed_frame, double timeout ) const
{
  if ( !isInitialized() )
    return QString( "Uninitialized" );
  if ( state_ == nullptr )
    return QString( "Invalid state" );
  std::string error;
  bool result;
  if ( timeout <= 0.0000001 ) {
    using namespace std::chrono_literals;
    result = state_->buffer.canTransform(
        target_frame.toStdString(), target_time, source_frame.toStdString(), source_time,
        fixed_frame.toStdString(), rclcpp::Duration( 0ns ), &error );
  } else {
    result = state_->buffer.canTransform(
        target_frame.toStdString(), target_time, source_frame.toStdString(), source_time,
        fixed_frame.toStdString(), qmlToRos2Duration( timeout ), &error );
  }
  if ( result )
    return true;
  if ( error.empty() )
    return false;
  return QString::fromStdString( error );
}

QVariantMap TfTransformListener::lookUpTransform( const QString &target_frame,
                                                  const QString &source_frame,
                                                  const rclcpp::Time &time, double timeout )
{
  geometry_msgs::msg::TransformStamped transform;
  if ( !isInitialized() ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "Uninitialized" );
    result.insert( "message", "ROS node is not yet initialized!" );
    return result;
  }
  if ( state_ == nullptr ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "Invalid state" );
    result.insert( "message", "TfTransformListener was not set up or already destructed!" );
    return result;
  }
  try {
    if ( timeout <= 1E-6 ) {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(),
                                                  source_frame.toStdString(), time );
    } else {
      transform =
          state_->buffer.lookupTransform( target_frame.toStdString(), source_frame.toStdString(),
                                          time, qmlToRos2Duration( timeout ) );
    }
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", true );
    return result;
  } catch ( tf2::LookupException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "LookupException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::ConnectivityException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ConnectivityException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::ExtrapolationException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ExtrapolationException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::InvalidArgumentException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "InvalidArgumentException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  }
}

QVariantMap TfTransformListener::lookUpTransform( const QString &target_frame,
                                                  const rclcpp::Time &target_time,
                                                  const QString &source_frame,
                                                  const rclcpp::Time &source_time,
                                                  const QString &fixed_frame, double timeout )
{
  geometry_msgs::msg::TransformStamped transform;
  if ( !isInitialized() ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "Uninitialized" );
    result.insert( "message", "ROS node is not yet initialized!" );
    return result;
  }
  if ( state_ == nullptr ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "Invalid state" );
    result.insert( "message", "TfTransformListener was not set up or already destructed!" );
    return result;
  }
  try {
    if ( timeout <= 0.0000001 ) {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), target_time,
                                                  source_frame.toStdString(), source_time,
                                                  fixed_frame.toStdString() );
    } else {
      transform = state_->buffer.lookupTransform(
          target_frame.toStdString(), target_time, source_frame.toStdString(), source_time,
          fixed_frame.toStdString(), qmlToRos2Duration( timeout ) );
    }
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", true );
    return result;
  } catch ( tf2::LookupException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "LookupException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::ConnectivityException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ConnectivityException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::ExtrapolationException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ExtrapolationException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::InvalidArgumentException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "InvalidArgumentException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  }
}

void TfTransformListener::registerWrapper()
{
  if ( wrapper_count_++ == 0 )
    initialize();
}

void TfTransformListener::unregisterWrapper()
{
  int count = --wrapper_count_;
  if ( count == 0 ) {
    if ( state_ != nullptr )
      Ros2Qml::getInstance().unregisterDependant();
    state_.reset();
  } else if ( count < 0 ) {
    QML_ROS2_PLUGIN_ERROR( "Unregister wrapper was called more often than registerWrapper for "
                           "TfTransformListener! This is a bug!" );
    wrapper_count_ += -count;
  }
}

tf2_ros::Buffer *TfTransformListener::buffer()
{
  return state_ == nullptr ? nullptr : &state_->buffer;
}

TfTransformListenerWrapper::TfTransformListenerWrapper()
{
  TfTransformListener::getInstance().registerWrapper();
}

TfTransformListenerWrapper::~TfTransformListenerWrapper()
{
  TfTransformListener::getInstance().unregisterWrapper();
}

void TfTransformListenerWrapper::initialize()
{
  /* only serves to create wrapper which will init anyway. */
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame,
                                                         const QString &source_frame,
                                                         const QDateTime &time, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, source_frame,
                                                             qmlToRos2Time( time ), timeout );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame,
                                                         const QString &source_frame,
                                                         const Time &time, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, source_frame,
                                                             time.getTime(), timeout );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame,
                                                         const QDateTime &target_time,
                                                         const QString &source_frame,
                                                         const QDateTime &source_time,
                                                         const QString &fixed_frame, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform(
      target_frame, qmlToRos2Time( target_time ), source_frame, qmlToRos2Time( source_time ),
      fixed_frame, timeout );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame,
                                                         const Time &target_time,
                                                         const QString &source_frame,
                                                         const Time &source_time,
                                                         const QString &fixed_frame, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, target_time.getTime(),
                                                             source_frame, source_time.getTime(),
                                                             fixed_frame, timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame,
                                                   const QString &source_frame,
                                                   const QDateTime &time, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, source_frame,
                                                          qmlToRos2Time( time ), timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame,
                                                   const QString &source_frame, const Time &time,
                                                   double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, source_frame,
                                                          time.getTime(), timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame,
                                                   const QDateTime &target_time,
                                                   const QString &source_frame,
                                                   const QDateTime &source_time,
                                                   const QString &fixed_frame, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, qmlToRos2Time( target_time ),
                                                          source_frame, qmlToRos2Time( source_time ),
                                                          fixed_frame, timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame,
                                                   const Time &target_time,
                                                   const QString &source_frame,
                                                   const Time &source_time,
                                                   const QString &fixed_frame, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, target_time.getTime(),
                                                          source_frame, source_time.getTime(),
                                                          fixed_frame, timeout );
}
} // namespace qml_ros2_plugin
