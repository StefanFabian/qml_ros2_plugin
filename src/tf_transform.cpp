// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/tf_transform.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/tf_transform_listener.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

TfTransform::TfTransform() : update_interval_( 1000 / 60 ), subscribed_( false ), enabled_( true )
{
  geometry_msgs::msg::TransformStamped transform;
  message_ = msgToMap( transform );
  message_.insert( "valid", false );
  connect( &update_timer_, &QTimer::timeout, this, &TfTransform::updateMessage );
  update_timer_.setSingleShot( false );
  update_timer_.setInterval( update_interval_ );
}

TfTransform::~TfTransform() { shutdown(); }

const QString &TfTransform::sourceFrame() const { return source_frame_; }

void TfTransform::setSourceFrame( const QString &value )
{
  source_frame_ = value;
  if ( source_frame_.isEmpty() )
    shutdown();
  else
    subscribe();

  emit sourceFrameChanged();
}

const QString &TfTransform::targetFrame() const { return target_frame_; }

void TfTransform::setTargetFrame( const QString &targetFrame )
{
  target_frame_ = targetFrame;
  if ( target_frame_.isEmpty() )
    shutdown();
  else
    subscribe();

  emit targetFrameChanged();
}

bool TfTransform::enabled() const { return enabled_; }

void TfTransform::setEnabled( bool value )
{
  if ( enabled_ == value )
    return;
  enabled_ = value;
  if ( enabled_ )
    subscribe();
  else
    shutdown();

  emit enabledChanged();
}

qreal TfTransform::rate() const
{
  if ( update_interval_.count() == 0 )
    return 0;
  return 1000.0 / update_interval_.count();
}

void TfTransform::setRate( qreal value )
{
  if ( value <= 0 )
    update_interval_ = std::chrono::milliseconds::zero();
  else
    update_interval_ = std::chrono::milliseconds(
        int( 1000 / value ) ); // Rounding down as value is strictly positive
  if ( update_interval_.count() == 0 )
    update_timer_.stop();
  else
    update_timer_.setInterval( update_interval_ );
  emit rateChanged();
}

const QVariantMap &TfTransform::message() { return message_; }

QVariant TfTransform::translation()
{
  if ( !message_.contains( "transform" ) )
    return {};
  const QVariantMap &transform = *static_cast<const QVariantMap *>( message_["transform"].data() );
  return transform.find( "translation" ).value();
}

QVariant TfTransform::rotation()
{
  if ( !message_.contains( "transform" ) )
    return {};
  const QVariantMap &transform = *static_cast<const QVariantMap *>( message_["transform"].data() );
  return transform.find( "rotation" ).value();
}

bool TfTransform::valid() { return message_.contains( "valid" ) && message_["valid"].toBool(); }

void TfTransform::subscribe()
{
  if ( source_frame_.isEmpty() || target_frame_.isEmpty() || !enabled_ || subscribed_ )
    return;
  subscribed_ = true;

  TfTransformListener::getInstance().registerWrapper();
  if ( update_interval_.count() > 0 )
    update_timer_.start();
  // Load transform
  updateMessage();
}

void TfTransform::shutdown()
{
  if ( !subscribed_ )
    return;
  subscribed_ = false;
  update_timer_.stop();
  TfTransformListener::getInstance().unregisterWrapper();
}

namespace
{
bool isDifferent( const geometry_msgs::msg::TransformStamped &new_transform,
                  const geometry_msgs::msg::TransformStamped &old_transform )
{
  if ( new_transform.header.stamp != old_transform.header.stamp )
    return true;
  if ( new_transform.header.frame_id != old_transform.header.frame_id )
    return true;
  if ( new_transform.child_frame_id != old_transform.child_frame_id )
    return true;
  return false;
}
} // namespace

void TfTransform::updateMessage()
{
  bool was_valid = valid();
  if ( TfTransformListener::getInstance().buffer() == nullptr )
    return;

  try {
    const auto &transform = TfTransformListener::getInstance().buffer()->lookupTransform(
        target_frame_.toStdString(), source_frame_.toStdString(), tf2::TimePointZero );
    if ( !isDifferent( transform, last_transform_ ) )
      return;
    last_transform_ = transform;

    QVariantMap result = msgToMap( transform );
    result.insert( "valid", true );
    message_ = result;
    if ( !was_valid )
      emit validChanged();
    emit rotationChanged();
    emit messageChanged();
    emit translationChanged();
  } catch ( tf2::LookupException &ex ) {
    QVariantMap result = msgToMap( geometry_msgs::msg::TransformStamped{} );
    result.insert( "valid", false );
    result.insert( "exception", "LookupException" );
    result.insert( "message", QString( ex.what() ) );
    if ( was_valid )
      emit validChanged();
    else if ( message_.contains( "message" ) && message_["message"].toString() == QString( ex.what() ) )
      return;
    message_ = result;
    emit messageChanged();
  } catch ( tf2::ConnectivityException &ex ) {
    QVariantMap result = msgToMap( geometry_msgs::msg::TransformStamped{} );
    result.insert( "valid", false );
    result.insert( "exception", "ConnectivityException" );
    result.insert( "message", QString( ex.what() ) );
    if ( was_valid )
      emit validChanged();
    else if ( message_.contains( "message" ) && message_["message"].toString() == QString( ex.what() ) )
      return;
    message_ = result;
    emit messageChanged();
  } catch ( tf2::ExtrapolationException &ex ) {
    QVariantMap result = msgToMap( geometry_msgs::msg::TransformStamped{} );
    result.insert( "valid", false );
    result.insert( "exception", "ExtrapolationException" );
    result.insert( "message", QString( ex.what() ) );
    if ( was_valid )
      emit validChanged();
    else if ( message_.contains( "message" ) && message_["message"].toString() == QString( ex.what() ) )
      return;
    message_ = result;
    emit messageChanged();
  } catch ( tf2::InvalidArgumentException &ex ) {
    QVariantMap result = msgToMap( geometry_msgs::msg::TransformStamped{} );
    result.insert( "valid", false );
    result.insert( "exception", "InvalidArgumentException" );
    result.insert( "message", QString( ex.what() ) );
    if ( was_valid )
      emit validChanged();
    else if ( message_.contains( "message" ) && message_["message"].toString() == QString( ex.what() ) )
      return;
    message_ = result;
    emit messageChanged();
  }
}
} // namespace qml_ros2_plugin
