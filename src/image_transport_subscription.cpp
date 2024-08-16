// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/image_transport_subscription.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/image_buffer.hpp"
#include "qml_ros2_plugin/image_transport_manager.hpp"
#include "qml_ros2_plugin/ros2.hpp"

namespace qml_ros2_plugin
{

ImageTransportSubscription::ImageTransportSubscription( QString topic, quint32 queue_size )
    : topic_( std::move( topic ) ), default_transport_( "compressed" ), clock_( RCL_ROS_TIME ),
      queue_size_( queue_size )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscription::onNoImageTimeout,
           Qt::AutoConnection );
  initSubscriber();
}

ImageTransportSubscription::ImageTransportSubscription()
    : default_transport_( "compressed" ), clock_( RCL_ROS_TIME ), queue_size_( 1 )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscription::onNoImageTimeout,
           Qt::AutoConnection );
}

QAbstractVideoSurface *ImageTransportSubscription::videoSurface() const { return surface_; }

void ImageTransportSubscription::setVideoSurface( QAbstractVideoSurface *surface )
{
  if ( surface == surface_ )
    return;
  if ( surface_ != nullptr && surface_->isActive() )
    surface_->stop();
  surface_ = surface;
  if ( surface_ == nullptr && subscribed_ ) {
    shutdownSubscriber();
    return;
  }
  if ( !subscribed_ )
    initSubscriber();
  if ( last_frame_.isValid() )
    presentFrame( last_frame_ );
}

void ImageTransportSubscription::onRos2Initialized() { initSubscriber(); }

void ImageTransportSubscription::onRos2Shutdown() { shutdownSubscriber(); }

void ImageTransportSubscription::initSubscriber()
{
  // This makes sure we lazy subscribe and only subscribe if there is a surface to write to
  if ( surface_ == nullptr )
    return;
  if ( !Ros2Qml::getInstance().isInitialized() )
    return;
  if ( topic_.isEmpty() )
    return;
  bool was_subscribed = subscribed_;
  if ( subscribed_ ) {
    blockSignals( true );
    shutdownSubscriber();
    blockSignals( false );
  }
  // TODO Transport hints
  const rclcpp::Node::SharedPtr &node = Ros2Qml::getInstance().node();
  image_transport::TransportHints transport_hints( node.get(), default_transport_.toStdString() );
  subscription_ = ImageTransportManager::getInstance().subscribe(
      node, topic_, queue_size_, transport_hints,
      [this]( const QVideoFrame &frame ) { presentFrame( frame ); }, surface_ );
  subscribed_ = subscription_ != nullptr;
  if ( !was_subscribed )
    emit subscribedChanged();
}

void ImageTransportSubscription::shutdownSubscriber()
{
  if ( !subscribed_ )
    return;
  subscription_.reset();
  if ( surface_ != nullptr && surface_->isActive() )
    surface_->stop();
  subscribed_ = false;
  emit subscribedChanged();
}

void ImageTransportSubscription::onNoImageTimeout()
{
  if ( surface_ == nullptr || !surface_->isActive() )
    return;
  int elapsed_time_milliseconds =
      static_cast<int>( ( clock_.now() - last_frame_timestamp_ ).nanoseconds() / 1000000 );

  if ( timeout_ == 0 )
    return;
  if ( elapsed_time_milliseconds < timeout_ ) {
    no_image_timer_.start( timeout_ - elapsed_time_milliseconds );
    return;
  }
  surface_->present( QVideoFrame() );
}

namespace
{
const char *videoSurfaceErrorToString( QAbstractVideoSurface::Error error )
{
  switch ( error ) {
  case QAbstractVideoSurface::NoError:
    return "NoError";
  case QAbstractVideoSurface::UnsupportedFormatError:
    return "UnsupportedFormatError";
  case QAbstractVideoSurface::IncorrectFormatError:
    return "IncorrectFormatError";
  case QAbstractVideoSurface::StoppedError:
    return "StoppedError";
  case QAbstractVideoSurface::ResourceError:
    return "ResourceError";
  default:
    return "UnknownError";
  }
}
} // namespace

void ImageTransportSubscription::presentFrame( const QVideoFrame &frame )
{
  if ( surface_ == nullptr )
    return;
  const QVideoSurfaceFormat &surface_format = surface_->surfaceFormat();
  if ( surface_format.frameWidth() != frame.width() ||
       surface_format.frameHeight() != frame.height() ||
       surface_format.pixelFormat() != frame.pixelFormat() ) {
    format_ = QVideoSurfaceFormat( frame.size(), frame.pixelFormat() );
    surface_->stop();
  }
  if ( !surface_->isActive() ) {
    format_ = QVideoSurfaceFormat( frame.size(), frame.pixelFormat() );
    if ( format_.pixelFormat() == QVideoFrame::Format_Invalid ) {
      QML_ROS2_PLUGIN_ERROR( "Could not find compatible format for video surface." );
      shutdownSubscriber();
      return;
    }
    if ( !surface_->start( format_ ) ) {
      QML_ROS2_PLUGIN_ERROR( "Failed to start video surface: %s",
                             videoSurfaceErrorToString( surface_->error() ) );
      shutdownSubscriber();
      return;
    }
  }
  last_frame_ = frame;
  surface_->present( frame );
  // Return if this is the last frame of our subscription.
  if ( subscription_ == nullptr )
    return;

  bool network_latency_changed = last_network_latency_ != subscription_->networkLatency();
  bool processing_latency_changed = last_processing_latency_ != subscription_->processingLatency();
  if ( network_latency_changed )
    emit networkLatencyChanged();
  if ( processing_latency_changed )
    emit processingLatencyChanged();
  if ( network_latency_changed || processing_latency_changed )
    emit latencyChanged();
  if ( std::abs( last_framerate_ - subscription_->framerate() ) > 0.1 )
    emit framerateChanged();
  last_framerate_ = subscription_->framerate();
  last_frame_timestamp_ = clock_.now();
  last_network_latency_ = subscription_->networkLatency();
  last_processing_latency_ = subscription_->processingLatency();
  if ( timeout_ != 0 ) {
    no_image_timer_.start( throttle_interval_ + timeout_ );
  }
}

QString ImageTransportSubscription::topic() const
{
  if ( subscription_ )
    return QString::fromStdString( subscription_->getTopic() );
  return topic_;
}

void ImageTransportSubscription::setTopic( const QString &value )
{
  if ( topic_ == value )
    return;
  topic_ = value;
  emit topicChanged();
}

const QString &ImageTransportSubscription::defaultTransport() const { return default_transport_; }

void ImageTransportSubscription::setDefaultTransport( const QString &value )
{
  if ( default_transport_ == value )
    return;
  default_transport_ = value;
  emit defaultTransportChanged();
}

bool ImageTransportSubscription::subscribed() const { return subscribed_; }

int ImageTransportSubscription::timeout() const { return timeout_; }

void ImageTransportSubscription::setTimeout( int value )
{
  timeout_ = value;
  emit timeoutChanged();
}

bool ImageTransportSubscription::enabled() const { return enabled_; }

void ImageTransportSubscription::setEnabled( bool value )
{
  if ( enabled_ == value )
    return;
  enabled_ = value;
  if ( enabled_ )
    initSubscriber();
  else
    shutdownSubscriber();
  emit enabledChanged();
}

double ImageTransportSubscription::framerate() const
{
  return subscription_ == nullptr ? 0 : subscription_->framerate();
}

int ImageTransportSubscription::latency() const
{
  return subscription_ == nullptr ? -1 : subscription_->latency();
}

int ImageTransportSubscription::networkLatency() const
{
  return subscription_ == nullptr ? -1 : subscription_->networkLatency();
}

int ImageTransportSubscription::processingLatency() const
{
  return subscription_ == nullptr ? -1 : subscription_->processingLatency();
}
} // namespace qml_ros2_plugin
