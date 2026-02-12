// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/image_transport_manager.hpp"

#include "./ring_buffer.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/helpers/rolling_average.h"
#include "qml_ros2_plugin/image_buffer.hpp"
#include "qml_ros2_plugin/qobject_ros2.hpp"

#include <QTimer>
#include <mutex>
#include <set>
#include <thread>

using namespace std::chrono_literals;

namespace qml_ros2_plugin
{

struct ImageTransportManager::SubscriptionManager {
  explicit SubscriptionManager( const rclcpp::Node::SharedPtr &node )
  {
    transport = std::make_unique<image_transport::ImageTransport>( node );
  }

  std::vector<std::shared_ptr<Subscription>> subscriptions;
  std::unique_ptr<image_transport::ImageTransport> transport;
};

class ImageTransportManager::Subscription final : public QObject
{
  Q_OBJECT
public:
  image_transport::TransportHints hints;
  std::shared_ptr<SubscriptionManager> subscription_manager;
  std::string topic;
  ImageTransportManager *manager = nullptr;
  quint32 queue_size = 0;
  std::thread subscribe_thread;
  std::future<void> subscribe_future_;
  // Use a timer to unsubscribe to avoid unsubscribing and resubscribing in quick succession
  QTimer unsubscribe_timer;

  explicit Subscription( image_transport::TransportHints hints ) : hints( std::move( hints ) )
  {
    unsubscribe_timer.setInterval( 1000 );
    unsubscribe_timer.setSingleShot( true );
    QObject::connect( &unsubscribe_timer, &QTimer::timeout, [this]() {
      std::lock_guard subscriptions_lock( subscriptions_mutex_ );
      if ( subscriptions_.empty() && subscriber_ ) {
        QML_ROS2_PLUGIN_DEBUG(
            "No more subscriptions left for topic '%s'. Shutting down subscriber.", topic.c_str() );
        subscriber_.shutdown();
        std::lock_guard image_lock( image_mutex_ );
        last_image_ = nullptr;
        last_buffer_ = nullptr;
      }
    } );
  }

  ~Subscription() override = default;

  void subscribe()
  {
    if ( subscriptions_.empty() )
      return;
    // Make sure we don't subscribe twice in a row due to a race condition
    std::unique_lock lock( subscribe_mutex_ );
    if ( subscribe_future_.valid() &&
         subscribe_future_.wait_for( 0ms ) == std::future_status::timeout ) {
      // Already subscribing
      return;
    }
    // Subscribing on background thread to reduce load on UI thread
    subscribe_future_ = std::async( std::launch::async, [this]() {
      if ( subscriber_ )
        return;
      try {
        auto subscriber = subscription_manager->transport->subscribe(
            topic, queue_size, &Subscription::imageCallback, this, &hints );
        std::lock_guard subscriptions_lock( subscriptions_mutex_ );
        if ( subscriptions_.empty() )
          subscriber.shutdown();
        else
          subscriber_ = std::move( subscriber );
      } catch ( std::exception &ex ) {
        QML_ROS2_PLUGIN_ERROR( "Failed to subscribe to topic '%s' with transport '%s': %s",
                               topic.c_str(), hints.getTransport().c_str(), e.what() );
      }
    } );
  }

  void addSubscription( const std::shared_ptr<ImageTransportSubscriptionHandle> &sub )
  {
    std::lock_guard subscriptions_lock( subscriptions_mutex_ );
    subscriptions_.push_back( sub.get() );
    subscription_handles_.push_back( sub );
    updateSupportedFormats();
    // If this was the first subscription, subscribe
    if ( !subscriber_ )
      subscribe();
  }

  void removeSubscription( const ImageTransportSubscriptionHandle *sub )
  {
    std::lock_guard subscriptions_lock( subscriptions_mutex_ );
    auto it = std::find_if(
        subscriptions_.begin(), subscriptions_.end(),
        [sub]( const ImageTransportSubscriptionHandle *handle ) { return handle == sub; } );
    if ( it == subscriptions_.end() ) {
      QML_ROS2_PLUGIN_ERROR(
          "Tried to remove a subscription that was not found! Please file a bug report!" );
      return;
    }
    size_t index = it - subscriptions_.begin();
    subscriptions_.erase( it );
    subscription_handles_.erase( subscription_handles_.begin() + index );
    bool subscriber_active = subscriber_;
    if ( subscriptions_.empty() && subscriber_active ) {
      unsubscribe_timer.start();
    }
    updateSupportedFormats();
  }

  std::string getTopic() const
  {
    const std::string &topic = subscriber_.getTopic();
    const std::string &transport = subscriber_.getTransport();
    if ( topic.size() < transport.size() + 1 ||
         0 != topic.compare( topic.size() - transport.size() - 1, transport.size() + 1,
                             "/" + transport ) )
      return topic;
    return topic.substr( 0, topic.size() - transport.size() - 1 );
  }

private:
  void imageCallback( const sensor_msgs::msg::Image::ConstSharedPtr &image )
  {
    rclcpp::Time received_stamp = clock_.now();
    QList<QVideoFrame::PixelFormat> formats;
    {
      std::lock_guard subscription_lock( subscriptions_mutex_ );
      if ( subscription_handles_.empty() )
        return;
      formats = supported_formats_;
    }
    auto buffer = std::make_unique<ImageBuffer>( image, formats );
    {
      std::lock_guard image_lock( image_mutex_ );
      last_received_stamp_ = received_stamp;
      last_image_ = image;
      last_buffer_ = std::move( buffer );
    }
    // Deliver frames on UI thread
    QMetaObject::invokeMethod( this, "imageDelivery", Qt::AutoConnection );
  }

  Q_INVOKABLE void imageDelivery()
  {
    ImageBuffer *buffer;
    sensor_msgs::msg::Image::ConstSharedPtr image;
    rclcpp::Time received;
    {
      std::lock_guard image_lock( image_mutex_ );
      if ( last_buffer_ == nullptr || last_image_ == nullptr )
        return;
      buffer = last_buffer_.release();
      image = last_image_;
      received = last_received_stamp_;
      last_buffer_ = nullptr;
    }
    QVideoFrame frame( buffer, QSize( image->width, image->height ), buffer->format() );
    std::vector<std::shared_ptr<ImageTransportSubscriptionHandle>> subscribers;
    {
      // This nested lock makes sure that our destruction of the subscription pointer will not lead to a deadlock
      std::lock_guard subscriptions_lock( subscriptions_mutex_ );
      for ( const auto &sub_weak : subscription_handles_ ) {
        if ( sub_weak.expired() )
          continue;
        subscribers.push_back( sub_weak.lock() );
      }
    }
    ImageInformation info;
    info.timestamp = image->header.stamp;
    info.encoding = image->encoding;
    int network_latency = info.timestamp.nanoseconds() != 0
                              ? static_cast<int>( ( received - info.timestamp ).seconds() * 1000 )
                              : -1;
    network_latency_average_.add( network_latency );
    auto processing_latency = static_cast<int>( ( clock_.now() - received ).seconds() * 1000 );
    processing_latency_average_.add( processing_latency );
    if ( info.timestamp.nanoseconds() != 0 ) {
      image_interval_average_.push_back( info.timestamp );
    } else {
      image_interval_average_.push_back( last_received_stamp_ );
    }
    const auto image_stamp_diff =
        ( image_interval_average_.back() - image_interval_average_.front() ).nanoseconds();
    const float framerate =
        image_stamp_diff > 0 ? ( image_interval_average_.size() - 1 ) * 1E9f / image_stamp_diff : 0;
    for ( const auto &sub : subscribers ) {
      if ( sub == nullptr || !sub->callback )
        continue;
      sub->network_latency = network_latency_average_;
      sub->processing_latency = processing_latency_average_;
      sub->framerate_ = std::round( framerate * 10 ) / 10;
      sub->callback( frame, info );
    }
  }

  // It's expected that a lock is held for the subscriptions when calling this method
  void updateSupportedFormats()
  {
    bool first = true;
    for ( const auto &sub_weak : subscription_handles_ ) {
      std::shared_ptr<ImageTransportSubscriptionHandle> sub = sub_weak.lock();
      if ( sub == nullptr )
        continue;
      const QList<QVideoFrame::PixelFormat> &surface_formats = sub->supported_pixel_formats;
      if ( first ) {
        supported_formats_ = surface_formats;
        first = false;
        continue;
      }
      for ( int i = supported_formats_.size() - 1; i >= 0; --i ) {
        if ( surface_formats.contains( supported_formats_[i] ) )
          continue;
        supported_formats_.removeAt( i );
      }
    }
  }

  std::mutex subscriptions_mutex_;
  std::mutex subscribe_mutex_;
  std::mutex image_mutex_;
  image_transport::Subscriber subscriber_;
  std::vector<ImageTransportSubscriptionHandle *> subscriptions_;
  std::vector<std::weak_ptr<ImageTransportSubscriptionHandle>> subscription_handles_;
  QList<QVideoFrame::PixelFormat> supported_formats_;
  RingBuffer<rclcpp::Time, 10> image_interval_average_;
  RollingAverage<int, 10> network_latency_average_;
  RollingAverage<int, 10> processing_latency_average_;
  rclcpp::Clock clock_ = rclcpp::Clock( RCL_ROS_TIME );
  rclcpp::Time last_received_stamp_;
  sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  std::unique_ptr<ImageBuffer> last_buffer_ = nullptr;
};

ImageTransportSubscriptionHandle::~ImageTransportSubscriptionHandle()
{
  try {
    subscription->removeSubscription( this );
  } catch ( std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Error while removing subscription: %s", ex.what() );
  }
}

std::string ImageTransportSubscriptionHandle::getTopic() const { return subscription->getTopic(); }

int ImageTransportSubscriptionHandle::latency() const
{
  return network_latency + processing_latency;
}

int ImageTransportSubscriptionHandle::networkLatency() const { return network_latency; }

int ImageTransportSubscriptionHandle::processingLatency() const { return processing_latency; }

double ImageTransportSubscriptionHandle::framerate() const { return framerate_; }

ImageTransportManager::ImageTransportManager() = default;

ImageTransportManager &ImageTransportManager::getInstance()
{
  static ImageTransportManager manager;
  return manager;
}

std::shared_ptr<ImageTransportSubscriptionHandle> ImageTransportManager::subscribe(
    const rclcpp::Node::SharedPtr &node, const QString &qtopic, quint32 queue_size,
    const image_transport::TransportHints &transport_hints, const ImageCallback &callback,
    const QList<QVideoFrame::PixelFormat> &supported_pixel_formats )
{
  if ( subscription_manager_ == nullptr ) {
    subscription_manager_ = std::make_shared<SubscriptionManager>( node );
  }
  std::string topic = qtopic.toStdString();
  std::vector<std::shared_ptr<Subscription>> &subscriptions = subscription_manager_->subscriptions;
  size_t i = 0;
  for ( ; i < subscriptions.size(); ++i ) {
    if ( subscriptions[i]->topic == topic && subscriptions[i]->queue_size == queue_size &&
         subscriptions[i]->hints.getTransport() == transport_hints.getTransport() )
      break; // We could also compare transport type and hints
  }
  auto handle = std::make_shared<ImageTransportSubscriptionHandle>();
  handle->supported_pixel_formats = supported_pixel_formats;
  handle->callback = callback;
  if ( i == subscriptions.size() ) {
    auto sub = std::make_shared<Subscription>( transport_hints );
    sub->manager = this;
    sub->subscription_manager = subscription_manager_;
    sub->topic = topic;
    sub->queue_size = queue_size;
    subscriptions.emplace_back( sub );
    QML_ROS2_PLUGIN_DEBUG( "Subscribed to '%s' with transport '%s'.", topic.c_str(),
                           transport_hints.getTransport().c_str() );
  }
  handle->subscription = subscriptions[i];
  subscriptions[i]->addSubscription( handle );
  return handle;
}

std::shared_ptr<ImageTransportSubscriptionHandle>
ImageTransportManager::subscribe( const rclcpp::Node::SharedPtr &node, const QString &qtopic,
                                  quint32 queue_size,
                                  const image_transport::TransportHints &transport_hints,
                                  const std::function<void( const QVideoFrame & )> &callback,
                                  const QList<QVideoFrame::PixelFormat> &supported_pixel_formats )
{
  return subscribe(
      node, qtopic, queue_size, transport_hints,
      [callback]( const QVideoFrame &frame, const ImageInformation & ) { callback( frame ); },
      supported_pixel_formats );
}
} // namespace qml_ros2_plugin

#include "image_transport_manager.moc"
