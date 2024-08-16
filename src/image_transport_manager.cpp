// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/image_transport_manager.hpp"

#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/helpers/rolling_average.h"
#include "qml_ros2_plugin/image_buffer.hpp"
#include "qml_ros2_plugin/qobject_ros2.hpp"

#include <QTimer>
#include <mutex>
#include <set>
#include <thread>

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

  explicit Subscription( image_transport::TransportHints hints ) : hints( std::move( hints ) ) { }

  ~Subscription() override = default;

  void subscribe()
  {
    if ( subscriptions_.empty() )
      return;
    // Subscribing on background thread to reduce load on UI thread
    std::thread( [this]() {
      // Make sure we don't subscribe twice in a row due to a race condition
      std::lock_guard<std::mutex> lock( subscribe_mutex_ );
      if ( subscriber_ )
        return;
      subscriber_ = subscription_manager->transport->subscribe(
          topic, queue_size, &Subscription::imageCallback, this, &hints );
    } ).detach();
  }

  void addSubscription( const std::shared_ptr<ImageTransportSubscriptionHandle> &sub )
  {
    std::lock_guard subscriptions_lock( subscriptions_mutex_ );
    subscriptions_.push_back( sub.get() );
    subscription_handles_.push_back( sub );
    updateSupportedFormats();
    // If this was the first subscription, subscribe
    if ( subscriptions_.size() == 1 )
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
      if ( last_image_ != nullptr ) {
        if ( rclcpp::Time( last_image_->header.stamp ).nanoseconds() == 0 )
          camera_base_interval_ = static_cast<int>(
              ( received_stamp - last_received_stamp_ ).nanoseconds() / ( 1000 * 1000 ) );
        else
          camera_base_interval_ = static_cast<int>(
              ( rclcpp::Time( image->header.stamp ) - last_image_->header.stamp ).nanoseconds() /
              ( 1000 * 1000 ) );
      }
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
    int base_interval;
    {
      std::lock_guard image_lock( image_mutex_ );
      if ( last_buffer_ == nullptr || last_image_ == nullptr )
        return;
      buffer = last_buffer_.release();
      image = last_image_;
      received = last_received_stamp_;
      base_interval = camera_base_interval_;
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
    const rclcpp::Time &image_stamp = image->header.stamp;
    int network_latency = image_stamp.nanoseconds() != 0
                              ? static_cast<int>( ( received - image_stamp ).seconds() * 1000 )
                              : -1;
    network_latency_average_.add( network_latency );
    auto processing_latency = static_cast<int>( ( clock_.now() - received ).seconds() * 1000 );
    processing_latency_average_.add( processing_latency );
    image_interval_average_.add( base_interval );
    for ( const auto &sub : subscribers ) {
      if ( sub == nullptr || !sub->callback )
        continue;
      sub->network_latency = network_latency_average_;
      sub->processing_latency = processing_latency_average_;
      sub->framerate_ = std::round( 1000.0 / image_interval_average_ * 10 ) / 10;
      sub->callback( frame );
    }
  }

  // It's expected that a lock is held for the subscriptions when calling this method
  void updateSupportedFormats()
  {
    bool first = true;
    for ( const auto &sub_weak : subscription_handles_ ) {
      std::shared_ptr<ImageTransportSubscriptionHandle> sub = sub_weak.lock();
      if ( sub == nullptr || sub->surface == nullptr )
        continue;
      const QList<QVideoFrame::PixelFormat> surface_formats = sub->surface->supportedPixelFormats();
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
  RollingAverage<int, 10> image_interval_average_;
  RollingAverage<int, 10> network_latency_average_;
  RollingAverage<int, 10> processing_latency_average_;
  rclcpp::Clock clock_ = rclcpp::Clock( RCL_ROS_TIME );
  rclcpp::Time last_received_stamp_;
  sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  std::unique_ptr<ImageBuffer> last_buffer_ = nullptr;
  int camera_base_interval_ = 0;
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
    const image_transport::TransportHints &transport_hints,
    const std::function<void( const QVideoFrame & )> &callback, QAbstractVideoSurface *surface )
{
  if ( subscription_manager_ == nullptr ) {
    subscription_manager_ = std::make_shared<SubscriptionManager>( node );
  }
  std::string topic = qtopic.toStdString();
  std::vector<std::shared_ptr<Subscription>> &subscriptions = subscription_manager_->subscriptions;
  try {
    size_t i = 0;
    for ( ; i < subscriptions.size(); ++i ) {
      if ( subscriptions[i]->topic == topic && subscriptions[i]->queue_size == queue_size )
        break; // We could also compare transport type and hints
    }
    auto handle = std::make_shared<ImageTransportSubscriptionHandle>();
    handle->surface = surface;
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
  } catch ( image_transport::TransportLoadException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Could not subscribe to image topic: %s", ex.what() );
  }
  return nullptr;
}
} // namespace qml_ros2_plugin

#include "image_transport_manager.moc"
