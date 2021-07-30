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

class ImageTransportManager::LoadBalancer : public QObject
{
  Q_OBJECT
public:
  LoadBalancer()
  {
    connect( &timer_, &QTimer::timeout, this, &ImageTransportManager::LoadBalancer::onTimeout );
    timer_.setInterval( 33 );
    timer_.setSingleShot( false );
    timer_.start();
  }

  void setEnabled( bool value ) { load_balancing_enabled_ = value; }

  void notifyImageReceived( ImageTransportManager::Subscription *subscription,
                            long throttle_interval );

  void registerThrottledSubscription( ImageTransportManager::Subscription *subscription );

  void unregister( ImageTransportManager::Subscription *subscription );

private slots:

  void onTimeout()
  {
    // Triggers subscription for all expired timeouts and restarts the timer unless no more timeouts are active
    std::lock_guard<std::mutex> lock( mutex_ );
    using namespace std::chrono;
    size_t i = 0;
    long now =
        duration_cast<std::chrono::milliseconds>( system_clock::now().time_since_epoch() ).count();
    std::vector<ImageTransportManager::Subscription *> ready;
    for ( ; i < timeouts_.size(); ++i ) {
      if ( timeouts_[i].first > now )
        break;
      ready.push_back( timeouts_[i].second );
    }
    for ( auto &sub : ready ) triggerSubscription( sub );
  }

private:
  void triggerSubscription( ImageTransportManager::Subscription *subscription );

  void insertTimeout( long desired_throttle_interval,
                      ImageTransportManager::Subscription *subscription );

  QTimer timer_;
  std::set<ImageTransportManager::Subscription *> waiting_subscriptions_;
  std::vector<ImageTransportManager::Subscription *> new_subscriptions_;
  std::vector<std::pair<long, ImageTransportManager::Subscription *>> timeouts_;
  std::mutex mutex_;
  bool load_balancing_enabled_ = true;
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

  Subscription( image_transport::TransportHints hints ) : hints( std::move( hints ) ) { }

  ~Subscription() final
  {
    if ( throttled_ )
      manager->load_balancer_->unregister( this );
  }

  int getThrottleInterval() const
  {
    int interval = std::accumulate(
        subscription_handles_.begin(), subscription_handles_.end(), std::numeric_limits<int>::max(),
        []( int current, const std::weak_ptr<ImageTransportSubscriptionHandle> &sub_weak ) {
          std::shared_ptr<ImageTransportSubscriptionHandle> sub = sub_weak.lock();
          if ( sub == nullptr )
            return current;
          return std::min( current, sub->throttle_interval );
        } );
    return interval == std::numeric_limits<int>::max() ? current_throttle_interval_ : interval;
  }

  void updateThrottling()
  {
    int min_interval = getThrottleInterval();
    if ( min_interval <= 0 ) {
      if ( throttled_ )
        manager->load_balancer_->unregister( this );
      throttled_ = false;
      if ( !subscriber_ )
        subscribe();
    } else if ( !throttled_ ) {
      throttled_ = true;
      manager->load_balancer_->registerThrottledSubscription( this );
    }
  }

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
    std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
    subscriptions_.push_back( sub.get() );
    subscription_handles_.push_back( sub );
    updateSupportedFormats();
    updateThrottling();
    // If this was the first subscription, subscribe
    if ( subscriptions_.size() == 1 )
      subscribe();
  }

  void removeSubscription( const ImageTransportSubscriptionHandle *sub )
  {
    std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
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
    updateThrottling();
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
    // Ignore image if timestamp did not differ which seems to be a bug of some gazebo camera plugins when throttling
    if ( throttled_ && rclcpp::Time( image->header.stamp ).nanoseconds() != 0 &&
         last_image_ != nullptr && last_image_->header.stamp == image->header.stamp )
      return;

    rclcpp::Time received_stamp = clock_.now();
    // Check if we have to throttle
    int interval = getThrottleInterval();
    if ( throttled_ ) {
      if ( interval != 0 && interval > camera_base_interval_ ) {
        {
          std::lock_guard<std::mutex> lock( subscribe_mutex_ );
          subscriber_.shutdown();
        }
        manager->load_balancer_->notifyImageReceived( this, interval );
      } else {
        // No need to throttle
        manager->load_balancer_->unregister( this );
        throttled_ = false;
      }
    } else if ( interval != 0 && interval > camera_base_interval_ *
                                                1.1 ) // 10% off for hysteresis to prevent oscillation
    {
      // Need to throttle now
      {
        std::lock_guard<std::mutex> lock( subscribe_mutex_ );
        subscriber_.shutdown();
      }
      manager->load_balancer_->registerThrottledSubscription( this );
      throttled_ = true;
    }
    QList<QVideoFrame::PixelFormat> formats;
    {
      std::lock_guard<std::mutex> subscription_lock( subscriptions_mutex_ );
      if ( subscription_handles_.empty() )
        return;
      formats = supported_formats_;
    }
    auto buffer = new ImageBuffer( image, formats );

    {
      std::lock_guard<std::mutex> image_lock( image_mutex_ );
      if ( !throttled_ && last_image_ != nullptr ) {
        // Update the base interval only if we are not throttled
        if ( rclcpp::Time( last_image_->header.stamp ).nanoseconds() == 0 )
          camera_base_interval_ = static_cast<int>(
              ( received_stamp - last_received_stamp_ ).nanoseconds() / ( 1000 * 1000 ) );
        else
          camera_base_interval_ = static_cast<int>(
              ( rclcpp::Time( image->header.stamp ) - last_image_->header.stamp ).nanoseconds() /
              ( 1000 * 1000 ) );
      } else if ( throttled_ ) {
        current_throttle_interval_ = interval;
      }
      last_received_stamp_ = received_stamp;
      last_image_ = image;
      delete last_buffer_;
      last_buffer_ = buffer;
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
      std::lock_guard<std::mutex> image_lock( image_mutex_ );
      if ( last_buffer_ == nullptr || last_image_ == nullptr )
        return;
      buffer = last_buffer_;
      image = last_image_;
      received = last_received_stamp_;
      base_interval = throttled_ ? current_throttle_interval_ : camera_base_interval_;
      last_buffer_ = nullptr;
    }
    QVideoFrame frame( buffer, QSize( image->width, image->height ), buffer->format() );
    std::vector<std::shared_ptr<ImageTransportSubscriptionHandle>> subscribers;
    {
      // This nested lock makes sure that our destruction of the subscription pointer will not lead to a deadlock
      std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
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
    int processing_latency = static_cast<int>( ( clock_.now() - received ).seconds() * 1000 );
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
  ImageBuffer *last_buffer_ = nullptr;
  int camera_base_interval_ = 0;
  int current_throttle_interval_ = 0;
  bool throttled_ = false;
};

// ============================= Load Balancing =============================
void ImageTransportManager::LoadBalancer::notifyImageReceived(
    ImageTransportManager::Subscription *subscription, long throttle_interval )
{
  std::lock_guard<std::mutex> lock( mutex_ );
  waiting_subscriptions_.erase( subscription );
  insertTimeout( throttle_interval, subscription );
  if ( !waiting_subscriptions_.empty() && !new_subscriptions_.empty() ) {
    // Can trigger the next camera if we have new subscriptions
    triggerSubscription( new_subscriptions_.front() );
  }
}

void ImageTransportManager::LoadBalancer::registerThrottledSubscription( Subscription *subscription )
{
  std::lock_guard<std::mutex> lock( mutex_ );
  for ( auto &timeout : timeouts_ )
    if ( timeout.second == subscription )
      return;
  insertTimeout( subscription->getThrottleInterval(), subscription );
  if ( !waiting_subscriptions_.empty() ) {
    new_subscriptions_.push_back( subscription );
    return;
  }
  triggerSubscription( subscription );
}

void ImageTransportManager::LoadBalancer::unregister( ImageTransportManager::Subscription *subscription )
{
  std::lock_guard<std::mutex> lock( mutex_ );
  waiting_subscriptions_.erase( subscription );
  new_subscriptions_.erase(
      std::remove( new_subscriptions_.begin(), new_subscriptions_.end(), subscription ),
      new_subscriptions_.end() );
  timeouts_.erase(
      std::remove_if( timeouts_.begin(), timeouts_.end(),
                      [subscription]( auto item ) { return item.second == subscription; } ),
      timeouts_.end() );
}

void ImageTransportManager::LoadBalancer::triggerSubscription(
    ImageTransportManager::Subscription *subscription )
{
  new_subscriptions_.erase(
      std::remove( new_subscriptions_.begin(), new_subscriptions_.end(), subscription ),
      new_subscriptions_.end() );
  timeouts_.erase(
      std::remove_if( timeouts_.begin(), timeouts_.end(),
                      [subscription]( auto item ) { return item.second == subscription; } ),
      timeouts_.end() );

  waiting_subscriptions_.insert( subscription );
  subscription->subscribe();
}

void ImageTransportManager::LoadBalancer::insertTimeout(
    const long desired_throttle_interval, ImageTransportManager::Subscription *subscription )
{
  const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now().time_since_epoch() )
                       .count();
  long timeout = now + desired_throttle_interval;
  if ( !load_balancing_enabled_ ) {
    std::pair<long, ImageTransportManager::Subscription *> value( timeout, subscription );
    timeouts_.insert( std::upper_bound( timeouts_.begin(), timeouts_.end(), value,
                                        []( auto a, auto b ) { return a.first < b.first; } ),
                      value );
    return;
  }
  long largest_gap = timeouts_.empty() ? 0 : timeouts_[0].first - now;
  size_t ind_largest_gap = 0;
  // Very unsophisticated method. We just find the largest gap before the desired interval and insert the desired_throttle_interval in the middle
  for ( size_t i = 1; i < timeouts_.size(); ++i ) {
    if ( timeouts_[i - 1].first > timeout )
      break;
    long gap;
    // Gap is either between two timeouts or between now and the first desired_throttle_interval that is not in the past
    if ( timeouts_[i - 1].first < now ) {
      if ( timeouts_[i].first <= now )
        continue;
      gap = timeouts_[i].first - now;
    } else {
      gap = timeouts_[i].first > timeout ? timeout - timeouts_[i - 1].first
                                         : timeouts_[i].first - timeouts_[i - 1].first;
    }
    if ( gap <= largest_gap )
      continue;
    ind_largest_gap = i;
    largest_gap = gap;
  }
  // Update timeout if inserting it with the desired timeout wouldn't create a larger gap (which can only happen if it is inserted at the end)
  if ( !timeouts_.empty() && timeout - timeouts_.back().first < largest_gap / 2 ) {
    timeout = timeouts_[ind_largest_gap].first - largest_gap / 2;
  }
  // We insert the desired_throttle_interval in the table so that the timeouts are sorted or before now
  std::pair<long, ImageTransportManager::Subscription *> value( timeout, subscription );
  timeouts_.insert( std::upper_bound( timeouts_.begin(), timeouts_.end(), value,
                                      []( auto a, auto b ) { return a.first < b.first; } ),
                    value );
}

ImageTransportSubscriptionHandle::~ImageTransportSubscriptionHandle()
{
  subscription->removeSubscription( this );
}

void ImageTransportSubscriptionHandle::updateThrottleInterval( int interval )
{
  throttle_interval = interval;
  subscription->updateThrottling();
}

std::string ImageTransportSubscriptionHandle::getTopic() const { return subscription->getTopic(); }

int ImageTransportSubscriptionHandle::latency() const
{
  return network_latency + processing_latency;
}

int ImageTransportSubscriptionHandle::networkLatency() const { return network_latency; }

int ImageTransportSubscriptionHandle::processingLatency() const { return processing_latency; }

double ImageTransportSubscriptionHandle::framerate() const { return framerate_; }

ImageTransportManager::ImageTransportManager()
{
  load_balancer_ = std::make_unique<ImageTransportManager::LoadBalancer>();
}

ImageTransportManager &ImageTransportManager::getInstance()
{
  static ImageTransportManager manager;
  return manager;
}

std::shared_ptr<ImageTransportSubscriptionHandle>
ImageTransportManager::subscribe( const rclcpp::Node::SharedPtr &node, const QString &qtopic,
                                  quint32 queue_size,
                                  const image_transport::TransportHints &transport_hints,
                                  const std::function<void( const QVideoFrame & )> &callback,
                                  QAbstractVideoSurface *surface, int throttle_interval )
{
  std::string ns =
      ""; // Support for namespaces like in qml_ros_plugin might be added in the future or dropped completely
  std::string topic = qtopic.toStdString();
  auto it = subscriptions_.find( ns );
  if ( it == subscriptions_.end() ) {
    it = subscriptions_.insert( { topic, std::make_shared<SubscriptionManager>( node ) } ).first;
  }
  std::shared_ptr<SubscriptionManager> &subscription_manager = it->second;
  std::vector<std::shared_ptr<Subscription>> &subscriptions = subscription_manager->subscriptions;
  try {
    size_t i = 0;
    for ( ; i < subscriptions.size(); ++i ) {
      if ( subscriptions[i]->topic == topic && subscriptions[i]->queue_size == queue_size )
        break; // We could also compare transport type and hints
    }
    auto handle = std::make_shared<ImageTransportSubscriptionHandle>();
    handle->surface = surface;
    handle->callback = callback;
    handle->throttle_interval = throttle_interval;
    if ( i == subscriptions.size() ) {
      auto sub = new Subscription( transport_hints );
      sub->manager = this;
      sub->subscription_manager = subscription_manager;
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

void ImageTransportManager::setLoadBalancingEnabled( bool value )
{
  load_balancer_->setEnabled( value );
}

void ImageTransportManagerSingletonWrapper::setLoadBalancingEnabled( bool value )
{
  ImageTransportManager::getInstance().setLoadBalancingEnabled( value );
}
} // namespace qml_ros2_plugin

#include "image_transport_manager.moc"
