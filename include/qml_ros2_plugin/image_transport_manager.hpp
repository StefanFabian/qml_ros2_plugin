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

#ifndef QML_ROS2_PLUGIN_IMAGE_TRANSPORT_MANAGER_HPP
#define QML_ROS2_PLUGIN_IMAGE_TRANSPORT_MANAGER_HPP

#include <QAbstractVideoSurface>
#include <image_transport/image_transport.hpp>
#include <mutex>

namespace qml_ros2_plugin
{
class ImageTransportSubscriptionHandle;

class ImageTransportManagerSingletonWrapper : public QObject
{
  Q_OBJECT
public:
  //! @copydoc ImageTransportManager::setLoadBalancingEnabled
  Q_INVOKABLE void setLoadBalancingEnabled( bool value );
};

/*!
 * Encapsulates the image transport communication to share the subscription resources, avoiding multiple conversions of
 *  the same image and subscription overhead if multiple cameras are set to throttle.
 */
class ImageTransportManager
{
  ImageTransportManager();

  struct SubscriptionManager;

  class Subscription;

  class LoadBalancer;

public:
  static ImageTransportManager &getInstance();

  //! Sets whether the manager should try to balance throttled subscriptions_ to ensure they don't update at the same
  //!   time which would result in network spikes.
  void setLoadBalancingEnabled( bool value );

  /*!
   * Note: Can only be called with a ready NodeHandle!
   *
   * Subscribes to the given topic with the given settings. Makes sure that multiple subscriptions_ (especially throttled)
   *  only result in a single (throttled) subscription.
   * If multiple subscriptions_ of the same topic and namespace are created, the settings of the first subscription are used.
   * Except for the throttle interval where the minimum value across all active subscriptions_ is used.
   * @param qtopic
   * @param queue_size
   * @param transport_hints
   * @param callback
   * @param surface
   * @param throttle_interval
   * @return
   */
  std::shared_ptr<ImageTransportSubscriptionHandle>
  subscribe( const rclcpp::Node::SharedPtr &node, const QString &qtopic, quint32 queue_size,
             const image_transport::TransportHints &transport_hints,
             const std::function<void( const QVideoFrame & )> &callback,
             QAbstractVideoSurface *surface = nullptr, int throttle_interval = 0 );

private:
  std::map<std::string, std::shared_ptr<SubscriptionManager>> subscriptions_;
  std::unique_ptr<LoadBalancer> load_balancer_;

  friend class ImageTransportSubscriptionHandle;
};

class ImageTransportSubscriptionHandle
{
public:
  ~ImageTransportSubscriptionHandle();

  //! The interval in ms the subscription waits between receiving images.
  int throttleInterval() const { return throttle_interval; }

  //! Set the interval in ms the subscription may wait between images.
  //! The images may still arrive at a higher rate if other subscriptions request it.
  void updateThrottleInterval( int interval );

  //! The subscribed topic. Once subscribed this is the full topic name without the transport.
  std::string getTopic() const;

  //! The full latency (in ms) from camera to your display excluding drawing time.
  int latency() const;

  //! The latency (in ms) from the camera to the reception of the image in this node.
  int networkLatency() const;

  //! The latency (in ms) from the reception of the image until it is in a displayable format.
  int processingLatency() const;

  //! The framerate (in frames per second).
  double framerate() const;

private:
  std::shared_ptr<ImageTransportManager::Subscription> subscription;
  QAbstractVideoSurface *surface = nullptr;
  std::function<void( const QVideoFrame & )> callback;
  double framerate_ = 0;
  int throttle_interval = 0;
  int network_latency = -1;
  int processing_latency = -1;

  friend class ImageTransportManager;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_IMAGE_TRANSPORT_MANAGER_HPP
