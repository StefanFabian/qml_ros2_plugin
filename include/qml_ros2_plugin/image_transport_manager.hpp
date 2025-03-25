// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_IMAGE_TRANSPORT_MANAGER_HPP
#define QML_ROS2_PLUGIN_IMAGE_TRANSPORT_MANAGER_HPP

#include <QAbstractVideoSurface>
#include <image_transport/image_transport.hpp>
#include <mutex>

namespace qml_ros2_plugin
{
class ImageTransportSubscriptionHandle;

/*!
 * Encapsulates the image transport communication to share the subscription resources, avoiding multiple conversions of
 *  the same image and subscription overhead if multiple cameras are set to throttle.
 */
class ImageTransportManager
{
  ImageTransportManager();

  struct SubscriptionManager;

  class Subscription;

public:
  static ImageTransportManager &getInstance();

  /*!
   * Note: Can only be called with a ready NodeHandle!
   *
   * Subscribes to the given topic with the given settings. Makes sure that multiple subscriptions
   *  only result in a single subscription.
   * If multiple subscriptions of the same topic and namespace are created, the settings of the
   * first subscription are used.
   */
  std::shared_ptr<ImageTransportSubscriptionHandle>
  subscribe( const rclcpp::Node::SharedPtr &node, const QString &qtopic, quint32 queue_size,
             const image_transport::TransportHints &transport_hints,
             const std::function<void( const QVideoFrame & )> &callback,
             const QList<QVideoFrame::PixelFormat> &supported_pixel_formats );

private:
  std::shared_ptr<SubscriptionManager> subscription_manager_;

  friend class ImageTransportSubscriptionHandle;
};

class ImageTransportSubscriptionHandle
{
public:
  ~ImageTransportSubscriptionHandle();

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
  QList<QVideoFrame::PixelFormat> supported_pixel_formats;
  std::function<void( const QVideoFrame & )> callback;
  double framerate_ = 0;
  int network_latency = -1;
  int processing_latency = -1;

  friend class ImageTransportManager;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_IMAGE_TRANSPORT_MANAGER_HPP
