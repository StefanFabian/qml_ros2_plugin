// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_SUBSCRIPTION_HPP
#define QML_ROS2_PLUGIN_SUBSCRIPTION_HPP

#include "qml_ros2_plugin/qobject_ros2.hpp"

#include <QMap>
#include <QTimer>
#include <QVariant>

#include <mutex>

#include <ros_babel_fish/babel_fish.hpp>

namespace qml_ros2_plugin
{

class Subscription : public QObjectRos2
{
  Q_OBJECT
  //! The topic this subscriber subscribes to.
  Q_PROPERTY( QString topic READ topic WRITE setTopic NOTIFY topicChanged )
  //! The maximum number of messages that are queued for processing. Default: 10
  Q_PROPERTY( quint32 queueSize READ queueSize WRITE setQueueSize NOTIFY queueSizeChanged )

  //! The last message that was received by this subscriber.
  Q_PROPERTY( QVariant message READ message NOTIFY messageChanged )

  //! Set to a specific type to subscribe to that type, e.g., geometry_msgs/msg/Pose, otherwise the type is
  //! automatically detected and if the topic has multiple available types, one is arbitrarily selected.
  Q_PROPERTY( QString messageType READ messageType WRITE setMessageType NOTIFY messageTypeChanged )

  //! Limits the frequency in which the notification for an updated message is emitted. Default: 20 Hz
  Q_PROPERTY( int throttleRate READ throttleRate WRITE setThrottleRate NOTIFY throttleRateChanged )

  //! Controls whether or not the subscriber is currently enabled, i.e., able to receive messages. Default: true
  Q_PROPERTY( bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged )

  //! Indicates whether a subscription is active or not.
  Q_PROPERTY( bool subscribed READ subscribed NOTIFY subscribedChanged )
public:
  Subscription();

  Subscription( QString topic, QString message_type, quint32 queue_size, bool enabled = true );

  ~Subscription() override;

  QString topic() const;

  void setTopic( const QString &value );

  quint32 queueSize() const;

  void setQueueSize( quint32 value );

  bool enabled() const;

  void setEnabled( bool value );

  int throttleRate() const;

  void setThrottleRate( int value );

  bool subscribed() const;

  const QVariant &message() const;

  const QString &messageType() const;

  void setMessageType( const QString &value );

  //! @return The number of publishers this subscriber is connected to.
  Q_INVOKABLE unsigned int getPublisherCount();

signals:

  void subscribedChanged();

  void topicChanged();

  void queueSizeChanged();

  void throttleRateChanged();

  void enabledChanged();

  void messageChanged();

  void messageTypeChanged();

  /*!
   * Emitted whenever a new message was received.
   * @param message The received message.
   */
  void newMessage( QVariant message );

protected slots:

  void try_subscribe();

  void updateMessage();

protected:
  void initTimers();

  void subscribe();

  void onRos2Initialized() override;

  void onRos2Shutdown() override;

  void shutdown();

  void messageCallback( const std::shared_ptr<ros_babel_fish::CompoundMessage> &msg );

  QTimer subscribe_timer_;
  ros_babel_fish::BabelFish babel_fish_;
  ros_babel_fish::BabelFishSubscription::SharedPtr subscription_;
  ros_babel_fish::CompoundMessage::ConstSharedPtr last_message_;
  std::mutex message_mutex_;
  QTimer throttle_timer_;

  QString topic_;
  QString user_message_type_;
  QString message_type_;
  QVariant message_;
  quint32 queue_size_ = 10;
  int throttle_rate_ = 20;
  bool running_ = true;
  bool is_subscribed_ = false;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_SUBSCRIPTION_HPP
