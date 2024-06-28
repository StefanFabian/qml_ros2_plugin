// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/subscription.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/ros2.hpp"

using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

Subscription::Subscription()
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  initTimers();
}

Subscription::Subscription( QString topic, QString message_type, quint32 queue_size, bool enabled )
    : topic_( std::move( topic ) ), user_message_type_( std::move( message_type ) ),
      queue_size_( queue_size ), running_( enabled )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  initTimers();
  subscribe();
}

Subscription::~Subscription() = default;

void Subscription::initTimers()
{
  connect( &subscribe_timer_, &QTimer::timeout, this, &Subscription::subscribe );
  subscribe_timer_.setInterval( std::chrono::milliseconds( 100 ) );
  connect( &throttle_timer_, &QTimer::timeout, this, &Subscription::updateMessage );
  throttle_timer_.setSingleShot( false );
  throttle_timer_.setInterval( 1000 / throttle_rate_ );
}

QString Subscription::topic() const
{
  return subscription_ == nullptr ? QString() : QString( subscription_->get_topic_name() );
}

void Subscription::setTopic( const QString &value )
{
  topic_ = value;
  subscribe();
  emit topicChanged();
}

quint32 Subscription::queueSize() const { return queue_size_; }

void Subscription::setQueueSize( quint32 value )
{
  queue_size_ = value;
  subscribe();
  emit queueSizeChanged();
}

bool Subscription::enabled() const { return running_; }

void Subscription::setEnabled( bool value )
{
  if ( value == running_ )
    return;
  running_ = value;
  if ( running_ )
    subscribe();
  else
    shutdown();
  emit enabledChanged();
}

int Subscription::throttleRate() const { return throttle_rate_; }

void Subscription::setThrottleRate( int value )
{
  throttle_rate_ = value;
  throttle_timer_.setInterval( 1000 / throttle_rate_ );
  emit throttleRateChanged();
}

bool Subscription::subscribed() const { return is_subscribed_; }

const QVariant &Subscription::message() const { return message_; }

const QString &Subscription::messageType() const { return message_type_; }

void Subscription::setMessageType( const QString &value )
{
  user_message_type_ = value;
  if ( user_message_type_ == message_type_ )
    return;
  message_type_ = user_message_type_;
  subscribe();
  emit messageTypeChanged();
}

unsigned int Subscription::getPublisherCount()
{
  return is_subscribed_ ? subscription_->get_publisher_count() : 0;
}

void Subscription::onRos2Initialized()
{
  if ( !is_subscribed_ )
    subscribe();
}

void Subscription::onRos2Shutdown() { shutdown(); }

void Subscription::subscribe()
{
  if ( is_subscribed_ ) {
    shutdown();
  }
  if ( topic_.isEmpty() ) {
    subscribe_timer_.stop();
    return;
  }

  QML_ROS2_PLUGIN_DEBUG( "All required information available, starting subscription process." );
  subscribe_timer_.start();
  try_subscribe();
}

void Subscription::try_subscribe()
{
  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();
  if ( node == nullptr )
    return;
  if ( user_message_type_.isEmpty() ) {
    subscription_ = babel_fish_.create_subscription(
        *node, topic_.toStdString(), queue_size_,
        [this]( ros_babel_fish::CompoundMessage::SharedPtr msg ) { messageCallback( msg ); },
        nullptr, {}, std::chrono::nanoseconds( 0 ) );
  } else {
    subscription_ = babel_fish_.create_subscription(
        *node, topic_.toStdString(), user_message_type_.toStdString(), queue_size_,
        [this]( ros_babel_fish::CompoundMessage::SharedPtr msg ) { messageCallback( msg ); },
        nullptr, {} );
  }
  if ( subscription_ == nullptr )
    return;
  subscribe_timer_.stop();
  const QString new_message_type = QString::fromStdString( subscription_->get_message_type() );
  if ( new_message_type != message_type_ ) {
    message_type_ = new_message_type;
    emit messageTypeChanged();
  }
  QML_ROS2_PLUGIN_DEBUG( "Subscribed to '%s' with type: '%s'.", topic_.toStdString().c_str(),
                         message_type_.toStdString().c_str() );
  is_subscribed_ = true;
  emit subscribedChanged();
  throttle_timer_.start();
}

void Subscription::shutdown()
{
  if ( !is_subscribed_ )
    return;
  subscription_.reset();
  throttle_timer_.stop();
  is_subscribed_ = false;
  emit subscribedChanged();
}

void Subscription::messageCallback( const ros_babel_fish::CompoundMessage::SharedPtr &msg )
{
  std::lock_guard<std::mutex> lock( message_mutex_ );
  last_message_ = msg;
}

void Subscription::updateMessage()
{
  std::unique_lock<std::mutex> lock( message_mutex_ );
  if ( last_message_ == nullptr )
    return;
  ros_babel_fish::CompoundMessage::ConstSharedPtr msg = std::move( last_message_ );
  last_message_ = nullptr;
  lock.unlock(); // Don't need the mutex anymore
  message_ = msgToMap( *msg );
  emit messageChanged();
  emit newMessage( message_ );
}
} // namespace qml_ros2_plugin
