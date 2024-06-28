// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/publisher.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/ros2.hpp"

using namespace ros_babel_fish;
using namespace qml_ros2_plugin::conversion;

namespace qml_ros2_plugin
{

Publisher::Publisher( QString topic, QString type, uint32_t queue_size )
    : is_advertised_( false ), type_( std::move( type ) ), topic_( std::move( topic ) ),
      queue_size_( queue_size )
{
  std_type_ = type_.toStdString();
  babel_fish_ = BabelFishDispenser::getBabelFish();

  if ( isRosInitialized() )
    advertise();
}

Publisher::~Publisher() = default;

QString Publisher::topic() const { return QString::fromStdString( publisher_->get_topic_name() ); }

const QString &Publisher::type() const { return type_; }

quint32 Publisher::queueSize() const { return queue_size_; }

bool Publisher::isAdvertised() const { return is_advertised_; }

unsigned int Publisher::getSubscriptionCount()
{
  return publisher_ == nullptr ? 0 : publisher_->get_subscription_count();
}

bool Publisher::publish( const QVariantMap &msg )
{
  if ( !is_advertised_ )
    return false;
  try {
    ros_babel_fish::CompoundMessage message = babel_fish_.create_message( type_.toStdString() );
    if ( !fillMessage( message, msg ) )
      return false;
    publisher_->publish( message );
    return true;
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to publish message: %s", ex.what() );
  }
  return false;
}

void Publisher::onRos2Initialized()
{
  if ( !is_advertised_ )
    advertise();
}

void Publisher::onRos2Shutdown() { publisher_.reset(); }

void Publisher::advertise()
{
  if ( is_advertised_ )
    publisher_.reset();
  if ( type_.isEmpty() )
    return;
  if ( topic_.isEmpty() )
    return;

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();
  if ( node == nullptr )
    return;
  try {
    auto qos = rclcpp::QoS( queue_size_ ); // TODO latched
    publisher_ = babel_fish_.create_publisher( *node, topic_.toStdString(), std_type_, qos, {} );
    advertise_timer_.stop();
    is_advertised_ = true;
    emit advertised();
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to create publisher: %s", ex.what() );
  }
}
} // namespace qml_ros2_plugin
