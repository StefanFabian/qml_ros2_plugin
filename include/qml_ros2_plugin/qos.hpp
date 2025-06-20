// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_QOS_HPP
#define QML_ROS2_PLUGIN_QOS_HPP

#include <QObject>
#include <QString>

#include <rclcpp/qos.hpp>

namespace qml_ros2_plugin
{

//! @brief Wrapper to enable setting QoS settings in QML.
//! Defaults to 1 depth, best effort reliability and volatile durability.
class QoSWrapper
{
  Q_GADGET
public:
  QoSWrapper();
  explicit QoSWrapper( rclcpp::QoS qos );

  //! @brief Sets the reliability policy to reliable. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml_ros2_plugin::QoSWrapper reliable();

  //! @brief Sets the reliability policy to best effort. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml_ros2_plugin::QoSWrapper best_effort();

  //! @brief Sets the durability policy to volatile. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml_ros2_plugin::QoSWrapper durability_volatile();

  //! @brief Sets the durability policy to transient local. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml_ros2_plugin::QoSWrapper transient_local();

  //! @brief Sets the history policy to keep all messages. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml_ros2_plugin::QoSWrapper keep_all();

  //! @brief Sets the history policy to keep last messages. Returns the QoSWrapper for chaining.
  //! @param depth The number of messages to keep.
  Q_INVOKABLE qml_ros2_plugin::QoSWrapper keep_last( int depth );

  //! @brief Returns the depth of the QoS policy. If history policy is set to keep_last, this is the
  //!        number of messages to keep.
  Q_INVOKABLE int depth() const;

  const rclcpp::QoS &rclcppQoS() const { return qos_; }

  std::string toString() const;

private:
  rclcpp::QoS qos_;
};

} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( qml_ros2_plugin::QoSWrapper )

#endif // QML_ROS2_PLUGIN_QOS_HPP
