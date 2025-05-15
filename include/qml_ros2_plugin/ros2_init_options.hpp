// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_ROS2_INIT_OPTIONS_HPP
#define QML_ROS2_PLUGIN_ROS2_INIT_OPTIONS_HPP

#include <QObject>
#include <QString>

#include <rclcpp/init_options.hpp>

namespace qml_ros2_plugin
{

class Ros2InitOptions : public QObject
{
  Q_OBJECT
public:
  explicit Ros2InitOptions( QObject *parent = nullptr );
  ~Ros2InitOptions() override;

  Q_INVOKABLE QObject *setNamespace( const QString &ns );

  Q_INVOKABLE QString getNamespace() const;

  Q_INVOKABLE QObject *setDomainId( int domain_id );

  Q_INVOKABLE int getDomainId();

  Q_INVOKABLE QObject *useDefaultDomainId();

  const rclcpp::InitOptions &rclcppInitOptions() const;

private:
  QString namespace_;
  rclcpp::InitOptions options_;
};

} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_ROS2_INIT_OPTIONS_HPP
