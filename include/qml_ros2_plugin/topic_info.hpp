// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_TOPIC_INFO_HPP
#define QML_ROS2_PLUGIN_TOPIC_INFO_HPP

#include <QArgument>
#include <QMetaType>
#include <QString>

namespace qml_ros2_plugin
{

class TopicInfo
{
  Q_GADGET
  //! The name of the topic, e.g., /front_camera/image_raw
  Q_PROPERTY( QString name READ name )
  //! The datatype(s) of the topic, e.g., sensor_msgs/msg/Image
  Q_PROPERTY( QStringList datatypes READ datatypes )
public:
  TopicInfo() = default;

  TopicInfo( QString name, QStringList datatypes )
      : name_( std::move( name ) ), datatypes_( std::move( datatypes ) )
  {
  }

  const QString &name() const { return name_; }

  const QStringList &datatypes() const { return datatypes_; }

private:
  QString name_;
  QStringList datatypes_;
};
} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( qml_ros2_plugin::TopicInfo )

#endif // QML_ROS2_PLUGIN_TOPIC_INFO_HPP
