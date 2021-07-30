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
