// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_IO_HPP
#define QML_ROS2_PLUGIN_IO_HPP

#include <QJSValue>

namespace qml_ros2_plugin
{

class IO
{
  Q_GADGET
public:
  /*!
   * Writes the given value to the given path in the yaml format.
   * @param path The path to the file.
   * @param value The value to write.
   * @return True if successful, false otherwise.
   */
  Q_INVOKABLE bool writeYaml( QString path, const QVariant &value );

  /*!
   * Reads a yaml file and returns the content in a QML compatible structure of 'QVariantMap's and 'QVariantList's.
   * @param path The path to the file.
   * @return A variant containing the file content or false.
   */
  Q_INVOKABLE QVariant readYaml( QString path );
};
} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( qml_ros2_plugin::IO )

#endif // QML_ROS2_PLUGIN_IO_HPP
