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
