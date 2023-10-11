// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_AMENT_INDEX_HPP
#define QML_ROS2_PLUGIN_AMENT_INDEX_HPP

#include <QtCore>

namespace qml_ros2_plugin
{

/*!
 * Wrapper for ament_index_cpp.
 */
class AmentIndex : public QObject
{
  Q_OBJECT

public:
  /*!
   * Queries the path to the share directory for a given package.
   * @param package_name The name of the package.
   * @return The fully-qualified path to the share directory of the given
   * package or an empty string if the package is not found.
   */
  Q_INVOKABLE QString getPackageShareDirectory( const QString &package_name );

  //! @return A list of all packages.
  Q_INVOKABLE QStringList getPackages();
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_AMENT_INDEX_HPP
