// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/io.hpp"
#include "qml_ros2_plugin/array.hpp"
#include "qml_ros2_plugin/conversion/qvariant_yaml_conversion.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"

#include <QAbstractListModel>
#include <QMetaProperty>

#include <fstream>
#include <yaml-cpp/yaml.h>

namespace qml_ros2_plugin
{

bool IO::writeYaml( QString path, const QVariant &value )
{
  if ( path.contains( QRegExp( "-*://" ) ) && !path.startsWith( "file://" ) ) {
    QML_ROS2_PLUGIN_ERROR( "Unsupported file path: %s", qPrintable( path ) );
    return false;
  }
  if ( path.startsWith( "file://" ) )
    path = path.mid( 7 );

  try {
    std::ofstream out( qPrintable( path ) );
    if ( !out ) {
      QML_ROS2_PLUGIN_ERROR( "Failed to open file: %s", qPrintable( path ) );
      return false;
    }
    YAML::Node yaml;
    yaml = value;
    out << yaml << std::endl;
    return true;
  } catch ( std::exception &e ) {
    return false;
  }
}

QVariant IO::readYaml( QString path )
{
  if ( path.contains( QRegExp( "-*://" ) ) && !path.startsWith( "file://" ) ) {
    QML_ROS2_PLUGIN_ERROR( "Unsupported file path: %s", qPrintable( path ) );
    return false;
  }
  if ( path.startsWith( "file://" ) )
    path = path.mid( 7 );

  try {
    YAML::Node node = YAML::LoadFile( path.toStdString() );
    return node.as<QVariant>();
  } catch ( std::exception &e ) {
    QML_ROS2_PLUGIN_ERROR( "Caught exception '%s' while trying to read file: %s", e.what(),
                           qPrintable( path ) );
    return QVariant::fromValue( false );
  }
}
} // namespace qml_ros2_plugin
