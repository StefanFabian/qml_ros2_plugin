// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/ament_index.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>

namespace qml_ros2_plugin
{

QString AmentIndex::getPackageShareDirectory( const QString &package_name )
{
  try {
    return QString::fromStdString(
        ament_index_cpp::get_package_share_directory( package_name.toStdString() ) );
  } catch ( ament_index_cpp::PackageNotFoundError &ex ) {
    return {};
  }
}

QStringList AmentIndex::getPackages()
{
  QStringList result;
  const std::map<std::string, std::string> &packages = ament_index_cpp::get_packages_with_prefixes();
  result.reserve( packages.size() );
  for ( const auto &s : packages ) result.append( QString::fromStdString( s.first ) );
  return result;
}
} // namespace qml_ros2_plugin
