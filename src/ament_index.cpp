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
