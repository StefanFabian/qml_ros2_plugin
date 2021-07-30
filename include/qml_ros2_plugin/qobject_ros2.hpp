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

#ifndef QML_ROS2_PLUGIN_QOBJECT_ROS2_HPP
#define QML_ROS2_PLUGIN_QOBJECT_ROS2_HPP

#include <QObject>

namespace qml_ros2_plugin
{

/*!
 * Base class for QObjects that require ROS functionality.
 * Provides virtual methods that are called once ROS was initialized and once it was shutdown to
 * enable initialization and clean-up of of functionality that requires ROS.
 */
class QObjectRos2 : public QObject
{
  Q_OBJECT
public:
  explicit QObjectRos2( QObject *parent = nullptr );

  ~QObjectRos2() override;

  //! @return Whether or not this object is initialized.
  bool isRosInitialized() const;

protected:
  /*!
   * Called once ROS was initialized in this application.
   *
   * Override to perform initialization of functionality that depends on ROS.
   */
  virtual void onRos2Initialized() { }

  /*!
   * Called once ROS or the application shuts down.
   *
   * Override to perform clean-up of functionality that depends on ROS, e.g., if the destruction
   * order does not guarantee destruction before required ROS objects are destructed.
   */
  virtual void onRos2Shutdown() { }

public slots:

  void _initialize();

  void _shutdown();

private:
  bool is_initialized_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_QOBJECT_ROS2_HPP
