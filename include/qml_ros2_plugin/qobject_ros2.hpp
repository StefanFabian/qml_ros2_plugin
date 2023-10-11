// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

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
