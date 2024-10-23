// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_SERVICE_CLIENT_HPP
#define QML_ROS2_PLUGIN_SERVICE_CLIENT_HPP

#include "qml_ros2_plugin/qobject_ros2.hpp"
#include <QJSValue>
#include <QTimer>
#include <QVariant>

#include <ros_babel_fish/babel_fish.hpp>

namespace qml_ros2_plugin
{

class ServiceClient : public QObjectRos2
{
  Q_OBJECT
  //! True if the ServiceClient is connected to the Service and the Service is ready, false otherwise.
  Q_PROPERTY( bool ready READ isServiceReady NOTIFY serviceReadyChanged )
  //! The service name (topic in ROS 1).
  Q_PROPERTY( QString name READ name CONSTANT )
  //! The type of the service, e.g., "example_interfaces/srv/AddTwoInts".
  Q_PROPERTY( QString type READ type CONSTANT )
public:
  /*!
   * @param name The service topic.
   * @param type The type of the service, e.g., "example_interfaces/srv/AddTwoInts"
   */
  ServiceClient( QString name, QString type );

  //! Returns whether the service is ready.
  bool isServiceReady() const;

  const QString &name() const;

  const QString &type() const;

  /*!
   * Calls a service asynchronously returning immediately.
   * Once the service call finishes, the optional callback is called with the result if provided.
   *
   * @param req The service request, i.e., a filled request message of the service type.
   * @param callback The callback that is called once the service has finished.
   *   If the request failed, the callback is called with false.
   */
  Q_INVOKABLE void sendRequestAsync( const QVariantMap &req, const QJSValue &callback );

signals:

  void serviceReadyChanged();

protected:
  void onRos2Initialized() override;

  void onRos2Shutdown() override;

private slots:

  void checkServiceReady();

  void invokeCallback( QJSValue value, QVariant result );

private:
  ros_babel_fish::BabelFish babel_fish_;
  QString name_;
  QString service_type_;
  ros_babel_fish::BabelFishServiceClient::SharedPtr client_;
  QTimer connect_timer_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_SERVICE_CLIENT_HPP
