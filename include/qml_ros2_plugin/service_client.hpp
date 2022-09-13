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

#ifndef QML_ROS2_PLUGIN_SERVICE_CLIENT_HPP
#define QML_ROS2_PLUGIN_SERVICE_CLIENT_HPP

#include "qml_ros2_plugin/qobject_ros2.hpp"
#include "qml_ros2_plugin/qos.hpp"

#include <QJSValue>
#include <QTimer>
#include <QVariant>

#include <ros2_babel_fish/babel_fish.hpp>

namespace qml_ros2_plugin
{

class ServiceClient : public QObjectRos2
{
  Q_OBJECT
  //! True if the ServiceClient is connected to the Service and the Service is ready, false otherwise.
  Q_PROPERTY( bool ready READ isServiceReady NOTIFY serviceReadyChanged )
  //! The quality of service settings for this service client's connection.
  //! See QoS and the ROS2 Docs for more info:
  //! https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
  Q_PROPERTY( qml_ros2_plugin::QoS qos READ qos )

public:
  /*!
   * @param name The service topic.
   * @param type The type of the service, e.g., "example_interfaces/srv/AddTwoInts"
   * @param qos The QoS settings to use for this client's connection.
   */
  ServiceClient( QString name, QString type, QoS qos );

  //! Returns whether the service is ready.
  bool isServiceReady() const;

  /*!
   * Calls a service asynchronously returning immediately.
   * Once the service call finishes, the optional callback is called with the result if provided.
   *
   * @param req The service request, i.e., a filled request message of the service type.
   * @param callback The callback that is called once the service has finished.
   *   If the request failed, the callback is called with false.
   */
  Q_INVOKABLE void sendRequestAsync( const QVariantMap &req, const QJSValue &callback );

  qml_ros2_plugin::QoS qos();

signals:

  void serviceReadyChanged();

protected:
  void onRos2Initialized() override;

  void onRos2Shutdown() override;

private slots:

  void checkServiceReady();

  void invokeCallback( QJSValue value, QVariant result );

private:
  ros2_babel_fish::BabelFish babel_fish_;
  QString name_;
  QString service_type_;
  ros2_babel_fish::BabelFishServiceClient::SharedPtr client_;
  QTimer connect_timer_;
  QoS qos_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_SERVICE_CLIENT_HPP
