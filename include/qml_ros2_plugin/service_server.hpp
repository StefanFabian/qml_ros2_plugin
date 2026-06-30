// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_SERVICE_SERVER_HPP
#define QML_ROS2_PLUGIN_SERVICE_SERVER_HPP

#include "qml_ros2_plugin/qobject_ros2.hpp"
#include "qml_ros2_plugin/qos.hpp"

#include <QJSValue>
#include <QPointer>
#include <QVariant>
#include <mutex>
#include <ros_babel_fish/babel_fish.hpp>
#include <unordered_map>

namespace qml_ros2_plugin
{

/*!
 * A directly-creatable QML element that advertises a ROS service and answers requests using a
 * QML/JavaScript callback.
 *
 * The callback can either return the response synchronously or defer the response and send it
 * later using sendResponse with the request id passed to the callback.
 *
 * Example:
 * @code
 * ServiceServer {
 *   name: "/add_two_ints"
 *   type: "example_interfaces/srv/AddTwoInts"
 *   processRequest: function (request, id) {
 *     return { sum: request.a + request.b } // synchronous response
 *     // ...or return undefined and later call sendResponse(id, { sum: ... })
 *   }
 * }
 * @endcode
 */
class ServiceServer : public QObjectRos2
{
  Q_OBJECT
  //! The service name (topic in ROS 1). Setting this (re-)creates the service server.
  Q_PROPERTY( QString name READ name WRITE setName NOTIFY nameChanged )
  //! The type of the service, e.g., "example_interfaces/srv/AddTwoInts". Setting this (re-)creates
  //! the service server.
  Q_PROPERTY( QString type READ type WRITE setType NOTIFY typeChanged )
  //! The QoS settings for this service server. Use ``Ros2.ServicesQoS()`` to create QoS settings.
  Q_PROPERTY( qml_ros2_plugin::QoSWrapper qos READ qos WRITE setQoS NOTIFY qosChanged )
  //! The callback that is invoked when a request is received. Signature: ``function (request,
  //! id)``. Return an object to answer the request synchronously, or return ``undefined`` and call
  //! ``sendResponse(id, response)`` later to answer asynchronously.
  Q_PROPERTY( QJSValue processRequest READ processRequest WRITE setProcessRequest NOTIFY
                  processRequestChanged )
  //! Whether the service server is currently advertised.
  Q_PROPERTY( bool advertised READ isAdvertised NOTIFY advertisedChanged )

public:
  ServiceServer();

  ~ServiceServer() override;

  const QString &name() const;

  void setName( const QString &value );

  const QString &type() const;

  void setType( const QString &value );

  const QoSWrapper &qos() const;

  void setQoS( const QoSWrapper &value );

  QJSValue processRequest() const;

  void setProcessRequest( const QJSValue &value );

  bool isAdvertised() const;

  /*!
   * Sends a response for a previously received (deferred) request.
   * Use this if the processRequest callback returned ``undefined`` to answer the request later.
   *
   * Every deferred request must eventually be answered with this method; an id that is never
   * answered is retained until the server is destroyed (and the client keeps waiting).
   *
   * @param id The id of the request as passed to the processRequest callback.
   * @param response The response message as a map of the service response type.
   */
  Q_INVOKABLE void sendResponse( int id, QVariantMap response );

signals:

  void nameChanged();

  void typeChanged();

  void qosChanged();

  void processRequestChanged();

  void advertisedChanged();

protected:
  void onRos2Initialized() override;

  void onRos2Shutdown() override;

private slots:

  void handleRequest( int id, QVariant request );

private:
  //! (Re-)creates the service server if name, type and ROS are available. Drops any prior server.
  void tryCreate();

  //! Executor-thread callback that stores the request and marshals it to the GUI thread.
  void onRequestReceived( const std::shared_ptr<rmw_request_id_t> &request_header,
                          const std::shared_ptr<ros_babel_fish::CompoundMessage> &request );

  ros_babel_fish::BabelFish babel_fish_;
  QoSWrapper qos_{ rclcpp::ServicesQoS() };
  QString name_;
  QString type_;
  QJSValue process_request_;
  ros_babel_fish::ServiceTypeSupport::ConstSharedPtr service_type_support_;
  ros_babel_fish::BabelFishService::SharedPtr service_;
  // Maps a request id to its rmw request header (a trivially copyable POD). Guarded by mutex_.
  std::unordered_map<int, rmw_request_id_t> pending_requests_;
  std::mutex mutex_;
  int next_request_id_ = 0;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_SERVICE_SERVER_HPP
