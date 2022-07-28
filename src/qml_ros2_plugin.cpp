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

#include "qml_ros2_plugin/action_client.hpp"
#include "qml_ros2_plugin/ament_index.hpp"
#include "qml_ros2_plugin/array.hpp"
#include "qml_ros2_plugin/goal_handle.hpp"
#include "qml_ros2_plugin/image_transport_subscription.hpp"
#include "qml_ros2_plugin/logger.hpp"
#include "qml_ros2_plugin/publisher.hpp"
#include "qml_ros2_plugin/ros2.hpp"
#include "qml_ros2_plugin/service_client.hpp"
#include "qml_ros2_plugin/subscription.hpp"
#include "qml_ros2_plugin/tf_transform.hpp"
#include "qml_ros2_plugin/tf_transform_listener.hpp"
#include "qml_ros2_plugin/time.hpp"

#include <QQmlExtensionPlugin>
#include <QtQml>

namespace qml_ros2_plugin
{

class QmlRos2Plugin : public QQmlExtensionPlugin
{
  Q_OBJECT
  Q_PLUGIN_METADATA( IID QQmlExtensionInterface_iid )
public:
  void registerTypes( const char *uri ) override
  {
    Q_ASSERT( uri == QLatin1String( "Ros2" ) );

    qmlRegisterType<Array>();
    QMetaType::registerConverter<Array, QVariantList>( &Array::toVariantList );
    qmlRegisterType<Logger>();
    qmlRegisterType<IO>();
    qmlRegisterUncreatableMetaObject( ros2_logger_levels::staticMetaObject, "Ros2", 1, 0,
                                      "Ros2LoggerLevel", "Error: Can not create enum object." );
    qmlRegisterSingletonType<Ros2QmlSingletonWrapper>(
        "Ros2", 1, 0, "Ros2", []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject * {
          Q_UNUSED( engine );
          Q_UNUSED( scriptEngine );
          return new Ros2QmlSingletonWrapper;
        } );
    qmlRegisterSingletonType<AmentIndex>(
        "Ros2", 1, 0, "AmentIndex", []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject * {
          Q_UNUSED( engine );
          Q_UNUSED( scriptEngine );
          return new AmentIndex;
        } );
    qmlRegisterUncreatableType<TopicInfo>(
        "Ros2", 1, 0, "TopicInfo",
        "Error: No point in creating TopicInfo in QML and it's not supported." );
    qRegisterMetaType<QoS>();
    qmlRegisterUncreatableType<Publisher>(
        "Ros2", 1, 0,
        "Publisher", "Error: Can not create Publisher manually in QML. Use one of the createPublisher functions." );

    qmlRegisterType<Subscription>( "Ros2", 1, 0, "Subscription" );
    qmlRegisterSingletonType<TfTransformListenerWrapper>(
        "Ros2", 1, 0, "TfTransformListener",
        []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject * {
          Q_UNUSED( engine );
          Q_UNUSED( scriptEngine );
          return new TfTransformListenerWrapper;
        } );
    qmlRegisterType<TfTransform>( "Ros2", 1, 0, "TfTransform" );

    // Image transport
    qmlRegisterType<ImageTransportSubscription>( "Ros2", 1, 0, "ImageTransportSubscription" );

    // Action Client
    qmlRegisterUncreatableMetaObject( action_goal_status::staticMetaObject, "Ros2", 1, 0,
                                      "ActionGoalStatus", "Error: Can not create enum object." );
    qmlRegisterUncreatableType<GoalHandle>(
        "Ros2", 1, 0, "GoalHandle",
        "Error: Can not create GoalHandle manually. A GoalHandle is obtained as a return value of "
        "ActionClient.sendGoalAsync or as callback parameter." );
    qmlRegisterUncreatableType<ActionClient>(
        "Ros2", 1, 0, "ActionClient",
        "Error: Can not create ActionClient manually in QML. Use the Ros2.createActionClient(name, "
        "type) factory method." );
    // Service
    qmlRegisterUncreatableType<ServiceClient>(
        "Ros2", 1, 0, "ServiceClient",
        "Error: Can not create ServiceClient manually in QML. Use the "
        "Ros2.createServiceClient(name, type) factory method." );

    // Time
    qRegisterMetaType<Time>();
    qRegisterMetaType<Duration>();
    qmlRegisterUncreatableMetaObject( ros_clock_types::staticMetaObject, "Ros2", 1, 0,
                                      "Ros2ClockTypes", "Error: Can not create enum object." );
  }
};
} // namespace qml_ros2_plugin

#include "qml_ros2_plugin.moc"
