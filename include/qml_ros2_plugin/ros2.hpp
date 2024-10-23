// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_ROS2_HPP
#define QML_ROS2_PLUGIN_ROS2_HPP

#include "qml_ros2_plugin/io.hpp"
#include "qml_ros2_plugin/logger.hpp"
#include "qml_ros2_plugin/time.hpp"
#include "qml_ros2_plugin/topic_info.hpp"

#include <QJSValue>
#include <QObject>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <ros_babel_fish/babel_fish.hpp>

//! @brief Project namespace.
namespace qml_ros2_plugin
{

class Ros2Qml final : public QObject
{
  Q_OBJECT
private:
  Ros2Qml();

public:
  static Ros2Qml &getInstance();

  Ros2Qml( const Ros2Qml & ) = delete;

  ~Ros2Qml() final;

  void operator=( const Ros2Qml & ) = delete;

  /*!
   * Checks whether ROS is initialized.
   * @return True if ROS is initialized, false otherwise.
   */
  bool isInitialized() const;

  /*!
   * Initializes the ros node with the given name and the command line arguments passed from the command line.
   * @param name The name of the ROS node.
   * @param options The options passed to ROS, see ros_init_options::Ros2InitOption.
   */
  void init( const QString &name, quint32 options = 0 );

  /*!
   * Initializes the ros node with the given args.
   * @param name The name of the ROS node.
   * @param args The args that are passed to ROS. Normally, these would be the command line arguments see init(const QString &, quint32)
   * @param options The options passed to ROS, see ros_init_options::Ros2InitOption.
   */
  void init( const QString &name, const QStringList &args, quint32 options = 0 );

  /*!
   * Can be used to query the state of ROS.
   * @return False if it's time to exit, true if still ok.
   */
  bool ok() const;

  /*!
   * Queries the internal node for its topics or using the optional datatype parameter for all topics with the given type.
   * @param datatype The message type to filter topics for, e.g., sensor_msgs/Image. Omit to query for all topics.
   * @return A list of topics that matches the given datatype or all topics if no datatype provided.
   */
  QStringList queryTopics( const QString &datatype = QString() ) const;

  /*!
   * Queries the internal node for its topics and their type.
   * @return A list of TopicInfo.
   */
  QList<TopicInfo> queryTopicInfo() const;

  /*!
   * Queries the internal node for a topic with the given name.
   * @param name The name of the topic, e.g., /front_camera/image_raw.
   * @return The types available on the topic if found, otherwise an empty string.
   */
  QStringList queryTopicTypes( const QString &name ) const;

  /*!
   * Queries the internal node for its known topics and their types.
   * See rclcpp::Node::get_topic_names_and_types() for more information.
   * @return A map with the topic names as keys and the types as values.
   */
  QMap<QString, QStringList> getTopicNamesAndTypes() const;

  /*!
   * Queries the internal node for its known services and their types.
   * See rclcpp::Node::get_service_names_and_types() for more information.
   * @return A map with the service names as keys and the types as values.
   */
  QMap<QString, QStringList> getServiceNamesAndTypes() const;

  /*!
   * Queries the internal node for its known actions and their types.
   * @return A map with the action names as keys and the types as values.
   */
  QMap<QString, QStringList> getActionNamesAndTypes() const;

  /*!
   * Creates an empty message for the given message type, e.g., "geometry_msgs/Point".
   * If the message type is known, an empty message with all members set to their default is returned.
   * If the message type is not found on the current machine, a warning message is printed and null is returned.
   * @param datatype The message datatype.
   * @return A message with all members set to their default.
   */
  QVariant createEmptyMessage( const QString &datatype ) const;

  /*!
   * Creates an empty service request for the given service type, e.g., "std_srvs/SetBool".
   * If the service type is known, an empty request is returned with all members of the request message set to their
   * default values.
   * If the service type is not found on the current machine, a warning message is printed and null is returned.
   * @param datatype The service datatype. NOT the request datatype.
   * @return A request message with all members set to their default.
   */
  QVariant createEmptyServiceRequest( const QString &datatype ) const;

  /*!
   * Creates an empty action goal request for the given action type, e.g., "example_interfaces/action/Fibonacci".
   * If the action type is known, an empty goal request is returned with all members of the goal message set to their
   * default values.
   * If the action type is not found on the current machine, a warning message is printed and null is returned.
   * @param datatype The action datatype. NOT the goal datatype.
   * @return A goal request message with all members set to their default.
   */
  QVariant createEmptyActionGoal( const QString &datatype ) const;

  //! Increases the dependant counter.
  void registerDependant();

  //! Decreases the dependant counter and if it gets to 0, frees all memory.
  void unregisterDependant();

  std::shared_ptr<rclcpp::Node> node();

signals:

  //! Emitted once when ROS was initialized.
  void initialized();

  //! Emitted when this ROS node was shut down and it is time to exit.
  void shutdown();

private:
  std::thread executor_thread_;
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> node_;
  ros_babel_fish::BabelFish babel_fish_;
  std::atomic<int> count_wrappers;
};

class Ros2QmlSingletonWrapper : public QObject
{
  Q_OBJECT
  Q_PROPERTY( qml_ros2_plugin::IO io READ io CONSTANT )
  Q_PROPERTY( QJSValue debug READ debug CONSTANT )
  Q_PROPERTY( QJSValue info READ info CONSTANT )
  Q_PROPERTY( QJSValue warn READ warn CONSTANT )
  Q_PROPERTY( QJSValue error READ error CONSTANT )
  Q_PROPERTY( QJSValue fatal READ fatal CONSTANT )
public:
  Ros2QmlSingletonWrapper();

  ~Ros2QmlSingletonWrapper() override;

  //! @copydoc Ros2Qml::isRosInitialized
  Q_INVOKABLE bool isInitialized() const;

  //! @copydoc Ros2Qml::init(const QString &, quint32)
  Q_INVOKABLE void init( const QString &name, quint32 options = 0 );

  //! @copydoc Ros2Qml::init(const QStringList &, const QString &, quint32)
  Q_INVOKABLE void init( const QString &name, const QStringList &args, quint32 options );

  //! @copydoc Ros2Qml::ok
  Q_INVOKABLE bool ok() const;

  //! @return The current time as given by the node's clock (if initialized, otherwise rclcpp::Time())
  Q_INVOKABLE qml_ros2_plugin::Time now() const;

  //! Returns the name of the node. Returns empty string before ROS node was initialized.
  Q_INVOKABLE QString getName();

  //! Returns the namespace of the node. Returns empty string before ROS node was initialized.
  Q_INVOKABLE QString getNamespace();

  //! @copydoc Ros2Qml::queryTopics
  Q_INVOKABLE QStringList queryTopics( const QString &datatype = QString() ) const;

  //! @copydoc Ros2Qml::queryTopicInfo
  Q_INVOKABLE QList<qml_ros2_plugin::TopicInfo> queryTopicInfo() const;

  //! @copydoc Ros2Qml::queryTopicTypes
  Q_INVOKABLE QStringList queryTopicTypes( const QString &name ) const;

  //! @copydoc Ros2Qml::getTopicNamesAndTypes
  Q_INVOKABLE QVariantMap getTopicNamesAndTypes() const;

  Q_INVOKABLE QStringList getTopicTypes( const QString &name ) const;

  //! @copydoc Ros2Qml::getServiceNamesAndTypes
  Q_INVOKABLE QVariantMap getServiceNamesAndTypes() const;

  Q_INVOKABLE QStringList getServiceTypes( const QString &name ) const;

  //! @copydoc Ros2Qml::getActionNamesAndTypes
  Q_INVOKABLE QVariantMap getActionNamesAndTypes() const;

  Q_INVOKABLE QStringList getActionTypes( const QString &name ) const;

  //! @copydoc Ros2Qml::createEmptyMessage
  Q_INVOKABLE QVariant createEmptyMessage( const QString &datatype ) const;

  //! @copydoc Ros2Qml::createEmptyServiceRequest
  Q_INVOKABLE QVariant createEmptyServiceRequest( const QString &datatype ) const;

  //! @copydoc Ros2Qml::createEmptyActionGoal
  Q_INVOKABLE QVariant createEmptyActionGoal( const QString &datatype ) const;

  IO io() const;

  /*!
   * Get the logger with the given name or if no name is passed, the node's logger.
   * @param name (Optional) Name of the logger.
   * @return An instance of Logger wrapping the requested rclcpp::Logger.
   */
  Q_INVOKABLE QObject *getLogger( const QString &name = QString() );

  //! Logs debug with the node's logger.
  //! @copydoc Logger::debug
  QJSValue debug();

  //! Logs info with the node's logger.
  //! @copydoc Logger::info
  QJSValue info();

  //! Logs warn with the node's logger.
  //! @copydoc Logger::warn
  QJSValue warn();

  //! Logs error with the node's logger.
  //! @copydoc Logger::error
  QJSValue error();

  //! Logs fatal with the node's logger.
  //! @copydoc Logger::fatal
  QJSValue fatal();

  /*!
   * Creates a Publisher to publish ROS messages.
   *
   * @param type The type of the messages published using this publisher.
   * @param topic The topic on which the messages are published.
   * @param queue_size The maximum number of outgoing messages to be queued for delivery to subscribers.
   * @return A Publisher instance.
   */
  Q_INVOKABLE QObject *createPublisher( const QString &topic, const QString &type,
                                        quint32 queue_size = 1 );

  /*!
   * Creates a Subscriber to createSubscription to ROS messages.
   * Convenience function to create a subscriber in a single line.
   *
   * @param topic The topic to createSubscription to.
   * @param queue_size The maximum number of incoming messages to be queued for processing.
   * @return A Subscriber instance.
   */
  Q_INVOKABLE QObject *createSubscription( const QString &topic, quint32 queue_size = 1 );

  /*!
   * Creates a Subscriber to createSubscription to ROS messages.
   * Convenience function to create a subscriber in a single line.
   *
   * @param topic The topic to createSubscription to.
   * @param message_type The type of the messages to subscribe to on the topic.
   * @param queue_size The maximum number of incoming messages to be queued for processing.
   * @return A Subscriber instance.
   */
  Q_INVOKABLE QObject *createSubscription( const QString &topic, const QString &message_type,
                                           quint32 queue_size = 1 );

  /*!
   * Creates a ServiceClient for the given type and the given name.
   * @param name The name of the service to connect to.
   * @param type The type of the service. Example: example_interfaces/srv/AddTwoInts
   * @return An instance of ServiceClient.
   */
  Q_INVOKABLE QObject *createServiceClient( const QString &name, const QString &type );

  /*!
   * Creates an ActionClient for the given type and the given name.
   * @param name The name of the action server to connect to.
   * @param type The type of the action. Example: action_tutorials_interfaces/action/Fibonacci
   * @return An instance of ActionClient.
   */
  Q_INVOKABLE QObject *createActionClient( const QString &name, const QString &type );

signals:

  //! @copydoc Ros2Qml::initialized
  void initialized();

  //! @copydoc Ros2Qml::shutdown
  void shutdown();

private:
  bool initLogging();

  QJSValue logger_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_ROS2_HPP
