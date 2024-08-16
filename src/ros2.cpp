// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/ros2.hpp"
#include "qml_ros2_plugin/action_client.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/publisher.hpp"
#include "qml_ros2_plugin/service_client.hpp"
#include "qml_ros2_plugin/subscription.hpp"

#include <QCoreApplication>
#include <QJSEngine>
#include <thread>

namespace qml_ros2_plugin
{

Ros2Qml &Ros2Qml::getInstance()
{
  static Ros2Qml instance;
  return instance;
}

Ros2Qml::Ros2Qml()
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  count_wrappers = 0;
}

bool Ros2Qml::isInitialized() const { return context_ != nullptr; }

void Ros2Qml::init( const QString &name, quint32 options )
{
  const QStringList &arguments = QCoreApplication::arguments();
  init( name, arguments, options );
}

void Ros2Qml::init( const QString &name, const QStringList &argv, quint32 )
{
  if ( context_ != nullptr ) {
    QML_ROS2_PLUGIN_WARN( "Was already initialized. Second call to init ignored." );
    return;
  }
  //  std::this_thread::sleep_for(std::chrono::seconds(10));
  int argc = argv.size();
  std::vector<const char *> p_args( argc );
  std::vector<std::string> args( argc );
  for ( int i = 0; i < argv.size(); ++i ) {
    args[i] = argv[i].toStdString();
    p_args[i] = args[i].c_str();
  }
  context_ = rclcpp::Context::make_shared();
  context_->init( argc, p_args.data() ); // TODO init options
  rclcpp::NodeOptions node_options;
  node_options.context( context_ );
  node_ = rclcpp::Node::make_shared( name.toStdString(),
                                     node_options ); // TODO namespace and init options
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  // StaticSingleThreadedExecutor may be a bit faster but will keep a reference to the subscription
  // and therefore not unsubscribe if the subscription is reset.
  auto executor = rclcpp::executors::SingleThreadedExecutor::make_unique( executor_options );
  executor->add_node( node_ );
  emit initialized();

  executor_thread_ = std::thread( [executor = std::move( executor )]() { executor->spin(); } );
  QML_ROS2_PLUGIN_DEBUG( "QML Ros2 initialized." );
}

bool Ros2Qml::ok() const { return rclcpp::ok(); }

namespace
{
//! Converts QString datatype to std::string and adds /msg/ if it is missing.
std::string toFullyQualifiedDatatype( const QString &datatype )
{
  std::string result = datatype.toStdString();
  std::string::size_type sep = result.find( '/' );
  if ( sep == std::string::npos )
    return result;
  if ( sep + 4 < result.size() && result[sep + 1] == 'm' && result[sep + 2] == 's' &&
       result[sep + 3] == 'g' && result[sep + 4] == '/' )
    return result;
  return result.substr( 0, sep ) + "/msg" + result.substr( sep );
}
} // namespace

QStringList Ros2Qml::queryTopics( const QString &datatype ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_ERROR( "Tried to query topics before node was initialized!" );
    return {};
  }
  std::map<std::string, std::vector<std::string>> topics_and_types =
      node_->get_topic_names_and_types();
  QStringList result;
  std::string std_datatype = toFullyQualifiedDatatype( datatype );
  for ( const auto &topic : topics_and_types ) {
    if ( !std_datatype.empty() &&
         std::find( topic.second.begin(), topic.second.end(), std_datatype ) == topic.second.end() )
      continue;
    result.append( QString::fromStdString( topic.first ) );
  }
  return result;
}

QList<TopicInfo> Ros2Qml::queryTopicInfo() const
{
  std::map<std::string, std::vector<std::string>> topics_and_types =
      node_->get_topic_names_and_types();
  QList<TopicInfo> result;
  for ( const auto &topic : topics_and_types ) {
    QStringList types;
    types.reserve( static_cast<int>( topic.second.size() ) );
    std::transform( topic.second.begin(), topic.second.end(), std::back_inserter( types ),
                    QString::fromStdString );
    result.append( { QString::fromStdString( topic.first ), types } );
  }
  return result;
}

QStringList Ros2Qml::queryTopicTypes( const QString &name ) const
{
  if ( name.isEmpty() )
    return {};
  std::map<std::string, std::vector<std::string>> topics_and_types =
      node_->get_topic_names_and_types();
  std::string std_name = name.toStdString();
  for ( const auto &topic : topics_and_types ) {
    if ( std_name != topic.first )
      continue;
    QStringList types;
    types.reserve( static_cast<int>( topic.second.size() ) );
    std::transform( topic.second.begin(), topic.second.end(), std::back_inserter( types ),
                    QString::fromStdString );
    return types;
  }
  return {};
}

QVariant Ros2Qml::createEmptyMessage( const QString &datatype ) const
{
  try {
    auto message = babel_fish_.create_message_shared( datatype.toStdString() );
    return conversion::msgToMap( message );
  } catch ( ros_babel_fish::BabelFishException &ex ) {
    QML_ROS2_PLUGIN_WARN( "Failed to create empty message for datatype '%s': %s",
                          datatype.toStdString().c_str(), ex.what() );
  }
  return QVariant();
}

QVariant Ros2Qml::createEmptyServiceRequest( const QString &datatype ) const
{
  try {
    auto message = babel_fish_.create_service_request_shared( datatype.toStdString() );
    return conversion::msgToMap( message );
  } catch ( ros_babel_fish::BabelFishException &ex ) {
    QML_ROS2_PLUGIN_WARN( "Failed to create empty service request for datatype '%s': %s",
                          datatype.toStdString().c_str(), ex.what() );
  }
  return {};
}

void Ros2Qml::registerDependant() { ++count_wrappers; }

void Ros2Qml::unregisterDependant()
{
  int count = --count_wrappers;
  if ( count == 0 ) {
    QML_ROS2_PLUGIN_DEBUG( "No dependants left. QML Ros2 shutting down." );
    rclcpp::shutdown(
        context_, "All dependants unregistered, usually that means the application is exiting." );
    emit shutdown();
    if ( executor_thread_.joinable() )
      executor_thread_.join();
    node_.reset();
    context_.reset();
    QML_ROS2_PLUGIN_DEBUG( "QML Ros2 shut down." );
  } else if ( count < 0 ) {
    QML_ROS2_PLUGIN_WARN(
        "Stop spinning was called more often than start spinning! This is a bug!" );
    ++count_wrappers;
  }
}

std::shared_ptr<rclcpp::Node> Ros2Qml::node() { return node_; }

/***************************************************************************************************/
/************************************* Ros2QmlSingletonWrapper **************************************/
/***************************************************************************************************/

Ros2QmlSingletonWrapper::Ros2QmlSingletonWrapper()
{
  connect( &Ros2Qml::getInstance(), &Ros2Qml::initialized, this,
           &Ros2QmlSingletonWrapper::initialized );
  connect( &Ros2Qml::getInstance(), &Ros2Qml::shutdown, this, &Ros2QmlSingletonWrapper::shutdown );
  Ros2Qml::getInstance().registerDependant();
}

Ros2QmlSingletonWrapper::~Ros2QmlSingletonWrapper()
{
  Ros2Qml::getInstance().unregisterDependant();
}

bool Ros2QmlSingletonWrapper::isInitialized() const
{
  return Ros2Qml::getInstance().isInitialized();
}

void Ros2QmlSingletonWrapper::init( const QString &name, quint32 options )
{
  Ros2Qml::getInstance().init( name, options );
}

void Ros2QmlSingletonWrapper::init( const QString &name, const QStringList &args, quint32 options )
{
  Ros2Qml::getInstance().init( name, args, options );
}

bool Ros2QmlSingletonWrapper::ok() const { return Ros2Qml::getInstance().ok(); }

Time Ros2QmlSingletonWrapper::now() const
{
  if ( !Ros2Qml::getInstance().isInitialized() )
    return Time();
  return Time( Ros2Qml::getInstance().node()->now() );
}

QString Ros2QmlSingletonWrapper::getName()
{
  if ( !isInitialized() )
    return {};
  return QString::fromStdString( Ros2Qml::getInstance().node()->get_name() );
}

QString Ros2QmlSingletonWrapper::getNamespace()
{
  if ( !isInitialized() )
    return {};
  return QString::fromStdString( Ros2Qml::getInstance().node()->get_namespace() );
}

QStringList Ros2QmlSingletonWrapper::queryTopics( const QString &datatype ) const
{
  return Ros2Qml::getInstance().queryTopics( datatype );
}

QList<TopicInfo> Ros2QmlSingletonWrapper::queryTopicInfo() const
{
  return Ros2Qml::getInstance().queryTopicInfo();
}

QStringList Ros2QmlSingletonWrapper::queryTopicTypes( const QString &name ) const
{
  return Ros2Qml::getInstance().queryTopicTypes( name );
}

QVariant Ros2QmlSingletonWrapper::createEmptyMessage( const QString &datatype ) const
{
  return Ros2Qml::getInstance().createEmptyMessage( datatype );
}

QVariant Ros2QmlSingletonWrapper::createEmptyServiceRequest( const QString &datatype ) const
{
  return Ros2Qml::getInstance().createEmptyServiceRequest( datatype );
}

IO Ros2QmlSingletonWrapper::io() const { return {}; }

QObject *Ros2QmlSingletonWrapper::getLogger( const QString &name )
{
  if ( name.isEmpty() ) {
    const rclcpp::Node::SharedPtr &node = Ros2Qml::getInstance().node();
    if ( node == nullptr ) {
      QML_ROS2_PLUGIN_ERROR( "Can not get node logger before Ros2 is initialized." );
      return nullptr;
    }
    return new Logger( node->get_logger() );
  }
  return new Logger( rclcpp::get_logger( name.toStdString() ) );
}

QJSValue Ros2QmlSingletonWrapper::debug()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->debug();
}

QJSValue Ros2QmlSingletonWrapper::info()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->info();
}

QJSValue Ros2QmlSingletonWrapper::warn()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->warn();
}

QJSValue Ros2QmlSingletonWrapper::error()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->error();
}

QJSValue Ros2QmlSingletonWrapper::fatal()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->fatal();
}

QObject *Ros2QmlSingletonWrapper::createPublisher( const QString &topic, const QString &type,
                                                   quint32 queue_size )
{
  return new Publisher( topic, type, queue_size );
}

QObject *Ros2QmlSingletonWrapper::createSubscription( const QString &topic, quint32 queue_size )
{
  return new Subscription( topic, QString(), queue_size );
}

QObject *Ros2QmlSingletonWrapper::createSubscription( const QString &topic,
                                                      const QString &message_type, quint32 queue_size )
{
  return new Subscription( topic, message_type, queue_size );
}

QObject *Ros2QmlSingletonWrapper::createServiceClient( const QString &name, const QString &type )
{
  return new ServiceClient( name, type );
}

QObject *Ros2QmlSingletonWrapper::createActionClient( const QString &name, const QString &type )
{
  return new ActionClient( name, type );
}

bool Ros2QmlSingletonWrapper::initLogging()
{
  if ( logger_.isNull() )
    return true;
  const std::shared_ptr<rclcpp::Node> &node = Ros2Qml::getInstance().node();
  if ( node == nullptr ) {
    QML_ROS2_PLUGIN_ERROR( "You need to initialize Ros2 before calling a log function!" );
    return false;
  }
  logger_ = qjsEngine( this )->newQObject( new Logger( node->get_logger() ) );
  return true;
}
} // namespace qml_ros2_plugin
