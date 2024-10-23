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
#include <rcl_action/graph.h>
#include <thread>

namespace qml_ros2_plugin
{

Ros2Qml &Ros2Qml::getInstance()
{
  static Ros2Qml instance;
  return instance;
}

Ros2Qml::Ros2Qml() : count_wrappers( 0 ) { babel_fish_ = BabelFishDispenser::getBabelFish(); }

Ros2Qml::~Ros2Qml()
{
  if ( node_ == nullptr )
    return;
  QML_ROS2_PLUGIN_DEBUG( "Ros2Qml destructing but context still alive. Shutting down context." );
  node_ = nullptr;
  rclcpp::shutdown( context_, "QML Ros2 was destroyed." );
  if ( executor_thread_.joinable() )
    executor_thread_.join();
  context_ = nullptr;
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

  executor_thread_ = std::thread( [exec = std::move( executor )]() { exec->spin(); } );
  QML_ROS2_PLUGIN_DEBUG( "QML Ros2 initialized." );
}

bool Ros2Qml::ok() const { return rclcpp::ok( context_ ); }

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
    QML_ROS2_PLUGIN_DEBUG( "Tried to query topics before node was initialized!" );
    return {};
  }
  auto topics_and_types = node_->get_topic_names_and_types();
  QStringList result;
  std::string std_datatype = toFullyQualifiedDatatype( datatype );
  for ( const auto &[topic, types] : topics_and_types ) {
    if ( !std_datatype.empty() &&
         std::find( types.begin(), types.end(), std_datatype ) == types.end() )
      continue;
    result.append( QString::fromStdString( topic ) );
  }
  return result;
}

QList<TopicInfo> Ros2Qml::queryTopicInfo() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to query topic info before node was initialized!" );
    return {};
  }
  auto topics_and_types = node_->get_topic_names_and_types();
  QList<TopicInfo> result;
  for ( const auto &[topic, types] : topics_and_types ) {
    QStringList result_types;
    result_types.reserve( static_cast<int>( types.size() ) );
    std::transform( types.begin(), types.end(), std::back_inserter( result_types ),
                    QString::fromStdString );
    result.append( { QString::fromStdString( topic ), result_types } );
  }
  return result;
}

QStringList Ros2Qml::queryTopicTypes( const QString &name ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to query topic types before node was initialized!" );
    return {};
  }
  if ( name.isEmpty() )
    return {};
  auto topics_and_types = node_->get_topic_names_and_types();
  std::string std_name = name.toStdString();
  for ( const auto &[topic, types] : topics_and_types ) {
    if ( std_name != topic )
      continue;
    QStringList result;
    result.reserve( static_cast<int>( types.size() ) );
    std::transform( types.begin(), types.end(), std::back_inserter( result ), QString::fromStdString );
    return result;
  }
  return {};
}

QMap<QString, QStringList> Ros2Qml::getTopicNamesAndTypes() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get topic names and types before node was initialized!" );
    return {};
  }
  auto topics_and_types = node_->get_topic_names_and_types();
  QMap<QString, QStringList> result;
  for ( const auto &[topic, types] : topics_and_types ) {
    QStringList result_types;
    result_types.reserve( static_cast<int>( types.size() ) );
    std::transform( types.begin(), types.end(), std::back_inserter( result_types ),
                    QString::fromStdString );
    result.insert( QString::fromStdString( topic ), result_types );
  }
  return result;
}

QMap<QString, QStringList> Ros2Qml::getServiceNamesAndTypes() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get service names and types before node was initialized!" );
    return {};
  }
  auto service_names_and_types = node_->get_service_names_and_types();
  QMap<QString, QStringList> result;
  for ( const auto &[service_name, types] : service_names_and_types ) {
    QStringList result_types;
    result_types.reserve( static_cast<int>( types.size() ) );
    std::transform( types.begin(), types.end(), std::back_inserter( result_types ),
                    QString::fromStdString );
    result.insert( QString::fromStdString( service_name ), result_types );
  }
  return result;
}

QMap<QString, QStringList> Ros2Qml::getActionNamesAndTypes() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get action names and types before node was initialized!" );
    return {};
  }
  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  const rcl_node_s *node_handle = node_->get_node_base_interface()->get_rcl_node_handle();
  rcl_allocator_t allocator = rcutils_get_default_allocator();
  if ( rcl_ret_t ret = rcl_action_get_names_and_types( node_handle, &allocator, &names_and_types );
       ret != RCL_RET_OK ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to get action names and types: %s", rcl_get_error_string().str );
    return {};
  }
  QMap<QString, QStringList> result;
  for ( size_t i = 0; i < names_and_types.names.size; ++i ) {
    const std::string name = names_and_types.names.data[i];
    const auto &types = names_and_types.types[i];
    QStringList result_types;
    result_types.reserve( static_cast<int>( types.size ) );
    for ( size_t j = 0; j < types.size; ++j ) {
      result_types.append( QString::fromStdString( types.data[j] ) );
    }
    result.insert( QString::fromStdString( name ), result_types );
  }
  if ( rcl_names_and_types_fini( &names_and_types ) != RCL_RET_OK ) {
    QML_ROS2_PLUGIN_ERROR(
        "Failed to free action names and types: '%s'. Might have lost some memory.",
        rcl_get_error_string().str );
  }
  return result;
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

QVariant Ros2Qml::createEmptyActionGoal( const QString &datatype ) const
{
  try {
    auto message = babel_fish_.create_action_goal_shared( datatype.toStdString() );
    return conversion::msgToMap( message );
  } catch ( ros_babel_fish::BabelFishException &ex ) {
    QML_ROS2_PLUGIN_WARN( "Failed to create empty action goal request for datatype '%s': %s",
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

QVariantMap Ros2QmlSingletonWrapper::getTopicNamesAndTypes() const
{
  QVariantMap result;
  QMap<QString, QStringList> topic_names_and_types = Ros2Qml::getInstance().getTopicNamesAndTypes();
  for ( auto it = topic_names_and_types.begin(); it != topic_names_and_types.end(); ++it ) {
    result.insert( it.key(), QVariant( it.value() ) );
  }
  return result;
}

QStringList Ros2QmlSingletonWrapper::getTopicTypes( const QString &name ) const
{
  return Ros2Qml::getInstance().queryTopicTypes( name );
}

QVariantMap Ros2QmlSingletonWrapper::getServiceNamesAndTypes() const
{
  QVariantMap result;
  QMap<QString, QStringList> topic_names_and_types = Ros2Qml::getInstance().getServiceNamesAndTypes();
  for ( auto it = topic_names_and_types.begin(); it != topic_names_and_types.end(); ++it ) {
    result.insert( it.key(), QVariant( it.value() ) );
  }
  return result;
}

QStringList Ros2QmlSingletonWrapper::getServiceTypes( const QString &name ) const
{
  QMap<QString, QStringList> service_names_and_types =
      Ros2Qml::getInstance().getServiceNamesAndTypes();
  return service_names_and_types.value( name );
}

QVariantMap Ros2QmlSingletonWrapper::getActionNamesAndTypes() const
{
  QVariantMap result;
  QMap<QString, QStringList> action_names_and_types = Ros2Qml::getInstance().getActionNamesAndTypes();
  for ( auto it = action_names_and_types.begin(); it != action_names_and_types.end(); ++it ) {
    result.insert( it.key(), QVariant( it.value() ) );
  }
  return result;
}

QStringList Ros2QmlSingletonWrapper::getActionTypes( const QString &name ) const
{
  QMap<QString, QStringList> action_names_and_types = Ros2Qml::getInstance().getActionNamesAndTypes();
  return action_names_and_types.value( name );
}

QVariant Ros2QmlSingletonWrapper::createEmptyMessage( const QString &datatype ) const
{
  return Ros2Qml::getInstance().createEmptyMessage( datatype );
}

QVariant Ros2QmlSingletonWrapper::createEmptyServiceRequest( const QString &datatype ) const
{
  return Ros2Qml::getInstance().createEmptyServiceRequest( datatype );
}

QVariant Ros2QmlSingletonWrapper::createEmptyActionGoal( const QString &datatype ) const
{
  return Ros2Qml::getInstance().createEmptyActionGoal( datatype );
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
  if ( !logger_.isUndefined() && !logger_.isNull() )
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
