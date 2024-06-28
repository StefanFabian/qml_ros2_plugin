// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "common.hpp"
#include "message_comparison.hpp"

#include <qml_ros2_plugin/action_client.hpp>
#include <qml_ros2_plugin/goal_handle.hpp>
#include <qml_ros2_plugin/publisher.hpp>
#include <qml_ros2_plugin/ros2.hpp>
#include <qml_ros2_plugin/service_client.hpp>
#include <qml_ros2_plugin/subscription.hpp>
#include <qml_ros2_plugin/tf_transform.hpp>
#include <qml_ros2_plugin/tf_transform_listener.hpp>
#include <qml_ros2_plugin/time.hpp>

#include <example_interfaces/srv/add_two_ints.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros_babel_fish_test_msgs/action/simple_test.hpp>
#include <std_srvs/srv/empty.hpp>

#include <QCoreApplication>
#include <QJSEngine>
#include <QSignalSpy>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace qml_ros2_plugin;
using namespace std::chrono_literals;

template<typename T>
struct MessageStorage {
  std::vector<T> messages;

  void callback( T msg ) { messages.push_back( msg ); }
};

rclcpp::Node::SharedPtr node;

void processEvents()
{
  QCoreApplication::processEvents();
  rclcpp::spin_some( node );
}

//! @param wait_count Max time to wait in increments of 33 ms
bool waitFor( const std::function<bool()> &pred, int wait_count = 10 )
{
  while ( --wait_count > 0 ) {
    if ( pred() )
      return true;
    processEvents();
    std::this_thread::sleep_for( 33ms );
  }
  return false;
}

TEST( Communication, publisher )
{
  Ros2QmlSingletonWrapper wrapper;
  auto pub_singleton_private = dynamic_cast<qml_ros2_plugin::Publisher *>(
      wrapper.createPublisher( "~/private_ns/test", "geometry_msgs/msg/Pose", 10 ) );
  ASSERT_NE( pub_singleton_private, nullptr );
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_private->topic().toStdString(), "/communication_qml/private_ns/test" );
  EXPECT_EQ( pub_singleton_private->queueSize(), 10U );
  EXPECT_EQ( pub_singleton_private->type().toStdString(), "geometry_msgs/msg/Pose" );
  //  EXPECT_EQ( pub_singleton_private_ns->isLatched(), false ); // TODO
  EXPECT_EQ( pub_singleton_private->getSubscriptionCount(), 0U );
  MessageStorage<geometry_msgs::msg::Pose> pub_singleton_private_storage;
  auto subscription_private_ns = node->create_subscription<geometry_msgs::msg::Pose>(
      "/communication_qml/private_ns/test", 10,
      [&pub_singleton_private_storage]( geometry_msgs::msg::Pose::UniquePtr msg ) {
        pub_singleton_private_storage.callback( *msg );
      } );
  processEvents();
  ASSERT_TRUE( waitFor( [&]() { return pub_singleton_private->getSubscriptionCount() == 1U; } ) );
  pub_singleton_private->publish( { { "position", QVariantMap{ { "x", 1.2 } } } } );
  if ( !waitFor( [&]() { return !pub_singleton_private_storage.messages.empty(); } ) )
    FAIL() << "Timeout while waiting for message!";
  ASSERT_EQ( pub_singleton_private_storage.messages.size(), 1UL );
  EXPECT_EQ( pub_singleton_private_storage.messages[0].position.x, 1.2 );
  delete pub_singleton_private;

  auto pub_singleton_glob_explicit = dynamic_cast<qml_ros2_plugin::Publisher *>(
      wrapper.createPublisher( "/pose", "geometry_msgs/msg/Pose", 10 ) );
  ASSERT_NE( pub_singleton_glob_explicit, nullptr );
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_glob_explicit->topic(), QString( "/pose" ) )
      << pub_singleton_glob_explicit->topic().toStdString();
  MessageStorage<geometry_msgs::msg::Pose> pub_singleton_glob_explicit_storage;
  auto subscription_glob_explicit = node->create_subscription<geometry_msgs::msg::Pose>(
      "/pose", 10, [&pub_singleton_glob_explicit_storage]( geometry_msgs::msg::Pose::UniquePtr msg ) {
        pub_singleton_glob_explicit_storage.callback( *msg );
      } );
  waitFor( []() { return false; }, 16 ); // Wait for half a second
  EXPECT_TRUE( pub_singleton_glob_explicit_storage.messages.empty() );
  pub_singleton_glob_explicit->publish( { { "position", QVariantMap{ { "y", 1.3 } } } } );
  if ( !waitFor( [&]() { return !pub_singleton_glob_explicit_storage.messages.empty(); } ) )
    FAIL() << "Timeout while waiting for message!";
  ASSERT_EQ( pub_singleton_glob_explicit_storage.messages.size(), 1UL );
  EXPECT_EQ( pub_singleton_glob_explicit_storage.messages[0].position.y, 1.3 );
  delete pub_singleton_glob_explicit;

  auto pub_singleton_glob = dynamic_cast<qml_ros2_plugin::Publisher *>(
      wrapper.createPublisher( "other_pose", "geometry_msgs/Pose", 10 ) );
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_glob->topic().toStdString(), "/other_pose" );
  MessageStorage<geometry_msgs::msg::Pose> pub_singleton_glob_storage;
  auto subscription_ns_glob = node->create_subscription<geometry_msgs::msg::Pose>(
      "/other_pose", 10, [&pub_singleton_glob_storage]( geometry_msgs::msg::Pose::UniquePtr msg ) {
        pub_singleton_glob_storage.callback( *msg );
      } );
  waitFor( []() { return false; }, 16 ); // Wait for half a second
  EXPECT_TRUE( pub_singleton_glob_storage.messages.empty() );
  pub_singleton_glob->publish( { { "position", QVariantMap{ { "y", 1.3 } } } } );
  if ( !waitFor( [&]() { return !pub_singleton_glob_storage.messages.empty(); } ) )
    FAIL() << "Timeout while waiting for message!";
  ASSERT_EQ( pub_singleton_glob_storage.messages.size(), 1UL );
  EXPECT_EQ( pub_singleton_glob_storage.messages[0].position.y, 1.3 );
  delete pub_singleton_glob;
}

TEST( Communication, subscriber )
{
  Ros2QmlSingletonWrapper wrapper;
  auto pub_pns = node->create_publisher<geometry_msgs::msg::Pose>( "~/test", 10 );
  ASSERT_EQ( pub_pns->get_topic_name(), std::string( "/communication/test" ) );
  auto subscriber_pns = dynamic_cast<qml_ros2_plugin::Subscription *>(
      wrapper.createSubscription( "/communication/test", 1 ) );
  processEvents();
  EXPECT_TRUE( subscriber_pns->isRosInitialized() );
  EXPECT_TRUE( subscriber_pns->enabled() );
  EXPECT_TRUE(
      waitFor( [&subscriber_pns]() { return subscriber_pns->getPublisherCount() == 1U; }, 10 ) );
  ASSERT_EQ( subscriber_pns->topic().toStdString(), "/communication/test" );
  //  EXPECT_EQ( subscriber_pns->ns(), QString( "/communication/private_ns" )) << subscriber_pns->ns().toStdString();
  EXPECT_EQ( subscriber_pns->queueSize(), 1U );
  if ( !waitFor( [&]() { return pub_pns->get_subscription_count() > 0; } ) )
    FAIL() << "Timout while waiting for subscriber num increasing.";
  geometry_msgs::msg::Pose pose;
  pose.position.x = 2.34;
  pub_pns->publish( pose );
  if ( !waitFor( [&]() { return subscriber_pns->message().isValid(); } ) )
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x,
                    subscriber_pns->message().toMap()["position"].toMap()["x"].toDouble() );
  delete subscriber_pns;

  auto pub_pns_glob = node->create_publisher<geometry_msgs::msg::Pose>( "/pose", 10 );
  ASSERT_EQ( pub_pns_glob->get_topic_name(), std::string( "/pose" ) );
  qml_ros2_plugin::Subscription subscriber_pns_glob;
  //  subscriber_pns_glob.setNs( "~private_ns" );
  subscriber_pns_glob.setTopic( "/pose" );
  subscriber_pns_glob.setQueueSize( 5 );
  QCoreApplication::processEvents();
  EXPECT_TRUE( subscriber_pns_glob.isRosInitialized() );
  EXPECT_EQ( subscriber_pns_glob.queueSize(), 5U );
  EXPECT_EQ( subscriber_pns_glob.topic(), QString( "/pose" ) )
      << subscriber_pns_glob.topic().toStdString();
  if ( !waitFor( [&]() { return pub_pns_glob->get_subscription_count() > 0; } ) )
    FAIL() << "Timout while waiting for subscriber num increasing.";
  pose.position.y = 3.44;
  pub_pns_glob->publish( pose );
  if ( !waitFor( [&]() { return subscriber_pns_glob.message().isValid(); } ) )
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x,
                    subscriber_pns_glob.message().toMap()["position"].toMap()["x"].toDouble() );
  EXPECT_DOUBLE_EQ( pose.position.y,
                    subscriber_pns_glob.message().toMap()["position"].toMap()["y"].toDouble() );

  auto pub_ns = node->create_publisher<geometry_msgs::msg::Pose>( "/other_pose", rclcpp::QoS( 1 ) );
  ASSERT_EQ( pub_ns->get_topic_name(), std::string( "/other_pose" ) );
  auto subscriber_ns =
      dynamic_cast<qml_ros2_plugin::Subscription *>( wrapper.createSubscription( "/other_pose", 1 ) );
  QCoreApplication::processEvents();
  EXPECT_TRUE( subscriber_ns->isRosInitialized() );
  EXPECT_EQ( subscriber_ns->queueSize(), 1U );
  EXPECT_EQ( subscriber_ns->topic(), QString( "/other_pose" ) )
      << subscriber_ns->topic().toStdString();
  if ( !waitFor( [&]() { return pub_ns->get_subscription_count() > 0; } ) )
    FAIL() << "Timout while waiting for subscriber num increasing.";
  pose.position.z = 5.16;
  pub_ns->publish( pose );
  if ( !waitFor( [&]() { return subscriber_ns->message().isValid(); } ) )
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x,
                    subscriber_ns->message().toMap()["position"].toMap()["x"].toDouble() );
  EXPECT_DOUBLE_EQ( pose.position.y,
                    subscriber_ns->message().toMap()["position"].toMap()["y"].toDouble() );
  EXPECT_DOUBLE_EQ( pose.position.z,
                    subscriber_ns->message().toMap()["position"].toMap()["z"].toDouble() );
  EXPECT_EQ( subscriber_ns->messageType().toStdString(), "geometry_msgs/msg/Pose" );
  subscriber_ns->setEnabled( false );
  EXPECT_FALSE( subscriber_ns->enabled() );
  pose.position.z = 1.0;
  pub_ns->publish( pose );
  if ( waitFor( [&]() {
         return std::abs( subscriber_ns->message().toMap()["position"].toMap()["z"].toDouble() -
                          1.0 ) < 1E-4;
       } ) )
    FAIL() << "Shouldn't have received the message that was published while the subscriber wasn't "
              "running.";
  subscriber_ns->setEnabled( true );
  EXPECT_TRUE( subscriber_ns->enabled() );
  ASSERT_FALSE( std::abs( subscriber_ns->message().toMap()["position"].toMap()["z"].toDouble() -
                          1.0 ) < 1E-4 );
  delete subscriber_ns;
}

TEST( Communication, queryTopics )
{
  auto pub1 = node->create_publisher<geometry_msgs::msg::Pose>( "/query_topics/pose1", 10 );
  auto pub2 = node->create_publisher<geometry_msgs::msg::Vector3>( "/query_topics/vector3", 10 );
  auto pub3 = node->create_publisher<geometry_msgs::msg::Point>( "/query_topics/point1", 10 );
  auto pub4 = node->create_publisher<geometry_msgs::msg::Point>( "/query_topics/point2", 10 );
  auto pub5 = node->create_publisher<geometry_msgs::msg::Pose>( "/query_topics/pose2", 10 );
  auto pub6 = node->create_publisher<geometry_msgs::msg::Pose>( "/query_topics/pose3", 10 );
  Ros2QmlSingletonWrapper wrapper;
  ASSERT_TRUE(
      waitFor( [&wrapper]() { return !wrapper.queryTopics().empty(); }, 30 ) ); // Wait for topics
  for ( const QString &topic :
        QStringList{ "/query_topics/pose1", "/query_topics/vector3", "/query_topics/point1",
                     "/query_topics/point2", "/query_topics/pose2", "/query_topics/pose3" } ) {
    ASSERT_TRUE( waitFor( [&wrapper, &topic]() { return wrapper.queryTopics().contains( topic ); }, 10 ) )
        << topic.toStdString() << " is not in topics.";
  }
  for ( const QString &topic : QStringList{ "/query_topics/point1", "/query_topics/point2" } ) {
    ASSERT_TRUE( waitFor(
        [&wrapper, &topic]() {
          return wrapper.queryTopics( "geometry_msgs/Point" ).contains( topic );
        },
        10 ) )
        << topic.toStdString() << " is not in topics of type Point.";
  }
  for ( const QString &topic :
        QStringList{ "/query_topics/pose1", "/query_topics/pose2", "/query_topics/pose3" } ) {
    ASSERT_TRUE( waitFor(
        [&wrapper, &topic]() {
          return wrapper.queryTopics( "geometry_msgs/msg/Pose" ).contains( topic );
        },
        10 ) )
        << topic.toStdString() << " is not in topics of type Pose.";
  }
  QList<TopicInfo> topic_info = wrapper.queryTopicInfo();
  for ( const QPair<QString, QStringList> &pair : QList<QPair<QString, QStringList>>{
            { "/query_topics/pose1", { "geometry_msgs/msg/Pose" } },
            { "/query_topics/vector3", { "geometry_msgs/msg/Vector3" } },
            { "/query_topics/point1", { "geometry_msgs/msg/Point" } },
            { "/query_topics/point2", { "geometry_msgs/msg/Point" } },
            { "/query_topics/pose2", { "geometry_msgs/msg/Pose" } },
            { "/query_topics/pose3", { "geometry_msgs/msg/Pose" } } } ) {
    bool found = false;
    std::string debug_topics = "";
    for ( const TopicInfo &info : topic_info ) {
      debug_topics += '\n' + info.name().toStdString();
      if ( info.name() != pair.first )
        continue;
      ASSERT_EQ( info.datatypes().size(), pair.second.size() )
          << "Sizes for " << pair.first.toStdString() << "not equal!";
      for ( const auto &datatype : info.datatypes() ) {
        ASSERT_TRUE( pair.second.contains( datatype ) )
            << datatype.toStdString() << " is not in expected types for "
            << pair.first.toStdString();
      }
      found = true;
    }
    EXPECT_TRUE( found ) << "Did not find " << pair.first.toStdString() << " in topic types."
                         << std::endl
                         << "Topics:" << debug_topics;
  }

  ASSERT_FALSE( wrapper.queryTopicTypes( "/query_topics/point1" ).isEmpty() );
  EXPECT_EQ( wrapper.queryTopicTypes( "/query_topics/point1" )[0], "geometry_msgs/msg/Point" );
  ASSERT_FALSE( wrapper.queryTopicTypes( "/query_topics/pose2" ).isEmpty() );
  EXPECT_EQ( wrapper.queryTopicTypes( "/query_topics/pose2" )[0], "geometry_msgs/msg/Pose" );
  EXPECT_TRUE( wrapper.queryTopicTypes( "/query_topics/pose20" ).isEmpty() );
}

TEST( Communication, serviceCallAsync )
{
  QJSEngine engine;

  Ros2QmlSingletonWrapper wrapper;
  auto *service = dynamic_cast<ServiceClient *>(
      wrapper.createServiceClient( "/service", "example_interfaces/srv/AddTwoInts" ) );
  engine.newQObject( service );
  ASSERT_NE( service, nullptr );
  bool service_called = false;
  bool returned = false;
  auto server = node->create_service<example_interfaces::srv::AddTwoInts>(
      "/service", [&]( example_interfaces::srv::AddTwoInts_Request::SharedPtr req,
                       example_interfaces::srv::AddTwoInts_Response::SharedPtr resp ) {
        service_called = true;
        std::this_thread::sleep_for( 1s );
        resp->sum = req->a + req->b;
        returned = true;
      } );
  QJSValue obj = engine.newObject();
  // Check that callback is called using a watcher object that will hold the passed response
  QJSValue callback =
      engine
          .evaluate( "(function (watcher) { return function (resp) { watcher.result = resp; }; })" )
          .call( { obj } );

  ASSERT_TRUE( waitFor( [&]() { return service->isServiceReady(); } ) );
  service->sendRequestAsync( { { "a", 1 }, { "b", 3 } }, callback );
  ASSERT_TRUE( !returned );
  waitFor( [&returned]() { return returned; }, 60 );
  ASSERT_TRUE( returned );
  processEvents();
  ASSERT_TRUE( obj.hasProperty( "result" ) );
  QVariant result = obj.property( "result" ).toVariant();
  EXPECT_TRUE( service_called ) << "Service was not called!";
  ASSERT_EQ( result.type(), QVariant::Map )
      << "Result was not map. Did the request fail? "
      << ( result.type() == QVariant::Bool && !result.toBool() ? "Yes" : "No" ) << std::endl
      << "Typename: " << result.typeName();
  EXPECT_EQ( result.toMap()["sum"].toInt(), 4 )
      << "Contains 'sum'? " << ( result.toMap().contains( "sum" ) ? "Yes" : "No" );
  delete service;

  service = dynamic_cast<ServiceClient *>(
      wrapper.createServiceClient( "/service_empty", "std_srvs/srv/Empty" ) );
  engine.newQObject( service );
  ASSERT_NE( service, nullptr );
  service_called = false;
  returned = false;
  auto server_empty = node->create_service<std_srvs::srv::Empty>(
      "/service_empty",
      [&]( std_srvs::srv::Empty_Request::SharedPtr, std_srvs::srv::Empty_Response::SharedPtr ) {
        service_called = true;
        returned = true;
      } );
  obj = engine.newObject();
  callback =
      engine
          .evaluate( "(function (watcher) { return function (resp) { watcher.result = resp; }; })" )
          .call( { obj } );

  ASSERT_TRUE( waitFor( [&]() { return service->isServiceReady(); } ) );
  service->sendRequestAsync( {}, callback );
  ASSERT_TRUE( !returned );
  waitFor( [&returned]() { return returned; } );
  ASSERT_TRUE( returned );
  processEvents();
  ASSERT_TRUE( obj.hasProperty( "result" ) );
  result = obj.property( "result" ).toVariant();
  // In ROS2 each message needs at least one member, hence empty will add a filler byte member
  ASSERT_EQ( result.type(), QVariant::Map )
      << "Result was not QVariantMap. Typename: " << result.typeName();
}

class ActionServer
{
public:
  using SimpleTest = ros_babel_fish_test_msgs::action::SimpleTest;

  ActionServer()
  {
    using namespace std::placeholders;
    server = rclcpp_action::create_server<ros_babel_fish_test_msgs::action::SimpleTest>(
        node, "action", std::bind( &ActionServer::handleGoal, this, _1, _2 ),
        std::bind( &ActionServer::cancelGoal, this, _1 ),
        std::bind( &ActionServer::handleAccepted, this, _1 ) );
  }

  rclcpp_action::GoalResponse handleGoal( const rclcpp_action::GoalUUID &,
                                          SimpleTest::Goal::ConstSharedPtr )
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  cancelGoal( const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleTest>> )
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted( const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleTest>> goal_handle )
  {
    std::thread execute_thread( &ActionServer::execute, this, goal_handle );
    execute_thread.detach();
  }

  void execute( const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleTest>> goal_handle )
  {
    int goal = goal_handle->get_goal()->goal;
    for ( int i = 0; i < goal; ++i ) {
      std::this_thread::sleep_for( 1ms );
      if ( goal_handle->is_canceling() ) {
        auto result = std::make_shared<ros_babel_fish_test_msgs::action::SimpleTest::Result>();
        result->result = goal_handle->get_goal()->goal * 2 - 1;
        goal_handle->canceled( result );
        return;
      }
      if ( i == goal / 2 ) {
        auto feedback = std::make_shared<ros_babel_fish_test_msgs::action::SimpleTest::Feedback>();
        feedback->feedback = goal + 1;
        goal_handle->publish_feedback( feedback );
      }
    }
    auto result = std::make_shared<ros_babel_fish_test_msgs::action::SimpleTest::Result>();
    result->result = goal_handle->get_goal()->goal * 2;
    goal_handle->succeed( result );
  }

  rclcpp_action::Server<SimpleTest>::SharedPtr server;
};

class ActionClientCallback : public QObject
{
  Q_OBJECT
public:
  Q_INVOKABLE void onGoalResponse( qml_ros2_plugin::GoalHandle *gh )
  {
    this->goal_handles.push_back( gh );
  }

  Q_INVOKABLE void onFeedback( qml_ros2_plugin::GoalHandle *, int fb ) { this->feedback = fb; }

  Q_INVOKABLE void onResult( QVariantMap s ) { this->results[s["goalId"].toString()] = s; }

  std::vector<GoalHandle *> goal_handles;
  int feedback = -1;
  std::map<QString, QVariantMap> results;
};

TEST( Communication, actionClient )
{
  ActionServer server;
  QJSEngine engine;
  auto *client_ptr = new ActionClient( "action", "ros_babel_fish_test_msgs/action/SimpleTest" );
  engine.newQObject( client_ptr );
  ActionClient &client = *client_ptr;
  EXPECT_FALSE( client.isServerReady() );
  // This should also print a warning "Tried to send goal when ActionClient was not connected!"
  EXPECT_EQ( client.sendGoalAsync( { { "goal", 8 } } ), nullptr );

  auto *callback_watcher = new ActionClientCallback;
  QJSValue callback_watcher_js = engine.newQObject( callback_watcher );
  QJSValue options = engine
                         .evaluate( R"!((function(watcher) {
return {
  onGoalResponse(gh) { watcher.onGoalResponse(gh) },
  onFeedback(gh, fb) { watcher.onFeedback(gh, fb.feedback) },
  onResult(result) { watcher.onResult(result) }
}
}))!" )
                         .call( { callback_watcher_js } );
  ASSERT_TRUE( waitFor( [&client]() { return client.isServerReady(); }, 150 ) ); // Wait max 5 seconds
  GoalHandle *handle =
      dynamic_cast<GoalHandle *>( client.sendGoalAsync( { { "goal", 400 } }, options ) );
  //  ASSERT_NE( handle, nullptr );
  ASSERT_TRUE( waitFor( [&callback_watcher]() { return !callback_watcher->goal_handles.empty(); } ) );
  handle = callback_watcher->goal_handles[0];
  //  EXPECT_EQ( handle->status(), action_goal_status::Executing );
  ASSERT_TRUE(
      waitFor( [&handle]() { return handle->status() == action_goal_status::Succeeded; }, 90 ) );
  EXPECT_EQ( callback_watcher->feedback, 401 );

  ASSERT_TRUE( waitFor( [&callback_watcher, handle]() {
    return callback_watcher->results.find( handle->goalId() ) != callback_watcher->results.end();
  } ) );
  QVariantMap result_map = callback_watcher->results[handle->goalId()];
  ASSERT_TRUE( result_map.contains( "goalId" ) )
      << "Keys: " << result_map.keys().join( ", " ).toStdString();
  EXPECT_EQ( result_map["goalId"].type(), QVariant::String );
  EXPECT_EQ( result_map["goalId"].toString(), handle->goalId() );
  ASSERT_TRUE( result_map.contains( "result" ) );
  EXPECT_EQ( result_map["result"].type(), QVariant::Map );
  ASSERT_TRUE( result_map["result"].toMap().contains( "result" ) );
  EXPECT_EQ( result_map["result"].toMap()["result"].type(), QVariant::Int );
  EXPECT_EQ( result_map["result"].toMap()["result"].toInt(), 800 );
  //  delete handle;

  // Cancel
  callback_watcher->goal_handles.clear();
  handle = dynamic_cast<GoalHandle *>( client.sendGoalAsync( { { "goal", 300 } }, options ) );
  //  ASSERT_NE( handle, nullptr );
  ASSERT_TRUE( waitFor( [&callback_watcher]() { return !callback_watcher->goal_handles.empty(); } ) );
  handle = callback_watcher->goal_handles[0];
  // For some reason the second request will stay in Accepted until it is done
  //  EXPECT_EQ( handle->status(), action_goal_status::Executing );
  //  ASSERT_TRUE( waitFor( [ &handle ]() { return handle->status() == action_goal_status::Executing; } )) << handle->status();
  std::this_thread::sleep_for( 5ms );
  handle->cancel();
  EXPECT_TRUE( waitFor( [&handle]() { return handle->status() == action_goal_status::Canceled; } ) )
      << handle->status();
  //  delete handle;

  // Cancel all goals
  callback_watcher->goal_handles.clear();
  GoalHandle *handle1 =
      dynamic_cast<GoalHandle *>( client.sendGoalAsync( { { "goal", 7000 } }, options ) );
  //  ASSERT_NE( handle1, nullptr );
  GoalHandle *handle2 =
      dynamic_cast<GoalHandle *>( client.sendGoalAsync( { { "goal", 8000 } }, options ) );
  //  ASSERT_NE( handle2, nullptr );
  GoalHandle *handle3 =
      dynamic_cast<GoalHandle *>( client.sendGoalAsync( { { "goal", 9000 } }, options ) );
  //  ASSERT_NE( handle3, nullptr );
  ASSERT_TRUE(
      waitFor( [&callback_watcher]() { return callback_watcher->goal_handles.size() == 3; } ) );
  handle1 = callback_watcher->goal_handles[0];
  handle2 = callback_watcher->goal_handles[1];
  handle3 = callback_watcher->goal_handles[2];
  // Again this will be stuck in Accepted, and we're not testing whether rclcpp_action works
  // correctly also it doesn't really matter as long as it is already running
  //  EXPECT_TRUE( waitFor( [ &handle1 ]() { return handle1->status() == action_goal_status::Executing; } ))
  //          << handle1->status();
  //  EXPECT_TRUE( waitFor( [ &handle2 ]() { return handle2->status() == action_goal_status::Executing; } ));
  //  EXPECT_TRUE( waitFor( [ &handle3 ]() { return handle3->status() == action_goal_status::Executing; } ));
  std::this_thread::sleep_for( 5ms );
  client.cancelAllGoals();
  EXPECT_TRUE( waitFor( [&handle1]() { return handle1->status() == action_goal_status::Canceled; }, 150 ) )
      << handle1->status();
  EXPECT_TRUE( waitFor( [&handle2]() { return handle2->status() == action_goal_status::Canceled; }, 150 ) )
      << handle2->status();
  EXPECT_TRUE( waitFor( [&handle3]() { return handle3->status() == action_goal_status::Canceled; }, 150 ) )
      << handle3->status();
  //  delete handle1;
  //  delete handle2;
  //  delete handle3;

  // Cancel all goals before and at time
  callback_watcher->goal_handles.clear();
  handle1 = dynamic_cast<GoalHandle *>( client.sendGoalAsync( { { "goal", 700 } }, options ) );
  //  ASSERT_NE( handle1, nullptr );
  processEvents();
  std::this_thread::sleep_for( 5ms );
  handle2 = dynamic_cast<GoalHandle *>( client.sendGoalAsync( { { "goal", 800 } }, options ) );
  //  ASSERT_NE( handle2, nullptr );
  processEvents();
  std::this_thread::sleep_for( 5ms );
  QDateTime now = rosToQmlTime( node->now() );
  std::this_thread::sleep_for( 5ms );
  handle3 = dynamic_cast<GoalHandle *>( client.sendGoalAsync( { { "goal", 190 } }, options ) );
  //  ASSERT_NE( handle3, nullptr );
  ASSERT_TRUE(
      waitFor( [&callback_watcher]() { return callback_watcher->goal_handles.size() == 3; } ) );
  handle1 = callback_watcher->goal_handles[0];
  handle2 = callback_watcher->goal_handles[1];
  handle3 = callback_watcher->goal_handles[2];
  EXPECT_NE( handle1->status(), action_goal_status::Succeeded );
  EXPECT_NE( handle2->status(), action_goal_status::Succeeded );
  EXPECT_NE( handle3->status(), action_goal_status::Succeeded );
  client.cancelGoalsBefore( now );
  EXPECT_TRUE( waitFor( [&handle1]() { return handle1->status() == action_goal_status::Canceled; } ) )
      << handle1->status();
  EXPECT_TRUE( waitFor( [&handle2]() { return handle2->status() == action_goal_status::Canceled; } ) )
      << handle2->status();
  EXPECT_TRUE( waitFor( [&handle3]() { return handle3->status() == action_goal_status::Succeeded; } ) )
      << handle3->status();

  ASSERT_TRUE( waitFor( [&callback_watcher, handle3]() {
    return callback_watcher->results.find( handle3->goalId() ) != callback_watcher->results.end();
  } ) );
  result_map = callback_watcher->results[handle3->goalId()];
  ASSERT_TRUE( result_map.contains( "goalId" ) );
  EXPECT_EQ( result_map["goalId"].type(), QVariant::String );
  EXPECT_EQ( result_map["goalId"].toString().toStdString(), handle3->goalId().toStdString() );
  ASSERT_TRUE( result_map.contains( "result" ) );
  EXPECT_EQ( result_map["result"].type(), QVariant::Map );
  ASSERT_TRUE( result_map["result"].toMap().contains( "result" ) );
  EXPECT_EQ( result_map["result"].toMap()["result"].type(), QVariant::Int );
  EXPECT_EQ( result_map["result"].toMap()["result"].toInt(), 380 );

  //  delete handle1;
  //  delete handle2;
  //  delete handle3;
}

TEST( Communication, tfTransform )
{
  qml_ros2_plugin::TfTransformListenerWrapper wrapper;
  qml_ros2_plugin::TfTransform transform;
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "world";
  transform_stamped.header.stamp = node->now();
  transform_stamped.child_frame_id = "base";
  transform_stamped.transform.translation.x = 1;
  transform_stamped.transform.translation.y = 2.4;
  transform_stamped.transform.translation.z = 0.3;
  transform_stamped.transform.rotation.w = 0.8431746;
  transform_stamped.transform.rotation.x = 0.06799795;
  transform_stamped.transform.rotation.y = 0.48208547;
  transform_stamped.transform.rotation.z = -0.22809313;
  tf2_ros::TransformBroadcaster broadcaster( *node );
  EXPECT_TRUE( transform.enabled() );
  ASSERT_TRUE( transform.message().contains( "valid" ) );
  EXPECT_FALSE( transform.message()["valid"].toBool() );
  EXPECT_FALSE( transform.valid() );
  broadcaster.sendTransform( transform_stamped );
  processEvents();
  transform.setSourceFrame( "base" );
  EXPECT_EQ( transform.sourceFrame(), QString( "base" ) ) << transform.sourceFrame().toStdString();
  transform.setTargetFrame( "world" );
  EXPECT_EQ( transform.targetFrame(), QString( "world" ) ) << transform.targetFrame().toStdString();
  if ( !waitFor( [&]() {
         transform_stamped.header.stamp = node->now();
         broadcaster.sendTransform( transform_stamped );
         return transform.valid();
       } ) )
    FAIL() << "Failed to get transform in time!";
  ASSERT_TRUE( transform.valid() );
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["x"].toDouble(), 1 );
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["y"].toDouble(), 2.4 );
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["z"].toDouble(), 0.3 );
  // TF will normalize the quaternion, hence, we check that it's very close instead of equality
  EXPECT_NEAR( transform.rotation().toMap()["w"].toDouble(), 0.8431746, 1E-6 );
  EXPECT_NEAR( transform.rotation().toMap()["x"].toDouble(), 0.06799795, 1E-6 );
  EXPECT_NEAR( transform.rotation().toMap()["y"].toDouble(), 0.48208547, 1E-6 );
  EXPECT_NEAR( transform.rotation().toMap()["z"].toDouble(), -0.22809313, 1E-6 );

  transform.setEnabled( false );
  EXPECT_FALSE( transform.enabled() );
  QCoreApplication::processEvents();
  transform_stamped.header.stamp = node->now();
  transform_stamped.transform.translation.x = 3.14;
  broadcaster.sendTransform( transform_stamped );
  waitFor( []() { return false; } );
  EXPECT_DOUBLE_EQ( transform.message()["transform"].toMap()["translation"].toMap()["x"].toDouble(), 1 )
      << "Shouldn't have received the \"x: 3.14\" transform!";
  QDateTime last_transform_datetime = QDateTime::currentDateTime();
  transform_stamped.header.stamp = node->now();
  transform_stamped.transform.translation.x = 0.577;
  broadcaster.sendTransform( transform_stamped );
  transform.setEnabled( true );
  ASSERT_TRUE( transform.enabled() );
  if ( !waitFor( [&]() {
         transform_stamped.header.stamp = node->now();
         broadcaster.sendTransform( transform_stamped );
         return std::abs( transform.translation().toMap()["x"].toDouble() - 0.577 ) < 1E-4;
       } ) )
    FAIL() << "Did not receive transform in time: "
           << transform.translation().toMap()["x"].toDouble();
  EXPECT_DOUBLE_EQ( transform.message()["transform"].toMap()["translation"].toMap()["x"].toDouble(),
                    0.577 );

  QVariant can_transform = wrapper.canTransform( "base", "world" ).toBool();
  ASSERT_EQ( can_transform.type(), QVariant::Bool ) << can_transform.toString().toStdString();
  EXPECT_TRUE( can_transform.toBool() );
  can_transform = wrapper.canTransform( "millionaire", "inheritance", QDateTime(), 500 );
  EXPECT_TRUE( can_transform.type() != QVariant::Bool || !can_transform.toBool() )
      << "Inheritance shouldn't be able to transform to millionaire!";
  can_transform = wrapper.canTransform( "base", last_transform_datetime, "world",
                                        last_transform_datetime, "world" );
  ASSERT_EQ( can_transform.type(), QVariant::Bool ) << can_transform.toString().toStdString();
  EXPECT_TRUE( can_transform.toBool() );

  EXPECT_TRUE( mapAndMessageEqual( wrapper.lookUpTransform( "world", "base", QDateTime(), 500 ),
                                   transform_stamped ) );
  QVariantMap lookup_result =
      wrapper.lookUpTransform( "world", Time( transform_stamped.header.stamp ), "base",
                               Time( transform_stamped.header.stamp ), "world" );
  ASSERT_TRUE( lookup_result["valid"].toBool() )
      << lookup_result["exception"].toString().toStdString() << std::endl
      << lookup_result["message"].toString().toStdString();
  EXPECT_TRUE( mapAndMessageEqual( lookup_result, transform_stamped, "msg", 1E-2 ) );
  lookup_result =
      wrapper.lookUpTransform( "world", rosToQmlTime( transform_stamped.header.stamp ), "base",
                               rosToQmlTime( transform_stamped.header.stamp ), "world", 500 );
  ASSERT_TRUE( lookup_result["valid"].toBool() )
      << lookup_result["exception"].toString().toStdString() << std::endl
      << lookup_result["message"].toString().toStdString();
  EXPECT_TRUE( mapAndMessageEqual( lookup_result, transform_stamped, "msg", 1E-2 ) );

  EXPECT_EQ( wrapper
                 .canTransform( "world", QDateTime::currentDateTime(), "base",
                                QDateTime::currentDateTime(), "world" )
                 .type(),
             QVariant::String );
  EXPECT_EQ( wrapper
                 .canTransform( "world", QDateTime::currentDateTime(), "base",
                                QDateTime::currentDateTime(), "world", 500 )
                 .type(),
             QVariant::String );
  EXPECT_EQ( wrapper
                 .lookUpTransform( "world", QDateTime::currentDateTime(), "base",
                                   QDateTime::currentDateTime(), "world" )["exception"]
                 .toString(),
             QString( "ExtrapolationException" ) );
  EXPECT_EQ(
      wrapper.lookUpTransform( "world", "base", QDateTime::currentDateTime() )["exception"].toString(),
      QString( "ExtrapolationException" ) );
  EXPECT_EQ(
      wrapper.lookUpTransform( "world", QDateTime(), "equality", QDateTime(), "world" )["exception"]
          .toString(),
      QString( "LookupException" ) );
  EXPECT_EQ( wrapper.lookUpTransform( "world", "equality" )["exception"].toString(),
             QString( "LookupException" ) );
  EXPECT_EQ( wrapper
                 .lookUpTransform( "world", QDateTime(), "billionaires", QDateTime(),
                                   "world" )["exception"]
                 .toString(),
             QString( "ConnectivityException" ) );
  EXPECT_EQ( wrapper.lookUpTransform( "world", "politics" )["exception"].toString(),
             QString( "ConnectivityException" ) );
  // Currently no idea how to test InvalidArgumentException since invalid quaternions get rejected when publishing already

  /// ==============================================================================================
  /// Test rate limiting
  /// ==============================================================================================
  transform.setRate( 30 );
  EXPECT_NEAR( transform.rate(), 30, 3 );
  QSignalSpy transform_changed_spy( &transform, SIGNAL( messageChanged() ) );
  std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
  for ( int i = 0; i < 4000; ++i ) {
    transform_stamped.header.stamp = node->now();
    broadcaster.sendTransform( transform_stamped );
    processEvents();
    usleep( 100 );
  }
  auto length = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - start );
  double sent_hz = 10000 * 1000.0 / length.count();
  ASSERT_GE( sent_hz, 60 ) << "Your computer seems to be too slow to run this test.";
  EXPECT_NEAR( transform_changed_spy.count() * 1000.0 / length.count(), 30, 3 )
      << "Data was sent with " << std::fixed << std::setprecision( 2 ) << sent_hz << "Hz. "
      << "Received " << transform_changed_spy.count() << " messages in " << length.count() << "ms.";

  transform.setRate( 120 );
  transform_changed_spy.clear();
  start = std::chrono::system_clock::now();
  for ( int i = 0; i < 4000; ++i ) {
    transform_stamped.header.stamp = node->now();
    broadcaster.sendTransform( transform_stamped );
    processEvents();
    usleep( 100 );
  }
  length = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() -
                                                                  start );
  sent_hz = 10000 * 1000.0 / length.count();
  ASSERT_GE( sent_hz, 200 ) << "Your computer seems to be too slow to run this test.";
  EXPECT_NEAR( transform.rate(), 120, 12 );
  EXPECT_NEAR( transform_changed_spy.count() * 1000.0 / length.count(), transform.rate(), 12 )
      << "Data was sent with " << std::fixed << std::setprecision( 2 ) << sent_hz << "Hz. "
      << "Received " << transform_changed_spy.count() << " messages in " << length.count() << "ms.";
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  rclcpp::init( argc, argv );
  node = rclcpp::Node::make_shared( "communication" );
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster( node );
  geometry_msgs::msg::TransformStamped static_transform;
  static_transform.header.frame_id = "billionaires";
  static_transform.child_frame_id = "politics";
  static_tf_broadcaster.sendTransform( static_transform );
  Ros2QmlSingletonWrapper wrapper;
  wrapper.init( "communication_qml" );
  int result = RUN_ALL_TESTS();
  node.reset();
  return result;
}

#include "communication.moc"
