// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "common.hpp"
#include "message_comparison.hpp"

#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml_ros2_plugin/ros2.hpp"
#include <ros_babel_fish_test_msgs/msg/test_message.hpp>

#include <QAbstractListModel>
#include <QCoreApplication>
#include <QDateTime>
#include <QQmlComponent>
#include <QQmlEngine>

using namespace qml_ros2_plugin;
using namespace qml_ros2_plugin::conversion;
using namespace ros_babel_fish;
using namespace ros_babel_fish_test_msgs::msg;

TEST( MessageConversion, babelFishDispenser )
{
  BabelFish fish1 = BabelFishDispenser::getBabelFish();
  BabelFish fish2 = BabelFishDispenser::getBabelFish();
  EXPECT_EQ( fish1.type_support_providers()[0], fish2.type_support_providers()[0] );
}

TEST( MessageConversion, emptyMessage )
{
  Ros2QmlSingletonWrapper ros;
  {
    QVariant map = ros.createEmptyMessage( "geometry_msgs/PoseStamped" );
    geometry_msgs::msg::PoseStamped msg;
    EXPECT_TRUE( mapAndMessageEqual( map, msg ) );
  }
  {
    QVariant map = ros.createEmptyMessage( "ros_babel_fish_test_msgs/TestMessage" );
    ros_babel_fish_test_msgs::msg::TestMessage msg;
    EXPECT_TRUE( mapAndMessageEqual( map, msg ) );
  }
  {
    QVariant map = ros.createEmptyServiceRequest( "example_interfaces/srv/AddTwoInts" );
    example_interfaces::srv::AddTwoInts_Request req;
    EXPECT_TRUE( mapAndMessageEqual( map, req ) );
  }
}

TEST( MessageConversion, msgToMapHeader )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  std_msgs::msg::Header msg;
  msg.frame_id = "Test";
  msg.stamp = rclcpp::Time( 1, 336 * 1000 * 1000 + 532 * 1000 );
  QVariantMap map = msgToMap( msg );
  ASSERT_TRUE( map.contains( "frame_id" ) );
  EXPECT_EQ( map["frame_id"].toString().toStdString(), msg.frame_id );
  ASSERT_TRUE( map.contains( "stamp" ) );
  EXPECT_EQ( map["stamp"].value<Time>().getTime(), msg.stamp );
  CompoundMessage::SharedPtr bf_msg = fish.create_message_shared( "std_msgs/Header" );
  EXPECT_TRUE( fillMessage( *bf_msg, map ) );
  auto &compound = *bf_msg;
  ASSERT_TRUE( compound.containsKey( "frame_id" ) );
  EXPECT_EQ( compound["frame_id"].value<std::string>(), msg.frame_id );
  ASSERT_TRUE( compound.containsKey( "stamp" ) );
  EXPECT_EQ( compound["stamp"].value<rclcpp::Time>(), rclcpp::Time( msg.stamp ) );
}

TEST( MessageConversion, msgToMapTransformStamped )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = "testframe";
  msg.header.stamp = rclcpp::Time( 42 );
  msg.child_frame_id = "child";
  msg.transform.translation.x = 1.4;
  msg.transform.translation.y = 2.1;
  msg.transform.translation.z = 3.5;
  msg.transform.rotation.w = 0.384;
  msg.transform.rotation.x = 0.412;
  msg.transform.rotation.y = 0.215;
  msg.transform.rotation.z = 0.79786;
  QVariantMap map = msgToMap( msg );
  EXPECT_EQ( map["header"].toHash()["frame_id"].toString().toStdString(), msg.header.frame_id );
  bool ok = false;
  EXPECT_EQ( map["header"].toHash()["stamp"].value<Time>().getTime(),
             rclcpp::Time( msg.header.stamp ) );
  EXPECT_EQ( map["child_frame_id"].toString().toStdString(), msg.child_frame_id );
  EXPECT_EQ( map["transform"].toHash()["translation"].toHash()["x"].toDouble( &ok ),
             msg.transform.translation.x );
  EXPECT_TRUE( ok );
  EXPECT_EQ( map["transform"].toHash()["translation"].toHash()["y"].toDouble( &ok ),
             msg.transform.translation.y );
  EXPECT_TRUE( ok );
  EXPECT_EQ( map["transform"].toHash()["translation"].toHash()["z"].toDouble( &ok ),
             msg.transform.translation.z );
  EXPECT_TRUE( ok );
  EXPECT_EQ( map["transform"].toHash()["rotation"].toHash()["w"].toDouble( &ok ),
             msg.transform.rotation.w );
  EXPECT_TRUE( ok );
  EXPECT_EQ( map["transform"].toHash()["rotation"].toHash()["x"].toDouble( &ok ),
             msg.transform.rotation.x );
  EXPECT_TRUE( ok );
  EXPECT_EQ( map["transform"].toHash()["rotation"].toHash()["y"].toDouble( &ok ),
             msg.transform.rotation.y );
  EXPECT_TRUE( ok );
  EXPECT_EQ( map["transform"].toHash()["rotation"].toHash()["z"].toDouble( &ok ),
             msg.transform.rotation.z );
  EXPECT_TRUE( ok );

  CompoundMessage::SharedPtr bf_msg =
      fish.create_message_shared( "geometry_msgs/TransformStamped" );
  EXPECT_TRUE( fillMessage( *bf_msg, map ) );
  auto &compound = *bf_msg;
  EXPECT_EQ( compound["header"]["frame_id"].value<std::string>(), msg.header.frame_id );
  EXPECT_EQ( compound["header"]["stamp"].value<rclcpp::Time>(), msg.header.stamp );
  EXPECT_EQ( compound["child_frame_id"].value<std::string>(), msg.child_frame_id );
  EXPECT_EQ( compound["transform"]["translation"]["x"].value<double>(), msg.transform.translation.x );
  EXPECT_EQ( compound["transform"]["translation"]["y"].value<double>(), msg.transform.translation.y );
  EXPECT_EQ( compound["transform"]["translation"]["z"].value<double>(), msg.transform.translation.z );
  EXPECT_EQ( compound["transform"]["rotation"]["w"].value<double>(), msg.transform.rotation.w );
  EXPECT_EQ( compound["transform"]["rotation"]["x"].value<double>(), msg.transform.rotation.x );
  EXPECT_EQ( compound["transform"]["rotation"]["y"].value<double>(), msg.transform.rotation.y );
  EXPECT_EQ( compound["transform"]["rotation"]["z"].value<double>(), msg.transform.rotation.z );
}

TEST( MessageConversion, msgToMapGoalInfo )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  action_msgs::msg::GoalInfo msg;
  //  msg.goal_id = "42";
  msg.stamp = rclcpp::Time( 1337 );
  QVariantMap map = msgToMap( msg );
  ASSERT_TRUE( map.contains( "goal_id" ) );
  //  EXPECT_EQ( map["goal_id"].toString().toStdString(), msg.goal_id ); // TODO
  ASSERT_TRUE( map.contains( "stamp" ) );
  EXPECT_EQ( map["stamp"].value<Time>().getTime(), msg.stamp );
}

TEST( MessageConversion, msgToMapGoalStatus )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  action_msgs::msg::GoalStatus msg;
  //  msg.goal_info.goal_id = "42";
  msg.goal_info.stamp = rclcpp::Time( 1337 );
  msg.status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  QVariantMap map = msgToMap( msg );
  ASSERT_TRUE( map.contains( "goal_info" ) );
  ASSERT_TRUE( obtainValueAsReference<QVariantMap>( map["goal_info"] ).contains( "goal_id" ) );
  //  EXPECT_EQ( obtainValueAsReference<QVariantMap>( map["goal_info"] )["goal_id"].toString().toStdString(), msg.goal_info.goal_id );
  ASSERT_TRUE( obtainValueAsReference<QVariantMap>( map["goal_info"] ).contains( "stamp" ) );
  EXPECT_EQ( obtainValueAsReference<QVariantMap>( map["goal_info"] )["stamp"].value<Time>().getTime(),
             rclcpp::Time( msg.goal_info.stamp ) );
  ASSERT_TRUE( map.contains( "status" ) );
  EXPECT_EQ( map["status"].toInt(), msg.status );
}

TEST( MessageConversion, msgToMapRBF )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  ros_babel_fish_test_msgs::msg::TestMessage test_message;
  test_message.header.stamp = rclcpp::Time( 42 );
  test_message.header.frame_id = "test";
  test_message.b = false;
  test_message.ui8 = 255;
  test_message.ui16 = 65535;
  test_message.ui32 = 4294967295;
  test_message.ui64 = std::numeric_limits<uint64_t>::max();
  test_message.i8 = -128;
  test_message.i16 = -32768;
  test_message.i32 = -2147483648;
  test_message.i64 = std::numeric_limits<int64_t>::min();
  test_message.f32 = 42.0;
  test_message.f64 = 1337.0;
  test_message.str = "test string";
  test_message.t = rclcpp::Time( 1337, 42 );
  test_message.d = rclcpp::Duration( 42, 1337 );
  for ( int i = 0; i < 5; ++i ) {
    geometry_msgs::msg::Point p;
    p.x = i * 0.1;
    p.y = 3 + i * 0.4;
    p.z = 15 + i * 3.14;
    test_message.point_arr.push_back( p );
  }

  CompoundMessage wrapped(
      *fish.get_message_type_support( "ros_babel_fish_test_msgs/TestMessage" ),
      std::shared_ptr<void>( &test_message, []( void * ) { /*empty deleter*/ } ) );
  QVariant map = msgToMap( wrapped );
  EXPECT_TRUE( mapAndMessageEqual( map, test_message ) );
  CompoundMessage::SharedPtr msg =
      fish.create_message_shared( "ros_babel_fish_test_msgs/TestMessage" );
  EXPECT_TRUE( fillMessage( *msg, map ) );
  auto &compound = msg->as<CompoundMessage>();
  EXPECT_TRUE( messageEqual( compound, test_message ) );

  // Int for duration
  map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>( map )["d"] = 42;
  EXPECT_TRUE( fillMessage( *msg, map ) );

  // Incompatible type
  QVariant broken_map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>( broken_map )["i32"] = QVariantList();
  EXPECT_FALSE( fillMessage( *msg, broken_map ) );

  broken_map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>( broken_map )["i32"] =
      QDateTime::fromMSecsSinceEpoch( 157000000000L );
  EXPECT_FALSE( fillMessage( *msg, broken_map ) );

  broken_map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>( broken_map )["i32"] = QString( "WRONG" );
  EXPECT_FALSE( fillMessage( *msg, broken_map ) );

  broken_map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>( broken_map )["str"] = 132;
  EXPECT_FALSE( fillMessage( *msg, broken_map ) );

  broken_map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>( broken_map )["str"] = QVariantMap();
  EXPECT_FALSE( fillMessage( *msg, broken_map ) );

  broken_map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>( broken_map )["header"] = 12;
  EXPECT_FALSE( fillMessage( *msg, broken_map ) );

  broken_map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>(
      obtainValueAsReference<QVariantMap>( broken_map )["header"] )["invalid"] = 12;
  EXPECT_FALSE( fillMessage( *msg, broken_map ) );

  broken_map = msgToMap( wrapped );
  obtainValueAsReference<QVariantMap>( broken_map )["i32"] = QDate( 2019, 11, 4 );
  EXPECT_FALSE( fillMessage( *msg, broken_map ) );
}

TEST( MessageConversion, array )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  // TODO: Test for fixed array length

  ros_babel_fish_test_msgs::msg::TestArray test_array;
  unsigned SEED = 42;
  fillArray( test_array.bools, SEED++ );
  fillArray( test_array.uint8s, SEED++ );
  fillArray( test_array.uint16s, SEED++ );
  fillArray( test_array.uint32s, SEED++ );
  fillArray( test_array.uint64s, SEED++ );
  fillArray( test_array.int8s, SEED++ );
  fillArray( test_array.int16s, SEED++ );
  fillArray( test_array.int32s, SEED++ );
  fillArray( test_array.int64s, SEED++ );
  fillArray( test_array.float32s, SEED++ );
  fillArray( test_array.float64s, SEED++ );
  fillArray( test_array.times, SEED++ );
  fillArray( test_array.durations, SEED++ );
  fillArray( test_array.strings, SEED++ );
  fillArray( test_array.subarrays_fixed, SEED++ );
  fillArray( test_array.subarrays, SEED++ );

  CompoundMessage wrapped( *fish.get_message_type_support( "ros_babel_fish_test_msgs/TestArray" ),
                           std::shared_ptr<void>( &test_array, []( void * ) { /*empty deleter*/ } ) );
  QVariant map = msgToMap( wrapped );
  ASSERT_TRUE( mapAndMessageEqual( map, test_array ) );
  CompoundMessage::SharedPtr msg =
      fish.create_message_shared( "ros_babel_fish_test_msgs/TestArray" );
  EXPECT_TRUE( fillMessage( *msg, map ) );
  ASSERT_TRUE( messageEqual( msg->as<CompoundMessage>(), test_array ) );

  EXPECT_FALSE( map.toHash()["float32s"].value<Array>()._isModified( 0 ) );

  // Remove element from end
  auto int32s_array = map.toMap()["int32s"].value<Array>();
  ASSERT_FALSE( int32s_array._inCache() );
  EXPECT_EQ( int32s_array.pop().toInt(), test_array.int32s[test_array.int32s.size() - 1] );
  test_array.int32s.pop_back();
  ASSERT_FALSE( int32s_array._inCache() );

  // Add an element to the back
  test_array.int32s.push_back( 420 );
  int32s_array.push( 420ULL );
  ASSERT_FALSE( int32s_array._inCache() );
  // Modification of the original message may also affect the wrapper unless it is in the cache already
  int32s_array.spliceList( 4, 1, { int32s_array.at( 4 ).toInt() - 42 } );
  test_array.int32s[4] -= 42;
  ASSERT_FALSE( int32s_array._inCache() );
  ASSERT_TRUE( mapAndMessageEqual( map, test_array ) );
  // Add an element to the front (this will require a deep copy)
  auto uint32_array = map.toMap()["uint32s"].value<Array>();
  uint32_array.unshift( 1337.f );
  test_array.uint32s.insert( test_array.uint32s.begin(), 1337 );
  ASSERT_TRUE( mapAndMessageEqual( map, test_array ) );
  // Insert a compatible element at a random position
  int32s_array.spliceList( 6, 0, { 421337.0 } );
  test_array.int32s.insert( test_array.int32s.begin() + 6, 421337 );
  ASSERT_TRUE( mapAndMessageEqual( map, test_array ) );
  int32s_array.spliceList( 8, 0, { 43172LL } );
  test_array.int32s.insert( test_array.int32s.begin() + 8, 43172 );
  ASSERT_TRUE( mapAndMessageEqual( map, test_array ) );
  // Remove an element from the beginning
  EXPECT_EQ( int32s_array.shift().toInt(), test_array.int32s[0] );
  test_array.int32s.erase( test_array.int32s.begin() );
  ASSERT_TRUE( mapAndMessageEqual( map, test_array ) );
  // Remove an element from the end
  int32s_array.setLength( int32s_array.length() - 1 );
  test_array.int32s.pop_back();
  EXPECT_TRUE( mapAndMessageEqual( map, test_array ) );
  // Obtain array as QML compatible type which should fill the cache
  EXPECT_TRUE( mapAndMessageEqual( int32s_array.toArray(), test_array.int32s ) );
  ASSERT_TRUE( int32s_array._inCache() );

  // Change array length
  test_array.uint32s.resize( 5 );
  uint32_array.setLength( 5 );

  // Remove multiple elements from the end and add some
  test_array.float32s.erase( test_array.float32s.begin() + 5, test_array.float32s.end() );
  test_array.float32s.push_back( 42.0 );
  test_array.float32s.push_back( 13.37 );
  map.toMap()["float32s"].value<Array>().spliceList( 5, 100000, { 42.0, 13.37 } );

  // All the different time and duration types
  auto time_array = map.toMap()["times"].value<Array>();
  // QML times are in ms, rclcpp uses nanoseconds
  time_array.push( static_cast<uint8_t>( 21 ) );
  test_array.times.push_back( rclcpp::Time( 21000000 ) );
  time_array.push( static_cast<uint16_t>( 24 ) );
  test_array.times.push_back( rclcpp::Time( 24000000 ) );
  time_array.push( static_cast<uint32_t>( 27 ) );
  test_array.times.push_back( rclcpp::Time( 27000000 ) );
  time_array.push( static_cast<qulonglong>( 30 ) );
  test_array.times.push_back( rclcpp::Time( 30000000 ) );
  time_array.push( static_cast<int8_t>( 33 ) );
  test_array.times.push_back( rclcpp::Time( 33000000 ) );
  time_array.push( static_cast<int16_t>( 36 ) );
  test_array.times.push_back( rclcpp::Time( 36000000 ) );
  time_array.push( static_cast<int32_t>( 39 ) );
  test_array.times.push_back( rclcpp::Time( 39000000 ) );
  time_array.push( static_cast<qlonglong>( 42 ) );
  test_array.times.push_back( rclcpp::Time( 42000000 ) );
  time_array.push( 45.0 );
  test_array.times.push_back( rclcpp::Time( 45000000 ) );

  auto duration_array = map.toMap()["durations"].value<Array>();
  duration_array.spliceList( 0, 1, { static_cast<uint8_t>( 21 ) } );
  test_array.durations[0] = rclcpp::Duration::from_seconds( 0.021 );
  duration_array.spliceList( 1, 1, { static_cast<uint16_t>( 24 ) } );
  test_array.durations[1] = rclcpp::Duration::from_seconds( 0.024 );
  duration_array.spliceList( 2, 1, { static_cast<uint32_t>( 27 ) } );
  test_array.durations[2] = rclcpp::Duration::from_seconds( 0.027 );
  duration_array.spliceList( 3, 1, { static_cast<qulonglong>( 30 ) } );
  test_array.durations[3] = rclcpp::Duration::from_seconds( 0.030 );
  duration_array.spliceList( 4, 1, { static_cast<int8_t>( 33 ) } );
  test_array.durations[4] = rclcpp::Duration::from_seconds( 0.033 );
  duration_array.spliceList( 5, 1, { static_cast<int16_t>( 36 ) } );
  test_array.durations[5] = rclcpp::Duration::from_seconds( 0.036 );
  duration_array.spliceList( 6, 1, { static_cast<int32_t>( 39 ) } );
  test_array.durations[6] = rclcpp::Duration::from_seconds( 0.039 );
  duration_array.spliceList( 7, 1, { static_cast<qlonglong>( 42 ) } );
  test_array.durations[7] = rclcpp::Duration::from_seconds( 0.042 );

  msg = fish.create_message_shared( "ros_babel_fish_test_msgs/TestArray" );
  EXPECT_TRUE( fillMessage( *msg, map ) );
  ASSERT_TRUE( messageEqual( msg->as<CompoundMessage>(), test_array ) );
  map = msgToMap( msg );
  // Out of bounds access
  EXPECT_FALSE( map.toMap()["uint32s"].value<Array>().at( 5 ).isValid() );

  // Incompatible type
  QVariant broken_map = msgToMap( msg );
  auto int32_array = broken_map.toMap()["int32s"].value<Array>();
  int32_array.push( 1337.42 );
  auto broken_msg = fish.create_message_shared( "ros_babel_fish_test_msgs/TestArray" );
  EXPECT_FALSE( fillMessage( *broken_msg, broken_map ) );

  broken_map = msgToMap( msg );
  broken_map.toMap()["int32s"].value<Array>().push( QVariantList() );
  EXPECT_FALSE( fillMessage( *broken_msg, broken_map ) );

  // Too long fixed array
  broken_map = msgToMap( msg );
  broken_map.toMap()["durations"].value<Array>().push( 3456.0 );
  EXPECT_FALSE( fillMessage( *broken_msg, broken_map ) );

  // Non string in string array but can be converted to string
  broken_map = msgToMap( msg );
  broken_map.toMap()["strings"].value<Array>().spliceList( 3, 0, { 3456.0 } );
  EXPECT_TRUE( fillMessage( *broken_msg, broken_map ) );

  // Non string in string array that cant be converted to string
  broken_map = msgToMap( msg );
  auto qobject = std::make_unique<QObject>();
  broken_map.toMap()["strings"].value<Array>().spliceList(
      3, 0, { QVariant::fromValue( qobject.get() ) } );
  EXPECT_FALSE( fillMessage( *broken_msg, broken_map ) );
  broken_map = msgToMap( msg );
  broken_map.toMap()["strings"].value<Array>().spliceList(
      3, 0, { QVariant::fromValue( QVariantList() ) } );
  EXPECT_FALSE( fillMessage( *broken_msg, broken_map ) );
  broken_map = msgToMap( msg );
  broken_map.toMap()["strings"].value<Array>().spliceList(
      3, 0, { QVariant::fromValue( QVariantMap() ) } );
  EXPECT_FALSE( fillMessage( *broken_msg, broken_map ) );

  // Primitive in compound array
  broken_map = msgToMap( msg );
  broken_map.toMap()["subarrays"].value<Array>().spliceList( 5, 0, { 3.141592 } );
  EXPECT_FALSE( fillMessage( *broken_msg, broken_map ) );
}

QObject *createModel( QQmlEngine *engine, const QByteArray &elements )
{
  QQmlComponent component( engine );
  component.setData( "import QtQuick 2.0 ListModel {" + elements + "}", QUrl() );
  return component.create();
}

TEST( MessageConversion, qobjectConversion )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  QQmlEngine engine;
  QQmlComponent component( &engine );
  component.setData( R"(
import QtQuick 2.0

QtObject {
  property bool b: false
  property int ui8: 42
  property ListModel point_arr: ListModel {
    ListElement { x: 1; y: 2; z: 3 }
    ListElement { x: 3; y: 2; z: 3 }
    ListElement { x: 1; y: 3.14; z: 1 }
    ListElement { x: 10000; y: -2; z: -300 }
  }
}
)",
                     QUrl() );
  auto obj = std::unique_ptr<QObject>( component.create() );
  CompoundMessage::SharedPtr msg =
      fish.create_message_shared( "ros_babel_fish_test_msgs/TestMessage" );
  ASSERT_NE( obj, nullptr ) << component.errorString().toStdString();
  fillMessage( *msg, QVariant::fromValue( obj.get() ) );

  TestMessage test_msg;
  test_msg.b = false;
  test_msg.ui8 = 42;
  test_msg.point_arr.resize( 4 );
  test_msg.point_arr[0].x = 1;
  test_msg.point_arr[0].y = 2;
  test_msg.point_arr[0].z = 3;
  test_msg.point_arr[1].x = 3;
  test_msg.point_arr[1].y = 2;
  test_msg.point_arr[1].z = 3;
  test_msg.point_arr[2].x = 1;
  test_msg.point_arr[2].y = 3.14;
  test_msg.point_arr[2].z = 1;
  ;
  test_msg.point_arr[3].x = 10000;
  test_msg.point_arr[3].y = -2;
  test_msg.point_arr[3].z = -300;
  EXPECT_TRUE( messageEqual( msg->as<CompoundMessage>(), test_msg ) );

  component.setData( R"(
import QtQuick 2.0

QtObject {
  property ListModel bools: ListModel {
    ListElement { value: true }
    ListElement { value: false }
    ListElement { value: true }
  }
  property var uint8s: ListModel {
    ListElement { value: 1 }
    ListElement { value: 2 }
    ListElement { value: 3 }
    ListElement { value: 5 }
  }
  property var uint16s: ListModel {
    ListElement { value: 13 }
    ListElement { value: 42 }
    ListElement { value: 1337 }
  }
  property var uint32s: ListModel {
    ListElement { value: 1 }
    ListElement { value: 3000 }
    ListElement { value: 50000000 }
  }
  property var uint64s: ListModel {
    ListElement { value: 5000000 }
    ListElement { value: 500000000000 }
  }
  property var int8s: ListModel {
    ListElement { value: 1 }
    ListElement { value: -2 }
    ListElement { value: 1 }
    ListElement { value: -1 }
    ListElement { value: 0 }
    ListElement { value: -1 }
  }
  property var int16s: ListModel {
    ListElement { value: 255 }
    ListElement { value: -255 }
  }
  property var int32s: ListModel {
    ListElement { value: 5000000 }
    ListElement { value: -5000000 }
  }
  property var int64s: ListModel {
    ListElement { value: 50000000000 }
    ListElement { value: -5000000000 }
  }
  property var float32s: ListModel {
    ListElement { value: 1.0 }
    ListElement { value: 3.14 }
  }
  property var float64s: ListModel { ListElement { value: 2.7 } }
  // Need to use the ms format for time here because javascript can not be used in ListElement properties
  property var times: ListModel { ListElement { value: 13000 } }
  property ListModel durations: ListModel {
    ListElement { value: 42000 }
    ListElement { value: 1337 }
    ListElement { value: -2000 }
    ListElement { value: 1 }
    ListElement { value: -1 }
    ListElement { value: 2 }
    ListElement { value: -2 }
    ListElement { value: 3 }
    ListElement { value: -3 }
    ListElement { value: -4 }
    ListElement { value: 4 }
    ListElement { value: 5 }
    // One too many
    ListElement { value: 1337 }
  }
  property ListModel strings: ListModel {
    id: stringModel
    ListElement { value: "This" }
    ListElement { value: "is" }
    ListElement { value: "a" }
    ListElement { value: "test" }

  }
  Component.onCompleted: {
    stringModel.append({ 'x': "An object", 'y': "that's not a string"})
  }
}
)",
                     QUrl() );
  obj = std::unique_ptr<QObject>( component.create() );
  msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  ASSERT_NE( obj, nullptr ) << component.errorString().toStdString();
  fillMessage( *msg, QVariant::fromValue( obj.get() ) );
  TestArray test_array;
  test_array.bools = { true, false, true };
  test_array.uint8s = { 1, 2, 3, 5 };
  test_array.uint16s = { 13, 42, 1337 };
  test_array.uint32s = { 1, 3000, 50000000 };
  test_array.uint64s = { 5000000, 500000000000 };
  test_array.int8s = { 1, -2, 1, -1, 0, -1 };
  test_array.int16s = { 255, -255 };
  test_array.int32s = { 5000000, -5000000 };
  test_array.int64s = { 50000000000, -5000000000 };
  test_array.float32s = { 1.0, 3.14 };
  test_array.float64s = { 2.7 };
  test_array.times = { rclcpp::Time( 13 * 1E9 ) };
  test_array.durations = {
      rclcpp::Duration::from_seconds( 42 ),     rclcpp::Duration::from_seconds( 1.337 ),
      rclcpp::Duration::from_seconds( -2 ),     rclcpp::Duration::from_seconds( 0.001 ),
      rclcpp::Duration::from_seconds( -0.001 ), rclcpp::Duration::from_seconds( 0.002 ),
      rclcpp::Duration::from_seconds( -0.002 ), rclcpp::Duration::from_seconds( 0.003 ),
      rclcpp::Duration::from_seconds( -0.003 ), rclcpp::Duration::from_seconds( -0.004 ),
      rclcpp::Duration::from_seconds( 0.004 ),  rclcpp::Duration::from_seconds( 0.005 ) };
  test_array.strings = { "This", "is", "a", "test" };
  EXPECT_TRUE( messageEqual( msg->as<CompoundMessage>(), test_array ) );
}

TEST( MessageConversion, timeConversion )
{
  //  qml_ros2_plugin::WallTimeSingleton wall_time; // TODO
  //  qml_ros2_plugin::TimeSingleton time;
  //  // It should always round down to prevent issues with look ups into the future
  //  EXPECT_EQ( ros::Time( 13, 1000000 ), qmlToRos2Time( rosToQmlTime( ros::Time( 13, 1500000 ))));
  //  EXPECT_EQ( ros::Time( 13, 0 ), qmlToRos2Time( rosToQmlTime( ros::Time( 13, 900000 ))));
  //  EXPECT_EQ( qmlToRos2Time( QDateTime::fromMSecsSinceEpoch( 13042 )), ros::Time( 13, 42000000 ));
  //  EXPECT_EQ( rosToQmlTime( ros::Time( 1337, 800000 )), QDateTime::fromMSecsSinceEpoch( 1337000 ));
  //  EXPECT_EQ( rosToQmlTime( ros::Time( 1337, 499999 )), QDateTime::fromMSecsSinceEpoch( 1337000 ));
  //  EXPECT_EQ( rosToQmlTime( ros::WallTime( 1337, 800000 )), QDateTime::fromMSecsSinceEpoch( 1337000 ));
  //  EXPECT_EQ( rosToQmlTime( ros::WallTime( 1337, 499999 )), QDateTime::fromMSecsSinceEpoch( 1337000 ));
  //  EXPECT_DOUBLE_EQ( rosToQmlDuration( ros::Duration( 5, 42 )), 5000.000042 );
  //  EXPECT_DOUBLE_EQ( rosToQmlDuration( ros::Duration( -1337, -42 )), -1337000.000042 );
  //  EXPECT_EQ( qmlToRos2Duration( 2020 ), ros::Duration( 2, 20000000 ));
  //  EXPECT_EQ( qmlToRos2Duration( -1337.000420 ), ros::Duration( -1, -337000420 ));
  //
  //  // Create custom time objects
  //  EXPECT_EQ( time.create( 2.0 ).value<Time>().getRos2Time(), ros::Time( 2.0 ));
  //  EXPECT_EQ( time.create( 13, 37 ).value<Time>().getRos2Time(), ros::Time( 13, 37 ));
  //  EXPECT_EQ( wall_time.create( 2.0 ).value<WallTime>().getRos2Time(), ros::WallTime( 2.0 ));
  //  EXPECT_EQ( wall_time.create( 13, 37 ).value<WallTime>().getRos2Time(), ros::WallTime( 13, 37 ));
  //  ros::WallTime now = ros::WallTime::now();
  //  WallTime then = wall_time.now().value<WallTime>();
  //  EXPECT_GE( then.getRos2Time(), now );
  //  now = ros::WallTime::now();
  //  EXPECT_LE( then.getRos2Time(), now );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  return RUN_ALL_TESTS();
}
