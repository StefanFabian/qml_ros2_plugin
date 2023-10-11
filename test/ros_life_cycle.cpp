// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "common.hpp"

#include <qml_ros2_plugin/publisher.hpp>
#include <qml_ros2_plugin/ros2.hpp>
#include <qml_ros2_plugin/tf_transform_listener.hpp>
#include <qml_ros2_plugin/time.hpp>

#include <QCoreApplication>

using namespace qml_ros2_plugin;
using namespace std::chrono_literals;

//! @param wait_count Max time to wait in increments of 33 ms
bool waitFor( const std::function<bool()> &pred, int wait_count = 10 )
{
  while ( --wait_count > 0 ) {
    if ( pred() )
      return true;
    QCoreApplication::processEvents();
    std::this_thread::sleep_for( 33ms );
  }
  return false;
}

class MockRos2 : public QObjectRos2
{
public:
  void onRos2Initialized() override { initialized = true; }

  void onRos2Shutdown() override { initialized = false; }

  bool initialized = false;
};

TEST( Ros2LifeCycle, testLifeCycle )
{
  {
    MockRos2 mock_ros;
    qml_ros2_plugin::Ros2QmlSingletonWrapper ros_wrapper;
    qml_ros2_plugin::TfTransformListenerWrapper tf_wrapper;
    EXPECT_FALSE( ros_wrapper.isInitialized() );
    EXPECT_FALSE( Ros2Qml::getInstance().isInitialized() );
    EXPECT_FALSE( qml_ros2_plugin::TfTransformListener::getInstance().isInitialized() );
    EXPECT_EQ( tf_wrapper.lookUpTransform( "frame2", "frame1" )["exception"].toString(),
               QString( "Uninitialized" ) );
    EXPECT_EQ(
        tf_wrapper
            .lookUpTransform( "frame2", QDateTime(), "frame1", QDateTime(), "frame1" )["exception"]
            .toString(),
        QString( "Uninitialized" ) );
    auto publisher_before_init = dynamic_cast<qml_ros2_plugin::Publisher *>(
        ros_wrapper.createPublisher( "/pose_stamped", "geometry_msgs/PoseStamped", 10 ) );
    ASSERT_NE( publisher_before_init, nullptr );
    QCoreApplication::processEvents();
    EXPECT_FALSE( publisher_before_init->isAdvertised() );
    QCoreApplication::processEvents();
    EXPECT_FALSE( mock_ros.initialized );

    // Initialize ROS
    ros_wrapper.init( "test_ros_life_cycle" );
    int wait_count = 0;
    while ( true ) {
      if ( ++wait_count > 10 )
        FAIL() << "Initialization did not finish in time!";
      if ( publisher_before_init->isAdvertised() )
        break;
      QCoreApplication::processEvents();
      std::this_thread::sleep_for( 5ms );
    }
    EXPECT_TRUE( ros_wrapper.isInitialized() );
    EXPECT_TRUE( Ros2Qml::getInstance().isInitialized() );
    EXPECT_TRUE( ros_wrapper.ok() );
    EXPECT_EQ( tf_wrapper.lookUpTransform( "frame2", "frame1" )["exception"].toString(),
               QString( "LookupException" ) );
    EXPECT_TRUE( qml_ros2_plugin::TfTransformListener::getInstance().isInitialized() );
    QCoreApplication::processEvents();

    auto publisher_after_init = dynamic_cast<qml_ros2_plugin::Publisher *>(
        ros_wrapper.createPublisher( "/pose_stamped2", "geometry_msgs/PoseStamped", 10 ) );
    EXPECT_TRUE( waitFor( [&]() { return publisher_after_init->isAdvertised(); } ) );
    QCoreApplication::processEvents();
    EXPECT_TRUE( mock_ros.initialized );
    delete publisher_before_init;
    delete publisher_after_init;
  }
  // Wrappers are destroyed, this should clear node etc.
  EXPECT_FALSE( qml_ros2_plugin::Ros2Qml::getInstance().isInitialized() );
  EXPECT_FALSE( qml_ros2_plugin::TfTransformListener::getInstance().isInitialized() );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  rclcpp::init( argc, argv );
  return RUN_ALL_TESTS();
}
