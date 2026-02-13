// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "message_comparison.hpp"

#include <qml_ros2_plugin/image_transport_subscription.hpp>
#include <qml_ros2_plugin/ros2.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <QCoreApplication>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

rclcpp::executors::SingleThreadedExecutor::UniquePtr executor;
rclcpp::Node::SharedPtr node;

void processSomeEvents( int n = 100, int sleep_duration_us = 1000 )
{
  for ( int i = 0; i < n; ++i ) {
    usleep( sleep_duration_us );
    QCoreApplication::processEvents();
    executor->spin_some( 1ms );
  }
}

//! @param wait_count Max time to wait in increments of 100 ms
bool waitFor( const std::function<bool()> &pred, int wait_count = 50 )
{
  while ( --wait_count > 0 ) {
    if ( pred() )
      return true;
    processSomeEvents();
  }
  return false;
}

::testing::AssertionResult compareImage( const uint8_t *data, const std::vector<uint8_t> &reference )
{
  for ( size_t i = 0; i < reference.size(); ++i ) {
    if ( data[i] != reference[i] )
      return ::testing::AssertionFailure() << "Image differed at i=" << i << "." << std::endl
                                           << "data[i]: " << static_cast<int>( data[i] ) << std::endl
                                           << "reference[i]: " << static_cast<int>( reference[i] );
  }
  return ::testing::AssertionSuccess();
}

struct MockSurface : public QAbstractVideoSurface {
  QList<QVideoFrame::PixelFormat> supportedPixelFormats( QAbstractVideoBuffer::HandleType ) const override
  {
    return { QVideoFrame::Format_RGB24 };
  }

  bool present( const QVideoFrame &frame ) override
  {
    last_frame = frame;
    return true;
  }

  QVideoFrame last_frame;
};

struct MockNoFormatSurface : public QAbstractVideoSurface {
  QList<QVideoFrame::PixelFormat> supportedPixelFormats( QAbstractVideoBuffer::HandleType ) const override
  {
    return {};
  }

  bool isFormatSupported( const QVideoSurfaceFormat & ) const override { return false; }

  bool present( const QVideoFrame &frame ) override
  {
    last_frame = frame;
    return true;
  }

  QVideoFrame last_frame;
};

TEST( ImageTransportSubscription, testCorrectFormat )
{
  auto img_pub = node->create_publisher<sensor_msgs::msg::Image>( "qml_ros2_plugin_test_image", 10 );
  ImageTransportSubscription subscriber( "test", 10 );
  EXPECT_EQ( subscriber.topic(), "test" );
  subscriber.setTopic( "qml_ros2_plugin_test_image" );
  // Before subscribing the topic name is not resolved.
  EXPECT_EQ( subscriber.topic(), "qml_ros2_plugin_test_image" );
  EXPECT_EQ( subscriber.defaultTransport(), "compressed" ); // compressed is default
  subscriber.setDefaultTransport( "raw" );
  EXPECT_EQ( subscriber.defaultTransport(), "raw" );
  MockSurface mock_surface;
  subscriber.setVideoSurface( &mock_surface );
  processSomeEvents();
  EXPECT_EQ( subscriber.topic().toStdString(), img_pub->get_topic_name() );
  ASSERT_TRUE( waitFor( [&img_pub]() { return img_pub->get_subscription_count() == 1U; } ) );

  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 3;
  image->encoding = sensor_msgs::image_encodings::RGB8;
  image->data = { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 };
  img_pub->publish( *image );
  processSomeEvents();

  ASSERT_TRUE( waitFor( [&mock_surface] { return mock_surface.last_frame.isValid(); }, 100 ) );
  ASSERT_EQ( mock_surface.last_frame.pixelFormat(), QVideoFrame::Format_RGB24 );
  EXPECT_TRUE( mock_surface.last_frame.map( QAbstractVideoBuffer::MapMode::ReadOnly ) );
  EXPECT_EQ( mock_surface.last_frame.mappedBytes(), 3 * 3 * 2 );
  EXPECT_EQ( mock_surface.last_frame.bytesPerLine(), 3 * 2 );
  EXPECT_TRUE( compareImage( mock_surface.last_frame.bits(), image->data ) );
}

TEST( ImageTransportSubscription, testWrongFormat )
{
  auto img_pub =
      node->create_publisher<sensor_msgs::msg::Image>( "qml_ros2_plugin_test_wrong_image", 10 );
  ImageTransportSubscription subscriber( "qml_ros2_plugin_test_wrong_image", 10 );
  // Before subscribing the topic name is not resolved.
  EXPECT_EQ( subscriber.topic(), "qml_ros2_plugin_test_wrong_image" );
  subscriber.setDefaultTransport( "raw" );
  EXPECT_EQ( subscriber.defaultTransport(), "raw" );
  MockNoFormatSurface mock_surface;
  subscriber.setVideoSurface( &mock_surface );
  processSomeEvents();
  EXPECT_EQ( subscriber.topic().toStdString(), img_pub->get_topic_name() );
  ASSERT_EQ( img_pub->get_subscription_count(), 1U );

  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 3;
  image->encoding = sensor_msgs::image_encodings::RGB8;
  image->data = { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 };
  img_pub->publish( *image );
  EXPECT_FALSE( mock_surface.last_frame.isValid() );
  EXPECT_TRUE( waitFor( [&subscriber]() { return !subscriber.subscribed(); }, 30 ) );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  rclcpp::init( argc, argv );
  node = rclcpp::Node::make_shared( "test_image_transport_subscriber" );
  executor = rclcpp::executors::SingleThreadedExecutor::make_unique();
  executor->add_node( node );
  Ros2Qml::getInstance().init( "test_image_transport_subscriber_qml" );
  Ros2Qml::getInstance().registerDependant();
  int result = RUN_ALL_TESTS();
  executor.reset();
  node.reset();
  Ros2Qml::getInstance().unregisterDependant();
  Ros2Qml::getInstance().shutdown();
  rclcpp::shutdown();
  return result;
}
