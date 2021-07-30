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

#include <qml_ros2_plugin/logger.hpp>
#include <qml_ros2_plugin/ros2.hpp>

#include <rcl_interfaces/msg/log.hpp>

#include <QCoreApplication>
#include <QJSEngine>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

using namespace qml_ros2_plugin;

rclcpp::Node::SharedPtr node;

bool waitFor( const std::function<bool()> &pred )
{
  using namespace std::chrono_literals;
  int wait_count = 0;
  while ( ++wait_count < 10 ) {
    if ( pred() )
      return true;
    QCoreApplication::processEvents();
    rclcpp::spin_some( node );
    std::this_thread::sleep_for( 33ms );
  }
  return false;
}

TEST( Logging, log )
{
  QJSEngine engine;
  auto *wrapper = new Ros2QmlSingletonWrapper;
  wrapper->init( "test_logging_qml" );
  engine.newQObject( wrapper );

  EXPECT_EQ( static_cast<RCUTILS_LOG_SEVERITY>( ros2_logger_levels::Debug ),
             RCUTILS_LOG_SEVERITY_DEBUG );
  EXPECT_EQ( static_cast<RCUTILS_LOG_SEVERITY>( ros2_logger_levels::Info ),
             RCUTILS_LOG_SEVERITY_INFO );
  EXPECT_EQ( static_cast<RCUTILS_LOG_SEVERITY>( ros2_logger_levels::Warn ),
             RCUTILS_LOG_SEVERITY_WARN );
  EXPECT_EQ( static_cast<RCUTILS_LOG_SEVERITY>( ros2_logger_levels::Error ),
             RCUTILS_LOG_SEVERITY_ERROR );
  EXPECT_EQ( static_cast<RCUTILS_LOG_SEVERITY>( ros2_logger_levels::Fatal ),
             RCUTILS_LOG_SEVERITY_FATAL );

  std::vector<std::pair<std::string, int>> log;
  auto rosout_sub = node->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 0, [&log]( std::shared_ptr<const rcl_interfaces::msg::Log> msg ) {
        if ( msg->line != 0 )
          return;
        log.emplace_back( msg->msg, msg->level );
      } );
  ASSERT_TRUE( wrapper->debug().isCallable() );
  wrapper->debug().call( { QJSValue( "Debug Message" ) } );
  EXPECT_FALSE( waitFor( [&log]() { return !log.empty(); } ) )
      << "Default level should not show debug messages.";
  {
    qml_ros2_plugin::Logger *logger = dynamic_cast<qml_ros2_plugin::Logger *>( wrapper->getLogger() );
    ASSERT_NE( logger, nullptr );
    ASSERT_TRUE( logger->setLoggerLevel( ros2_logger_levels::Debug ) )
        << "Failed to set logging level.";
    delete logger;
  }

  QJSValue result = wrapper->debug().call( { QJSValue( "Debug Message" ) } );
  ASSERT_FALSE( result.isError() );
  ASSERT_TRUE( waitFor( [&log]() { return !log.empty(); } ) );
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Debug Message" );
  EXPECT_EQ( log[0].second, rcl_interfaces::msg::Log::DEBUG );
  log.clear();

  result = wrapper->info().call( { QJSValue( "Info Message" ) } );
  ASSERT_FALSE( result.isError() );
  ASSERT_TRUE( waitFor( [&log]() { return !log.empty(); } ) );
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Info Message" );
  EXPECT_EQ( log[0].second, rcl_interfaces::msg::Log::INFO );
  log.clear();

  result = wrapper->warn().call( { QJSValue( "Warn Message" ) } );
  ASSERT_FALSE( result.isError() );
  ASSERT_TRUE( waitFor( [&log]() { return !log.empty(); } ) );
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Warn Message" );
  EXPECT_EQ( log[0].second, rcl_interfaces::msg::Log::WARN );
  log.clear();

  result = wrapper->error().call( { QJSValue( "Error Message" ) } );
  ASSERT_FALSE( result.isError() );
  ASSERT_TRUE( waitFor( [&log]() { return !log.empty(); } ) );
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Error Message" );
  EXPECT_EQ( log[0].second, rcl_interfaces::msg::Log::ERROR );
  log.clear();

  result = wrapper->fatal().call( { QJSValue( "Fatal Message" ) } );
  ASSERT_FALSE( result.isError() );
  ASSERT_TRUE( waitFor( [&log]() { return !log.empty(); } ) );
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Fatal Message" );
  EXPECT_EQ( log[0].second, rcl_interfaces::msg::Log::FATAL );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  QCoreApplication app( argc, argv );
  node = rclcpp::Node::make_shared( "test_logging" );
  int result = RUN_ALL_TESTS();
  node.reset();
  return result;
}
