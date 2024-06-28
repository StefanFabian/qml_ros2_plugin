// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TEST_COMMON_H
#define ROS_BABEL_FISH_TEST_COMMON_H

#include <ros_babel_fish_test_msgs/msg/test_array.hpp>

#include <QString>
#include <gtest/gtest.h>
#include <random>
#include <rclcpp/time.hpp>
#include <vector>

QT_BEGIN_NAMESPACE
inline void PrintTo( const QString &qString, ::std::ostream *os )
{
  *os << qUtf8Printable( qString );
}

QT_END_NAMESPACE

std::ostream &operator<<( std::ostream &stream, const QString &value )
{
  stream << value.toStdString();
  return stream;
}

std::ostream &operator<<( std::ostream &stream, const rclcpp::Time &value )
{
  stream << value.seconds();
  return stream;
}

std::ostream &operator<<( std::ostream &stream, const rclcpp::Duration &value )
{
  stream << value.seconds();
  return stream;
}

template<typename T>
void fillArray( std::vector<T> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  typedef
      typename std::conditional<std::is_floating_point<T>::value, std::uniform_real_distribution<T>,
                                std::uniform_int_distribution<T>>::type Distribution;
  Distribution distribution( std::numeric_limits<T>::min(), std::numeric_limits<T>::max() );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i ) { msg.push_back( distribution( generator ) ); }
}

template<typename T, size_t L>
void fillArray( rosidl_runtime_cpp::BoundedVector<T, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  typedef
      typename std::conditional<std::is_floating_point<T>::value, std::uniform_real_distribution<T>,
                                std::uniform_int_distribution<T>>::type Distribution;
  Distribution distribution( std::numeric_limits<T>::min(), std::numeric_limits<T>::max() );
  for ( size_t i = 0; i < L; ++i ) { msg.push_back( distribution( generator ) ); }
}

template<typename T, size_t L>
void fillArray( std::array<T, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  typedef
      typename std::conditional<std::is_floating_point<T>::value, std::uniform_real_distribution<T>,
                                std::uniform_int_distribution<T>>::type Distribution;
  Distribution distribution( std::numeric_limits<T>::min(), std::numeric_limits<T>::max() );
  for ( size_t i = 0; i < L; ++i ) { msg.at( i ) = distribution( generator ); }
}

template<>
void fillArray( std::vector<bool> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<uint8_t> distribution( 0, 1 );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i ) { msg.push_back( distribution( generator ) == 1 ); }
}

template<>
void fillArray<builtin_interfaces::msg::Time>( std::vector<builtin_interfaces::msg::Time> &msg,
                                               unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<int32_t> distribution( 0, 1000000 );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i ) {
    builtin_interfaces::msg::Time time;
    time.sec = distribution( generator );
    time.nanosec = distribution( generator );
    msg.emplace_back( time );
  }
}

template<size_t L>
void fillArray( std::array<builtin_interfaces::msg::Time, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( 0, 1E9 );
  for ( size_t i = 0; i < L; ++i ) {
    builtin_interfaces::msg::Time time;
    time.sec = distribution( generator );
    time.nanosec = distribution( generator );
    msg.at( i ) = time;
  }
}

template<>
void fillArray<builtin_interfaces::msg::Duration>( std::vector<builtin_interfaces::msg::Duration> &msg,
                                                   unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( -1E9, 1E9 );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i ) {
    builtin_interfaces::msg::Duration duration;
    duration.sec = distribution( generator );
    duration.nanosec = distribution( generator );
    msg.emplace_back( duration );
  }
}

template<size_t L>
void fillArray( std::array<builtin_interfaces::msg::Duration, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( -1E9, 1E9 );
  for ( size_t i = 0; i < L; ++i ) {
    builtin_interfaces::msg::Duration duration;
    duration.sec = distribution( generator );
    duration.nanosec = distribution( generator );
    msg.at( i ) = duration;
  }
}

std::string randomString( unsigned seed, int length = -1 )
{
  std::default_random_engine generator( seed );
  static const char alphanum[] = "0123456789"
                                 "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                 "abcdefghijklmnopqrstuvwxyz";
  std::uniform_int_distribution<size_t> distribution( 0, sizeof( alphanum ) - 2 );
  if ( length == -1 ) {
    std::uniform_int_distribution<int> length_distribution( 1, 1000 );
    length = length_distribution( generator );
  }
  std::string result( length, ' ' );
  for ( int i = 0; i < length; ++i ) { result[i] = alphanum[distribution( generator )]; }
  return result;
}

template<>
void fillArray<std::string>( std::vector<std::string> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min() );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i ) {
    msg.push_back( randomString( distribution( generator ), i == 0 ? 1 : -1 ) );
  }
}

template<size_t L>
void fillArray( rosidl_runtime_cpp::BoundedVector<std::string, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min() );
  std::uniform_int_distribution<size_t> length_distribution( 1, L );
  size_t length = length_distribution( generator );
  for ( size_t i = 0; i < length; ++i ) {
    msg.push_back( randomString( distribution( generator ), i == 0 ? 1 : -1 ) );
  }
}

template<size_t L>
void fillArray( std::array<std::string, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min() );
  for ( size_t i = 0; i < msg.length(); ++i ) {
    msg.at( i ) = randomString( distribution( generator ), i == 0 ? 1 : -1 );
  }
}

template<>
void fillArray<ros_babel_fish_test_msgs::msg::TestSubArray>(
    std::vector<ros_babel_fish_test_msgs::msg::TestSubArray> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min() );
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.reserve( length );
  for ( size_t i = 0; i < length; ++i ) {
    ros_babel_fish_test_msgs::msg::TestSubArray message;
    fillArray( message.ints, seed++ );
    fillArray( message.strings, seed++ );
    fillArray( message.times, seed++ );
    msg.push_back( message );
  }
}

template<size_t L>
void fillArray( std::array<ros_babel_fish_test_msgs::msg::TestSubArray, L> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min() );
  for ( size_t i = 0; i < L; ++i ) {
    fillArray( msg[i].ints, seed++ );
    fillArray( msg[i].strings, seed++ );
    fillArray( msg[i].times, seed++ );
  }
}

#endif // ROS_BABEL_FISH_TEST_COMMON_H
