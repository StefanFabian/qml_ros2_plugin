// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/conversion/message_conversions.hpp"

#include "qml_ros2_plugin/array.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"
#include "qml_ros2_plugin/time.hpp"

#include <QAbstractListModel>
#include <QDateTime>
#include <QMetaProperty>
#include <QQuaternion>
#include <QUrl>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>

using namespace ros_babel_fish;

namespace qml_ros2_plugin
{
namespace conversion
{

QVariantMap msgToMap( const std_msgs::msg::Header &msg )
{
  QVariantMap result;
  result.insert( "frame_id", QString::fromStdString( msg.frame_id ) );
  result.insert( "stamp", QVariant::fromValue( Time( msg.stamp ) ) );
  return result;
}

QVariantMap msgToMap( const geometry_msgs::msg::Transform &msg )
{
  QVariantMap result;
  result.insert( "translation", QVariant::fromValue( msgToMap( msg.translation ) ) );
  result.insert( "rotation", QVariant::fromValue( msgToMap( msg.rotation ) ) );
  return result;
}

QVariantMap msgToMap( const geometry_msgs::msg::TransformStamped &msg )
{
  QVariantMap result;
  result.insert( "header", QVariant::fromValue( msgToMap( msg.header ) ) );
  result.insert( "child_frame_id", QString::fromStdString( msg.child_frame_id ) );
  result.insert( "transform", QVariant::fromValue( msgToMap( msg.transform ) ) );
  return result;
}

QVariantMap msgToMap( const geometry_msgs::msg::Vector3 &msg )
{
  QVariantMap result;
  result.insert( "x", msg.x );
  result.insert( "y", msg.y );
  result.insert( "z", msg.z );
  return result;
}

QVariantMap msgToMap( const geometry_msgs::msg::Quaternion &msg )
{
  QVariantMap result;
  result.insert( "w", msg.w );
  result.insert( "x", msg.x );
  result.insert( "y", msg.y );
  result.insert( "z", msg.z );
  return result;
}

QString uuidToString( const rclcpp_action::GoalUUID &uuid )
{
  static constexpr char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
  static_assert( std::is_same<std::remove_const<std::remove_reference<decltype( uuid )>::type>::type,
                              std::array<uint8_t, 16>>::value,
                 "The UUID definition was changed. Please open an issue!" );
  QString result_uuid;
  result_uuid.resize( 36 );
  int input_index = 0;
  int output_index = -1;
  for ( ; input_index < 4; ++input_index ) {
    result_uuid[++output_index] = hex[uuid[input_index] & 0xf];
    result_uuid[++output_index] = hex[( uuid[input_index] >> 4 ) & 0xf];
  }
  result_uuid[++output_index] = '-';
  for ( ; input_index < 6; ++input_index ) {
    result_uuid[++output_index] = hex[uuid[input_index] & 0xf];
    result_uuid[++output_index] = hex[( uuid[input_index] >> 4 ) & 0xf];
  }
  result_uuid[++output_index] = '-';
  for ( ; input_index < 8; ++input_index ) {
    result_uuid[++output_index] = hex[uuid[input_index] & 0xf];
    result_uuid[++output_index] = hex[( uuid[input_index] >> 4 ) & 0xf];
  }
  result_uuid[++output_index] = '-';
  for ( ; input_index < 10; ++input_index ) {
    result_uuid[++output_index] = hex[uuid[input_index] & 0xf];
    result_uuid[++output_index] = hex[( uuid[input_index] >> 4 ) & 0xf];
  }
  result_uuid[++output_index] = '-';
  for ( ; input_index < 16; ++input_index ) {
    result_uuid[++output_index] = hex[uuid[input_index] & 0xf];
    result_uuid[++output_index] = hex[( uuid[input_index] >> 4 ) & 0xf];
  }
  return result_uuid;
}

QVariantMap msgToMap( const unique_identifier_msgs::msg::UUID &msg )
{
  QVariantMap result;
  result.insert( "uuid", uuidToString( msg.uuid ) );
  return result;
}

QVariantMap msgToMap( const action_msgs::msg::GoalInfo &msg )
{
  QVariantMap result;
  result.insert( "goal_id", QVariant::fromValue( msgToMap( msg.goal_id ) ) );
  result.insert( "stamp", QVariant::fromValue( Time( msg.stamp ) ) );
  return result;
}

QVariantMap msgToMap( const action_msgs::msg::GoalStatus &msg )
{
  QVariantMap result;
  result.insert( "goal_info", QVariant::fromValue( msgToMap( msg.goal_info ) ) );
  result.insert( "status", msg.status );
  return result;
}

namespace
{
struct MessageToQVariantConverter {
  template<typename T>
  QVariant operator()( const ValueMessage<T> &msg )
  {
    return QVariant::fromValue( msg.getValue() );
  }

  QVariant operator()( const ValueMessage<std::string> &msg )
  {
    return QVariant::fromValue( QString::fromStdString( msg.getValue() ) );
  }

  QVariant operator()( const ValueMessage<std::wstring> &msg )
  {
    return QVariant::fromValue( QString::fromStdWString( msg.getValue() ) );
  }

  QVariant operator()( const ValueMessage<uint8_t> &msg )
  {
    return QVariant::fromValue( uint( msg.getValue() ) );
  }

  QVariant operator()( const ValueMessage<int8_t> &msg )
  {
    return QVariant::fromValue( int( msg.getValue() ) );
  }

  QVariant operator()( const ValueMessage<long double> &msg )
  {
    return QVariant::fromValue( static_cast<double>( msg.getValue() ) );
  }

  QVariant operator()( const ValueMessage<char16_t> &msg )
  {
    return QVariant::fromValue( QChar( msg.getValue() ) );
  }
};
} // namespace

QVariant msgToMap( const Message::ConstSharedPtr &msg )
{
  if ( msg->type() == MessageTypes::Compound ) {
    QVariantMap result;
    const auto &compound = msg->as<CompoundMessage>();
    // Special cases for time and duration
    if ( compound.datatype() == "builtin_interfaces::msg::Time" ) {
      return QVariant::fromValue( Time( compound.value<rclcpp::Time>() ) );
    }
    if ( compound.datatype() == "builtin_interfaces::msg::Duration" ) {
      return QVariant::fromValue( Duration( compound.value<rclcpp::Duration>() ) );
    }
    const auto &keys = compound.keys();
    const auto &values = compound.values();
    for ( size_t i = 0; i < keys.size(); ++i ) {
      result.insert( QString::fromStdString( keys[i] ), msgToMap( values[i] ) );
    }
    return result;
  } else if ( msg->type() == MessageTypes::Array ) {
    return QVariant::fromValue( Array( std::dynamic_pointer_cast<const ArrayMessageBase>( msg ) ) );
  }

  return invoke_for_value_message( *msg, MessageToQVariantConverter{} );
}

namespace
{

struct ArrayToQVariantListConverter {

  template<typename T, bool BOUNDED, bool FIXED_LENGTH>
  QVariantList operator()( const ArrayMessage_<T, BOUNDED, FIXED_LENGTH> &array )
  {
    QVariantList result;
    result.reserve( array.size() );
    for ( size_t i = 0; i < array.size(); ++i ) {
      result.append( QVariant::fromValue( array[i] ) );
    }
    return result;
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  QVariantList operator()( const ArrayMessage_<std::string, BOUNDED, FIXED_LENGTH> &array )
  {
    QVariantList result;
    result.reserve( array.size() );
    for ( size_t i = 0; i < array.size(); ++i ) {
      result.append( QVariant::fromValue( QString::fromStdString( array[i] ) ) );
    }
    return result;
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  QVariantList operator()( const ArrayMessage_<std::wstring, BOUNDED, FIXED_LENGTH> &array )
  {
    QVariantList result;
    result.reserve( array.size() );
    for ( size_t i = 0; i < array.size(); ++i ) {
      result.append( QVariant::fromValue( QString::fromStdWString( array[i] ) ) );
    }
    return result;
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  QVariantList operator()( const ArrayMessage_<uint8_t, BOUNDED, FIXED_LENGTH> &array )
  {
    QVariantList result;
    result.reserve( array.size() );
    for ( size_t i = 0; i < array.size(); ++i ) {
      result.append( QVariant::fromValue( uint( array[i] ) ) );
    }
    return result;
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  QVariantList operator()( const ArrayMessage_<int8_t, BOUNDED, FIXED_LENGTH> &array )
  {
    QVariantList result;
    result.reserve( array.size() );
    for ( size_t i = 0; i < array.size(); ++i ) {
      result.append( QVariant::fromValue( int( array[i] ) ) );
    }
    return result;
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  QVariantList operator()( const ArrayMessage_<long double, BOUNDED, FIXED_LENGTH> &array )
  {
    QVariantList result;
    result.reserve( array.size() );
    for ( size_t i = 0; i < array.size(); ++i ) {
      result.append( QVariant::fromValue( static_cast<double>( array[i] ) ) );
    }
    return result;
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  QVariantList operator()( const ArrayMessage_<char16_t, BOUNDED, FIXED_LENGTH> &array )
  {
    QVariantList result;
    result.reserve( array.size() );
    for ( size_t i = 0; i < array.size(); ++i ) {
      result.append( QVariant::fromValue( QChar( array[i] ) ) );
    }
    return result;
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  QVariantList operator()( const CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &array )
  {
    QVariantList result;
    result.reserve( array.size() );
    for ( const auto &val : array.values() ) {
      result.append( QVariant::fromValue( msgToMap( val ) ) );
    }
    return result;
  }
};
} // namespace

QVariant msgToMap( const Message &msg )
{
  if ( msg.type() == MessageTypes::Compound ) {
    QVariantMap result;
    const auto &compound = msg.as<CompoundMessage>();
    // Special cases for time and duration
    if ( compound.datatype() == "builtin_interfaces::msg::Time" ) {
      return QVariant::fromValue( Time( compound.value<rclcpp::Time>() ) );
    }
    if ( compound.datatype() == "builtin_interfaces::msg::Duration" ) {
      return QVariant::fromValue( Duration( compound.value<rclcpp::Duration>() ) );
    }
    const auto &keys = compound.keys();
    const auto &values = compound.values();
    for ( size_t i = 0; i < keys.size(); ++i ) {
      result.insert( QString::fromStdString( keys[i] ), msgToMap( values[i] ) );
    }
    return result;
  } else if ( msg.type() == MessageTypes::Array ) {
    auto &arr = msg.as<ArrayMessageBase>();
    return invoke_for_array_message( arr, ArrayToQVariantListConverter{} );
  }

  return invoke_for_value_message( msg, MessageToQVariantConverter{} );
}

namespace
{

template<typename T>
using limits = std::numeric_limits<T>;

template<typename TargetType, typename ValueType>
typename std::enable_if<std::is_same<ValueType, bool>::value, bool>::type isCompatible( const bool & )
{
  return std::is_same<TargetType, bool>::value;
}

template<typename TargetType, typename ValueType>
typename std::enable_if<std::is_integral<TargetType>::value && std::is_integral<ValueType>::value &&
                            std::is_signed<TargetType>::value == std::is_signed<ValueType>::value &&
                            !std::is_same<ValueType, bool>::value,
                        bool>::type
isCompatible( const ValueType &value )
{
  return std::numeric_limits<TargetType>::min() <= value &&
         value <= std::numeric_limits<TargetType>::max();
}

template<typename TargetType, typename ValueType>
typename std::enable_if<std::is_integral<TargetType>::value && std::is_integral<ValueType>::value &&
                            std::is_signed<TargetType>::value && !std::is_signed<ValueType>::value &&
                            !std::is_same<ValueType, bool>::value,
                        bool>::type
isCompatible( const ValueType &value )
{
  return value <= static_cast<typename std::make_unsigned<TargetType>::type>(
                      std::numeric_limits<TargetType>::max() );
}

template<typename TargetType, typename ValueType>
typename std::enable_if<std::is_integral<TargetType>::value && std::is_integral<ValueType>::value &&
                            !std::is_signed<TargetType>::value && std::is_signed<ValueType>::value &&
                            !std::is_same<ValueType, bool>::value,
                        bool>::type
isCompatible( const ValueType &value )
{
  if ( value < 0 )
    return false;
  return static_cast<typename std::make_unsigned<ValueType>::type>( value ) <=
         std::numeric_limits<TargetType>::max();
}

template<typename TargetType, typename ValueType>
typename std::enable_if<
    std::is_floating_point<TargetType>::value && std::is_arithmetic<ValueType>::value, bool>::type
isCompatible( const ValueType & )
{
  return true;
}

template<typename TargetType, typename ValueType>
typename std::enable_if<std::is_integral<TargetType>::value && std::is_floating_point<ValueType>::value,
                        bool>::type
isCompatible( const ValueType &value )
{
  if ( std::abs( value - std::round( value ) ) > 1E-12 )
    return false;
  return std::numeric_limits<TargetType>::min() <= value &&
         value <= std::numeric_limits<TargetType>::max();
}

template<typename ValueType>
struct ValueSetter {
  template<typename MessageType>
  typename std::enable_if<std::is_assignable<MessageType &, ValueType>::value, bool>::type
  operator()( ValueMessage<MessageType> &msg, const ValueType &value )
  {
    if ( !isCompatible<MessageType, ValueType>( value ) ) {
      QML_ROS2_PLUGIN_WARN( "Tried to fill '%s' field with incompatible type!",
                            typeid( ValueType ).name() );
      return false;
    }
    msg.setValue( value );
    return true;
  }

  template<typename MessageType>
  typename std::enable_if<!std::is_assignable<MessageType &, ValueType>::value, bool>::type
  operator()( ValueMessage<MessageType> &, const ValueType & )
  {
    return false;
  }

  template<typename MessageType>
  typename std::enable_if<std::is_same<MessageType, ValueType>::value, bool>::type
  setIfSameType( ValueMessage<MessageType> &msg, ValueType value )
  {
    msg.setValue( value );
    return true;
  }

  template<typename MessageType>
  typename std::enable_if<!std::is_same<MessageType, ValueType>::value, bool>::type
  setIfSameType( ValueMessage<MessageType> &, ValueType )
  {
    return false;
  }

  bool operator()( ValueMessage<bool> &msg, ValueType value )
  {
    return setIfSameType( msg, value );
  }

  bool operator()( ValueMessage<std::string> &msg, const ValueType &value )
  {
    return setIfSameType( msg, value );
  }

  bool operator()( ValueMessage<std::wstring> &msg, const ValueType &value )
  {
    return setIfSameType( msg, value );
  }

  template<typename T = ValueType>
  typename std::enable_if<std::is_assignable<double &, T>::value, bool>::type
  operator()( CompoundMessage &msg, const ValueType &value )
  {
    if ( msg.isDuration() ) {
      // Value is milliseconds
      msg = rclcpp::Duration( std::chrono::nanoseconds( static_cast<int64_t>( value * 1E6 ) ) );
      return true;
    }
    if ( msg.isTime() ) {
      msg = rclcpp::Time( static_cast<int64_t>( value * 1E6 ) );
      return true;
    }
    return false;
  }

  template<typename T = ValueType>
  typename std::enable_if<!std::is_assignable<double &, T>::value, bool>::type
  operator()( CompoundMessage &, const ValueType & )
  {
    return false;
  }

  bool operator()( ArrayMessageBase &, const ValueType & ) { return false; }
};

template<typename ValueType>
bool fillValue( Message &msg, const ValueType &value )
{
  return invoke_for_message( msg, ValueSetter<ValueType>{}, value );
}

template<>
bool fillValue<rclcpp::Time>( Message &msg, const rclcpp::Time &value )
{
  if ( !msg.isTime() ) {
    QML_ROS2_PLUGIN_WARN( "Tried to put Time into field with incompatible type!" );
    return false;
  }
  msg = value;
  return true;
}

template<>
bool fillValue<rclcpp::Duration>( Message &msg, const rclcpp::Duration &value )
{
  if ( !msg.isDuration() ) {
    QML_ROS2_PLUGIN_WARN( "Tried to put Duration into field with incompatible type!" );
    return false;
  }
  msg = value;
  return true;
}

template<typename TBounds, typename TVal>
typename std::enable_if<std::is_signed<TVal>::value, bool>::type inBounds( TVal val )
{
  typedef typename std::make_signed<TBounds>::type STBounds;
  typedef typename std::make_unsigned<TBounds>::type UTBounds;
  typedef typename std::make_unsigned<TVal>::type UTVal;
  // val is in bounds of a signed target type if it is greater than the min of the signed type and either smaller than
  // zero or smaller than its max. If val is greater or equal to zero, the val and max are casted to unsigned types to
  // prevent compiler warnings if the TVal and TBounds are not both signed or both unsigned.
  return static_cast<STBounds>( std::numeric_limits<TBounds>::min() ) <= val &&
         ( val < 0 || static_cast<UTBounds>( val ) <=
                          static_cast<UTVal>( std::numeric_limits<TBounds>::max() ) );
}

template<typename TBounds, typename TVal>
typename std::enable_if<std::is_unsigned<TVal>::value, bool>::type inBounds( TVal val )
{
  return val <= static_cast<typename std::make_unsigned<TBounds>::type>(
                    std::numeric_limits<TBounds>::max() );
}

// Default implementation is for all integer types
template<typename T>
bool isCompatible( const QVariant &variant )
{
  switch ( (int)variant.type() ) {
  case QMetaType::UChar: {
    auto val = variant.value<uint8_t>();
    return inBounds<T>( val );
  }
  case QMetaType::UShort: {
    auto val = variant.value<uint16_t>();
    return inBounds<T>( val );
  }
  case QVariant::UInt: {
    uint val = variant.toUInt();
    return inBounds<T>( val );
  }
  case QMetaType::ULong: {
    auto val = variant.value<unsigned long>();
    return inBounds<T>( val );
  }
  case QVariant::ULongLong: {
    qulonglong val = variant.toULongLong();
    return inBounds<T>( val );
  }
  case QMetaType::SChar:
  case QMetaType::Char: {
    auto val = variant.value<int8_t>();
    return inBounds<T>( val );
  }
  case QMetaType::Short: {
    auto val = variant.value<int16_t>();
    return inBounds<T>( val );
  }
  case QVariant::Int: {
    int val = variant.toInt();
    return inBounds<T>( val );
  }
  case QMetaType::Long: {
    auto val = variant.value<long>();
    return inBounds<T>( val );
  }
  case QVariant::LongLong: {
    qlonglong val = variant.toLongLong();
    return inBounds<T>( val );
  }
  case QMetaType::Float: {
    auto val = variant.value<float>();
    return val == std::round( val ) && std::numeric_limits<T>::min() <= val &&
           val <= std::numeric_limits<T>::max();
  }
  case QVariant::Double: {
    double val = variant.toDouble();
    return val == std::round( val ) && std::numeric_limits<T>::min() <= val &&
           val <= std::numeric_limits<T>::max();
  }
  default:
    break;
  }
  return false;
}

template<>
bool isCompatible<bool>( const QVariant &variant )
{
  return variant.type() == QVariant::Bool;
}

template<>
bool isCompatible<float>( const QVariant &variant )
{
  return (int)variant.type() == QMetaType::Float || variant.type() == QVariant::Double ||
         variant.type() == QVariant::UInt || variant.type() == QVariant::Int ||
         variant.type() == QVariant::ULongLong || variant.type() == QVariant::LongLong;
}

template<>
bool isCompatible<double>( const QVariant &variant )
{
  return (int)variant.type() == QMetaType::Float || variant.type() == QVariant::Double ||
         variant.type() == QVariant::UInt || variant.type() == QVariant::Int ||
         variant.type() == QVariant::ULongLong || variant.type() == QVariant::LongLong;
}

template<>
bool isCompatible<long double>( const QVariant &variant )
{
  return (int)variant.type() == QMetaType::Float || variant.type() == QVariant::Double ||
         variant.type() == QVariant::UInt || variant.type() == QVariant::Int ||
         variant.type() == QVariant::ULongLong || variant.type() == QVariant::LongLong;
}

template<>
bool isCompatible<std::string>( const QVariant &variant )
{
  return variant.canConvert<QString>();
}

template<>
bool isCompatible<std::wstring>( const QVariant &variant )
{
  return variant.canConvert<QString>();
}

template<>
bool isCompatible<rclcpp::Time>( const QVariant &variant )
{
  return variant.type() == QVariant::Double || variant.type() == QVariant::UInt ||
         variant.type() == QVariant::Int || variant.type() == QVariant::ULongLong ||
         variant.type() == QVariant::LongLong || variant.type() == QVariant::DateTime ||
         variant.typeName() == std::string( "qml_ros2_plugin::Time" );
}

template<>
bool isCompatible<rclcpp::Duration>( const QVariant &variant )
{
  return variant.type() == QVariant::Double || variant.type() == QVariant::UInt ||
         variant.type() == QVariant::Int || variant.type() == QVariant::ULongLong ||
         variant.type() == QVariant::LongLong ||
         variant.typeName() == std::string( "qml_ros2_plugin::Duration" );
}

template<typename T>
T getValue( const QVariant &variant )
{
  switch ( (int)variant.type() ) {
  case QVariant::Bool:
    return variant.toBool();
  case QMetaType::SChar:
    return static_cast<T>( variant.value<int8_t>() );
  case QMetaType::UChar:
    return static_cast<T>( variant.value<uint8_t>() );
  case QMetaType::Short:
    return static_cast<T>( variant.value<int16_t>() );
  case QMetaType::UShort:
    return static_cast<T>( variant.value<uint16_t>() );
  case QVariant::Int:
    return static_cast<T>( variant.toInt() );
  case QVariant::UInt:
    return static_cast<T>( variant.toUInt() );
  case QMetaType::Long:
    return static_cast<T>( variant.value<long>() );
  case QMetaType::ULong:
    return static_cast<T>( variant.value<unsigned long>() );
  case QVariant::LongLong:
    return static_cast<T>( variant.toLongLong() );
  case QVariant::ULongLong:
    return static_cast<T>( variant.toULongLong() );
  case QMetaType::Float:
    return static_cast<T>( variant.value<float>() );
  case QVariant::Double:
    return static_cast<T>( variant.toDouble() );
  default:
    QML_ROS2_PLUGIN_WARN( "Tried to get '%s' from incompatible type! Type: %s", typeid( T ).name(),
                          variant.typeName() );
  }
  return T();
}

template<>
std::string getValue<std::string>( const QVariant &variant )
{
  return variant.toString().toStdString();
}

template<>
std::wstring getValue<std::wstring>( const QVariant &variant )
{
  return variant.toString().toStdWString();
}

template<>
rclcpp::Time getValue<rclcpp::Time>( const QVariant &variant )
{
  switch ( variant.type() ) {
  case QVariant::Int:
    return qmlToRos2Time( variant.toInt() );
  case QVariant::UInt:
    return qmlToRos2Time( variant.toUInt() );
  case QVariant::LongLong:
    return qmlToRos2Time( variant.toLongLong() );
  case QVariant::ULongLong:
    return qmlToRos2Time( variant.toULongLong() );
  case QVariant::Double:
    return qmlToRos2Time( variant.toDouble() );
  case QVariant::DateTime:
    return qmlToRos2Time( variant.toDateTime().toMSecsSinceEpoch() );
  case QVariant::Date:
  case QVariant::Time:
  default:
    if ( variant.canConvert<Time>() )
      return variant.value<Time>().getTime();
    QML_ROS2_PLUGIN_WARN( "Tried to get rclcpp::Time from incompatible type! Type: %s",
                          variant.typeName() );
    return rclcpp::Time();
  }
}

template<>
rclcpp::Duration getValue<rclcpp::Duration>( const QVariant &variant )
{
  switch ( variant.type() ) {
  case QVariant::Int:
    return qmlToRos2Duration( variant.toInt() );
  case QVariant::UInt:
    return qmlToRos2Duration( variant.toUInt() );
  case QVariant::LongLong:
    return qmlToRos2Duration( variant.toLongLong() );
  case QVariant::ULongLong:
    return qmlToRos2Duration( variant.toULongLong() );
  case QVariant::Double:
    return qmlToRos2Duration( variant.toDouble() );
  case QVariant::Date:
  case QVariant::Time:
  case QVariant::DateTime:
  default:
    if ( variant.canConvert<Duration>() )
      return variant.value<Duration>().getDuration();
    QML_ROS2_PLUGIN_WARN( "Tried to get rclcpp::Duration from incompatible type! Type: %s",
                          variant.typeName() );
    return rclcpp::Duration( std::chrono::nanoseconds( 0 ) );
  }
}
} // namespace

bool fillMessage( Message &msg, const QVariant &value )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  return fillMessage( fish, msg, value );
}

namespace
{

template<bool BOUNDED, bool FIXED_LENGTH>
size_t limitCount( const ArrayMessageBase &array, int count )
{
  if ( ( BOUNDED || FIXED_LENGTH ) && static_cast<size_t>( count ) > array.maxSize() ) {
    QML_ROS2_PLUGIN_WARN(
        "Too many values for bounded or fixed size array (%d vs %lu)! Only using first %lu.", count,
        array.maxSize(), array.maxSize() );
    return array.maxSize();
  }
  return count;
}

struct QVariantListToMessageConverter {
  template<typename T, bool BOUNDED, bool FIXED_LENGTH, typename ArrayType>
  bool operator()( ArrayMessage_<T, BOUNDED, FIXED_LENGTH> &array, BabelFish &, const ArrayType &list )
  {
    int count = limitCount<BOUNDED, FIXED_LENGTH>( array, list.size() );
    bool no_error = count == list.size();
    if ( !FIXED_LENGTH )
      array.clear();
    for ( int i = 0; i < count; ++i ) {
      const QVariant &variant = list.at( i );
      if ( !isCompatible<T>( variant ) ) {
        QML_ROS2_PLUGIN_WARN(
            "Tried to fill array of '%s' with incompatible value! Skipped. (Type: %s)",
            typeid( T ).name(), variant.typeName() );
        no_error = false;
        continue;
      }
      if ( array.isFixedSize() )
        array.assign( i, getValue<T>( variant ) );
      else
        array.push_back( getValue<T>( variant ) );
    }
    return no_error;
  }

  template<bool BOUNDED, bool FIXED_LENGTH, typename Array>
  bool operator()( CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &array, BabelFish &fish,
                   const Array &list )
  {
    const int count = limitCount<BOUNDED, FIXED_LENGTH>( array, list.size() );
    bool no_error = count == list.size();
    if ( !FIXED_LENGTH )
      array.clear();
    for ( int i = 0; i < count; ++i ) {
      const QVariant &variant = list.at( static_cast<int>( i ) );
      auto &child = FIXED_LENGTH ? array[i] : array.appendEmpty();
      if ( variant.type() != QVariant::Map ) {
        if ( child.isTime() ) {
          if ( !isCompatible<rclcpp::Time>( variant ) ) {
            QML_ROS2_PLUGIN_WARN(
                "Tried to fill array of 'time' with incompatible value! Skipped. (Type: %s)",
                variant.typeName() );
            no_error = false;
            continue;
          }
          child = getValue<rclcpp::Time>( variant );
          continue;
        }
        if ( child.isDuration() ) {
          if ( !isCompatible<rclcpp::Duration>( variant ) ) {
            QML_ROS2_PLUGIN_WARN(
                "Tried to fill array of 'duration' with incompatible value! Skipped. (Type: %s)",
                variant.typeName() );
            no_error = false;
            continue;
          }
          child = getValue<rclcpp::Duration>( variant );
          continue;
        }
        const char *name = array.elementIntrospection()->name_;
        QML_ROS2_PLUGIN_WARN( "Tried to fill compound array '%s' with non-map value! Skipped.", name );
        if ( !FIXED_LENGTH )
          array.pop_back();
        std::vector<int> test;
        no_error = false;
        continue;
      }
      fillMessage( fish, child, variant );
    }
    return no_error;
  }
};

struct QAbstractListModelToMessageConverter {
  template<typename T, bool BOUNDED, bool FIXED_LENGTH>
  bool operator()( ArrayMessage_<T, BOUNDED, FIXED_LENGTH> &array, BabelFish &,
                   const QAbstractListModel &list )
  {
    int count = limitCount<BOUNDED, FIXED_LENGTH>( array, list.rowCount() );
    bool no_error = count == list.rowCount();
    if ( !FIXED_LENGTH )
      array.clear();
    for ( int i = 0; i < count; ++i ) {
      const QModelIndex &index = list.index( i, 0 );
      const QVariant &variant = list.data( index );
      if ( !isCompatible<T>( variant ) ) {
        QML_ROS2_PLUGIN_WARN(
            "Tried to fill array of '%s' with incompatible value! Skipped. (Type: %s)",
            typeid( T ).name(), variant.typeName() );
        no_error = false;
        continue;
      }
      if ( FIXED_LENGTH )
        array.assign( i, getValue<T>( variant ) );
      else
        array.push_back( getValue<T>( variant ) );
    }
    return no_error;
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  bool operator()( CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &array, BabelFish &fish,
                   const QAbstractListModel &list )
  {
    int count = limitCount<BOUNDED, FIXED_LENGTH>( array, list.rowCount() );
    QHash<int, QByteArray> roleNames = list.roleNames();
    if ( roleNames.empty() )
      return true;
    std::vector<std::string> names;
    {
      int max_key = 0;
      for ( auto key : roleNames.keys() ) max_key = std::max( max_key, key );
      names.resize( max_key + 1 );
    }
    // Collect keys
    QHashIterator<int, QByteArray> it( roleNames );
    while ( it.hasNext() ) {
      it.next();
      names[it.key()] = it.value().data();
    }
    bool no_error = count == list.rowCount();
    if ( !FIXED_LENGTH )
      array.clear();
    // Check that all keys are in message
    std::vector<std::string> compound_names;
    const auto &introspection = array.elementIntrospection();
    assert( introspection->members_->typesupport_identifier ==
            std::string( "rosidl_typesupport_introspection_cpp" ) );
    const auto *members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        introspection->members_->data );
    compound_names.reserve( members->member_count_ );
    for ( size_t i = 0; i < members->member_count_; ++i ) {
      compound_names.emplace_back( members->members_[i].name_ );
    }
    bool any_name_valid = false;
    for ( auto &name : names ) {
      const std::string &key = name;
      if ( key.empty() )
        continue;
      if ( std::find( compound_names.begin(), compound_names.end(), key ) == compound_names.end() ) {
        QML_ROS2_PLUGIN_DEBUG( "Message doesn't have field '%s'! Skipped.", key.c_str() );
        no_error = false;
        name = std::string();
        continue;
      }
      any_name_valid = true;
    }
    if ( !any_name_valid ) {
      if ( members->message_namespace_ != std::string( "builtin_interfaces::msg" ) )
        return no_error;
      if ( members->message_name_ == std::string( "Duration" ) ) {
        no_error = true;
        for ( int i = 0; i < count; ++i ) {
          QModelIndex index = list.index( i, 0 );
          const QVariant &variant = list.data( index );
          if ( !isCompatible<rclcpp::Duration>( variant ) ) {
            QML_ROS2_PLUGIN_WARN(
                "Tried to fill array of '%s::%s' with incompatible value! Skipped. (Type: %s)",
                members->message_namespace_, members->message_name_, variant.typeName() );
            no_error = false;
            continue;
          }
          auto &child = FIXED_LENGTH ? array[i] : array.appendEmpty();
          child = getValue<rclcpp::Duration>( variant );
        }
      } else if ( members->message_name_ == std::string( "Time" ) ) {
        no_error = true;
        for ( int i = 0; i < count; ++i ) {
          QModelIndex index = list.index( i, 0 );
          const QVariant &variant = list.data( index );
          if ( !isCompatible<rclcpp::Time>( variant ) ) {
            QML_ROS2_PLUGIN_WARN(
                "Tried to fill array of '%s::%s' with incompatible value! Skipped. (Type: %s)",
                members->message_namespace_, members->message_name_, variant.typeName() );
            no_error = false;
            continue;
          }
          auto &child = FIXED_LENGTH ? array[i] : array.appendEmpty();
          child = getValue<rclcpp::Time>( variant );
        }
      }
      return no_error;
    }
    for ( int i = 0; i < count; ++i ) {
      QModelIndex index = list.index( i );
      auto &child = FIXED_LENGTH ? array[i] : array.appendEmpty();
      for ( size_t j = 0; j < names.size(); ++j ) {
        const std::string &key = names[j];
        if ( key.empty() )
          continue;
        no_error &= fillMessage( fish, child[key], index.data( j ) );
      }
    }
    return no_error;
  }
};
} // namespace

bool fillMessage( BabelFish &fish, Message &msg, const QVariant &value )
{
  if ( value.canConvert<QVariantMap>() && msg.type() == MessageTypes::Compound ) {
    auto &compound = msg.as<CompoundMessage>();
    const QVariantMap &map = value.value<QVariantMap>();
    bool no_error = true;
    for ( const auto &key : map.keys() ) {
      std::string skey = key.toStdString();
      if ( !compound.containsKey( skey ) ) {
        QML_ROS2_PLUGIN_WARN( "Message doesn't have field '%s'!", skey.c_str() );
        no_error = false;
        continue;
      }
      no_error &= fillMessage( fish, compound[skey], map[key] );
    }
    return no_error;
  } else if ( value.canConvert<Array>() && msg.type() == MessageTypes::Array ) {
    try {
      return invoke_for_array_message( msg.as<ArrayMessageBase>(), QVariantListToMessageConverter{},
                                       fish, value.value<Array>() );
    } catch ( BabelFishException &ex ) {
      QML_ROS2_PLUGIN_ERROR( "BabelFishException occurred while converting QML to ROS message: %s",
                             ex.what() );
      return false;
    }
  } else if ( value.canConvert<QVariantList>() && msg.type() == MessageTypes::Array ) {
    try {
      return invoke_for_array_message( msg.as<ArrayMessageBase>(), QVariantListToMessageConverter{},
                                       fish, value.value<QVariantList>() );
    } catch ( BabelFishException &ex ) {
      QML_ROS2_PLUGIN_ERROR( "BabelFishException occurred while converting QML to ROS message: %s",
                             ex.what() );
      return false;
    }
  } else if ( value.canConvert<QObject *>() ) {
    const auto *list_model = value.value<QAbstractListModel *>();
    if ( list_model != nullptr ) {
      return invoke_for_array_message( msg.as<ArrayMessageBase>(),
                                       QAbstractListModelToMessageConverter{}, fish, *list_model );
    }

    if ( msg.type() == MessageTypes::Compound ) {
      const auto *obj = value.value<QObject *>();
      const QMetaObject *metaObj = obj->metaObject();
      auto &compound = msg.as<CompoundMessage>();
      bool no_error = true;
      for ( int i = metaObj->propertyOffset(); i < metaObj->propertyCount(); ++i ) {
        QMetaProperty prop = metaObj->property( i );
        std::string skey = prop.name();
        // Skip keys that don't exist, we don't warn here because there will very likely be properties that don't exist.
        if ( !compound.containsKey( skey ) )
          continue;
        no_error &= fillMessage( fish, compound[skey], prop.read( obj ) );
      }
      return no_error;
    }
  } else if ( value.type() == QVariant::Vector2D && msg.type() == MessageTypes::Compound ) {
    auto vector = value.value<QVector2D>();
    auto &compound = msg.as<CompoundMessage>();
    bool no_error = true;
    int i = 0;
    for ( auto &key : { "x", "y" } ) {
      if ( !compound.containsKey( key ) ) {
        QML_ROS2_PLUGIN_ERROR(
            "Tried to set QVector2D to compound message that has no '%s' property!", key );
        no_error = false;
        continue;
      }
      no_error &= fillMessage( fish, compound[key], vector[i++] );
    }
    return no_error;
  } else if ( value.type() == QVariant::Vector3D && msg.type() == MessageTypes::Compound ) {
    auto vector = value.value<QVector3D>();
    auto &compound = msg.as<CompoundMessage>();
    bool no_error = true;
    int i = 0;
    for ( auto &key : { "x", "y", "z" } ) {
      if ( !compound.containsKey( key ) ) {
        QML_ROS2_PLUGIN_ERROR(
            "Tried to set QVector3D to compound message that has no '%s' property!", key );
        no_error = false;
        continue;
      }
      no_error &= fillMessage( fish, compound[key], vector[i++] );
    }
    return no_error;
  } else if ( value.type() == QVariant::Vector4D && msg.type() == MessageTypes::Compound ) {
    auto vector = value.value<QVector4D>();
    auto &compound = msg.as<CompoundMessage>();
    bool no_error = true;
    int i = 0;
    for ( auto &key : { "x", "y", "z", "w" } ) {
      if ( !compound.containsKey( key ) ) {
        QML_ROS2_PLUGIN_ERROR(
            "Tried to set QVector4D to compound message that has no '%s' property!", key );
        no_error = false;
        continue;
      }
      no_error &= fillMessage( fish, compound[key], vector[i++] );
    }
    return no_error;
  } else if ( value.type() == QVariant::Quaternion && msg.type() == MessageTypes::Compound ) {
    auto quaternion = value.value<QQuaternion>();
    auto &compound = msg.as<CompoundMessage>();
    bool no_error = true;
    for ( const std::string &key : std::array<std::string, 4>{ "w", "x", "y", "z" } ) {
      if ( !compound.containsKey( key ) ) {
        QML_ROS2_PLUGIN_ERROR(
            "Tried to set QQuaternion to compound message that has no '%s' property!", key.c_str() );
        no_error = false;
        continue;
      }
      if ( key == "w" )
        no_error &= fillMessage( fish, compound[key], quaternion.scalar() );
      else if ( key == "x" )
        no_error &= fillMessage( fish, compound[key], quaternion.x() );
      else if ( key == "y" )
        no_error &= fillMessage( fish, compound[key], quaternion.y() );
      else
        no_error &= fillMessage( fish, compound[key], quaternion.z() );
    }
    return no_error;
  }

  if ( msg.type() == MessageTypes::Array ) {
    QML_ROS2_PLUGIN_WARN( "Invalid type for array message: %s (%u)", value.typeName(), value.type() );
    return false;
  }
  if ( msg.type() == MessageTypes::Compound && !msg.isTime() && !msg.isDuration() ) {
    QML_ROS2_PLUGIN_WARN( "Invalid type for compound message: %s (%u)", value.typeName(),
                          value.type() );
    return false;
  }
  switch ( (int)value.type() ) {

  case QVariant::Invalid:
    break;
  case QVariant::Bool:
    return fillValue<bool>( msg, value.toBool() );
  case QMetaType::Short:
  case QMetaType::SChar:
  case QVariant::Int:
    return fillValue<int>( msg, value.toInt() );
  case QMetaType::UShort:
  case QMetaType::UChar:
  case QVariant::UInt:
    return fillValue<uint>( msg, value.toUInt() );
  case QMetaType::Long:
  case QVariant::LongLong:
    return fillValue<qlonglong>( msg, value.toLongLong() );
  case QMetaType::ULong:
  case QVariant::ULongLong:
    return fillValue<qulonglong>( msg, value.toULongLong() );
  case QMetaType::Float:
    return fillValue( msg, value.toFloat() );
  case QVariant::Double:
    return fillValue<double>( msg, value.toDouble() );
  case QVariant::String:
    return fillValue<std::string>( msg, value.toString().toStdString() );
  case QVariant::Url:
    return fillValue<std::string>( msg, value.toUrl().toString().toStdString() );
  case QVariant::DateTime:
    return fillValue<rclcpp::Time>( msg, qmlToRos2Time( value.toDateTime() ) );
  case QVariant::Date: // Not sure if any of these types should be supported
  case QVariant::Time:
  case QVariant::Char:
  default:
    if ( value.canConvert<Time>() ) {
      return fillValue<rclcpp::Time>( msg, value.value<Time>().getTime() );
    }
    if ( value.canConvert<Duration>() ) {
      return fillValue<rclcpp::Duration>( msg, value.value<Duration>().getDuration() );
    }
    QML_ROS2_PLUGIN_WARN( "Unsupported QVariant type '%s' encountered while filling message!",
                          value.typeName() );
    break;
  }
  return false;
}
} // namespace conversion
} // namespace qml_ros2_plugin
