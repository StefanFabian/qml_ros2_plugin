//  MIT License
//
//  Copyright (c) 2021 Stefan Fabian
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//    of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//    copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//    copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.

#ifndef QVARIANT_YAML_CONVERSION_H
#define QVARIANT_YAML_CONVERSION_H

#include "qml_ros2_plugin/time.hpp"

#include <QAbstractListModel>
#include <QDateTime>
#include <QMetaProperty>
#include <QVariant>
#include <yaml-cpp/yaml.h>

namespace YAML
{
template<>
struct convert<qml_ros2_plugin::Time> {
  static Node encode( const qml_ros2_plugin::Time &time )
  {
    YAML::Node result;
    builtin_interfaces::msg::Time msg = time.getTime();
    result["type"] = "time";
    result["sec"] = msg.sec;
    result["nanosec"] = msg.nanosec;
    result["clocktype"] = static_cast<int>( time.getTime().get_clock_type() );
    return result;
  }

  static bool decode( const Node &node, qml_ros2_plugin::Time &out )
  {
    if ( !node.IsMap() )
      return false;
    Node node_clock_type = node["clocktype"];
    rcl_clock_type_t clock_type = RCL_ROS_TIME;
    if ( node_clock_type.IsDefined() ) {
      clock_type = static_cast<rcl_clock_type_t>( node_clock_type.as<int>() );
    }
    builtin_interfaces::msg::Time msg;
    msg.sec = node["sec"].as<int32_t>();
    msg.nanosec = node["nanosec"].as<uint32_t>();
    rclcpp::Time time = msg;
    out = qml_ros2_plugin::Time( rclcpp::Time( time.nanoseconds(), clock_type ) );
    return true;
  }
};

template<>
struct convert<qml_ros2_plugin::Duration> {
  static Node encode( const qml_ros2_plugin::Duration &duration )
  {
    YAML::Node result;
    builtin_interfaces::msg::Duration msg = duration.getDuration();
    result["type"] = "duration";
    result["sec"] = msg.sec;
    result["nanosec"] = msg.nanosec;
    return result;
  }

  static bool decode( const Node &node, qml_ros2_plugin::Duration &out )
  {
    if ( !node.IsMap() )
      return false;
    builtin_interfaces::msg::Duration msg;
    msg.sec = node["sec"].as<int32_t>();
    msg.nanosec = node["nanosec"].as<uint32_t>();
    out = qml_ros2_plugin::Duration( rclcpp::Duration( msg ) );
    return true;
  }
};

template<>
struct convert<QVariantMap> {
  static Node encode( const QVariantMap &map )
  {
    YAML::Node result;
    for ( auto &key : map.keys() ) { result[key.toStdString()] = map[key]; }
    return result;
  }

  static bool decode( const Node &node, QVariantMap &out )
  {
    if ( !node.IsMap() )
      return false;
    for ( auto it = node.begin(); it != node.end(); ++it ) {
      out.insert( QString::fromStdString( it->first.as<std::string>() ), it->second.as<QVariant>() );
    }
    return true;
  }
};

template<>
struct convert<QVariantList> {
  static Node encode( const QVariantList &list )
  {
    YAML::Node result;
    for ( int i = 0; i < list.size(); ++i ) { result.push_back( list[i] ); }
    return result;
  }

  static bool decode( const Node &node, QVariantList &list )
  {
    if ( !node.IsSequence() )
      return false;

    for ( const YAML::Node &subNode : node ) { list.append( subNode.as<QVariant>() ); }
    return true;
  }
};

template<>
struct convert<QAbstractListModel> {
  static Node encode( const QAbstractListModel &list )
  {
    YAML::Node result;
    QHash<int, QByteArray> roleNames = list.roleNames();
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
    for ( int i = 0; i < list.rowCount(); ++i ) {
      YAML::Node item;
      QModelIndex index = list.index( i );
      for ( size_t j = 0; j < names.size(); ++j ) {
        const std::string &key = names[j];
        if ( key.empty() )
          continue;
        item[key] = index.data( j );
      }
      result.push_back( item );
    }
    return result;
  }
};

template<>
struct convert<QVariant> {
  static Node encode( const QVariant &variant )
  {
    YAML::Node result;
    if ( variant.type() == QVariant::UserType && variant.userType() == qMetaTypeId<QJSValue>() ) {
      // Need to unwrap QJSValue because it can be cast to both QVariantList and QVariantMap but
      // will be empty if it is not the contained type.
      result = variant.value<QJSValue>().toVariant();
      return result;
    }
    if ( variant.canConvert<QVariantList>() ) {
      const auto &list = variant.value<QVariantList>();
      result = list;
      return result;
    } else if ( variant.canConvert<QVariantMap>() ) {
      const auto &map = variant.value<QVariantMap>();
      result = map;
      return result;
    } else if ( variant.canConvert<qml_ros2_plugin::Time>() ) {
      result = variant.value<qml_ros2_plugin::Time>();
      return result;
    } else if ( variant.canConvert<qml_ros2_plugin::Duration>() ) {
      result = variant.value<qml_ros2_plugin::Duration>();
      return result;
    } else if ( variant.canConvert<QDateTime>() && variant.value<QDateTime>().isValid() ) {
      result = variant.value<QDateTime>().toString( Qt::ISODateWithMs ).toStdString();
      return result;
    } else if ( variant.canConvert<QObject *>() ) {
      const auto *list_model = variant.value<QAbstractListModel *>();
      if ( list_model != nullptr ) {
        result = *list_model;
        return result;
      }
      const auto *obj = variant.value<QObject *>();
      const QMetaObject *metaObj = obj->metaObject();
      for ( int i = metaObj->propertyOffset(); i < metaObj->propertyCount(); ++i ) {
        QMetaProperty prop = metaObj->property( i );
        result[prop.name()] = prop.read( obj );
      }
      return result;
    }
    QString sval = variant.toString();
    result = ( sval.toStdString() );
    if ( variant.type() == QVariant::String )
      result.SetTag( "tag:yaml.org,2002:str" );
    return result;
  }

  static bool decode( const YAML::Node &node, QVariant &out )
  {
    switch ( node.Type() ) {
    case YAML::NodeType::Undefined:
    case YAML::NodeType::Null:
      out = QVariant();
      return true;
    case YAML::NodeType::Scalar: {
      // Since yaml-cpp doesn't properly encode strings with quotes and will ignore quotes when converting to bool etc.
      // we have to add a tag to prevent the string "y" from being interpreted as a bool.
      if ( node.Tag() == "tag:yaml.org,2002:str" ) {
        out = QVariant( QString::fromStdString( node.as<std::string>() ) );
        return true;
      }
      try {
        out = QVariant( node.as<bool>() );
        return true;
      } catch ( YAML::BadConversion & ) {
      }
      try {
        qlonglong lval = node.as<long long>();
        if ( lval >= 0 ) {
          out = QVariant( node.as<qulonglong>() );
          return true;
        }
        out = QVariant( lval );
        return true;
      } catch ( YAML::BadConversion & ) {
      }
      try {
        out = QVariant( node.as<qulonglong>() );
        return true;
      } catch ( YAML::BadConversion & ) {
      }
      try {
        out = QVariant::fromValue( node.as<qreal>() );
        return true;
      } catch ( YAML::BadConversion & ) {
      }
      QString value = QString::fromStdString( node.as<std::string>() );
      QDateTime datetime = QDateTime::fromString( value, Qt::ISODateWithMs );
      if ( datetime.isValid() ) {
        out = QVariant( datetime );
        return true;
      }
      out = QVariant( value );
      return true;
    }
    case YAML::NodeType::Sequence: {
      out = node.as<QVariantList>();
      return true;
    }
    case YAML::NodeType::Map: {
      if ( node["type"].IsDefined() ) {
        const std::string &type = node["type"].as<std::string>();
        if ( type == "time" ) {
          out = QVariant::fromValue( node.as<qml_ros2_plugin::Time>() );
          return true;
        } else if ( type == "duration" ) {
          out = QVariant::fromValue( node.as<qml_ros2_plugin::Duration>() );
          return true;
        }
      }
      out = node.as<QVariantMap>();
      return true;
    }
    }
    return false;
  }
};
} // namespace YAML

#endif // QVARIANT_YAML_CONVERSION_H
