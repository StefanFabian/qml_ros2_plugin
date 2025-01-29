// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/array.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml_ros2_plugin/time.hpp"
#include <ros_babel_fish/method_invoke_helpers.hpp>

using namespace qml_ros2_plugin::conversion;
using namespace ros_babel_fish;

namespace qml_ros2_plugin
{

struct Array::Data {
  QVariantList cache;
  QList<bool> modified;
  ros_babel_fish::ArrayMessageBase::ConstSharedPtr message = nullptr;
  bool all_in_cache = true;
  int length = 0;
};

Array::Array() { p_ = std::make_shared<Data>(); }

Array::Array( const ArrayMessageBase::ConstSharedPtr &message )
{
  p_ = std::make_shared<Data>();
  p_->message = message;
  p_->all_in_cache = false;
  p_->length = message == nullptr ? 0 : int( message->size() );
}

int Array::length() const { return p_->length; }

void Array::setLength( int value )
{
  p_->length = value;
  for ( int i = p_->modified.length(); i > p_->length; --i ) p_->modified.pop_back();
  for ( int i = p_->cache.length(); i > p_->length; --i ) p_->cache.pop_back();
}

namespace
{
struct ElementGetter {
  template<typename T, ros_babel_fish::ArraySize SIZE>
  QVariant operator()( const ArrayMessage_<T, SIZE> &array, int index,
                       const std::shared_ptr<Array::Data> & )
  {
    if ( static_cast<size_t>( index ) >= array.size() )
      return {};
    if constexpr ( std::is_same_v<T, std::string> ) {
      return QVariant::fromValue( QString::fromStdString( array[index] ) );
    } else if constexpr ( std::is_same_v<T, std::wstring> ) {
      return QVariant::fromValue( QString::fromStdWString( array[index] ) );
    } else if constexpr ( std::is_same_v<T, uint8_t> ) {
      return QVariant::fromValue( quint32( array[index] ) );
    } else if constexpr ( std::is_same_v<T, int8_t> ) {
      return QVariant::fromValue( qint32( array[index] ) );
    } else if constexpr ( std::is_same_v<T, char16_t> ) {
      return QVariant::fromValue( QChar( array[index] ) );
    } else if constexpr ( std::is_same_v<T, long double> ) {
      // long double is not supported by QVariant
      return QVariant::fromValue( static_cast<double>( array[index] ) );
    } else {
      return QVariant::fromValue( array[index] );
    }
  }

  template<ros_babel_fish::ArraySize SIZE>
  QVariant operator()( const CompoundArrayMessage_<SIZE> &array, int index,
                       const std::shared_ptr<Array::Data> &p )
  {
    if ( static_cast<size_t>( index ) >= array.size() )
      return {};
    if ( p->cache.size() <= index ) {
      p->cache.reserve( index + 1 );
      p->modified.reserve( index + 1 );
      for ( int i = p->cache.size(); i <= index; ++i ) {
        p->cache.push_back( QVariant() );
        p->modified.push_back( false );
      }
    }
    if ( !p->cache[index].isValid() ) {
      if ( static_cast<size_t>( index ) < p->message->size() )
        p->cache[index] = msgToMap( array[index] );
      else
        p->cache[index] = QVariantMap();
    }
    return p->cache[index];
  }
};
} // namespace

QVariant Array::at( int index ) const
{
  if ( index < 0 || index >= length() ) {
    return {};
  }
  if ( p_->cache.size() > index && p_->cache[index].isValid() )
    return p_->cache[index];
  return ros_babel_fish::invoke_for_array_message( *p_->message, ElementGetter{}, index, p_ );
}

void Array::spliceList( int start, int delete_count, const QVariantList &items )
{
  if ( start > p_->length )
    start = p_->length;
  else if ( start < 0 )
    start = p_->length + start;
  if ( start < 0 )
    start = 0;
  if ( start + delete_count >= length() ) {
    // cheap case where we can just remove the last elements and add the new items.
    if ( !p_->all_in_cache ) {
      enlargeCache( length() );
      for ( int i = p_->modified.length(); i > start; --i ) p_->modified.pop_back();
    }
    for ( int i = p_->cache.length(); i > start; --i ) p_->cache.pop_back();
    for ( auto &item : items ) {
      if ( !p_->all_in_cache )
        p_->modified.push_back( true );
      p_->cache.push_back( item );
    }
    p_->length = p_->cache.size();
    return;
  }
  if ( delete_count == 1 && items.size() == 1 ) {
    // this is a simple replace operation
    if ( !p_->all_in_cache ) {
      enlargeCache( start + 1 );
      p_->modified[start] = true;
    }
    p_->cache[start] = items[0];
    return;
  }
  // otherwise we have to copy the entire message
  fillCache();
  for ( int i = 0; i < delete_count; ++i ) p_->cache.removeAt( start );
  for ( int i = 0; i < items.size(); ++i ) p_->cache.insert( start + i, items[i] );
  p_->length = p_->cache.size();
}

void Array::push( const QVariant &value )
{
  enlargeCache( length() );
  p_->cache.append( value );
  if ( !p_->all_in_cache )
    p_->modified.push_back( true );
  ++p_->length;
}

void Array::unshift( const QVariant &value )
{
  fillCache();
  p_->cache.prepend( value );
  ++p_->length;
}

QVariant Array::pop()
{
  if ( length() == 0 )
    return QVariant();
  QVariant result = at( length() - 1 ); // This automatically grows the cache if not a primitive type
  if ( p_->cache.size() == length() )
    p_->cache.pop_back();
  if ( !p_->all_in_cache ) {
    if ( p_->modified.size() == p_->length )
      p_->modified.pop_back();
  }
  --p_->length;
  return result;
}

QVariant Array::shift()
{
  if ( length() == 0 )
    return QVariant();
  fillCache();
  QVariant result = at( 0 );
  p_->cache.pop_front();
  --p_->length;
  return result;
}

QVariantList Array::toArray()
{
  fillCache();
  return p_->cache;
}

bool Array::_isModified( int index ) const
{
  return p_->all_in_cache || ( index < p_->modified.size() && p_->modified[index] );
}

ros_babel_fish::ArrayMessageBase::ConstSharedPtr Array::_message() const { return p_->message; }

QVariantList Array::toVariantList() const
{
  fillCache();
  return p_->cache;
}

void Array::enlargeCache( int size ) const
{
  if ( p_->cache.size() >= size )
    return;
  p_->cache.reserve( size );
  for ( int i = p_->cache.size(); i < size; ++i ) { p_->cache.push_back( QVariant() ); }
  for ( size_t i = p_->modified.size(); i < static_cast<uint>( size ); ++i ) {
    p_->modified.push_back( false );
  }
}

bool Array::_inCache() const { return p_->all_in_cache; }

void Array::fillCache() const
{
  if ( p_->all_in_cache )
    return;
  p_->cache.reserve( length() );
  for ( int i = 0; i < p_->length; ++i ) {
    if ( ( p_->modified.size() > i && p_->modified[i] ) ||
         ( p_->cache.size() > i && p_->cache[i].isValid() ) )
      continue;
    const QVariant &variant = at( i );
    if ( p_->cache.size() <= i )
      p_->cache.push_back( variant );
    else
      p_->cache[i] = variant;
  }
  p_->all_in_cache = true;
  p_->modified.clear();
}
} // namespace qml_ros2_plugin
