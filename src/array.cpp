// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/array.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml_ros2_plugin/time.hpp"

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
template<typename T, bool BOUNDED, bool FIXED_LENGTH>
void getElement( const ArrayMessage_<T, BOUNDED, FIXED_LENGTH> &array, int index, QVariant &result,
                 const std::shared_ptr<Array::Data> & )
{
  if ( static_cast<size_t>( index ) >= array.size() )
    return;
  result = QVariant::fromValue( array[index] );
}

template<bool BOUNDED, bool FIXED_LENGTH>
void getElement( const ArrayMessage_<std::string, BOUNDED, FIXED_LENGTH> &array, int index,
                 QVariant &result, const std::shared_ptr<Array::Data> & )
{
  if ( static_cast<size_t>( index ) >= array.size() )
    return;
  result = QVariant::fromValue( QString::fromStdString( array[index] ) );
}

template<bool BOUNDED, bool FIXED_LENGTH>
void getElement( const ArrayMessage_<std::wstring, BOUNDED, FIXED_LENGTH> &array, int index,
                 QVariant &result, const std::shared_ptr<Array::Data> & )
{
  if ( static_cast<size_t>( index ) >= array.size() )
    return;
  result = QVariant::fromValue( QString::fromStdWString( array[index] ) );
}

template<bool BOUNDED, bool FIXED_LENGTH>
void getElement( const ArrayMessage_<uint8_t, BOUNDED, FIXED_LENGTH> &array, int index,
                 QVariant &result, const std::shared_ptr<Array::Data> & )
{
  if ( static_cast<size_t>( index ) >= array.size() )
    return;
  result = QVariant::fromValue( uint( array[index] ) );
}

template<bool BOUNDED, bool FIXED_LENGTH>
void getElement( const ArrayMessage_<int8_t, BOUNDED, FIXED_LENGTH> &array, int index,
                 QVariant &result, const std::shared_ptr<Array::Data> & )
{
  if ( static_cast<size_t>( index ) >= array.size() )
    return;
  result = QVariant::fromValue( int( array[index] ) );
}

template<bool BOUNDED, bool FIXED_LENGTH>
void getElement( const ArrayMessage_<long double, BOUNDED, FIXED_LENGTH> &array, int index,
                 QVariant &result, const std::shared_ptr<Array::Data> & )
{
  if ( static_cast<size_t>( index ) >= array.size() )
    return;
  result = QVariant::fromValue( static_cast<double>( array[index] ) );
}

template<bool BOUNDED, bool FIXED_LENGTH>
void getElement( const ArrayMessage_<char16_t, BOUNDED, FIXED_LENGTH> &array, int index,
                 QVariant &result, const std::shared_ptr<Array::Data> & )
{
  if ( static_cast<size_t>( index ) >= array.size() )
    return;
  result = QVariant::fromValue( QChar( array[index] ) );
}

template<bool BOUNDED, bool FIXED_LENGTH>
void getElement( const CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &array, int index,
                 QVariant &result, const std::shared_ptr<Array::Data> &p )
{
  if ( static_cast<size_t>( index ) >= array.size() )
    return;
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
  result = p->cache[index];
}
} // namespace

QVariant Array::at( int index ) const
{
  if ( index < 0 || index >= length() ) {
    return QVariant();
  }
  if ( p_->cache.size() > index && p_->cache[index].isValid() )
    return p_->cache[index];
  QVariant result;
  RBF2_TEMPLATE_CALL_ARRAY_TYPES( getElement, *p_->message, index, result, p_ );
  return result;
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
