// Copyright (c) 2021 Aljoscha Schmidt, Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_RING_BUFFER_HPP
#define QML_ROS2_PLUGIN_RING_BUFFER_HPP

#include <array>
#include <cassert>
#include <stdexcept>
#include <type_traits>

namespace qml_ros2_plugin
{

/*!
 * A RingBuffer is a limited size storage container that once full will overwrite the oldest
 * elements when a new element is added.
 * @tparam T The type of the elements stored in this RingBuffer.
 * @tparam Nm The maximum number of elements stored in the RingBuffer at any time.
 */
template<typename T, std::size_t N>
class RingBuffer
{
public:
  static constexpr std::size_t Size = N;
  using value_type = T;
  using pointer = value_type *;
  using const_pointer = const value_type *;
  using reference = value_type &;
  using const_reference = const value_type &;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  template<typename Iterator>
  struct ring_iterator;
  using iterator = ring_iterator<pointer>;
  using const_iterator = ring_iterator<const_pointer>;

  //! @returns true if the container is empty, false otherwise
  [[nodiscard]] bool empty() const { return size_ == 0; }
  //! @returns true if the container is full, false otherwise. Appending to a full container will overwrite old elements.
  [[nodiscard]] bool full() const { return size_ == Size; }
  //! @returns the number of elements
  [[nodiscard]] std::size_t size() const { return size_; }
  //! @returns the maximum number of elements which is equal to the template parameter TSize.
  [[nodiscard]] constexpr std::size_t capacity() const { return Size; }
  //! @returns the maximum number of elements which is equal to the template parameter TSize.
  [[nodiscard]] constexpr std::size_t max_size() const { return Size; }

  /*!
   * Adds an element to the end of the RingBuffer.
   * If the RingBuffer is full the oldest element will be overwritten.
   * @param value element to be appended.
   */
  void push_back( const_reference value )
  {
    items_[tail_index_] = value;
    added_element_tail_adapt_indices();
  }

  //! Deletes the oldest element in the RingBuffer.
  void pop_front()
  {
    if ( size_ > 0 ) {
      front().~value_type(); // front is last element -> oldest element
      removed_element_at_head_adapt_indices();
    }
  }

  /*!
   * Reads and deletes the oldest element of the RingBuffer.
   * @return the oldest not yet overwritten element from The RingBuffer.
   */
  value_type read_and_pop_front()
  {
    if ( size_ <= 0 )
      throw std::length_error( "RingBuffer is empty!" );
    value_type tmp = std::move( items_[get_head_index()] );
    removed_element_at_head_adapt_indices();
    return tmp;
  }
  /*!
   * Constructs an element and appends it to the RingBuffer.
   * If the RingBuffer is already full, the oldest element is overwritten.
   * @param args The arguments that are passed to the constructor of the element.
   */
  template<typename... Args>
  void emplace_back( Args &&...args )
  {
    items_[tail_index_] = value_type( std::forward<Args>( args )... );
    added_element_tail_adapt_indices();
  }

  //! @returns an iterator pointing to the oldest element in the ringbuffer.
  iterator begin() noexcept { return iterator( this, get_head_index(), 0 ); }

  //! @returns an iterator pointing to the position after the newest element in the ringbuffer.
  iterator end() noexcept { return iterator( this, get_head_index(), size_ ); }

  //! @returns a const iterator pointing to the oldest element in the ringbuffer.
  const_iterator cbegin() noexcept { return const_iterator( this, get_head_index() ); }

  //! @returns a const iterator pointing to the position after the newest element in the ringbuffer.
  const_iterator cend() noexcept { return const_iterator( this, get_head_index(), size_ ); }

  //! @returns a reference to the oldest element in the buffer.
  reference front() { return items_[get_head_index()]; }
  //! @returns a const reference to the oldest element in the buffer.
  const_reference front() const { return items_[get_head_index()]; }

  //! @returns a reference to the newest element in the buffer.
  reference back() { return items_[get_last_index()]; }
  //! @returns a const reference to the newest element in the buffer.
  const_reference back() const { return items_[get_last_index()]; }

  //! Clears the contents of the RingBuffer.
  void clear()
  {
    if ( std::is_trivially_destructible<value_type>::value ) {
      tail_index_ = 0;
      size_ = 0;
    } else {
      while ( size_ > 0 ) pop_front();
    }
  }

  const_reference operator[]( std::size_t index ) const
  {
    return items_[get_normalised_index( index )];
  }

  reference operator[]( std::size_t index ) { return items_[get_normalised_index( index )]; }

private:
  // normalises index to be as if head would reside at position 0
  std::size_t get_normalised_index( std::size_t index ) const
  {
    const std::size_t result = get_head_index() + index;
    return result < Size ? result : result - Size;
  }

  void added_element_tail_adapt_indices()
  {
    if ( !full() )
      size_++;
    tail_index_ = tail_index_ == ( Size - 1 ) ? 0 : ( tail_index_ + 1 );
  }

  void removed_element_at_head_adapt_indices() { size_--; }

  //! The index of the oldest element
  std::size_t get_head_index() const
  {
    return tail_index_ >= size_ ? ( tail_index_ - size_ ) : ( tail_index_ + Size - size_ );
  }
  std::size_t get_last_index() const
  {
    return tail_index_ == 0 ? ( Size - 1 ) : ( tail_index_ - 1 );
  }
  std::array<value_type, Size> items_;
  std::size_t size_ = 0;
  std::size_t tail_index_ = 0;
};

template<typename T, std::size_t N>
template<typename Iterator>
struct RingBuffer<T, N>::ring_iterator {
private:
  using _traits = std::iterator_traits<Iterator>;

public:
  using iterator_type = Iterator;
  using iterator_category = std::random_access_iterator_tag;
  using value_type = typename _traits::value_type;
  using difference_type = typename _traits::difference_type;
  using reference = typename _traits::reference;
  using pointer = typename _traits ::pointer;

  ring_iterator( RingBuffer<T, N> *buffer, int offset, int index = 0 ) noexcept
      : buffer_( buffer ), offset_( offset ), iterator_index_( index ),
        buffer_index_( ( offset + index ) % N )
  {
  }

  reference operator*() const noexcept { return buffer_->items_[buffer_index_]; }
  pointer operator->() const noexcept { return &( buffer_->items_[buffer_index_] ); }

  ring_iterator<Iterator> &operator++() noexcept
  {
    buffer_index_ = buffer_index_ == ( Size - 1 ) ? 0 : ( buffer_index_ + 1 );
    ++iterator_index_;
    return *this;
  }

  // clang-format off
  // NOLINTNEXTLINE(cert-dcl21-cpp) lvalue ref-qualify to prevent (it++)++ mistakes without const.
  ring_iterator<Iterator> operator++( int ) & noexcept
  // clang-format on
  {
    auto tmp = *this;
    ++( *this );
    return tmp;
  }

  ring_iterator<Iterator> &operator+=( int nums ) noexcept
  {
    iterator_index_ += nums;
    buffer_index_ = ( buffer_index_ + nums ) % N;
    return *this;
  }

  ring_iterator<Iterator> &operator--() noexcept
  {
    buffer_index_ = buffer_index_ == 0 ? ( Size - 1 ) : ( buffer_index_ - 1 );
    --iterator_index_;
    return *this;
  }

  // clang-format off
  ring_iterator<Iterator> operator--( int ) & noexcept // NOLINT(cert-dcl21-cpp)
  // clang-format on
  {
    auto tmp = *this;
    --( *this );
    return tmp;
  }

  ring_iterator<Iterator> &operator-=( int nums ) noexcept
  {
    iterator_index_ -= nums;
    buffer_index_ = ( buffer_index_ - nums ) % N;
    return *this;
  }

  difference_type operator-( const ring_iterator<Iterator> &other ) const noexcept
  {
    assert( buffer_ == other.buffer_ && "Iterators have to point to the same buffer!" );
    assert( offset_ == other.offset_ && "You are performing arithmetic with two iterators from "
                                        "different head points. This indicates a bug." );
    return iterator_index_ - other.iterator_index_;
  }

  ring_iterator<Iterator> operator+( difference_type nums ) const noexcept
  {
    return ring_iterator<Iterator>( buffer_, offset_, iterator_index_ + nums );
  }

  ring_iterator<Iterator> operator-( difference_type nums ) const noexcept
  {
    return ring_iterator<Iterator>( buffer_, offset_, iterator_index_ - nums );
  }

  // Always allow implicit cast to a const iterator.
  // NOLINTNEXTLINE(google-explicit-constructor)
  operator RingBuffer<T, N>::const_iterator() const noexcept
  {
    return RingBuffer<T, N>::const_iterator( buffer_, offset_, iterator_index_ );
  }

  template<typename IteratorR>
  bool operator==( const ring_iterator<IteratorR> &other ) const noexcept
  {
    assert( buffer_ == other.buffer_ &&
            "You are comparing two iterators from different ring buffers. This indicates a bug." );
    assert( offset_ == other.offset_ &&
            "You are comparing two iterators from different head points. This indicates a bug." );
    return iterator_index_ == other.iterator_index_;
  }

  template<typename IteratorR>
  bool operator!=( const ring_iterator<IteratorR> &other ) const noexcept
  {
    assert( buffer_ == other.buffer_ &&
            "You are comparing two iterators from different ring buffers. This indicates a bug." );
    assert( offset_ == other.offset_ &&
            "You are comparing two iterators from different head points. This indicates a bug." );
    return iterator_index_ != other.iterator_index_;
  }

private:
  RingBuffer<T, N> *buffer_;
  int offset_;
  int iterator_index_;
  int buffer_index_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_RING_BUFFER_HPP