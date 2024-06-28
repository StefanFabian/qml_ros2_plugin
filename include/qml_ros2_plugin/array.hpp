// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_ARRAY_HPP
#define QML_ROS2_PLUGIN_ARRAY_HPP

#include <QVariant>
#include <ros_babel_fish/messages/array_message.hpp>

namespace qml_ros2_plugin
{

/*!
 * @brief View on an array field of a message.
 * This allows access on array elements with lazy copy mechanism.
 * Copies of an Array point to the same data and modifications of one array will be mirrored by the other.
 */
class Array
{
  Q_GADGET
  //! The length of the array, i.e., the number of elements.
  Q_PROPERTY( int length READ length WRITE setLength )
public:
  struct Data;

  Array();

  explicit Array( const ros_babel_fish::ArrayMessageBase::ConstSharedPtr &message );

  //  Array( const Array &other );

  ~Array() = default;

  int length() const;

  int size() const { return length(); }

  void setLength( int value );

  /*!
   * If the index is out of the bounds of the array, an empty QVariant is returned.
   *
   * @param index Index of the retrieved element.
   * @return The array element at the given index.
   */
  Q_INVOKABLE QVariant at( int index ) const;

  /*!
   * Changes the array content by removing delete_count elements at index and inserting the elements
   * in items. This method can be used to remove, replace or add elements to the array.
   *
   * @warning If the operation is not limited to the end of the array, it requires a deep copy of
   * the message array.
   *
   * @param start The index at which to start changing the array. If greater than the length of the
   * array, start will be set to the length of the array. If negative, it will begin that many
   * elements from the end of the array (with origin -1, meaning -n is the index of the nth last
   * element and is therefore equivalent to the index of array.length - n). If the absolute value of
   * start is greater than the length of the array, it will begin from index 0.
   * @param delete_count The number of elements to delete starting at index start. If delete_count
   * is greater than the number of elements after start, all elements from start to the length of
   * the array are removed. If delete_count is 0, no elements are removed, e.g., for a insert only
   * operation.
   * @param items The items that will be inserted at start.
   */
  Q_INVOKABLE void spliceList( int start, int delete_count, const QVariantList &items );

  /*!
   * Adds the given value to the end of the array.
   *
   * @param value The item that is added.
   */
  Q_INVOKABLE void push( const QVariant &value );

  /*!
   * Adds the given value to the front of the array.
   *
   * @warning This requires a deep copy of the message array on first call whereas appending can be
   * done without copying the array.
   * @param value The item that is added.
   */
  Q_INVOKABLE void unshift( const QVariant &value );

  /*!
   * Removes the last element and returns it.
   * @return The removed element or an empty QVariant if the array is empty.
   */
  Q_INVOKABLE QVariant pop();

  /*!
   * Removes the first element and returns it.
   * @warning This requires a deep copy of the message array on first call whereas appending can be
   * done without copying the array.
   * @return The removed element or an empty QVariant if the array is empty.
   */
  Q_INVOKABLE QVariant shift();

  /*!
   * Converts the array to a QVariantList which can be used in place of a JS array in QML.
   * This method performs a deep copy of the message array on the first call.
   * @return The array as a QVariantList.
   */
  Q_INVOKABLE QVariantList toArray();

  /* Internal functions */
  bool _isModified( int index ) const;

  bool _inCache() const;

  ros_babel_fish::ArrayMessageBase::ConstSharedPtr _message() const;

  Q_INVOKABLE QVariantList toVariantList() const;

private:
  void enlargeCache( int size ) const;

  void fillCache() const;

  // Copies of array share the data
  std::shared_ptr<Data> p_;
};
} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( qml_ros2_plugin::Array )

#endif // QML_ROS2_PLUGIN_ARRAY_HPP
