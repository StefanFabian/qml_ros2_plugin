// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/message_item_model.hpp"
#include "logging.hpp"
#include "qml_ros2_plugin/array.hpp"
#include "qml_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml_ros2_plugin/conversion/message_conversions.hpp"
#include "qml_ros2_plugin/time.hpp"

#include <QMetaProperty>
#include <ros_babel_fish/idl/exceptions.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <utility>

namespace qml_ros2_plugin
{

MessageTreeItem::MessageTreeItem( std::shared_ptr<QVariant> root_data,
                                  ros_babel_fish::MessageMembersIntrospection introspection )
    : root_introspection_(
          std::make_unique<ros_babel_fish::MessageMembersIntrospection>( introspection ) ),
      root_data_( std::move( root_data ) )
{
}
MessageTreeItem::MessageTreeItem( int index, const QString &name,
                                  ros_babel_fish::MessageMemberIntrospection introspection,
                                  MessageTreeItem *parentItem )
    : introspection_( std::make_unique<ros_babel_fish::MessageMemberIntrospection>( introspection ) ),
      name_( name ), parent_item_( parentItem ), index_( index ), type_( CompoundMember )
{
}
MessageTreeItem::MessageTreeItem( int index, ros_babel_fish::MessageMemberIntrospection introspection,
                                  MessageTreeItem *parent_item )
    : introspection_( std::make_unique<ros_babel_fish::MessageMemberIntrospection>( introspection ) ),
      name_( QString::number( index ) ), parent_item_( parent_item ), index_( index ),
      type_( ArrayElement )
{
}

MessageTreeItem *MessageTreeItem::child( int row )
{
  if ( row >= childCount() || row < 0 )
    return nullptr;
  if ( row >= static_cast<int>( child_items_.size() ) )
    child_items_.resize( row + 1 );
  if ( child_items_[row] != nullptr )
    return child_items_[row].get();
  if ( root_introspection_ ) {
    auto member = root_introspection_->getMember( row );
    child_items_[row] =
        std::make_unique<MessageTreeItem>( row, QString( member->name_ ), member, this );
    return child_items_[row].get();
  }

  if ( introspection_->value->is_array_ && type_ != ArrayElement ) {
    child_items_[row] = std::make_unique<MessageTreeItem>( row, *introspection_, this );
    return child_items_[row].get();
  }

  ros_babel_fish::MessageMembersIntrospection members = *introspection_;
  auto member = members.getMember( row );
  child_items_[row] =
      std::make_unique<MessageTreeItem>( row, QString( member->name_ ), member, this );
  return child_items_[row].get();
}

int MessageTreeItem::childCount() const
{
  if ( root_introspection_ ) {
    return static_cast<int>( root_introspection_->value->member_count_ );
  }
  if ( introspection_->value->is_array_ && type_ != ArrayElement ) {
    const bool fixed_length =
        !introspection_->value->is_upper_bound_ && introspection_->value->array_size_ > 0;
    if ( fixed_length )
      return introspection_->value->array_size_;

    auto item_data = getItemData();
    if ( item_data.type() == QMetaType::QVariantList ) {
      auto *arr_msg = static_cast<const QVariantList *>( item_data.data() );
      return static_cast<int>( arr_msg->size() );
    }
    if ( item_data.type() == qMetaTypeId<qml_ros2_plugin::Array>() ) {
      const auto *array = static_cast<const qml_ros2_plugin::Array *>( item_data.data() );
      return array->length();
    }
    return 0;
  }
  if ( introspection_->value->members_ == nullptr )
    return 0;
  ros_babel_fish::MessageMembersIntrospection members = *introspection_;
  return members->member_count_;
}

int MessageTreeItem::row() const
{
  if ( parent_item_ == nullptr )
    return 0;
  return index_;
}
int MessageTreeItem::columnCount() const { return 2; }

QVariant MessageTreeItem::data( int column ) const
{
  if ( column == 0 )
    return name_;
  auto item_data = getItemData();
  if ( item_data.isNull() )
    return createNewChild();
  return item_data;
}

bool MessageTreeItem::setData( int column, const QVariant &value )
{
  if ( column != 1 )
    return false;
  return setItemData( value );
}

MessageTreeItem *MessageTreeItem::parentItem() { return parent_item_; }

QString MessageTreeItem::type() const
{
  if ( root_introspection_ ) {
    return "compound";
  }
  if ( type_ != ArrayElement && introspection_->value->is_array_ ) {
    return "array";
  }

  switch ( introspection_->value->type_id_ ) {
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
    return "compound";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
    return "float";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
    return "double";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
    return "long double";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
    return "char";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
    return "wchar";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
    return "bool";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    return "octet";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
    return "uint8";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
    return "int8";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
    return "uint16";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
    return "int16";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
    return "uint32";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
    return "int32";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
    return "uint64";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
    return "int64";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
    return "string";
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
    return "wstring";
  default:
    Q_ASSERT( false && "Unknown message type!" );
    return "unknown";
  }
}

QVariant *MessageTreeItem::getItemDataPtr() const
{
  if ( !parent_item_ ) {
    return root_data_.get();
  }
  QVariant *item_data = parent_item_->getItemDataPtr();
  if ( item_data == nullptr )
    return nullptr;
  if ( type_ == ArrayElement ) {
    if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Array>() ) {
      auto *array = static_cast<qml_ros2_plugin::Array *>( item_data->data() );
      return &( array->atRef( index_ ) ); // Not supported
    }
    if ( item_data->type() != QMetaType::QVariantList ) {
      QML_ROS2_PLUGIN_ERROR( "MessageTreeItem::getItemDataPtr: Unexpected type '%d'!",
                             item_data->type() );
      return nullptr;
    }
    auto *arr_msg = static_cast<QVariantList *>( item_data->data() );
    return &( ( *arr_msg )[index_] );
  }
  if ( item_data->type() != QMetaType::QVariantMap ) {
    QML_ROS2_PLUGIN_ERROR( "MessageTreeItem::getItemDataPtr: Unexpected type '%d'!",
                           item_data->type() );
    return nullptr;
  }
  auto *compound_msg = static_cast<QVariantMap *>( item_data->data() );
  return &( ( *compound_msg )[name_] );
}

QVariant MessageTreeItem::getItemData() const
{
  if ( !parent_item_ ) {
    return *root_data_;
  }
  QVariant *item_data = parent_item_->getItemDataPtr();
  if ( type_ == ArrayElement ) {
    if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Array>() ) {
      auto *array = static_cast<qml_ros2_plugin::Array *>( item_data->data() );
      return ( array->at( index_ ) );
    }
    if ( item_data->type() != QMetaType::QVariantList ) {
      QML_ROS2_PLUGIN_ERROR( "MessageTreeItem::getItemData: Unexpected type '%d'!",
                             item_data->type() );
      return {};
    }
    auto *arr_msg = static_cast<QVariantList *>( item_data->data() );
    return ( ( *arr_msg )[index_] );
  }
  if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Time>() ) {
    auto time = static_cast<qml_ros2_plugin::Time *>( item_data->data() );
    builtin_interfaces::msg::Time t = time->getTime();
    return name_ == "sec" ? t.sec : t.nanosec;
  }
  if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Duration>() ) {
    auto duration = static_cast<qml_ros2_plugin::Duration *>( item_data->data() );
    builtin_interfaces::msg::Duration d = duration->getDuration();
    return name_ == "sec" ? d.sec : d.nanosec;
  }
  if ( item_data->type() != QMetaType::QVariantMap ) {
    QML_ROS2_PLUGIN_ERROR( "MessageTreeItem::getItemData: Unknown type of member '%d'!",
                           item_data->type() );
    return {};
  }
  auto *compound_msg = static_cast<QVariantMap *>( item_data->data() );
  return ( ( *compound_msg )[name_] );
}

bool MessageTreeItem::setItemData( QVariant value )
{
  if ( QVariant::Type type_id = getItemData().type();
       value.type() != type_id && !value.convert( type_id ) )
    return false;
  if ( !parent_item_ ) {
    *root_data_ = value;
    return true;
  }
  QVariant *item_data = parent_item_->getItemDataPtr();
  if ( type_ == ArrayElement ) {
    if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Array>() ) {
      auto *array = static_cast<qml_ros2_plugin::Array *>( item_data->data() );
      array->replace( index_, value );
      return true;
    }
    if ( item_data->type() != QMetaType::QVariantList ) {
      QML_ROS2_PLUGIN_ERROR( "MessageTreeItem::setItemData: Unexpected type '%d'!",
                             item_data->type() );
      return false;
    }
    auto *arr_msg = static_cast<QVariantList *>( item_data->data() );
    ( *arr_msg )[index_] = value;
    return true;
  }
  if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Time>() ) {
    auto time = static_cast<qml_ros2_plugin::Time *>( item_data->data() );
    builtin_interfaces::msg::Time t = time->getTime();
    bool ok = false;
    uint val = value.toUInt( &ok );
    if ( !ok )
      return false;
    if ( name_ == "sec" )
      t.sec = static_cast<int>( val ); // int but values < 0 are not allowed
    else
      t.nanosec = val;
    if ( t.sec < 0 )
      return false; // values < 0 are not allowed

    *time = qml_ros2_plugin::Time( t );
    return ok;
  }
  if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Duration>() ) {
    auto duration = static_cast<qml_ros2_plugin::Duration *>( item_data->data() );
    builtin_interfaces::msg::Duration d = duration->getDuration();
    bool ok = false;
    if ( name_ == "sec" )
      d.sec = value.toInt( &ok );
    else
      d.nanosec = value.toUInt( &ok );
    if ( !ok )
      return false;
    *duration = qml_ros2_plugin::Duration( d );
    return ok;
  }
  if ( item_data->type() != QMetaType::QVariantMap ) {
    QML_ROS2_PLUGIN_ERROR( "MessageTreeItem::setItemData: Unknown type of member '%d'!",
                           item_data->type() );
    return {};
  }
  auto *compound_msg = static_cast<QVariantMap *>( item_data->data() );
  ( *compound_msg )[name_] = value;
  return true;
}

bool MessageTreeItem::insertChildren( int index, int count )
{
  if ( type_ == ArrayElement || root_introspection_ != nullptr || !introspection_->value->is_array_ ) {
    return false;
  }
  if ( index < 0 || index > childCount() )
    return false;
  const auto max_size = static_cast<int>( introspection_->value->array_size_ );
  const bool is_bounded = introspection_->value->is_upper_bound_;
  const bool is_fixed_length = !introspection_->value->is_upper_bound_ && max_size > 0;
  if ( is_fixed_length || ( is_bounded && childCount() + count > max_size ) ) {
    // Fixed size array, cannot insert rows, or extend beyond upper bound
    return false;
  }
  QVariant *item_data = getItemDataPtr();
  if ( item_data == nullptr )
    return false;

  if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Array>() ) {
    auto *array = static_cast<qml_ros2_plugin::Array *>( item_data->data() );
    QVariantList values;
    for ( int i = 0; i < count; ++i ) { values.append( createNewChild() ); }
    array->spliceList( index, 0, values );
    return true;
  }
  if ( item_data->type() != QMetaType::QVariantList ) {
    QML_ROS2_PLUGIN_ERROR( "Unexpected type in MessageItemModel when trying to insert child." );
    return false;
  }
  auto &arr_msg = *static_cast<QVariantList *>( item_data->data() );
  for ( int i = 0; i < count; ++i ) {
    if ( index + i == arr_msg.size() )
      arr_msg.append( createNewChild() );
    else
      arr_msg.insert( index + i, createNewChild() );
  }
  return true;
}

bool MessageTreeItem::removeChildren( int index, int count )
{
  if ( type_ == ArrayElement || root_introspection_ != nullptr || !introspection_->value->is_array_ ) {
    return false;
  }
  if ( index < 0 || index + count > childCount() )
    return false;
  const auto max_size = static_cast<int>( introspection_->value->array_size_ );
  const bool is_fixed_length = !introspection_->value->is_upper_bound_ && max_size > 0;
  if ( is_fixed_length ) {
    // Fixed size array, cannot remove Rows
    return false;
  }
  QVariant *item_data = getItemDataPtr();
  if ( item_data->type() == qMetaTypeId<qml_ros2_plugin::Array>() ) {
    auto *array = static_cast<qml_ros2_plugin::Array *>( item_data->data() );
    array->spliceList( index, count, {} );
    return true;
  }
  auto &arr_msg = *static_cast<QVariantList *>( item_data->data() );
  arr_msg.erase( arr_msg.begin() + index, arr_msg.begin() + index + count );
  return true;
}

bool MessageTreeItem::isArrayElement() const { return type_ == ArrayElement; }

bool MessageTreeItem::isArray() const
{
  return type_ != ArrayElement && root_introspection_ == nullptr && introspection_->value->is_array_;
}

std::string MessageTreeItem::_arrayElementType() const
{
  if ( introspection_ == nullptr )
    return {};
  ros_babel_fish::MessageMembersIntrospection members = *introspection_;
  return members.getMessageDatatype();
}

QVariant MessageTreeItem::createNewChild() const
{
  switch ( introspection_->value->type_id_ ) {
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
    auto babel_fish = BabelFishDispenser::getBabelFish();
    ros_babel_fish::MessageMembersIntrospection members = *introspection_;
    auto value = babel_fish.create_message_shared( members.getMessageName() );
    return QVariant::fromValue( conversion::msgToMap( value ) );
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
    return QVariant::fromValue( QString() );
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
    return QVariant::fromValue( false );
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
    return QVariant::fromValue<QChar>( {} );
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
    return QVariant::fromValue<int>( 0 );
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
    return QVariant::fromValue<uint>( 0 );
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
    return QVariant::fromValue<qint64>( 0 );
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
    return QVariant::fromValue<quint64>( 0 );
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
    return QVariant::fromValue<float>( 0 );
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
    return QVariant::fromValue<double>( 0 );
  default:
    return {};
  }
}

// ==================================================
// MessageItemModel Implementation
// ==================================================

MessageItemModel::MessageItemModel( QObject *parent ) : QAbstractItemModel( parent ) { }

MessageItemModel::~MessageItemModel() = default;

QModelIndex MessageItemModel::index( int row, int column, const QModelIndex &parent ) const
{
  if ( !hasIndex( row, column, parent ) )
    return {};

  MessageTreeItem *parentItem = parent.isValid()
                                    ? static_cast<MessageTreeItem *>( parent.internalPointer() )
                                    : root_item_.get();
  if ( !parentItem )
    return {};

  if ( auto *childItem = parentItem->child( row ) )
    return createIndex( row, column, childItem );
  return {};
}

QModelIndex MessageItemModel::parent( const QModelIndex &child ) const
{
  if ( !child.isValid() )
    return {};

  auto *childItem = static_cast<MessageTreeItem *>( child.internalPointer() );
  const MessageTreeItem *parentItem = childItem->parentItem();
  if ( parentItem == nullptr )
    return {};

  return parentItem != root_item_.get() ? createIndex( parentItem->row(), 0, (void *)parentItem )
                                        : QModelIndex{};
}
int MessageItemModel::rowCount( const QModelIndex &parent ) const
{
  const MessageTreeItem *parentItem =
      parent.isValid() ? static_cast<const MessageTreeItem *>( parent.internalPointer() )
                       : root_item_.get();
  if ( !parentItem )
    return 0;
  return parentItem->childCount();
}
int MessageItemModel::columnCount( const QModelIndex &parent ) const
{
  if ( parent.isValid() )
    return static_cast<MessageTreeItem *>( parent.internalPointer() )->columnCount();
  return root_item_ != nullptr ? root_item_->columnCount() : 0;
}
QVariant MessageItemModel::data( const QModelIndex &index, int role ) const
{
  if ( !index.isValid() )
    return {};
  if ( role == TreeIndexRole ) {
    return QVariant::fromValue( index );
  }
  const auto *item = static_cast<const MessageTreeItem *>( index.internalPointer() );
  if ( !item )
    return {};
  if ( role == TypeRole ) {
    return item->type();
  }
  if ( role == IsArrayElementRole ) {
    return item->isArrayElement();
  }
  if ( index.column() == 1 && role == Qt::EditRole ) {
    return item->data( index.column() ); // Only value is editable.
  }
  if ( role == Qt::DisplayRole ) {
    return item->data( index.column() );
  }
  return {};
}

Qt::ItemFlags MessageItemModel::flags( const QModelIndex &index ) const
{
  if ( !index.isValid() )
    return Qt::NoItemFlags;

  Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  if ( index.column() > 0 )
    flags |= Qt::ItemIsEditable; // Only allow editing for the second column
  return flags;
}

bool MessageItemModel::hasChildren( const QModelIndex &parent ) const
{
  if ( !parent.isValid() )
    return false;
  const auto *item = static_cast<const MessageTreeItem *>( parent.internalPointer() );
  return item->childCount() > 0;
}

bool MessageItemModel::setData( const QModelIndex &index, const QVariant &value, int role )
{
  if ( !index.isValid() || role != Qt::EditRole )
    return false;

  if ( auto *item = static_cast<MessageTreeItem *>( index.internalPointer() );
       item->setData( index.column(), value ) ) {
    emit modified();
    // Not emiting data changed for EditRole here because that would trigger rerender while the user
    // might still be modifying the data.
    emit dataChanged( index, index, { Qt::DisplayRole } );
    return true;
  }
  return false;
}

QHash<int, QByteArray> MessageItemModel::roleNames() const
{
  auto result = QAbstractItemModel::roleNames();
  result[TypeRole] = "type";           // Custom role for field type
  result[TreeIndexRole] = "treeIndex"; // Custom role because the index property is just the row
  result[IsArrayElementRole] = "isArrayElement";
  return result;
}

bool MessageItemModel::insertRows( int row, int count, const QModelIndex &parent )
{
  if ( !parent.isValid() )
    return false;
  beginInsertRows( parent, row, row + count - 1 );
  emit layoutAboutToBeChanged( { parent.parent() } );
  auto *item = static_cast<MessageTreeItem *>( parent.internalPointer() );
  bool success = item->insertChildren( row, count );
  if ( success ) {
    emit modified();
    emit dataChanged( parent.parent(), parent.parent() );
  }
  endInsertRows();
  emit layoutChanged( { parent.parent() } );
  return success;
}

bool MessageItemModel::removeRows( int row, int count, const QModelIndex &parent )
{
  if ( !parent.isValid() )
    return false;
  beginRemoveRows( parent, row, row + count - 1 );
  emit layoutAboutToBeChanged( { parent.parent() } );
  auto *item = static_cast<MessageTreeItem *>( parent.internalPointer() );
  bool success = item->removeChildren( row, count );
  if ( success ) {
    emit modified();
  }
  endRemoveRows();
  emit layoutChanged( { parent.parent() } );
  return success;
}

QVariant MessageItemModel::message() const { return content_ == nullptr ? QVariant() : *content_; }
void MessageItemModel::setMessage( const QVariant &message )
{
  beginResetModel();
  root_item_.reset();
  content_ = {};
  if ( message.isNull() ) {
    endResetModel();
    emit messageChanged();
    return;
  }
  if ( !message.canConvert<QVariantMap>() ) {
    QML_ROS2_PLUGIN_ERROR( "MessageItemModel::setMessage: Invalid message format, expected "
                           "QVariantMap or compatible type." );
    endResetModel();
    emit messageChanged();
    return;
  }
  auto map = message.value<QVariantMap>();
  QString message_type;
  if ( map.contains( "#messageType" ) )
    message_type = map["#messageType"].toString();
  if ( message_type.isEmpty() )
    message_type = message_type_; // Fallback to messageType property if not in message
  if ( message_type.isEmpty() ) {
    QML_ROS2_PLUGIN_ERROR( "MessageItemModel::setMessage: Message does not contain '#messageType' "
                           "field and no messageType is set. Can not determine type of message!" );
    endResetModel();
    emit messageChanged();
    return;
  }
  try {
    auto fish = BabelFishDispenser::getBabelFish();
    auto type_support = fish.get_message_type_support( message_type.toStdString() );
    ros_babel_fish::MessageMembersIntrospection introspection( *type_support );
    // Turn content into message and back to get missing fields and correct any broken data.
    auto msg = fish.create_message_shared( message_type.toStdString() );
    conversion::fillMessage( fish, *msg, map );
    content_ = std::make_shared<QVariant>();
    *content_ = conversion::msgToMap( msg );
    root_item_ = std::make_unique<MessageTreeItem>( content_, introspection );
    endResetModel();
    if ( message_type != message_type_ ) {
      message_type_ = message_type;
      emit messageTypeChanged();
    }
    emit messageChanged();
  } catch ( ros_babel_fish::TypeSupportException &ex ) {
    QML_ROS2_PLUGIN_ERROR(
        "MessageItemModel::setMessage: Failed to load type support for message '%s'. Error: %s",
        message_type.toStdString().c_str(), ex.what() );
    endResetModel();
    emit messageChanged();
  }
}

QString MessageItemModel::messageType() const { return message_type_; }

void MessageItemModel::setMessageType( const QString &value )
{
  if ( message_type_ == value )
    return;
  beginResetModel();
  message_type_ = value;
  bool had_content = root_item_ != nullptr;
  content_ = nullptr;
  root_item_.reset();
  endResetModel();
  emit messageTypeChanged();
  if ( had_content )
    emit messageChanged();
}

} // namespace qml_ros2_plugin
