// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_MESSAGE_ITEM_MODEL_HPP
#define QML_ROS2_PLUGIN_MESSAGE_ITEM_MODEL_HPP

#include <QAbstractItemModel>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <ros_babel_fish/messages/value_message.hpp>

namespace qml_ros2_plugin
{

class MessageTreeItem
{
  enum Type {
    CompoundMember,
    ArrayElement,
  };

public:
  MessageTreeItem( std::shared_ptr<QVariant> root_data,
                   ros_babel_fish::MessageMembersIntrospection introspection );
  MessageTreeItem( int index, const QString &name,
                   ros_babel_fish::MessageMemberIntrospection introspection,
                   MessageTreeItem *parent_item = nullptr );
  MessageTreeItem( int index, ros_babel_fish::MessageMemberIntrospection introspection,
                   MessageTreeItem *parent_item = nullptr );

  MessageTreeItem *child( int row );
  int childCount() const;
  int columnCount() const;
  QVariant data( int column ) const;
  bool setData( int column, const QVariant &value );
  bool insertChildren( int index, int count );
  bool removeChildren( int index, int count );
  int row() const;
  MessageTreeItem *parentItem();
  QString type() const;

  //! @returns True if this item is an element in an array.
  bool isArrayElement() const;
  //! @returns True if the type of this field is an array.
  bool isArray() const;
  //! @returns The type of the elements in this array. Only valid if isArray() is true.
  std::string _arrayElementType() const;

private:
  QVariant getItemData() const;
  QVariant *getItemDataPtr() const;
  bool setItemData( QVariant value );

  QVariant createNewChild() const;

  std::vector<std::unique_ptr<MessageTreeItem>> child_items_;
  std::unique_ptr<ros_babel_fish::MessageMembersIntrospection> root_introspection_;
  std::unique_ptr<ros_babel_fish::MessageMemberIntrospection> introspection_;
  std::shared_ptr<QVariant> root_data_ = nullptr;
  QString name_;
  MessageTreeItem *parent_item_ = nullptr;
  int index_ = 0;
  Type type_;
};

class MessageItemModel : public QAbstractItemModel
{
  Q_OBJECT
  //! The message displayed by this model. The message content will be copied and changes do not
  //! reflect back to the original message.
  //! Use the @modified@ signal to detect changes to the message content.
  Q_PROPERTY( QVariant message READ message WRITE setMessage NOTIFY messageChanged )
  //! The type of the message displayed by this model.
  //! Changing this manually will reset the content so first set this before setting the message content.
  //! If the message contains a '\#messageType' field, it will overwrite the current messageType.
  Q_PROPERTY( QString messageType READ messageType WRITE setMessageType NOTIFY messageTypeChanged )

  static constexpr int TypeRole = Qt::UserRole;
  static constexpr int TreeIndexRole = Qt::UserRole + 1;
  static constexpr int IsArrayElementRole = Qt::UserRole + 2;

public:
  explicit MessageItemModel( QObject *parent = nullptr );
  ~MessageItemModel() override;

  QModelIndex index( int row, int column, const QModelIndex &parent ) const override;
  QModelIndex parent( const QModelIndex &child ) const override;
  int rowCount( const QModelIndex &parent ) const override;
  int columnCount( const QModelIndex &parent ) const override;
  QVariant data( const QModelIndex &index, int role ) const override;
  bool insertRows( int row, int count, const QModelIndex &parent ) override;
  bool removeRows( int row, int count, const QModelIndex &parent ) override;

  bool hasChildren( const QModelIndex &parent ) const override;
  bool setData( const QModelIndex &index, const QVariant &value, int role ) override;
  Qt::ItemFlags flags( const QModelIndex &index ) const override;
  QHash<int, QByteArray> roleNames() const override;

  QVariant message() const;
  void setMessage( const QVariant &message );

  QString messageType() const;
  void setMessageType( const QString &value );

signals:
  //! Emitted when the message property is changed by the user.
  void messageChanged();

  //! Emitted when the messageType property is changed either directly or due to
  //! the message changing it.
  void messageTypeChanged();

  //! Emitted whenever the message content is modified.
  void modified();

private:
  std::shared_ptr<QVariant> content_;
  std::unique_ptr<MessageTreeItem> root_item_;
  QString message_type_;
  std::function<void()> on_modified_callback_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_MESSAGE_ITEM_MODEL_HPP
