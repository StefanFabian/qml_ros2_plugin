================
MessageItemModel
================

The ``MessageItemModel`` is derived from ``QAbstractListModel`` and provides a way to display and edit ROS2 messages in QML
views such as ``ListView``, ``GridView``, or ``TreeView``.

Example Usage:
--------------

.. code-block:: qml

    TreeView {
      Subscription {
        id: subscription
        topic: topicSelect.currentText
        onTopicChanged: {
          subscription.enabled = true
        }
        onNewMessage: {
          // Disable after receiving the first message, otherwise the TreeView keeps resetting
          enabled = false
        }
      }
      model: MessageItemModel {
        message: subscription.message
      }
      delegate: TreeViewDelegate {
        contentItem: Text {
          Layout.fillWidth: model.edit != null
          color: "#cccccc"
          text: {
            if (model.edit == null) return "" + model.display || "";
            if (model.type === "compound" || "model.type" === "array") return "";
            if (model.type === "bool") return model.edit ? "true" : "false";
            return model.edit + "";
          }
        }
      }
    }

In this example, a ``TreeView`` is used to display the structure of a ROS2 message received from a subscription.
The delegate will display both the field names and their corresponding values.
If ``model.edit`` is not null, it indicates the value of the message field and can be used to edit the message (this is
not part of this example).

 .. note::
   The message is copied when assigned to the ``message`` property of the ``MessageItemModel``.
   If you want to reflect changes made in the model back to the original message, you can use the ``onModified`` signal
   to copy the modified message back.

The ``MessageItemModel`` will automatically determine the type of the message if the message contains a ``#messageType`` field
which is automatically added for messages that are created through the QML ROS2 module.
If you pass your own message data, either provide the field or set the ``messageType`` property of the ``MessageItemModel``
before setting the message property.

API:
----
.. doxygenclass:: qml6_ros2_plugin::MessageItemModel
   :members:
