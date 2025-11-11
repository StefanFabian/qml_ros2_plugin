import QtQuick.Controls.Material
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Ros2

ApplicationWindow {
  id: page
  width: 620
  height: 400

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_message_item_model_demo")
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    Label {
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      text: "This example will subscribe to the selected topic and display the first message received in a tree view using the MessageItemModel."
    }
    RowLayout {
      Layout.fillWidth: true
      ComboBox {
        id: topicSelect
        Layout.fillWidth: true
        editable: true
        model: Ros2.queryTopics()
      }
      Button {
        text: "Refresh"
        onClicked: {
          topicSelect.model = Ros2.queryTopics()
          subscription.enabled = true
        }
      }
    }
    TreeView {
      Layout.fillWidth: true
      Layout.fillHeight: true
      clip: true
      flickableDirection: Flickable.AutoFlickIfNeeded

      Subscription {
        id: subscription
        topic: topicSelect.currentText
        onTopicChanged: {
          subscription.enabled = true
        }
        onNewMessage: function (message) {
          // Disable after receiving the first message
          enabled = false
          console.log(JSON.stringify(message))
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
            if (model.type === "compound" || model.type === "array") return "";
            if (model.type === "bool") return model.edit ? "true" : "false";
            return model.edit + " (" + model.type + ")";
          }
        }
      }
      columnWidthProvider: function (column) {
        if (column === 0)
          return explicitColumnWidth(column) || implicitColumnWidth(column);

        return Math.max(160, width - columnWidth(0) - columnSpacing);
      }
    }
  }
}
