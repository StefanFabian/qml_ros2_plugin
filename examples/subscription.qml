import QtQuick 2.3
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.0
import Ros2 1.0

ApplicationWindow {
  id: page
  width: 620
  height: 400

  // This connection makes sure the application exits if this ROS node is requested to shutdown
  Connections {
    target: Ros2
    function onShutdown() {
      Qt.quit()
    }
  }

  Subscription {
    id: mySubscription
    //messageType: "example_interfaces/msg/Int32" // This is optional, if not specified, type is selected automatically
    topic: "/intval"
  }
  // Alternatively:
  //property var mySubscription: Ros2.createSubscription("/intval", 10)
  // or
  //property var mySubscription: Ros2.createSubscription("/intval", "example_interfaces/msg/Int32", 10)

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    Text {
      text: "Paste the following in a terminal and run it:"
    }
    TextInput {
      Layout.margins: 12
      readOnly: true
      selectByMouse: true
      text: "i=0; while [ 1 ]; do ros2 topic pub -1 /intval example_interfaces/msg/Int32 \"data: $i\"; sleep 1; let i++; done"
    }

    Text {
      text: "Received:\n" +
            "  Message type: " + mySubscription.messageType + "\n" +
            "  Message content: " + (mySubscription.message ? mySubscription.message.data : "No message received yet") + "\n" +
            "  Enabled: " + mySubscription.enabled + "\n" +
            "  Subscribed: " + mySubscription.subscribed
    }

    Button {
      id: activeButton
      state: "active"
      onClicked: {
        mySubscription.enabled = !mySubscription.enabled
        state = state == "active" ? "paused" : "active"
      }
      states: [
        State {
          name: "active"
          PropertyChanges {
            target: activeButton
            text: "Unsubscribe"
          }
        },
        State {
          name: "paused"
          PropertyChanges {
            target: activeButton
            text: "Subscribe"
          }
        }
      ]
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_subscriber_demo")
  }

}
