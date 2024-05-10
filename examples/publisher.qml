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

  // Arguments are: Topic, Message Type, Queue Size
  property var intPublisher: Ros2.createPublisher("/intval", "example_interfaces/msg/Int32", 10)

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    Text {
      text: "Enter an integer to publish it on /intval:"
    }

    SpinBox {
      id: numberInput
      Layout.margins: 12
    }

    Button {
      id: activeButton
      state: "ready"
      onClicked: {
        state = "sending"
        intPublisher.publish({ data: numberInput.value })
        state = "ready"
      }
      states: [
        State {
          name: "ready"
          PropertyChanges {
            target: activeButton
            text: "Send"
          }
        },
        State {
          name: "sending"
          PropertyChanges {
            target: activeButton
            text: "Sending"
          }
        }
      ]
    }

    Text {
      text: "Is advertised: " + (intPublisher.isAdvertised ? "true" : "false")
    }

    Text {
      id: subscriberText
      property var count: 0
      text: "Subscribers: " + count

      Timer {
        interval: 250 //ms
        repeat: true
        running: true
        onTriggered: parent.count = intPublisher.getSubscriptionCount()
      }
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_publisher_demo")
  }

}
