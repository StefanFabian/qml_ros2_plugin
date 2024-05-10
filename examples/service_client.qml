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

  property var serviceClient: Ros2.createServiceClient("/add_two_ints", "example_interfaces/srv/AddTwoInts")

  GridLayout {
    anchors.fill: parent
    anchors.margins: 12
    columns: 2
    Text {
      Layout.columnSpan: 2
      text: "For this example,\n run the following in a separate terminal:"
    }

    TextInput {
      Layout.columnSpan: 2
      readOnly: true
      selectByMouse: true
      text: "ros2 run examples_rclcpp_minimal_service service_main"
    }

    Text {
      Layout.columnSpan: 2
      text: serviceClient.ready ? "Server is ready!" : "Waiting for server..."
    }

    Text {
      Layout.alignment: Qt.AlignRight
      text: "A:"
    }

    SpinBox {
      id: inputA
      Layout.margins: 12
    }

    Text {
      Layout.alignment: Qt.AlignRight
      text: "B:"
    }

    SpinBox {
      id: inputB
      Layout.margins: 12
    }

    Button {
      id: activeButton
      state: "ready"
      onClicked: {
        state = "sending"
        textResult.text = "Pending..."
        serviceClient.sendRequestAsync(
          { a: inputA.value, b: inputB.value },
          function (result) {
            textResult.text = !!result ? ("Result: " + result.sum) : "Failed"
            state = "ready"
          }
        )
      }
      states: [
        State {
          name: "ready"
          PropertyChanges {
            target: activeButton
            text: "Call Service"
          }
        },
        State {
          name: "sending"
          PropertyChanges {
            target: activeButton
            text: "Processing"
          }
        }
      ]
    }

    Text {
      id: textResult
      text: "No result yet"
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_service_demo")
  }

}
