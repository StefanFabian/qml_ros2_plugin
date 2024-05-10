import QtQuick 2.3
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.0
import Ros2 1.0

// This demo hasn't been updated for ROS2 yet where there is no WallTime and Time anymore
// but rather different clocks.
ApplicationWindow {
  id: page
  width: 1200
  height: 400

  // This connection makes sure the application exits if this ROS node is requested to shutdown
  Connections {
    target: Ros2
    function onShutdown() {
      Qt.quit()
    }
  }

  property var time: Ros2.now()

  function printClockType(clock) {
    if (clock === Ros2ClockTypes.Uninitialized) {
      return "Uninitialized"
    } else if (clock === Ros2ClockTypes.Ros) {
      return "ROS"
    } else if (clock === Ros2ClockTypes.System) {
      return "System"
    } else if (clock === Ros2ClockTypes.Steady) {
      return "Steady"
    } else {
      return "Unknown"
    }
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12

    Text { text: "Time (ROS)" }

    Text { text: "seconds: " + page.time.seconds }

    Text { text: "nanoseconds: " + page.time.nanoseconds }

    Text { text: "clock type: " + printClockType(page.time.clockType) }

    Text { text: "Date: " + page.time.toJSDate() }

    Button {
      text: "Update"
      onClicked: {
        page.time = Ros2.now()
      }
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_time_demo")
  }

}
