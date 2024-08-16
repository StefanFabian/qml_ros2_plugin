import QtQuick 2.3
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.0
import QtMultimedia 5.4
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

  ImageTransportSubscription {
    id: imageSubscription
    // Enter a valid image topic here
    topic: "/image_raw"
    // This is the default transport, change if compressed is not available
    // Requires the compressed_image_transport package to be installed
    defaultTransport: "compressed"
  }
  GridLayout {
    anchors.fill: parent
    anchors.margins: 12
    columns: 1
    Text {
      text: "For this example,\n run the following in a separate terminal:"
    }
    TextInput {
      readOnly: true
      selectByMouse: true
      text: "ros2 run examples_rclcpp_minimal_service service_main"
    }
    VideoOutput {
      // Can be used in increments of 90 to rotate the video
      orientation: 0
      source: imageSubscription
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_image_subscription_demo")
  }

}
