import QtQuick 2.3
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.0
import Ros2 1.0

ApplicationWindow {
  id: page
  width: 620
  height: 400
  visible: true

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    Ros2.init("qml_service_server_demo")
  }

  ServiceServer {
    id: addServer
    name: "/add_two_ints"
    type: "example_interfaces/srv/AddTwoInts"
    // Synchronous response: return an object to answer the request immediately.
    // If you do not return anything, you need to capture the id and call
    // sendResponse(id, response) later to answer the request asynchronously.
    processRequest: function (request, id) {
      log.text = "Received request: " + request.a + " + " + request.b
      return { sum: request.a + request.b }
    }
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12

    Text {
      text: "For this example, call the service in a separate terminal:"
    }

    TextInput {
      Layout.fillWidth: true
      readOnly: true
      selectByMouse: true
      text: "ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts \"{a: 3, b: 4}\""
    }

    Text {
      text: addServer.advertised ? "Service advertised." : "Service not advertised."
    }

    Text {
      id: log
      Layout.fillWidth: true
      text: "No request yet."
      wrapMode: Text.WordWrap
    }
  }
}
