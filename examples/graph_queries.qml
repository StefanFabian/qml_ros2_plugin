import QtQuick 2.6
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
  property var topics: ({})
  property var services: ({})
  property var actions: ({})

  function update() {
    Ros2.info("Updating.")
    topics = Ros2.getTopicNamesAndTypes()
    services = Ros2.getServiceNamesAndTypes()
    actions = Ros2.getActionNamesAndTypes()
  }
  function printNamesAndTypes(map) {
    let result = ""
    for (let key in map) {
      result += key + ": " + map[key] + "\n"
    }
    return result
  }

  ScrollView {
    anchors.fill: parent

    ColumnLayout {
      anchors.margins: 12


      Button {
        Layout.columnSpan: 3
        text: "Update"
        onClicked: page.update()
      }

      Text {
        text: "Topics:"
      }
      Text {
        leftPadding: 12
        text: printNamesAndTypes(topics)
      }
      Text {
        text: "Services:"
      }
      Text {
        leftPadding: 12
        text: printNamesAndTypes(services)
      }
      Text {
        text: "Actions:"
      }
      Text {
        leftPadding: 12
        text: printNamesAndTypes(actions)
      }
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_graph_queries_demo")
    update()
  }

}
