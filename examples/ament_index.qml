import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
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
    Ros2.init("qml_ament_index_demo")
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    Text {
      text: "Select a package to see its prefix and share directory:"
    }
    ComboBox {
      id: packageComboBox
      Layout.fillWidth: true
      model: AmentIndex.getPackages()
    }
    Text {
      text: "Package prefix: " + (packageComboBox.currentText && AmentIndex.getPackagePrefix(packageComboBox.currentText))
    }
    Text {
      text: "Share directory: " + (packageComboBox.currentText && AmentIndex.getPackageShareDirectory(packageComboBox.currentText))
    }
  }
}
