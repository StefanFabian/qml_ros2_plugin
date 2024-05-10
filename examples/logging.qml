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

  ButtonGroup {
    id: outputLoggingLevelGroup
    onCheckedButtonChanged: {
      // Note: This might be called before onCompleted initializes Ros2 and before Ros2 is initialized we can not get
      // the node logger.
      if (!Ros2.isInitialized()) return
      switch (checkedButton.text) {
        case 'Debug':
          Ros2.getLogger().setLoggerLevel(Ros2LoggerLevel.Debug);
          break;
        case 'Info':
          Ros2.getLogger().setLoggerLevel(Ros2LoggerLevel.Info);
          break;
        case 'Warn':
          Ros2.getLogger().setLoggerLevel(Ros2LoggerLevel.Warn);
          break;
        case 'Error':
          Ros2.getLogger().setLoggerLevel(Ros2LoggerLevel.Error);
          break;
        case 'Fatal':
          Ros2.getLogger().setLoggerLevel(Ros2LoggerLevel.Fatal);
          break;
      }
    }
  }
  ButtonGroup { id: messageLoggingLevelGroup }

  GridLayout {
    anchors.fill: parent
    columns: 5

    // Output logging level
    Text {
      Layout.columnSpan: parent.columns
      text: "Output logging level"
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Debug"
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Info"
      checked: true
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Warn"
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Error"
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Fatal"
    }

    // Message logging level
    Text {
      Layout.columnSpan: parent.columns
      text: "Message logging level"
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Debug"
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Info"
      checked: true
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Warn"
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Error"
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Fatal"
    }


    TextField {
      id: logMessageField
      Layout.columnSpan: 4
      Layout.fillWidth: true
      Layout.margins: 4
      placeholderText: "Enter a message to log"
    }

    Button {
      text: "Log"

      onClicked: {
        switch (messageLoggingLevelGroup.checkedButton.text) {
          case 'Debug':
            // These log functions are convenience methods which will log using the internal node's logger.
            // Before you can use them, you need to call Ros2.init first.
            Ros2.debug(logMessageField.text);
            break;
          case 'Info':
            Ros2.info(logMessageField.text);
            break;
          case 'Warn':
            // You can also get the node's logger explicitly or by passing a name, get a different logger.
            // But this creates a new instance of the logger wrapper, so, you should probably save and reuse it.
            Ros2.getLogger().warn(logMessageField.text);
            break;
          case 'Error':
            Ros2.error(logMessageField.text);
            break;
          case 'Fatal':
            Ros2.fatal(logMessageField.text);
            break;
          default:
            Ros2.fatal("Unexpected logging level")
        }
      }
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_logging_demo")
  }

}
