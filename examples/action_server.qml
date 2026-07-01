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
    Ros2.init("qml_action_server_demo")
  }

  ActionServer {
    id: fibonacciServer
    name: "/fibonacci"
    type: "example_interfaces/action/Fibonacci"

    // Optional: reject goals that ask for too many numbers.
    handleGoal: function (goal, goalId) {
      return goal.order > 0 && goal.order <= 25
    }

    // Optional: accept all cancel requests.
    handleCancel: function (handle) {
      return true
    }

    onGoalAccepted: function (handle) {
      // The execution is driven step by step using a Timer to keep the UI responsive.
      var sequence = [0, 1]
      var order = handle.goal.order

      var timer = Qt.createQmlObject(
        'import QtQuick 2.3; Timer { interval: 200; repeat: true }', page)
      handle.cancelRequested.connect(function () {
        timer.stop()
        timer.destroy()
        log.text = "Goal " + handle.goalId + " canceled."
        handle.canceled({ sequence: sequence })
      })
      timer.triggered.connect(function () {
        if (sequence.length >= order) {
          timer.stop()
          timer.destroy()
          log.text = "Goal order=" + order + " (" + handle.goalId + ") succeeded.\nResult: " + sequence.join(", ")
          handle.succeed({ sequence: sequence })
          return
        }
        log.text = "Goal order=" + order + " (" + handle.goalId + ") in progress.\nCurrent sequence: " + sequence.join(", ")
        sequence.push(sequence[sequence.length - 1] + sequence[sequence.length - 2])
        handle.publishFeedback({ sequence: sequence })
      })
      timer.start()
    }
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12

    Text {
      text: "For this example, send a goal in a separate terminal:"
    }

    TextInput {
      Layout.fillWidth: true
      readOnly: true
      selectByMouse: true
      text: "ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci \"{order: 10}\""
    }

    Text {
      text: fibonacciServer.advertised ? "Action server advertised." : "Action server not advertised."
    }

    Text {
      id: log
      Layout.fillWidth: true
      text: "No goal yet."
      wrapMode: Text.WordWrap
    }
  }
}
