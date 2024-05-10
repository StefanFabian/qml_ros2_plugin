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

  // Arguments are: Name, Message Type
  property var fibonacciClient: Ros2.createActionClient("fibonacci", "action_tutorials_interfaces/action/Fibonacci")
  property var goal_handle
  property string status: ActionGoalStatus.Unknown
  property string feedback: "[]"
  property string result: "[]"

  GridLayout {
    columns: 2
    anchors.fill: parent
    anchors.margins: 12
    Text {
      Layout.fillWidth: true
      Layout.columnSpan: 2
      text: "Enter an integer to send it to the fibonacci tutorial action server:"
    }

    TextInput {
      Layout.columnSpan: 2
      readOnly: true
      selectByMouse: true
      text: "ros2 run action_tutorials_py fibonacci_action_server"
    }

    SpinBox {
      id: numberInput
      Layout.margins: 12
    }

    GridLayout {
      Layout.fillWidth: true
      Layout.preferredWidth: parent.width * 2 / 3
      columns: 2
      Text {
        text: "Status:"
      }
      Text {
        text: status || ""
      }
      Text {
        text: "Feedback:"
      }
      Text {
        Layout.fillWidth: true
        text: feedback || ""
      }
      Text {
        text: "Result:"
      }
      Text {
        text: result || ""
      }
    }

    Button {
      id: activeButton
      state: "ready"
      onClicked: {
        if (state == "sending") {
          goal_handle.cancel();
          return
        }
        state = "sending"
        status = ActionGoalStatus.Unknown
        fibonacciClient.sendGoalAsync({ order: numberInput.value }, {
          // These callbacks are optional
          onGoalResponse(goal_handle) {
            if (!goal_handle) {
              // Goal was rejected
              return
            }
            page.goal_handle = goal_handle
            status = goal_handle.status
          },
          onFeedback(gh, feedback) {
            status = gh.status
            var newFeedback = "["
            for (var i = 0; i < feedback.partial_sequence.length; ++i) {
              newFeedback += feedback.partial_sequence.at(i)
              if (i != feedback.partial_sequence.length - 1) newFeedback += ", "
            }
            page.feedback = newFeedback + "]"
          },
          onResult(result) {
            Ros2.info("Received result for goal: " + JSON.stringify(result.goalId))
            activeButton.state = "ready"
            status = result.code
            page.result = "["
            for (var i = 0; i < result.result.sequence.length; ++i) {
              page.result += result.result.sequence.at(i)
              if (i != result.result.sequence.length - 1) page.result += ", "
            }
            page.result += "]"
          }
        })
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
            text: "Cancel"
          }
        }
      ]
    }

    Text {
      text: "Server ready: " + (fibonacciClient.ready ? "true" : "false")
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_action_client_demo")
  }
}
