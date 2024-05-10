import QtQuick 2.3
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.0
import Ros2 1.0

ApplicationWindow {
  id: page
  width: 800
  height: 600

  // This connection makes sure the application exits if this ROS node is requested to shutdown
  Connections {
    target: Ros2
    function onShutdown() {
      Qt.quit()
    }
  }

  // Arguments are: Message Type, Topic, Queue Size
  property var cmdVelPublisher: Ros2.createPublisher("/turtle1/cmd_vel", "geometry_msgs/msg/Twist", 10)

  TfTransform {
    id: tfTransform
    enabled: true // This is the default, if false no updates will be received
    sourceFrame: "turtle1"
    targetFrame: "world"
  }
  
  // Helper functions to print the translation and rotation of the transform
  function printVector3(pos) {
    return "    x: " + pos.x.toFixed(3) + "\n    y: " + pos.y.toFixed(3) + "\n    z: " + pos.z.toFixed(3)
  }

  function printRotation(q) {
    return "    w: " + q.w.toFixed(4) + "\n    x: " + q.x.toFixed(4) + "\n    y: " + q.y.toFixed(4) + "\n    z: " + q.z.toFixed(4)
  }

  function extractYaw(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
  }

  GridLayout {
    id: grid
    anchors.fill: parent
    columns: 3
    rows: 2

    Canvas {
      Layout.row: 0
      Layout.column: 0
      Layout.fillHeight: true
      Layout.fillWidth: true
      Layout.margins: 4
      Layout.preferredWidth: grid.width / 2

      onPaint: {
        var ctx = getContext("2d")
        ctx.resetTransform()
        ctx.fillStyle = Qt.rgba(0.8, 0.8, 0.8, 1)
        ctx.fillRect(0, 0, width, height)
        var posX = tfTransform.translation.x * width / 12
        var posY = height - tfTransform.translation.y * height / 11
        ctx.save()
        ctx.translate(posX, posY)
        ctx.rotate(1.57 - extractYaw(tfTransform.rotation))
        ctx.beginPath()
        ctx.fillStyle = "rgb(255, 0, 0)"
        ctx.lineTo(8, 2)
        ctx.lineTo(-8, 2)
        ctx.lineTo(0, -6)
        ctx.closePath()
        ctx.fill()
        ctx.fillStyle = "rgb(0, 0, 255)"
        ctx.fillRect(-8, 2, 16, 4)
        ctx.restore() // Restore untransformed state
        ctx.translate(width / 2, height / 2)
        ctx.fillStyle = "rgb(0, 0, 0)"
        ctx.fillRect(-2, -2, 4, 4)
      }

      Timer {
        interval: 32; running: true; repeat: true
        onTriggered: parent.requestPaint()
      }
    }

    ColumnLayout {
      Layout.column: 0
      Layout.row: 1
      Layout.preferredWidth: grid.width / 2

      RowLayout {
        Layout.alignment: Qt.AlignHCenter
        Layout.bottomMargin: 4
        spacing: 12

        Text {
          Layout.alignment: Qt.AlignTop
          text: "Position:\n" + printVector3(tfTransform.translation)
        }
        Text {
          Layout.alignment: Qt.AlignTop
          text: "Orientation:\n" + printRotation(tfTransform.rotation)
        }
      }

      Text {
        function pad(val) {
          return val < 10 ? '0' + val : val
        }

        function printDateTime(d) {
          return pad(d.getDate()) + "." + pad(d.getMonth() + 1) + "." + d.getYear()
              + " " + pad(d.getHours()) + ":" + pad(d.getMinutes())
              + ":" + pad(d.getSeconds()) + "." + d.getMilliseconds()
        }

        text: "Last update: " + printDateTime(tfTransform.message.header.stamp.toJSDate())
      }
    }

    Item {
      Layout.row: 0
      Layout.column: 1
      Layout.fillHeight: true
      Layout.fillWidth: true
      Slider {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        id: sliderForward
        orientation: Qt.Vertical
        from: -1
        to: 1
        value: 0
      }

      Button {
        anchors.left: sliderForward.right
        anchors.verticalCenter: sliderForward.verticalCenter
        text: "0"
        onClicked: sliderForward.value = 0
      }
    }

    ColumnLayout {
      Layout.row: 1
      Layout.column: 1
      Layout.preferredWidth: grid.width / 4
      Slider {
        Layout.fillWidth: true
        id: sliderTurn
        from: -3
        to: 3
        value: 0
      }

      Button {
        Layout.alignment: Qt.AlignHCenter
        text: "0"
        onClicked: sliderTurn.value = 0
      }
    }

    Subscription {
      id: cmdVelSubscriber
      topic: "/turtle1/cmd_vel"
    }

    Text {
      Layout.row: 0
      Layout.column: 2
      Layout.margins: 12
      Layout.alignment: Qt.AlignHCenter
      text: "Linear:\n" + (cmdVelSubscriber.message && printVector3(cmdVelSubscriber.message.linear) || "Unknown")
      + "\nAngular:\n" + (cmdVelSubscriber.message && printVector3(cmdVelSubscriber.message.angular) || "Unknown")
    }
  }

  // Timer is bound to animation timer which is usually set to 60 fps so resolution will be at best 16ms
  Timer {
    interval: 32; running: true; repeat: true;
    // Angular has to be inverted because right is positive 
    onTriggered: page.cmdVelPublisher.publish({ linear: { x: sliderForward.value }, angular: { z: -sliderTurn.value } })
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("turtle_tutorial_control")
  }

}
