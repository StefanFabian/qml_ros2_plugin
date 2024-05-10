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
    columns: 2
    rows: 2

    Canvas {
      Layout.row: 0
      Layout.column: 0
      Layout.fillHeight: true
      Layout.fillWidth: true
      Layout.margins: 4
      width: grid.width / 2

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

        text: "Last update: " + (tfTransform.valid ? printDateTime(tfTransform.message.header.stamp.toJSDate()) : "Never")
      }
    }

    ColumnLayout {
      id: lookUpLayout
      Layout.column: 1
      Layout.preferredWidth: grid.width / 2
      Text {
        text: "Look up your own transform:"
      }

      RowLayout {
        Layout.fillWidth: true
        Text {
          text: "Source Frame:"
        }
        Rectangle {
          Layout.alignment: Qt.AlignRight
          width: 120
          height: inputSourceFrame.height
          border.color: "black"
          border.width: 1
          clip: true
          TextInput {
            width: parent.width
            id: inputSourceFrame
            text: "turtle2"
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        Text {
          text: "Target Frame:"
        }
        Rectangle {
          Layout.alignment: Qt.AlignRight
          width: 120
          height: inputTargetFrame.height
          border.color: "black"
          border.width: 1
          clip: true
          TextInput {
            width: parent.width
            id: inputTargetFrame
            text: "world"
          }
        }
      }

      Button {
        text: "Look Up"
        onClicked: {
          var transform = TfTransformListener.lookUpTransform(inputTargetFrame.text, inputSourceFrame.text)
          if (!transform.valid)
          {
            transformResult.text = "Transform from '" + inputSourceFrame.text + "' to '" + inputTargetFrame.text + "' was not valid!\n" +
                                   "Exception: " + transform.exception + "\nMessage: " + transform.message
            return
          }
          transformResult.text = "Position:\n" + printVector3(transform.transform.translation) + "\nOrientation:\n" + printRotation(transform.transform.rotation)
        }
      }
    }

    Text {
      id: transformResult
      Layout.column: 1
      Layout.row: 1
      Layout.margins: 4
      Layout.preferredWidth: grid.width / 2
      wrapMode: Text.WordWrap
    }
  }


  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_tf_transforms_demo")
    // Since not every application needs tf2, the subscription is created on-demand. That means
    // if you use a TfTransform element, or the TfTransformListener singleton, the tf2 subscription
    // is created. However, the first time the TfTransformListener is used, the subscription may not yet
    // be initialized and the first call(s) will fail (until tf2 data is received).
    // This can be avoided by using at the start of the application or some time before using one of the lookup methods:
    TfTransformListener.initialize()
  }

}
