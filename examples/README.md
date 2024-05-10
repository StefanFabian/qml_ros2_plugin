# Examples
Examples on how to use the QML ROS Plugin

| Dependencies            |                                                         |
|-------------------------|---------------------------------------------------------|
| qmlscene                | `sudo apt install qmlscene`                             |
| QtMultimedia            | `sudo apt install qml-module-qtmultimedia`              |
| QtQuick 2               | `sudo apt install qml-module-qtquick2`                  |
| QtQuick.Controls 2      | `sudo apt install qml-module-qtquick-controls2`         |
| QtQuick.Layouts         | `sudo apt install qml-module-qtquick-layouts`           |
| QtQuick.Window 2        | `sudo apt install qml-module-qtquick-window2`           |
| Image Transport Plugins | `sudo apt install ros-{DISTRO}-image-transport-plugins` |
| USB Cam                 | `sudo apt install ros-{DISTRO}-usb-cam`                 |

Quick install all dependencies for ROS 2 jazzy (otherwise replace jazzy with your distro)
```
sudo apt install qmlscene qml-module-qtmultimedia qml-module-qtquick2 qml-module-qtquick-controls2 qml-module-qtquick-layouts qml-module-qtquick-window2 ros-jazzy-image-transport-plugins ros-jazzy-turtle-tf2-py ros-jazzy-usb-cam
```

Run using: `qmlscene FILE`

**Example:**
```
qmlscene publisher.qml
```
----------
**Note:**
For the `tf_transforms.qml` and `turtle_demo_control.qml`, you can use the following simulation:
```
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
```


For the `image_subscription.qml`, you can use the usb_cam package and run
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=mjpeg2rgb
```
to stream your webcam.

**Troubleshooting:** If you get an error that compressed is not available, you're missing the `image_transport_plugins`.   
Install on Ubuntu using
```
sudo apt install ros-{DISTRO}-image-transport-plugins
```

If you get "Could not find compatible format for video surface.", you most likely used yuyv which currently is not supported as the compression in ROS does not work correctly.
