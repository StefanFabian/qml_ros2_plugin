[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=StefanFabian_qml_ros2_plugin&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=StefanFabian_qml_ros2_plugin)
[![Maintainability Rating](https://sonarcloud.io/api/project_badges/measure?project=StefanFabian_qml_ros2_plugin&metric=sqale_rating)](https://sonarcloud.io/summary/new_code?id=StefanFabian_qml_ros2_plugin)
[![Reliability Rating](https://sonarcloud.io/api/project_badges/measure?project=StefanFabian_qml_ros2_plugin&metric=reliability_rating)](https://sonarcloud.io/summary/new_code?id=StefanFabian_qml_ros2_plugin)
[![Security Rating](https://sonarcloud.io/api/project_badges/measure?project=StefanFabian_qml_ros2_plugin&metric=security_rating)](https://sonarcloud.io/summary/new_code?id=StefanFabian_qml_ros2_plugin)
[![Documentation Status](https://readthedocs.org/projects/qml-ros2-plugin/badge/?version=latest)](https://qml-ros2-plugin.readthedocs.io/en/latest/?badge=latest)

## Scientific Works
If you are using this module in a scientific context, feel free to cite [this paper](https://ieeexplore.ieee.org/document/9568801):
```
@INPROCEEDINGS{fabian2021hri,
  author = {Stefan Fabian and Oskar von Stryk},
  title = {Open-Source Tools for Efficient ROS and ROS2-based 2D Human-Robot Interface Development},
  year = {2021},
  booktitle = {2021 European Conference on Mobile Robots (ECMR)},
}
```

# QML ROS2 Plugin

Connects QML user interfaces to the Robot Operating System 2 (ROS2). [For the ROS 1 version click here](https://github.com/StefanFabian/qml_ros_plugin).  
Please be aware that this loses some of the semantic information that the type of a message would normally provide.

Currently, has support for the following:  
Logging, Publisher, Subscription, ImageTransportSubscription, Service client, ActionClient, TfTransform, Ament index and querying topics  

**License:** MIT

https://github.com/StefanFabian/qml_ros2_plugin/assets/2090520/c45280cf-24fe-4ff1-8423-30035deda10d

This demo interface uses Tf and a velocity publisher to control and display the turtle demo with less than 200 lines of code for the entire interface.
It is available in the examples folder as `turtle_demo_control.qml`.

**Note:** For full examples including ROS init calls and shutdown handling checkout the examples directory.

## Logging

Logging is supported and correctly reports from which qml file and line the message came!

```qml
import Ros2 1.0

Item {
  function doesWork() {
    Ros2.debug("A debug message")
    // Set the logging level to Debug (default is usually Info)
    Ros2.getLogger().setLoggerLevel(Ros2LoggerLevel.Debug);
    Ros2.debug("A debug message that is actually logged.")
    Ros2.info("I have some information")
    Ros2.warn("This is the last warning")
    Ros2.error("Great! Now there's an error.")
    Ros2.fatal("I'm dead")
    Ros2.info("Just so you know, fatal does not kill a node. Though they usually die after logging fatal")
  }
  // ...
}
```

## Subscribers

Can be used to create a Subscription to any topic and message type that is available on your system.  
The type does not need to be known at the time of compilation.

Usage example:

```qml
import Ros2 1.0

Item {
  width: 600
  height: 400
  
  Subscription {
    id: subscriber
    topic: "/test"
    onNewMessage: textField.text = message.data 
  }
  
  Text {
    text: "You can use the message directly: " + subscriber.message.data
  }
  
  Text {
    id: textField
    text: "Or you can use the newMessage signal."
  }
}
```

## Image Transport

Can be used to stream camera images.
The default transport used is "compressed".  
The stream is exposed to QML as a `QObject` with a `QAbstractVideoSurface` based `videoSurface` property
(see [QML VideoOutput docs](https://doc.qt.io/qt-5/qml-qtmultimedia-videooutput.html#source-prop)) and can be used
directly as source for the `VideoOutput` control.

Multiple ImageTransportSubscribers for the same topic share a subscription to ensure the image is converted
to a QML compatible format only once. Additionally, a throttleRate property allows to throttle the camera rate by
subscribing for one frame and shutting down again at the given rate (see documentation).

Usage example:

```qml
import QtMultimedia 5.4
import Ros2 1.0

Item {
  width: 600
  height: 400

  ImageTransportSubscription {
    id: imageSubscriber
    topic: "/front_rgb_cam"
    throttleRate: 0.2 // 1 frame every 5 seconds
  }

  VideoOutput {
    source: imageSubscriber
  }
}

```

## Tf Lookup

### TfTransformListener

A singleton class that can be used to look up tf transforms.  
Usage example:

```qml
import Ros2 1.0

Item {
  // ...
  Connections {
    target: TfTransformListener
    onTransformChanged: {
      var message = TfTransformListener.lookUpTransform("base_link", "world");
      if (!message.valid) {
        // Check message.exception and message.message for more info if it is available.
        return;
      }
      var translation = message.transform.translation;
      var orientation = message.transform.rotation;
      // DO something with the information
   }
  }
}
```

**Explanation**:  
You can use the TfTransformListener.lookUpTransform (and canTransform) methods anywhere in your QML code.
However, they only do this look up once and return the result. If you want to continuously monitor the transform, you
have to either connect to the *transformChanged* signal or use the convenience component TfTransform.
The message structure is identical to the ROS message, except for an added *valid* field (`message.valid`) indicating if
the transform returned is valid or not. If it is not valid, there may be a field *exception* containing the name of the
exception that occured and a field *message* with the message of the exception.

### TfTransform

A convenience component that watches a transform.
Usage example:

```qml
import Ros2 1.0

Item {
  // ...
  TfTransform {
    id: tfTransform
    sourceFrame: "base_link"
    targetFrame: "world"
  }
  
  Text {
    width: parent.width
    // The translation and rotation can either be accessed using the message field as in the lookUpTransform case or,
    // alternatively, using the convenience properties translation and rotation which resolve to the message fields.
    // In either case, the message.valid field should NOT be ignored.
    text: "- Position: " + tfTransform.message.transform.translation.x + ", " + tfTransform.translation.y + ", " + tfTransform.translation.z + "\n" +
          "- Orientation: " + tfTransform.message.transform.rotation.w + ", " + tfTransform.rotation.x + ", " + tfTransform.rotation.y + ", " + tfTransform.rotation.z + "\n" +
          "- Valid: " + tfTransform.message.valid + "\n" +
          "- Exception: " + tfTransform.message.exception + "\n" +
          "- Message: " + tfTransform.message.message
    wrapMode: Text.WordWrap
  }
}
```

**Explanation**:  
This component can be used to watch a transform. Whenever the transform changes, the message and the properties of the
TfTransform change and the changes are propagated by QML.

## Installation

You can either build this repository as part of your ROS2 workspace as you would any other ROS2 package, or
set the CMake option `GLOBAL_INSTALL` to `ON` which installs the plugin in your global qml module directory.  
**Please note** that the plugin will still require a ROS2 environment when loaded to be able to load the message
libraries.

Other than the source dependencies which are currently not available in the package sources, you can install
the dependencies using rosdep:  
*The following command assumes you are in the `src` folder of your ROS 2 workspace*

```
rosdep install --from-paths . --ignore-packages-from-source
```

### Source Dependencies

* [ros_babel_fish](https://github.com/LOEWE-emergenCity/ros2_babel_fish)

## Documentation

You can find the documentation on [readthedocs.io](https://qml-ros2-plugin.readthedocs.io/en/latest/index.html).

Alternatively, you can follow the steps below to build it yourself.

### Dependencies

* Doxygen
* Sphinx
* sphinx_rtd_theme
* Breathe

**Example for Ubuntu**  
Install dependencies

```bash
sudo apt install doxygen
pip3 install sphinx sphinx_rtd_theme breathe
```

#### Build documentation

```bash
cd REPO/docs
make html
```

### Known limitations

* JavaScript doesn't have long double, hence, they are cast to double with a possible loss of precision
