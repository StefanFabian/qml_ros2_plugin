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

Connects QML user interfaces to the Robot Operating System 2 (ROS2).
Please be aware that this loses some of the semantic information that the message type would normally provide since
you can subscribe to any message without specifying the type and the type is only checked if you do specify it.

* [For the Qt6 version click here](https://github.com/StefanFabian/qml6_ros2_plugin)
* [For the ROS 1 version click here](https://github.com/StefanFabian/qml_ros_plugin).

Currently, has support for the following:  
Logging, Publisher, Subscription, ImageTransportSubscription, Service client, ActionClient, TfTransform, Ament index and querying topics

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

## Quickstart

Check the examples in the [examples](examples) directory for a quick introduction on how to use this module.
Can be used to create a Subscription to any topic and message type that is available on your system.  
The type does not need to be known at the time of compilation.

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
sudo apt install doxygen python3-sphinx python3-sphinx-rtd-theme python3-breathe 
```

#### Build documentation

```bash
cd REPO/docs
make html
```

### Known limitations

* JavaScript doesn't have long double, hence, they are cast to double with a possible loss of precision
