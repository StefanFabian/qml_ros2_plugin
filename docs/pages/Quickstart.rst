==========
Quickstart
==========

This library provides convenient access of ROS2 concepts and functionalities
in QML.

Installation
============

*Note:* Currently, only Linux is supported. Other platforms have not been tested.

From Source
-----------

To install ``qml_ros2_plugin`` from source, clone the
`repo <https://github.com/StefanFabian/qml_ros2_plugin>`_.
Now, you have two options: You can either install the plugin in your ROS2 overlay which makes the plugin
available only if you've sourced the overlay in your environment.
Alternatively, you can enable the global install, to install it system-wide on linux.

Local Install
.............
``cd`` into your workspace root directory and ``colcon build``. Re-source your ``ìnstall/setup.bash``.

Global install
..............

``cd`` into the repo folder.
To install create a build folder, ``cd`` into that folder and run
``cmake -DGLOBAL_INSTALL=ON ..`` followed by ``make`` and ``sudo make install``.

.. code-block:: bash

   mkdir build && cd build
   cmake -DGLOBAL_INSTALL=ON ..
   make -j8 # Replace 8 by the number of cpu cores
   sudo make install

Usage
=====

To use the plugin import ``Ros2`` in QML.

.. code-block:: qml

  import Ros2 1.0

Now, you can use the provided components such as ``Subscription`` and
``TfTransform`` and the :ref:`Ros2 Singleton` to create a ``Publisher``, a
``ServiceClient``, or an ``ActionClient``.

As a simple example, a ``Subscription`` can be created as follows:

.. code-block:: qml
  :linenos:

  Subscription {
    id: mySubscription
    topic: "/intval"
  }

or a button calling a service or action client:

.. code-block:: qml
  :linenos:

  Button {
    property var client: Ros2.createServiceClient("/add_two_ints", "example_interfaces/srv/AddTwoInts")
    property bool sending: false
    text: !sending ? "Do something" : "Working..."
    onClicked: {
      // Limit to one service call at a time but this is optional, you can do multiple calls in parallel
      if (sending) return
      sending = true
      myServiceClient.callAsync({ a: 42, b: 1337 }, function(result) {
        // The response callback will be called once the service response is received or the call failed.
        // In that case result will be false.
        sending = false
        if (!result) {
          console.log("Service call failed!")
          return
        }
        console.log("Service response:", result.sum)
      })
    }
  }

For more in-depth examples, check out the :ref:`Examples` section.

Initialization
--------------
Before a ``Subscription`` can receive messages, a ``Publisher`` can publish messages, etc. the node has to be
initialized. This has to be done once per application.

.. code-block:: qml
  :linenos:

  ApplicationWindow {
    /* ... */
    Component.onCompleted: {
      Ros2.init("node_name");
    }
  }

Shutdown
--------
 .. note::
   The following is not always necessary, only add it if Ctrl+C does not properly exit your application.
   The plugin will clean automatically if it can properly detect the Qt application shutdown.

To make your application quit when ROS shuts down, e.g., because of a ``Ctrl+C`` in the console, you can connect
to the ``Shutdown`` signal:

.. code-block:: qml
  :linenos:

  ApplicationWindow {
    Connections {
      target: Ros2
      function onShutdown() {
        Qt.quit()
      }
    }
    /* ... */
  }

For more on that, check out the :ref:`Ros2 Singleton`.
