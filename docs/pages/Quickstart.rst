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

For more in-depth examples, check out the :ref:`Examples` section.

Initialization
--------------
Before a ``Subscription`` can receive messages, a ``Publisher`` can publish
messages, etc. the node has to be initialized.

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
To make your application quit when ROS shuts down, e.g., because of a
``Ctrl+C`` in the console, you can connect
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
