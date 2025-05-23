==========
Publishers
==========

A Publisher is used to publish messages on a given topic for delivery
to subscribers.

Simple Example
--------------
Contrary to Subscribers, a Publisher can not be instantiated but is
created using a factory method of the Ros2 singleton.

.. code-block:: qml
  :linenos:

  /* ... */
  ApplicationWindow {
    property var intPublisher: Ros2.createPublisher("/intval", "example_interfaces/msg/Int32", 10)
    /* ... */
    property var intTransientLocalPublisher: Ros2.createPublisher("/intval_tl", "example_interfaces/msg/Int32", Ros2.QoS().transient_local())
  }

In order, the arguments are the ``topic``, the ``type`` and the ``qos``.
For backward compatibility and convenience, you can also pass an integer which will set the `history_policy` of the QoS to
``keep_last`` and the ``depth`` to the integer value.
If you only pass the topic and type, the default QoS will be used which are best effort, durability volatile and depth 1.
See the Ros2 singleton for different preconfigured QoS profiles.

To publish a message using our Publisher, we can simply use the ``intPublisher`` variable defined earlier.

.. code-block:: qml
  :linenos:

  SpinBox {
    id: numberInput
  }

  Button {
    onClicked: {
      intPublisher.publish({ data: numberInput.value })
    }
  }

where we pass an object with a data field containing the (integer) number of the ``SpinBox``.
This is according to the ``example_interfaces/msg/Int32`` `message definition <https://github.com/ros2/example_interfaces/blob/master/msg/Int32.msg>`_.

API
---

Publisher
=========
.. doxygenclass:: qml_ros2_plugin::Publisher
  :members:

.. doxygenclass:: qml_ros2_plugin::QoSWrapper
  :members:
