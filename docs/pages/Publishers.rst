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
    property var intPublisher: Ros2.advertise("/intval", "example_interfaces/msg/Int32", 10)
    /* ... */
  }

In order, the arguments are the ``topic``, the ``type`` and the ``queueSize`` (defaults to 1).
Additional QoS options are currently not supported.

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
