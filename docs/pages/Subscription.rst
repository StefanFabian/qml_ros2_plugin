============
Subscription
============

A subscription listens for messages on a given topic.

Simple example
--------------
First, let's start with a simple example:

.. code-block:: qml
  :linenos:

  Subscription {
    id: mySubscription
    topic: "/intval"
  }

This creates a subscription on the topic ``/intval``.
The message type will be determined when the subscription is established.
Let's assume the topic publishes an ``example_interfaces/msg/Int32`` message.

The ``example_interfaces/msg/Int32`` message is defined as follows:

.. code-block::

  int32 data

We can display the published value using a text field:

.. code-block:: qml
  :linenos:

  Text {
    text: "Published value was: " + mySubscription.message.data
  }

Whenever a new message is received on ``/intval`` the message property
is updated and the change is propagated to the text field. Thus, the text
field will always display the latest received value.

Full example
------------
In most cases, the above Subscription is sufficient. However, the Subscription
has more properties to give you more fine-grained control.

.. code-block:: qml
  :linenos:

  Subscriber {
    id: mySubscriber
    topic: "/intval"
    // Using messageType sets the type explicitly, will not connect to a
    //  publisher if the type does not match
    messageType: "example_interfaces/msg/Int32"
    throttleRate: 30 // Update rate of message property in Hz. Default: 20
    queueSize: 10
    enabled: true // Can be used to pause/unpause the subscription
    onNewMessage: doStuff(message)
  }

The ``queueSize`` property controls how many incoming messages are queued for
processing before the oldest are dropped.
Note that due to the ``throttleRate`` messages may be dropped even if the ``queueSize`` is large enough.

The ``throttleRate`` limits the rate in which QML receives updates from the given topic.
By default the Subscriber polls with 20 Hz on the UI thread and will notify of property changes
with at most this rate.
This is to reduce load and prevent race conditions that could otherwise update the message while QML
is using it since the subscriber is receiving messages in a background thread by default.

Using the ``enabled`` property, the subscription can be enabled and disabled.
If the property is set to ``false``, the subscription is shut down until it is
set to ``true`` again and subscribes to the topic again.
For example, the state of a Subscription can be toggled using a button:

.. code-block:: qml
  :linenos:

  Button {
    id: myButton
    state: "active"
    onClicked: {
      mySubscription.enabled = !mySubscription.enabled
      state = state == "active" ? "paused" : "active"
    }
    states: [
      State {
        name: "active"
        PropertyChanges {
          target: myButton
          text: "Unsubscribe"
        }
      },
      State {
        name: "paused"
        PropertyChanges {
          target: myButton
          text: "Subscribe"
        }
      }
    ]
  }

Whenever a new message is received, the newMessage signal is emitted and the
message is passed and can be accessed as ``message`` which technically refers
to the received message and not the message property of the Subscriber.
Untechnically, they are the same, though.

Finally, there's also the messageType property which holds the type of the
received message, e.g., ``example_interfaces/msg/Int32``.
If it isn't set, the type is determined from the first available publisher,
otherwise, the subscription will only connect to publishers with the correct
message type.

API
---

.. doxygenclass:: qml_ros2_plugin::Subscription
   :members:
