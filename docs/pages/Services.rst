========
Services
========

You can create a ``ServiceClient`` using the :ref:`Ros2 Singleton`
Here's a short modified example of the service example provided in the 
:doc:`Examples`.

.. code-block:: qml

  Button {
    property var serviceClient: Ros2.createServiceClient("/add_two_ints", "example_interfaces/srv/AddTwoInts")
    onClicked: {
      var result = serviceClient.sendRequestAsync(
        { a: 1, b: 3 },
        function (result) {
          textResult.text = !!result ? ("Result: " + result.sum) : "Failed"
        })
    }
  }

In the first step, a service client is created. Here, the first argument is the ``service``
that is called and the second is the ``type`` of the service.

When the button is clicked, the service client is used to send a request with the first argument
being the ``request`` and the second is a callback that is called once the service returns.
The callback receives the result of the service call or the boolean value ``false`` if the
call failed.
The code will continue execution while the service call is processed, hence, if you want to
prohibit concurrent calls to the same service, you'll have to add your own logic to check
whether a service call is currently active.

API
---

.. doxygenclass:: qml_ros2_plugin::ServiceClient
  :members:
