==============
Service Server
==============

The ``ServiceServer`` is a directly-creatable QML element that advertises a ROS service and answers
requests using a QML/JavaScript callback.

.. code-block:: qml

  ServiceServer {
    name: "/add_two_ints"
    type: "example_interfaces/srv/AddTwoInts"
    processRequest: function (request, id) {
      return { sum: request.a + request.b }
    }
  }

The ``name`` and ``type`` properties select the advertised service. They can be changed at runtime
to re-target the server.

The ``processRequest`` callback receives the ``request`` as a map and an ``id`` that identifies the
request. It can answer the request in two ways:

Synchronous response
====================

Return an object from ``processRequest`` to answer the request immediately:

.. code-block:: qml

  processRequest: function (request, id) {
    return { sum: request.a + request.b }
  }

Deferred (asynchronous) response
===============================

Return ``undefined`` from ``processRequest`` and call ``sendResponse(id, response)`` later, e.g.,
after an asynchronous operation finished:

.. code-block:: qml

  ServiceServer {
    id: server
    name: "/add_two_ints"
    type: "example_interfaces/srv/AddTwoInts"
    processRequest: function (request, id) {
      doSomethingAsync(function (result) {
        server.sendResponse(id, { sum: result })
      })
      return undefined
    }
  }

You must eventually call ``sendResponse`` for every deferred request. Otherwise the caller is left
waiting until the server is destroyed.

Notes
=====

* A partial or empty response map sends an all-defaults message; missing fields are default
  initialized, this is not reported as an error.
* If no ``processRequest`` callback is set, requests are answered with a default response so callers
  are not stranded.

API
---

.. doxygenclass:: qml_ros2_plugin::ServiceServer
  :members:
