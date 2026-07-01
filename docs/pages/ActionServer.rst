=============
Action Server
=============

The ``ActionServer`` is a directly-creatable QML element that advertises a ROS action server and
handles goals using QML/JavaScript callbacks.

.. code-block:: qml

  ActionServer {
    name: "/fibonacci"
    type: "example_interfaces/action/Fibonacci"
    onGoalAccepted: function (handle) {
      // drive execution, e.g., using a Timer
      handle.publishFeedback({ sequence: [0, 1] })
      handle.succeed({ sequence: [0, 1, 1, 2, 3] })
    }
  }

The ``name`` and ``type`` properties select the advertised action. They can be changed at runtime to
re-target the server.

Goal handling
=============

When a goal is accepted, ``onGoalAccepted`` is called with an
:doc:`ActionServerGoalHandle <ActionServer>` ``handle``. Use it to publish feedback and to terminate
the goal:

* ``handle.publishFeedback(feedback)`` publishes a feedback message.
* ``handle.succeed(result)`` finishes the goal successfully.
* ``handle.abort(result)`` aborts the goal.
* ``handle.canceled(result)`` finishes a goal that was being canceled.

After a terminal call (``succeed``/``abort``/``canceled``) the handle is invalid and further calls
are ignored.

The handle exposes ``goal``, ``goalId`` and the state properties ``isActive``, ``isExecuting`` and
``isCanceling`` (each notified via ``statusChanged``), and the ``cancelRequested`` signal.

Accepting and rejecting goals
============================

The optional ``handleGoal`` callback decides whether a goal is accepted. It receives the ``goal`` map
and the ``goalId`` and returns a boolean. If not set, all goals are accepted.

.. code-block:: qml

  handleGoal: function (goal, goalId) { return goal.order <= 100 }

Cancellation
===========

To support cancellation, set the ``handleCancel`` callback. It receives the ``handle`` and returns a
boolean deciding whether the cancellation is accepted. When accepted, the handle emits
``cancelRequested`` (and ``isCanceling`` becomes true) so you can finalize the goal with
``handle.canceled(...)``:

.. code-block:: qml

  handleCancel: function (handle) { return true }
  onGoalAccepted: function (handle) {
    handle.cancelRequested.connect(function () {
      handle.canceled({ sequence: partialSequence })
    })
  }

Without ``handleCancel`` all cancellation requests are rejected and the goal keeps running, so an
action server must set ``handleCancel`` to support cancelling.

Caveats
=======

* ``handleGoal`` and ``handleCancel`` are invoked synchronously on the single ROS executor and block
  it until they return. Keep them fast.
* ``handleCancel`` is best-effort for a cancel that races goal acceptance: if a cancel arrives before
  the accepted goal handle was delivered to QML, it is rejected without consulting ``handleCancel``.
* A partial or empty result/feedback map sends an all-defaults message; missing fields are default
  initialized, this is not reported as an error.

API
---

.. doxygenclass:: qml_ros2_plugin::ActionServer
  :members:

.. doxygenclass:: qml_ros2_plugin::ActionServerGoalHandle
  :members:
