============
ActionClient
============

An action client can be used to send goals to an ActionServer.
One ActionClient has to have a single type of action but can send multiple goals simultaneously.
Process can be tracked using either a goal handle manually or using callbacks.

An ActionClient can be created using the :ref:`Ros2 Singleton` as follows:

.. code-block:: qml

  Item {
    // ...
    property var fibonacciClient: Ros2.createActionClient("fibonacci", "action_tutorials_interfaces/action/Fibonacci")
    // ...
  }

In this example an action client is created using the ``action_tutorials_interfaces/action/Fibonacci`` action
(it is important to use the complete action type here, not any part like the ActionGoal) using the name
fibonacci on which an ActionServer should be registered (You can use the action tutorials fibonacci_server).

To send a goal, you can use the `sendGoalAsync` function:

.. code-block:: qml

  if (!fibonacciClient.ready) {
    Ros2.error("Action client not ready yet!")
    return
  }
  goal_handle = fibonacciClient.sendGoalAsync({ order: numberInput.value }, {
      // These callbacks are optional
      onGoalResponse(goal_handle) {
        if (!goal_handle) {
          // Goal was rejected
          return
        }
        // Handle goal accepted
      },
      onFeedback(goal_hamdle, feedback) {
        // Handle feedback from action server
      },
      onResult(result) {
        // Handle result from action server
        let goalId = result.goalId
        if (result.code == ActionGoalStatus.Succeeded) {
          // Handle success
          let goalResult = result.result
        } else if (result.code == ActionGoalStatus.Canceled) {
          // Handle canceled
        } else if (result.code == ActionGoalStatus.Aborted) {
          // Handle aborted
        }
      }
    })

The ``sendGoalAsync`` function takes 2 parameters: the goal and an optional options object with optional
callbacks for `onGoalResponse`, `onFeedback` and `onResult`.
Both callbacks are optional. It returns a GoalHandle which can be used query to state of the goal or
to cancel the goal. The goal_handle passed to the callbacks and the one returned are the same.

API
---

.. doxygenclass:: qml_ros2_plugin::ActionClient
   :members:

.. doxygenclass:: qml_ros2_plugin::GoalHandle
   :members:

.. doxygenenum:: qml_ros2_plugin::action_goal_status::GoalStatus
   :project: project
