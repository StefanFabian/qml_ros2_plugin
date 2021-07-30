=============
Tf Transforms
=============

There are two methods for looking up **tf2** transforms.

Component
---------
The ``TfTransform`` component can be used to createSubscription to transforms between
two frames.

.. code-block:: qml

  TfTransform {
    id: tfTransform
    active: true // This is the default, if false no updates will be received
    sourceFrame: "turtle1"
    targetFrame: "world"
  }

It provides a ``valid`` property that indicates if a valid transform has been received.
If it is valid, it contains a ``transform`` property with the stamped transform ``geometry_msgs/msg/TransformStamped``
and for convenience also a ``translation`` and a ``rotation`` property which refer to the translation and rotation in
the transform.

Using the ``rate`` property, you can also change the maximum rate at which the transform is updated.

Static
------
You can also use the ``TfTransformListener`` singleton to look up transforms if you just need it once.

.. code-block:: qml

  Button {
    text: "Look Up"
    onClicked: {
      var transformStamped = TfTransformListener.lookUpTransform(inputTargetFrame.text, inputSourceFrame.text)
      if (!transformStamped.valid)
      {
        transformResult.text = "Transform from '" + inputSourceFrame.text + "' to '" + inputTargetFrame.text + "' was not valid!\n" +
                                "Exception: " + transformStamped.exception + "\nMessage: " + transformStamped.message
        return
      }
      transformResult.text = "Position:\n" + printVector3(transformStamped.transform.translation) + "\nOrientation:\n" + printRotation(transformStamped.transform.rotation)
    }
  }

Use the provided ``Ros2.now()`` static methods to look up at specific time
points. For the latest, you can pass ``new Date(0)``.
Be aware that in JavaScript durations are given in milliseconds.


.. Warning:: Be aware that `canLookUp` can return a ``boolean`` value or
  a ``string`` error message. You should explicitly test for that since strings
  are truthy, too.

API
---

.. doxygenclass:: qml_ros2_plugin::TfTransformListener
  :members:

.. doxygenclass:: qml_ros2_plugin::TfTransform
  :members:
