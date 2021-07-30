==============
ImageTransport
==============

Seeing what the robot sees is one of the most important features of any user interface.
To enable this, this library provides the
:cpp:class:`ImageTransportSubscription <qml_ros2_plugin::ImageTransportSubscription>`.
It allows easy subscription of camera messages and provides them in a QML native format as a VideoSource.

Example:

.. code-block:: qml

  ImageTransportSubscription {
    id: imageSubscription
    // Enter a valid image topic here
    topic: "/front_rgbd_cam/color/image_rect_color"
    // This is the default transport, change if compressed is not available
    defaultTransport: "compressed"
  }

  VideoOutput {
    anchors.fill: parent
    // Can be used in increments of 90 to rotate the video
    orientation: 90
    source: imageSubscription
  }

The :cpp:class:`ImageTransportSubscription <qml_ros2_plugin::ImageTransportSubscription>` can be used as
the source of a ``VideoOutput`` to display the camera images as they are received.
Additionally, it can be configured to show a blank image after x milliseconds using the ``timeout`` property
which is set to 3000ms (3s) by default. This can be disabled by setting the ``timeout`` to 0.
If you do not want the full camera rate, you can throttle the rate by setting ``throttleRate`` to a value
greater than 0 (which is the default and disables throttling). E.g. a rate of 0.2 would show a new frame
every 5 seconds.
Since there is no ROS functionality for a throttled subscription, this means the ``image_transport::Subscriber``
is shut down and newly subscribed for each frame. This comes at some overhead, hence, it should only be used
to throttle to low rates <1.
To avoid all throttled subscribers subscribing at the same time causing huge network spikes, the throttled rates
are load balanced by default. This can be disabled globally using
:cpp:func:`ImageTransportManager::setLoadBalancingEnabled <qml_ros2_plugin::ImageTransportManagerSingletonWrapper::setLoadBalancingEnabled>`
which is available in QML using the singleton ``ImageTransportManager``.

API
---

.. doxygenclass:: qml_ros2_plugin::ImageTransportSubscription
   :members:

.. doxygenclass:: qml_ros2_plugin::ImageTransportManagerSingletonWrapper
   :members:
