^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qml_ros2_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.26.31 (2026-03-10)
--------------------
* Interface changes for kilted.
* Contributors: Stefan Fabian

3.26.30 (2026-03-10)
--------------------
* Fixed yaml conversion not handling QJSValue correctly.
* More control over time to be in line with qml6_ros2_plugin.
* Feature/backport qml6 fixes (`#14 <https://github.com/StefanFabian/qml_ros2_plugin/issues/14>`_)
  * Backported fixes from qml6_ros2_plugin.
  * Add backport workflow.
  * Backport fixes.
* Also install to lib to enable other packages to use this as a library.
* Fixed a weird threading bug with the wait mechanism in ServiceClient when the Service was not ready.
  Added pendingRequests property.
* Try to improve test robustness involving ROS communication.
* Renamed aboutToShutdown signal in wrapper in accordance with Ros2Qml singleton.
  Fixed test after change of how cleanup is done.
* Wait in service request until service is ready.
* Updated logging docs.
* Added more clean shutdown method to avoid middleware crashes due to destruction order of static singletons.
* Added name and type property to ActionClient.
* Added query methods for services and actions.
* Hopefully fixed a weird crash in service client destructor.
* Fixed QJSValue not assigned to CompoundArrayMessage element.
* Fixed crash in action client when trying to extract callback and updated docs.
* Wait for service client to connect when sending request instead of failing. This enables use right after creation without having to wait until the service is connected.
* Added update watcher to fire status changed events for GoalHandle.
* Updated CI for rolling.
* QoS wrapper to set Quality of Service settings for Publishers, Subscriptions and Services (`#12 <https://github.com/StefanFabian/qml_ros2_plugin/issues/12>`_)
  * Added QoS settings to configure publishers, subscriptions and service clients.
  * Improved throttle rate logic and allow to set it to 0 to receive all message.
  This is especially useful if you want to use a keep_last policy >1, maybe even with transient_local.
  * Updated docs.
  * Fixed sendGoalSync not actually returning a GoalHandle ever.
  Will now return one with the future and a status of Unknown until it the future is done.
  * Improved test wait logic.
  * Improved subscribe logic.
* Added init options to set namespace or domain id.
* Add getter for hostname.
* Fix Subscription Full example code
* Fix standard library operator = of thread calling terminate if the previous thread is joinable which causes the process to die.
* Fixed handling of transport load exception broken by move of subscription to background thread.
* Enable logger service for qml Node.
* Don't list hidden _action services in getServiceNames methods.
* Install include dirs and export properly to enable use as library.
* Changed ImageTransportManager to only require supported pixel formats instead of surface so it can be used in abstractions.
* Added support yuyv and uyvy if no conversion necessary.
* Contributors: Stefan Fabian, Tomohiko Sashimura

2.25.2 (2025-02-07)
-------------------
* Apply required changes due to refactoring of array size in ros_babel_fish.
* Updated communication test message fields in accordance to renaming in ros_babel_fish_test_msgs.
* Fixed crashes when exiting application due to node still being used.
* Added method to create an empty action goal for a given action with the Ros2 singleton.
* Fixed possible crash if querying services/actions before node is initialized and downgraded error to warning.
  Will just return no results if not initialized yet.
* Added convenience functions to get types for given topic/service/action.
* Added name and type properties to ServiceClient.
* Made image transport test more robust.
* Added graph queries getTopicNamesAndTypes, getServiceNamesAndTypes and getActionNamesAndTypes to Ros2 singleton.
* Small quality refactorings.
* Contributors: Stefan Fabian

1.0.1 (2024-08-19)
------------------
* Added missing dependencies.
* Contributors: Stefan Fabian

1.0.0 (2024-08-16)
------------------
* Initial release.
* Contributors: Stefan Fabian
