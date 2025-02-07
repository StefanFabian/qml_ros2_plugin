^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qml_ros2_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.25.2 (2025-02-07)
-------------------
* Apply required changes due to change of array template parameters in ros_babel_fish.
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
