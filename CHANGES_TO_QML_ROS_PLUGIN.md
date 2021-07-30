## Changes coming from qml_ros_plugin

 * No process global ros initialization anymore
   * You now have to call `Ros2.init` once which will create a custom context and node for the UI.
 * Topics can have multiple types
   * `queryTopicType` became `queryTopicTypes` and gives a list instead of a single string
   * `TopicInfo`'s `datatype` became `datatypes` and is also now a list of strings

## Ros2 Singleton
 * init now always takes the name first and the optional command line args second

### Subscriber
 * Now called Subscription
 * Support for subscriber specific namespaces has been dropped
 * `getNumPublishers` is replaced by `getPublisherCount`
 * You can specify a `messageType` to subscribe to on the topic, if no type is specified, the type is determined automatically.
 * `running` changed to `enabled` to be consistent with other items

### Service
 * Currently no singleton available. Instead a service client can be created using the Ros2 singleton.

### Action
 * `isServerConnected()` renamed to `isServerReady()`
 * sendGoalAsync params changed and instead of callbacks now receives an options object which may
   contain callback members for `onGoalResponse`, `onFeedback` and `onResult`.  
   See docs for details. 
 * swapped order of name and type in arguments to conform with subscription etc.

### TfTransform
 * Rate of 0 will now disable updates instead of removing throttling.
 * Removed `active` alias for `enabled`
