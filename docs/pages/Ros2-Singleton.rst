=============
Ros2 Singleton
=============

The ``Ros2`` singleton provides interfaces to static methods and convenience
methods.

In QML it is available as ``Ros2``, e.g.:

.. code-block:: qml

  if (Ros2.ok()) console.log("Ros2 is ok!")

ROS Initialization
------------------

First, you need to initialize the node used by your QML application, e.g., in the ``onCompleted`` handler:

.. code-block:: qml

  Component.onCompleted: {
    Ros2.init("node_name")
  }

You can also conditionally initialize by checking if it was already initialized using ``Ros2.isRosInitialized``.
As described in the API documentation for :cpp:func:`Ros2.init <qml_ros2_plugin::Ros2QmlSingletonWrapper::init>`, you can pass either just the
node name or additionally use provided command line args instead of the command
line args provided to your executable.

Additionally, you can use the init options to set options for the node, such as the namespace or domain id.

.. code-block:: qml

  Component.onCompleted: {
    let initOptions = Ros2.createInitOptions()
    initOptions.setNamespace("/my_namespace")
    initOptions.setDomainId(42) // or initOptions.useDefaultDomainId()
    Ros2.init("node_name", initOptions)
  }

Query Graph
------------

You can also use the Ros2 singleton to query the available topics, services and actions.
These are available as:

* | ``QVariantMap getTopicNamesAndTypes()``
  | Retrieves a map of topic names and their types as a list of strings.
* | ``QVariantMap getServiceNamesAndTypes()``
  | Retrieves a map of service names and their types as a list of strings.
* | ``QVariantMap getActionNamesAndTypes()``
  | Retrieves a map of action names and their types as a list of strings.

Note that the return value is a ``QVariantMap``, which is a wrapped ``QMap<QString, QStringList>``
to make it compatible with QML.
Also check out the ``graph_queries.qml`` example in the repo's examples folder.

Additionally, for topics three convenience methods are also provided:

* | ``QStringList queryTopics( const QString &datatype = QString())``
  | Queries a list of topics with the given datatype or all topics if no type provided.
* | ``QList<TopicInfo> queryTopicInfo()``
  | Retrieves a list of all advertised topics including their datatypes. See :cpp:class:`TopicInfo`
* | ``QString queryTopicTypes( const QString &name )``
  | Retrieves the datatypes for a given topic.

Example:

.. code-block:: qml

  // Retrieves a list of topics with the type sensor_msgs/Image
  var topics = Ros2.queryTopics("sensor_msgs/msg/Image")
  // Another slower and less clean method of this would be
  var cameraTopics = []
  var topics = Ros2.queryTopicInfo()
  for (var i = 0; i < topics.length; ++i) {
    if (topics[i].datatypes.includes("sensor_msgs/msg/Image")) cameraTopics.push(topics[i].name)
  }
  // The types of a specific topic can be retrieved as follows
  var datatypes = Ros2.queryTopicTypes("/topic/that/i/care/about")
  // Using this we can make an even worse implementation of the same functionality
  var cameraTopics = []
  var topics = Ros2.queryTopics() // Gets all topics
  for (var i = 0; i < topics.length; ++i) {
    if (Ros2.queryTopicTypes(topics[i]).includes("sensor_msgs/msg/Image")) cameraTopics.push(topics[i])
  }

Create Empty Message
--------------------
You can also create empty messages and service requests as javascript objects using the ``Ros2`` singleton.

.. code-block:: qml

  var message = Ros2.createEmptyMessage("geometry_msgs/msg/Point")
  // This creates an empty instance of the mssage, we can override the fields
  message.x = 1; message.y = 2; message.z = 1

  // Same can be done with service requests
  var serviceRequest = Ros2.createEmptyServiceRequest("std_srvs/srv/SetBool")
  // This creates an empty instance of the service request with all members set to their
  // default, we can override the fields
  serviceRequest.data = true

Logging
-------
The Ros2 singleton also provides access to the ``Ros2`` logging functionality.
See `Logging`:ref:.

IO
--
You can also save and read data that can be serialized in the yaml format using:

.. code-block:: qml

  var obj = {"key": [1, 2, 3], "other": "value"}
  if (!Ros2.io.writeYaml("/home/user/file.yaml", obj))
    Ros2.error("Could not write file!")
  // and read it back
  obj = Ros2.io.readYaml("/home/user/file.yaml")
  if (!obj) Ros2.error("Failed to load file!")

API
---

.. doxygenclass:: qml_ros2_plugin::Ros2InitOptions
  :members:

.. doxygenclass:: qml_ros2_plugin::TopicInfo
  :members:

.. doxygenclass:: qml_ros2_plugin::IO
  :members:

.. doxygenclass:: qml_ros2_plugin::QoSWrapper
  :members:

.. doxygenclass:: qml_ros2_plugin::Ros2QmlSingletonWrapper
  :members:
