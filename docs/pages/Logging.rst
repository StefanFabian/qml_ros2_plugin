=======
Logging
=======

Logging is done using the :ref:`Ros2 Singleton`.

Output
------

To log a message you can use one of the following methods ``debug``, ``info``, ``warn``, ``error`` and ``fatal``.

.. code-block:: qml

  Button {
    // ...
    onClicked: Ros2.info("Button clicked.")
  }

This will produce the following output:

:: code-block:: bash

  [ INFO] [1583062360.048922959]: Button clicked.

and publish the following on ``/rosout`` (unless ``NoRos2out`` was specified in the ``Ros2InitOptions``).

:: code-block:: bash

  header:
    seq: 1
    stamp:
      secs: 1583062360
      nsecs:  49001300
    frame_id: ''
  level: 2
  name: "/qml_logging_demo"
  msg: "Button clicked."
  file: "/home/stefan/qml_ros2_plugin/examples/logging.qml"
  function: "onClicked"
  line: 130
  topics: [/rosout]

The ``file``, ``function`` and ``line`` info is automatically extracted when you call the log function.

Set Verbosity
-------------

You can change the verbosity, i.e., the minimal level of logging message that is printed
(and published if enabled), using ``Ros2.console.setLoggerLevel``.
By default the logging level is set to `Info`.
To enable debug messages you can set it to `Debug` as follows:

.. code-block:: qml

  Ros2.console.setLoggerLevel(Ros2.console.defaultName, Ros2ConsoleLevels.Debug);

The first argument to that method is the name of the console to which the logging is printed.
These are identifiers used by ros to enable you to change the verbosity of a submodule of your node using
``rqt_console``.

| You can optionally change to which console you're writing by passing a second
  argument to the logging function, e.g., ``debug("Some message", "ros.my_pkg.my_submodule")``.
| This name should contain only letters, numbers, dots and underscores.
| **Important:** The name has to start with "``ros.``".

By default the value of ``Ros2.console.defaultName`` is used which evaluates to ``ros.qml_ros2_plugin``.

Possible values for the console level are: ``Debug``, ``Info``, ``Warn``, ``Error`` and ``Fatal``.
