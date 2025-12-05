=============
Ament Index
=============

The Ament Index is a system used in ROS 2 to locate resources such as packages, libraries, and other files.
It is available in QML through the ``AmentIndex`` singleton.

Example

.. code-block:: qml

  ComboBox {
    id: packageSelector
    model: AmentIndex.getPackages()
    onCurrentTextChanged: {
      let packageShare = AmentIndex.getPackageShareDirectory(currentText)
      console.log("Package share directory for", currentText, "is", packageShare)
    }
  }

API
---

.. doxygenclass:: qml6_ros2_plugin::AmentIndex
   :members:
