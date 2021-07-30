======
Arrays
======
Due to the lazy copy mechanism, arrays differ from the standard access in javascript.
Because the array is not copied into a QML compatible array container, access happens with methods.

**Example**: Instead of ``path.to.array[1].someproperty``, you would write
``path.to.array.at(1).someproperty``.

If you need the array as a javascript array, you can use :cpp:func:`toArray <qml_ros2_plugin::Array::toArray>`
to copy the entire array and return it as a javascript array.
The copy is only performed on the first call, subsequent calls should have
less overhead.

API
---
.. doxygenclass:: qml_ros2_plugin::Array
  :members:
