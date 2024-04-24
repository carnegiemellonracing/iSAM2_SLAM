// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from eufs_msgs:msg/ConeWithCovariancePlus.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "eufs_msgs/msg/detail/cone_with_covariance_plus__struct.h"
#include "eufs_msgs/msg/detail/cone_with_covariance_plus__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool eufs_msgs__msg__cone_with_covariance_plus__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[64];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("eufs_msgs.msg._cone_with_covariance_plus.ConeWithCovariancePlus", full_classname_dest, 63) == 0);
  }
  eufs_msgs__msg__ConeWithCovariancePlus * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // blue_prob
    PyObject * field = PyObject_GetAttrString(_pymsg, "blue_prob");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->blue_prob = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yellow_prob
    PyObject * field = PyObject_GetAttrString(_pymsg, "yellow_prob");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yellow_prob = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // orange_prob
    PyObject * field = PyObject_GetAttrString(_pymsg, "orange_prob");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->orange_prob = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // big_orange_prob
    PyObject * field = PyObject_GetAttrString(_pymsg, "big_orange_prob");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->big_orange_prob = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // unknown_prob
    PyObject * field = PyObject_GetAttrString(_pymsg, "unknown_prob");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->unknown_prob = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // point
    PyObject * field = PyObject_GetAttrString(_pymsg, "point");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__point__convert_from_py(field, &ros_message->point)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // covariance
    PyObject * field = PyObject_GetAttrString(_pymsg, "covariance");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 4;
      double * dest = ros_message->covariance;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * eufs_msgs__msg__cone_with_covariance_plus__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ConeWithCovariancePlus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("eufs_msgs.msg._cone_with_covariance_plus");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ConeWithCovariancePlus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  eufs_msgs__msg__ConeWithCovariancePlus * ros_message = (eufs_msgs__msg__ConeWithCovariancePlus *)raw_ros_message;
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // blue_prob
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->blue_prob);
    {
      int rc = PyObject_SetAttrString(_pymessage, "blue_prob", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yellow_prob
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yellow_prob);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yellow_prob", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // orange_prob
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->orange_prob);
    {
      int rc = PyObject_SetAttrString(_pymessage, "orange_prob", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // big_orange_prob
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->big_orange_prob);
    {
      int rc = PyObject_SetAttrString(_pymessage, "big_orange_prob", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // unknown_prob
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->unknown_prob);
    {
      int rc = PyObject_SetAttrString(_pymessage, "unknown_prob", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // point
    PyObject * field = NULL;
    field = geometry_msgs__msg__point__convert_to_py(&ros_message->point);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "point", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // covariance
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "covariance");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->covariance[0]);
    memcpy(dst, src, 4 * sizeof(double));
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
