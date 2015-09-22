// PythonObject.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef PYTHON_OBJECT_H
#define PYTHON_OBJECT_H

#ifdef USE_PYTHON
#include <Python.h>

class PythonObject final
{

public:

  explicit PythonObject( PyObject* object );

  PythonObject( PythonObject&& other );

  ~PythonObject();

  PythonObject& operator=( PythonObject&& other );

  operator PyObject*() const;

private:

  PythonObject( PythonObject& ) = delete;
  void operator=( const PythonObject& ) = delete;

  PyObject* m_object;

};

#endif

#endif
