// PythonObject.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef PYTHON_OBJECT_H
#define PYTHON_OBJECT_H

#ifdef USE_PYTHON
#include <Python.h>

class PythonObject final
{

public:

  PythonObject( PyObject* object );

  PythonObject( PythonObject&& other );

  ~PythonObject();

  PythonObject& operator=( PythonObject&& other );

  operator PyObject*() const;

private:

  PyObject* m_object;

};
#endif

#endif
