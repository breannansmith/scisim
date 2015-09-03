// PythonTools.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef PYTHON_TOOLS_H
#define PYTHON_TOOLS_H

#ifdef USE_PYTHON
#include <Python.h>
#endif

#include <string>

class PythonObject;

namespace PythonTools
{
  // TODO: Have these functions throw errors in lieu of exiting
  #ifndef USE_PYTHON
  [[noreturn]]
  #endif
  void pythonCommand( const std::string& command );
  #ifndef USE_PYTHON
  [[noreturn]]
  #endif
  void loadModule( const std::string& path, const std::string& module_name, PythonObject& loaded_module );
  #ifndef USE_PYTHON
  [[noreturn]]
  #endif
  void loadFunction( const std::string& function_name, PythonObject& loaded_module, PythonObject& function );
}

#endif
