// PythonTools.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "PythonTools.h"

#include <iostream>

#ifdef USE_PYTHON
#include "PythonObject.h"
#endif

void PythonTools::pythonCommand( const std::string& command )
{
  #ifdef USE_PYTHON
  if( PyRun_SimpleString( ( command + "\n" ).c_str() ) != 0 )
  {
    std::cerr << "Failed to execute Python command: " << command << std::endl;
    std::cerr << "Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  #else
  std::cerr << "Error, PythonTools::pythonCommand must be compiled with Python support. Exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

// TODO: For debugging, try printing sys.modules, see if the module is already here on loading... if so
//       have to reload it? 'sys' in sys.modules
void PythonTools::loadModule( const std::string& path, const std::string& module_name, PythonObject& loaded_module )
{
  #ifdef USE_PYTHON
  // Load the module
  PythonTools::pythonCommand( "sys.path.insert( 0, '" + path + "' )" );
  loaded_module = PythonObject{ PyImport_ImportModule( module_name.c_str() ) };
  // Ensure that the module loaded without error
  if( loaded_module == nullptr )
  {
    assert( PyErr_Occurred() != nullptr );
    PyErr_Print();
    std::cerr << "Failed to load python module " << module_name << ", exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( PyErr_Occurred() == nullptr );
  // Force a reload of the module in case it was previously loaded and the user edited source
  PyObject* reloaded_module{ PyImport_ReloadModule( loaded_module ) };
  if( reloaded_module == nullptr )
  {
    assert( PyErr_Occurred() != nullptr );
    PyErr_Print();
    std::cerr << "Failed to reload python module " << module_name << ", exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( reloaded_module == loaded_module );
  PythonTools::pythonCommand( "sys.path.pop( 0 )" );
  #else
  std::cerr << "Error, PythonTools::loadModule must be compiled with Python support. Exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void PythonTools::loadFunction( const std::string& function_name, PythonObject& loaded_module, PythonObject& function )
{
  #ifdef USE_PYTHON
  PythonObject new_function{ PyObject_GetAttrString( loaded_module, function_name.c_str() ) };
  if( new_function == nullptr )
  {
    // Silence the error message
    PyErr_Clear();
  }
  if( new_function != nullptr && PyCallable_Check( new_function ) == 0 )
  {
    std::cout << "Error, " << function_name << " is not callable, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  using std::swap;
  swap( function, new_function );
  #else
  std::cerr << "Error, PythonTools::loadFunction must be compiled with Python support. Exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}
