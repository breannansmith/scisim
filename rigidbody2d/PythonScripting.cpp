// PythonScripting.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "PythonScripting.h"

#ifdef USE_PYTHON
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include "scisim/PythonTools.h"
#include "scisim/Math/Rational.h"
#include "RigidBody2DState.h"
#endif

#include "scisim/StringUtilities.h"

#include <iostream>

#ifdef USE_PYTHON
static scalar s_timestep;
static unsigned s_next_iteration;
static RigidBody2DState* s_state;
static VectorXs* s_mu;
static VectorXs* s_cor;
static const std::vector<std::unique_ptr<Constraint>>* s_active_set;
#endif

PythonScripting::PythonScripting()
: m_path()
, m_module_name()
#ifdef USE_PYTHON
, m_loaded_module( nullptr )
, m_loaded_start_of_sim_callback( nullptr )
, m_loaded_end_of_sim_callback( nullptr )
, m_loaded_start_of_step_callback( nullptr )
, m_loaded_end_of_step_callback( nullptr )
, m_loaded_friction_coefficient_callback( nullptr )
, m_loaded_restitution_coefficient_callback( nullptr )
#endif
{}

PythonScripting::PythonScripting( const std::string& path, const std::string& module_name )
: m_path( path )
, m_module_name( module_name )
#ifdef USE_PYTHON
, m_loaded_module( nullptr )
, m_loaded_start_of_sim_callback( nullptr )
, m_loaded_end_of_sim_callback( nullptr )
, m_loaded_start_of_step_callback( nullptr )
, m_loaded_end_of_step_callback( nullptr )
, m_loaded_friction_coefficient_callback( nullptr )
, m_loaded_restitution_coefficient_callback( nullptr )
#endif
{
  intializePythonCallbacks();
}

PythonScripting::PythonScripting( std::istream& input_stream )
: m_path( StringUtilities::deserializeString( input_stream ) )
, m_module_name( StringUtilities::deserializeString( input_stream ) )
#ifdef USE_PYTHON
, m_loaded_module( nullptr )
, m_loaded_start_of_sim_callback( nullptr )
, m_loaded_end_of_sim_callback( nullptr )
, m_loaded_start_of_step_callback( nullptr )
, m_loaded_end_of_step_callback( nullptr )
, m_loaded_friction_coefficient_callback( nullptr )
, m_loaded_restitution_coefficient_callback( nullptr )
#endif
{
  intializePythonCallbacks();
}

void PythonScripting::intializePythonCallbacks()
{
  #ifdef USE_PYTHON
  // If no module name was provided, nothing to do
  if( m_module_name.empty() )
  {
    return;
  }

  // Load the module
  assert( m_loaded_module == nullptr );
  PythonTools::loadModule( m_path, m_module_name, m_loaded_module );

  // Hook up the start of sim callback
  assert( m_loaded_start_of_sim_callback == nullptr );
  PythonTools::loadFunction( "startOfSim", m_loaded_module, m_loaded_start_of_sim_callback );

  // Hook up the end of sim callback
  assert( m_loaded_end_of_sim_callback == nullptr );
  PythonTools::loadFunction( "endOfSim", m_loaded_module, m_loaded_end_of_sim_callback );

  // Hook up the start of step callback
  assert( m_loaded_start_of_step_callback == nullptr );
  PythonTools::loadFunction( "startOfStep", m_loaded_module, m_loaded_start_of_step_callback );

  // Hook up the end of step callback
  assert( m_loaded_end_of_step_callback == nullptr );
  PythonTools::loadFunction( "endOfStep", m_loaded_module, m_loaded_end_of_step_callback );

  // Hook up the friction coefficient callback
  assert( m_loaded_friction_coefficient_callback == nullptr );
  PythonTools::loadFunction( "frictionCoefficient", m_loaded_module, m_loaded_friction_coefficient_callback );

  // Hook up the restitution coefficient callback
  assert( m_loaded_restitution_coefficient_callback == nullptr );
  PythonTools::loadFunction( "restitutionCoefficient", m_loaded_module, m_loaded_restitution_coefficient_callback );
  assert( PyErr_Occurred() == nullptr );
  #else
  if( !m_module_name.empty() )
  {
    std::cerr << "Error, Python callback " << m_module_name << " requested, but program is not compiled with Python support. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  #endif
}

void swap( PythonScripting& first, PythonScripting& second )
{
  using std::swap;
  swap( first.m_path, second.m_path );
  swap( first.m_module_name, second.m_module_name );
  #ifdef USE_PYTHON
  swap( first.m_loaded_module, second.m_loaded_module );
  swap( first.m_loaded_start_of_sim_callback, second.m_loaded_start_of_sim_callback );
  swap( first.m_loaded_end_of_sim_callback, second.m_loaded_end_of_sim_callback );
  swap( first.m_loaded_start_of_step_callback, second.m_loaded_start_of_step_callback );
  swap( first.m_loaded_end_of_step_callback, second.m_loaded_end_of_step_callback );
  swap( first.m_loaded_friction_coefficient_callback, second.m_loaded_friction_coefficient_callback );
  swap( first.m_loaded_restitution_coefficient_callback, second.m_loaded_restitution_coefficient_callback );
  #endif
}

void PythonScripting::restitutionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor )
{
  #ifdef USE_PYTHON
  assert( !m_module_name.empty() );
  assert( m_loaded_module != nullptr );
  if( m_loaded_restitution_coefficient_callback == nullptr )
  {
    return;
  }
  // Get data ready for Python
  s_cor = &cor;
  s_active_set = &active_set;
  // Make the function call
  const PythonObject value{ PyObject_CallObject( m_loaded_restitution_coefficient_callback, nullptr ) };
  if( value == nullptr )
  {
    PyErr_Print();
    std::cerr << "Python callback restitutionCoefficient failed, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_cor = nullptr;
  s_active_set = nullptr;
  assert( PyErr_Occurred() == nullptr );
  #else
  std::cerr << "PythonScripting::restitutionCoefficient must be compiled with Python support, exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void PythonScripting::frictionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu )
{
  #ifdef USE_PYTHON
  assert( !m_module_name.empty() );
  assert( m_loaded_module != nullptr );
  if( m_loaded_friction_coefficient_callback == nullptr )
  {
    return;
  }
  // Get data ready for Python
  s_mu = &mu;
  s_active_set = &active_set;
  // Make the function call
  const PythonObject value{ PyObject_CallObject( m_loaded_friction_coefficient_callback, nullptr ) };
  if( value == nullptr )
  {
    PyErr_Print();
    std::cerr << "Python callback frictionCoefficient failed, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_mu = nullptr;
  s_active_set = nullptr;
  assert( PyErr_Occurred() == nullptr );
  #else
  std::cerr << "PythonScripting::frictionCoefficient must be compiled with Python support, exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void PythonScripting::startOfSim()
{
  #ifdef USE_PYTHON
  assert( !m_module_name.empty() );
  assert( m_loaded_module != nullptr );
  if( m_loaded_start_of_sim_callback == nullptr )
  {
    return;
  }
  // Make the function call
  const PythonObject value{ PyObject_CallObject( m_loaded_start_of_sim_callback, nullptr ) };
  if( value == nullptr )
  {
    PyErr_Print();
    std::cerr << "Python callback startOfSim failed, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( PyErr_Occurred() == nullptr );
  #else
  std::cerr << "PythonScripting::startOfSim must be compiled with Python support, exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void PythonScripting::endOfSim()
{
  #ifdef USE_PYTHON
  assert( !m_module_name.empty() );
  assert( m_loaded_module != nullptr );
  if( m_loaded_end_of_sim_callback == nullptr )
  {
    return;
  }
  // Make the function call
  const PythonObject value{ PyObject_CallObject( m_loaded_end_of_sim_callback, nullptr ) };
  if( value == nullptr )
  {
    PyErr_Print();
    std::cerr << "Python callback endOfSim failed, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( PyErr_Occurred() == nullptr );
  #else
  std::cerr << "PythonScripting::endOfSim must be compiled with Python support, exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void PythonScripting::startOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
{
  #ifdef USE_PYTHON
  assert( !m_module_name.empty() );
  assert( m_loaded_module != nullptr );
  if( m_loaded_start_of_step_callback == nullptr )
  {
    return;
  }
  // Get data ready for Python
  s_timestep = scalar( dt );
  s_next_iteration = next_iteration;
  // Make the function call
  const PythonObject value{ PyObject_CallObject( m_loaded_start_of_step_callback, nullptr ) };
  if( value == nullptr )
  {
    PyErr_Print();
    std::cerr << "Python callback startOfStep failed, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_timestep = std::numeric_limits<scalar>::signaling_NaN();
  s_next_iteration = 0;
  assert( PyErr_Occurred() == nullptr );
  #else
  std::cerr << "PythonScripting::startOfStep must be compiled with Python support, exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void PythonScripting::endOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
{
  #ifdef USE_PYTHON
  assert( !m_module_name.empty() );
  assert( m_loaded_module != nullptr );
  if( m_loaded_end_of_step_callback == nullptr )
  {
    return;
  }
  // Get data ready for Python
  s_timestep = scalar( dt );
  s_next_iteration = next_iteration;
  // Make the function call
  const PythonObject value{ PyObject_CallObject( m_loaded_end_of_step_callback, nullptr ) };
  if( value == nullptr )
  {
    PyErr_Print();
    std::cerr << "Python callback endOfStep failed, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_timestep = std::numeric_limits<scalar>::signaling_NaN();
  s_next_iteration = 0;
  assert( PyErr_Occurred() == nullptr );
  #else
  std::cerr << "PythonScripting::endOfStep must be compiled with Python support, exiting." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

std::string PythonScripting::name() const
{
  return m_module_name;
}

void PythonScripting::setState( RigidBody2DState& state )
{
  #ifdef USE_PYTHON
  s_state = &state;
  #endif
  // No need to handle state cache if scripting is disabled
}

void PythonScripting::forgetState()
{
  #ifdef USE_PYTHON
  s_state = nullptr;
  #endif
  // No need to handle state cache if scripting is disabled
}

void PythonScripting::serialize( std::ostream& output_stream )
{
  StringUtilities::serializeString( m_path, output_stream );
  StringUtilities::serializeString( m_module_name, output_stream );
  // Python variables are re-initialized on deserializaiton, so no action needed here
}

#ifdef USE_PYTHON
static PyObject* timestep( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return Py_BuildValue( is_same<scalar,double>::value ? "d" : "f", s_timestep );
}

static PyObject* nextIteration( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  return Py_BuildValue( "I", s_next_iteration );
}

static PyObject* numStaticPlanes( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  assert( s_state != nullptr );
  return Py_BuildValue( "I", s_state->planes().size() );
}

static PyObject* deleteStaticPlane( PyObject* self, PyObject* args )
{
  unsigned plane_idx;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, "I", &plane_idx ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for deleteStaticPlane, parameters are: unsigned plane_idx. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_state != nullptr );
  if( plane_idx > s_state->planes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in deleteStaticPlane, plane_idx must be less than " << s_state->planes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_state->planes().erase( s_state->planes().begin() + plane_idx );
  return Py_BuildValue( "" );
}

static PyMethodDef RigidBody2DFunctions[] = {
  { "timestep", timestep, METH_NOARGS, "Returns the timestep." },
  { "nextIteration", nextIteration, METH_NOARGS, "Returns the end of step iteration." },
  { "numStaticPlanes", numStaticPlanes, METH_NOARGS, "Returns the number of static planes." },
  { "deleteStaticPlane", deleteStaticPlane, METH_VARARGS, "Deletes a static plane." },
  { nullptr, nullptr, 0, nullptr }
};

void PythonScripting::initializeCallbacks()
{
  if( _import_array() < 0 )
  {
    std::cerr << "Bad import array!" << std::endl;
    std::exit( EXIT_FAILURE );
  }
  Py_InitModule( "rigidbody2d", RigidBody2DFunctions );
}
#endif
