// PythonScripting.cpp
//
// Breannan Smith
// Last updated: 09/14/2015

#include "PythonScripting.h"

#ifdef USE_PYTHON
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include "scisim/Math/Rational.h"
#include "scisim/Constraints/Constraint.h"
#include "scisim/PythonTools.h"
#include "rigidbody3d/RigidBody3DState.h"
#include "rigidbody3d/Forces/NearEarthGravityForce.h"
#include "scisim/Utilities.h"
#include "StaticGeometry/StaticCylinder.h"
#include "StaticGeometry/StaticPlane.h"
#endif

#include "scisim/StringUtilities.h"
#include <iostream>

#ifdef USE_PYTHON
static scalar s_timestep;
static unsigned s_next_iteration;
static RigidBody3DState* s_sim_state;
static unsigned* s_initial_iterate;
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
: m_path( StringUtilities::deserialize( input_stream ) )
, m_module_name( StringUtilities::deserialize( input_stream ) )
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
  if( m_module_name != "" )
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

void PythonScripting::setState( RigidBody3DState& state )
{
  #ifdef USE_PYTHON
  s_sim_state = &state;
  #endif
  // No need to handle state cache if scripting is disabled
}

void PythonScripting::setInitialIterate( unsigned& initial_iterate )
{
  #ifdef USE_PYTHON
  s_initial_iterate = &initial_iterate;
  #endif
  // No need to handle state cache if scripting is disabled
}

void PythonScripting::forgetState()
{
  #ifdef USE_PYTHON
  s_sim_state = nullptr;
  s_initial_iterate = nullptr;
  #endif
  // No need to handle state cache if scripting is disabled
}

void PythonScripting::serialize( std::ostream& output_stream )
{
  StringUtilities::serialize( m_path, output_stream );
  StringUtilities::serialize( m_module_name, output_stream );
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

static PyObject* configuration( PyObject* self, PyObject* args )
{
  assert( s_sim_state != nullptr );
  assert( args == nullptr );
  npy_intp dims[1] = { s_sim_state->q().size() };
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return PyArray_SimpleNewFromData( 1, dims, (is_same<scalar,double>::value ? NPY_DOUBLE : NPY_FLOAT), s_sim_state->q().data() );
}

static PyObject* velocity( PyObject* self, PyObject* args )
{
  assert( s_sim_state != nullptr );
  assert( args == nullptr );
  npy_intp dims[1] = { s_sim_state->v().size() };
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return PyArray_SimpleNewFromData( 1, dims, (is_same<scalar,double>::value ? NPY_DOUBLE : NPY_FLOAT), s_sim_state->v().data() );
}

static PyObject* numStaticPlanes( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  assert( s_sim_state != nullptr );
  return Py_BuildValue( "I", s_sim_state->staticPlanes().size() );
}

static PyObject* setStaticPlanePosition( PyObject* self, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned plane_idx;
  Vector3s position;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Iddd" : "Ifff", &plane_idx, &position.x(), &position.y(), &position.z() ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setStaticPlanePosition, parameters are: plane_idx, x, y, z. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_sim_state != nullptr );
  if( plane_idx > s_sim_state->staticPlanes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in setStaticPlanePosition, plane_idx must be less than " << s_sim_state->staticPlanes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_sim_state->staticPlane(plane_idx).x() = position;
  return Py_BuildValue( "" );
}

static PyObject* setStaticPlaneVelocity( PyObject* self, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned plane_idx;
  Vector3s velocity;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Iddd" : "Ifff", &plane_idx, &velocity.x(), &velocity.y(), &velocity.z() ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setStaticPlaneVelocity, parameters are: plane_idx, vx, vy, vz. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_sim_state != nullptr );
  if( plane_idx > s_sim_state->staticPlanes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in setStaticPlaneVelocity, plane_idx must be less than " << s_sim_state->staticPlanes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_sim_state->staticPlane(plane_idx).v() = velocity;
  return Py_BuildValue( "" );
}

static PyObject* numStaticCylinders( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  assert( s_sim_state != nullptr );
  return Py_BuildValue( "I", s_sim_state->staticCylinders().size() );
}

static PyObject* setStaticCylinderOrientation( PyObject* self, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned cylinder_idx;
  Vector3s axis;
  scalar theta;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Idddd" : "Iffff", &cylinder_idx, &axis.x(), &axis.y(), &axis.z(), &theta ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setStaticCylinderOrientation, parameters are: cylinder_idx, n_x, n_y, n_z, theta. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_sim_state != nullptr );
  if( cylinder_idx > s_sim_state->staticCylinders().size() )
  {
    std::cerr << "Invalid cylinder_idx parameter of " << cylinder_idx << " in setStaticCylinderOrientation, cylinder_idx must be less than " << s_sim_state->staticCylinders().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( fabs( axis.norm() - 1.0 ) > 1.0e-6 )
  {
    std::cerr << "Invalid cylynder axis for setStaticCylinderOrientation, axis must have unit norm." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_sim_state->staticCylinder(cylinder_idx).setOrientation( axis, theta );
  return Py_BuildValue( "" );
}

static PyObject* setStaticCylinderAngularVelocity( PyObject* self, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned cylinder_idx;
  Vector3s omega;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Iddd" : "Ifff", &cylinder_idx, &omega.x(), &omega.y(), &omega.z() ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setStaticCylinderAngularVelocity, parameters are: cylinder_idx, omegax, omegay, omegaz. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_sim_state != nullptr );
  if( cylinder_idx > s_sim_state->staticCylinders().size() )
  {
    std::cerr << "Invalid cylinder_idx parameter of " << cylinder_idx << " in setStaticCylinderAngularVelocity, cylinder_idx must be less than " << s_sim_state->staticCylinders().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_sim_state->staticCylinder(cylinder_idx).omega() = omega;
  return Py_BuildValue( "" );
}

static PyObject* mu( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  assert( s_mu != nullptr );
  npy_intp dims[1] = { s_mu->size() };
  assert( s_mu->data() != nullptr );
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return PyArray_SimpleNewFromData( 1, dims, (is_same<scalar,double>::value ? NPY_DOUBLE : NPY_FLOAT), s_mu->data() );
}

static PyObject* cor( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  assert( s_cor != nullptr );
  npy_intp dims[1] = { s_cor->size() };
  assert( s_cor->data() != nullptr );
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return PyArray_SimpleNewFromData( 1, dims, (is_same<scalar,double>::value ? NPY_DOUBLE : NPY_FLOAT), s_cor->data() );
}

static PyObject* numCollisions( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  assert( s_active_set != nullptr );
  return Py_BuildValue( "I", s_active_set->size() );
}

static PyObject* collisionType( PyObject* self, PyObject* args )
{
  unsigned collision_idx;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, "I", &collision_idx ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for collisionType, parameters are: collision_idx. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_active_set != nullptr );
  if( collision_idx > s_active_set->size() )
  {
    std::cerr << "Invalid collision_idx parameter of " << collision_idx << " for collisionType, collision_idx must be less than " << s_active_set->size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  return Py_BuildValue( "s", (*s_active_set)[collision_idx]->name().c_str() );
}

static PyObject* collisionIndices( PyObject* self, PyObject* args )
{
  unsigned collision_idx;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, "I", &collision_idx ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for collisionIndices, parameters are: collision_idx. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_active_set != nullptr );
  if( collision_idx > s_active_set->size() )
  {
    std::cerr << "Invalid collision_idx parameter of " << collision_idx << " for collisionIndices, collision_idx must be less than " << s_active_set->size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  std::pair<int,int> body_indices;
  (*s_active_set)[collision_idx]->getBodyIndices( body_indices );
  npy_intp dims[1] = { 2 };
  PyObject* object = PyArray_SimpleNew( 1, dims, NPY_INT );
  int* int_data = (int*)( PyArray_DATA( (PyArrayObject*)( object ) ) );
  int_data[0] = body_indices.first;
  int_data[1] = body_indices.second;
  return object;
}

static PyObject* numForces( PyObject* self, PyObject* args )
{
  assert( args == nullptr );
  assert( s_sim_state != nullptr );
  return Py_BuildValue( "I", s_sim_state->forces().size() );
}

static PyObject* forceType( PyObject* self, PyObject* args )
{
  unsigned force_idx;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, "I", &force_idx ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for forceType, parameters are: force_idx. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_sim_state != nullptr );
  if( force_idx > s_sim_state->forces().size() )
  {
    std::cerr << "Invalid force_idx parameter of " << force_idx << " for forceType, force_idx must be less than " << s_sim_state->forces().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  return Py_BuildValue( "s", s_sim_state->forces()[force_idx]->name().c_str() );
}

static PyObject* setGravityForce( PyObject* self, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned force_idx;
  Vector3s new_gravity;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Iddd" : "Ifff", &force_idx, &new_gravity.x(), &new_gravity.y(), &new_gravity.z() ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setGravityForce, parameters are: force_idx, fx, fy, fz. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_sim_state != nullptr );
  if( force_idx > s_sim_state->forces().size() )
  {
    std::cerr << "Invalid force_idx parameter of " << force_idx << " in setGravityForce, force_idx must be less than " << s_sim_state->forces().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  NearEarthGravityForce& near_earth_gravity = static_cast<NearEarthGravityForce&>( *s_sim_state->forces()[force_idx] );
  near_earth_gravity.setForce( new_gravity );
  return Py_BuildValue( "" );
}

static PyObject* setInitialIterate( PyObject* self, PyObject* args )
{
  unsigned initial_iterate;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, "I", &initial_iterate ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setInitialIterate, parameters are: initial_iterate. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_initial_iterate != nullptr );
  *s_initial_iterate = initial_iterate;
  return Py_BuildValue( "" );
}

static PyMethodDef RigidBody3DFunctions[] = {
  { "timestep", timestep, METH_NOARGS, "Returns the timestep." },
  { "nextIteration", nextIteration, METH_NOARGS, "Returns the end of step iteration." },
  { "configuration", configuration, METH_NOARGS, "Returns the system's configuration." },
  { "velocity", velocity, METH_NOARGS, "Returns the system's velocity." },
  { "numStaticPlanes", numStaticPlanes, METH_NOARGS, "Returns the number of static planes." },
  { "setStaticPlanePosition", setStaticPlanePosition, METH_VARARGS, "Sets the position of a static plane." },
  { "setStaticPlaneVelocity", setStaticPlaneVelocity, METH_VARARGS, "Sets the velocity of a static plane." },
  { "numStaticCylinders", numStaticCylinders, METH_NOARGS, "Returns the number of static cylinders." },
  { "setStaticCylinderOrientation", setStaticCylinderOrientation, METH_VARARGS, "Sets the orientation of a given static cylinder via an axis and rotation about that axis." },
  { "setStaticCylinderAngularVelocity", setStaticCylinderAngularVelocity, METH_VARARGS, "Sets the angular velocity of a static cylinder." },
  { "mu", mu, METH_NOARGS, "Returns the coefficients of friction." },
  { "cor", cor, METH_NOARGS, "Returns the coefficients of restitution." },
  { "numCollisions", numCollisions, METH_NOARGS, "Returns the number of collisions." },
  { "collisionType", collisionType, METH_VARARGS, "Returns the type of a collision." },
  { "collisionIndices", collisionIndices, METH_VARARGS, "Returns the indices of bodies involved in a given collision." },
  { "numForces", numForces, METH_NOARGS, "Returns the number of forces." },
  { "forceType", forceType, METH_VARARGS, "Returns the type of a force." },
  { "setGravityForce", setGravityForce, METH_VARARGS, "Sets a gravity force." },
  { "setInitialIterate", setInitialIterate, METH_VARARGS, "Sets the initial iteration count." },
  { nullptr, nullptr, 0, nullptr }
};

void PythonScripting::initializeCallbacks()
{
  if( _import_array() < 0 )
  {
    std::cerr << "Bad import array!" << std::endl;
    std::exit( EXIT_FAILURE );
  }
  Py_InitModule( "rigidbody3d", RigidBody3DFunctions );
}
#endif
