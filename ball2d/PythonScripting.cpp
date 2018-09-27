#include "PythonScripting.h"

#ifdef USE_PYTHON
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include "scisim/Math/Rational.h"
#include "scisim/Constraints/Constraint.h"
#include "Ball2DState.h"
#include "scisim/PythonTools.h"
#include "ball2d/StaticGeometry/StaticPlane.h"
#endif

#include "scisim/StringUtilities.h"
#include <iostream>

#ifdef USE_PYTHON
static scalar s_timestep;
static unsigned s_next_iteration;
static Ball2DState* s_ball_state;
static VectorXs* s_mu;
static VectorXs* s_cor;
static const std::vector<std::unique_ptr<Constraint>>* s_active_set;
#endif

// User-provided callbacks
typedef void (*BallInsertCallback)(void*, int);
static void* s_ball_insert_context = nullptr;
static BallInsertCallback s_ball_insert_call_back = nullptr;

typedef void (*PlaneDeleteCallback)(void*, int);
static void* s_plane_delete_context = nullptr;
static PlaneDeleteCallback s_plane_delete_call_back = nullptr;


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

#ifdef USE_PYTHON
void PythonScripting::restitutionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor )
#else
void PythonScripting::restitutionCoefficient( const std::vector<std::unique_ptr<Constraint>>& /*active_set*/, VectorXs& /*cor*/ )
#endif
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

#ifdef USE_PYTHON
void PythonScripting::frictionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu )
#else
void PythonScripting::frictionCoefficient( const std::vector<std::unique_ptr<Constraint>>& /*active_set*/, VectorXs& /*mu*/ )
#endif
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

#ifdef USE_PYTHON
void PythonScripting::startOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
#else
void PythonScripting::startOfStep( const unsigned /*next_iteration*/, const Rational<std::intmax_t>& /*dt*/ )
#endif
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

#ifdef USE_PYTHON
void PythonScripting::endOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
#else
void PythonScripting::endOfStep( const unsigned /*next_iteration*/, const Rational<std::intmax_t>& /*dt*/ )
#endif
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

#ifdef USE_PYTHON
void PythonScripting::setState( Ball2DState& state )
#else
void PythonScripting::setState( Ball2DState& /*state*/ )
#endif
{
  #ifdef USE_PYTHON
  s_ball_state = &state;
  #endif
  // No need to handle state cache if scripting is disabled
}

void PythonScripting::registerBallInsertCallback( void* context, void (*callback)(void*, int) )
{
  s_ball_insert_context = context;
  s_ball_insert_call_back = callback;
}

void PythonScripting::registerPlaneDeleteCallback( void* context, void (*callback)(void*, int) )
{
  s_plane_delete_context = context;
  s_plane_delete_call_back = callback;
}

void PythonScripting::forgetState()
{
  #ifdef USE_PYTHON
  s_ball_state = nullptr;
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
static PyObject* timestep( PyObject* /*self*/, PyObject* /*args*/ )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return Py_BuildValue( is_same<scalar,double>::value ? "d" : "f", s_timestep );
}

static PyObject* nextIteration( PyObject* /*self*/, PyObject* /*args*/ )
{
  return Py_BuildValue( "I", s_next_iteration );
}

static PyObject* configuration( PyObject* /*self*/, PyObject* /*args*/ )
{
  assert( s_ball_state != nullptr );
  npy_intp dims[1] = { s_ball_state->q().size() };
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return PyArray_SimpleNewFromData( 1, dims, (is_same<scalar,double>::value ? NPY_DOUBLE : NPY_FLOAT), s_ball_state->q().data() );
}

static PyObject* velocity( PyObject* /*self*/, PyObject* /*args*/ )
{
  assert( s_ball_state != nullptr );
  npy_intp dims[1] = { s_ball_state->v().size() };
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return PyArray_SimpleNewFromData( 1, dims, (is_same<scalar,double>::value ? NPY_DOUBLE : NPY_FLOAT), s_ball_state->v().data() );
}

static PyObject* insertBall( PyObject* /*self*/, PyObject* args )
{
  Vector2s q;
  Vector2s v;
  scalar r;
  scalar m;
  int fixed;
  assert( args != nullptr );
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "ddddddi" : "ffffffi", &q.x(), &q.y(), &v.x(), &v.y(), &r, &m, &fixed ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for insertBall, parameters are: double x, double y, double vx, double vy, double radius, double mass, bool fixed. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( r <= 0.0 )
  {
    std::cerr << "Error in insert ball, radius must be positive. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( m <= 0.0 )
  {
    std::cerr << "Error in insert ball, mass must be positive. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( fixed != 0 && fixed != 1 )
  {
    std::cerr << "Error in insert ball, fixed value must be 0 or 1. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_ball_state != nullptr );
  s_ball_state->pushBallBack( q, v, r, m, fixed );
  if( s_ball_insert_call_back != nullptr )
  {
    s_ball_insert_call_back( s_ball_insert_context, s_ball_state->nballs() );
  }
  return Py_BuildValue( "" );
}

static PyObject* numStaticPlanes( PyObject* /*self*/, PyObject* /*args*/ )
{
  assert( s_ball_state != nullptr );
  return Py_BuildValue( "I", s_ball_state->staticPlanes().size() );
}

static PyObject* setStaticPlanePosition( PyObject* /*self*/, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned plane_idx;
  scalar x_position;
  scalar y_position;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Idd" : "Iff", &plane_idx, &x_position, &y_position ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setStaticPlanePosition, parameters are: plane_idx, x, y. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_ball_state != nullptr );
  if( plane_idx > s_ball_state->staticPlanes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in setStaticPlanePosition, plane_idx must be less than " << s_ball_state->staticPlanes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_ball_state->staticPlanes()[ plane_idx ].x() << x_position, y_position;
  return Py_BuildValue( "" );
}

static PyObject* setStaticPlaneVelocity( PyObject* /*self*/, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned plane_idx;
  scalar x_velocity;
  scalar y_velocity;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Idd" : "Iff", &plane_idx, &x_velocity, &y_velocity ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setStaticPlaneVelocity, parameters are: unsigned plane_idx, double vx, double vy. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_ball_state != nullptr );
  if( plane_idx > s_ball_state->staticPlanes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in setStaticPlaneVelocity, plane_idx must be less than " << s_ball_state->staticPlanes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_ball_state->staticPlanes()[ plane_idx ].v() << x_velocity, y_velocity;
  return Py_BuildValue( "" );
}

static PyObject* deleteStaticPlane( PyObject* /*self*/, PyObject* args )
{
  unsigned plane_idx;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, "I", &plane_idx ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for deleteStaticPlane, parameters are: unsigned plane_idx. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_ball_state != nullptr );
  if( plane_idx > s_ball_state->staticPlanes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in deleteStaticPlane, plane_idx must be less than " << s_ball_state->staticPlanes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_ball_state->staticPlanes().erase( s_ball_state->staticPlanes().begin() + plane_idx );
  if( s_plane_delete_call_back != nullptr )
  {
    s_plane_delete_call_back( s_plane_delete_context, plane_idx );
  }
  return Py_BuildValue( "" );
}

static PyObject* mu( PyObject* /*self*/, PyObject* /*args*/ )
{
  assert( s_mu != nullptr );
  npy_intp dims[1] = { s_mu->size() };
  assert( s_mu->data() != nullptr );
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return PyArray_SimpleNewFromData( 1, dims, (is_same<scalar,double>::value ? NPY_DOUBLE : NPY_FLOAT), s_mu->data() );
}

static PyObject* cor( PyObject* /*self*/, PyObject* /*args*/ )
{
  assert( s_cor != nullptr );
  npy_intp dims[1] = { s_cor->size() };
  assert( s_cor->data() != nullptr );
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  return PyArray_SimpleNewFromData( 1, dims, (is_same<scalar,double>::value ? NPY_DOUBLE : NPY_FLOAT), s_cor->data() );
}

static PyObject* numCollisions( PyObject* /*self*/, PyObject* /*args*/ )
{
  assert( s_active_set != nullptr );
  return Py_BuildValue( "I", s_active_set->size() );
}

static PyObject* collisionType( PyObject* /*self*/, PyObject* args )
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

static PyObject* collisionIndices( PyObject* /*self*/, PyObject* args )
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
    std::cerr << "Invalid collision_idx parameter of " << collision_idx << " for collisionType, collision_idx must be less than " << s_active_set->size() << ". Exiting." << std::endl;
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

static PyMethodDef Balls2DFunctions[] = {
  { "timestep", timestep, METH_NOARGS, "Returns the timestep." },
  { "nextIteration", nextIteration, METH_NOARGS, "Returns the end of step iteration." },
  { "configuration", configuration, METH_NOARGS, "Returns the system's configuration." },
  { "velocity", velocity, METH_NOARGS, "Returns the system's velocity." },
  { "insertBall", insertBall, METH_VARARGS, "Adds a new ball to the system." },
  { "numStaticPlanes", numStaticPlanes, METH_NOARGS, "Returns the number of static planes." },
  { "setStaticPlanePosition", setStaticPlanePosition, METH_VARARGS, "Sets the position of a static plane." },
  { "setStaticPlaneVelocity", setStaticPlaneVelocity, METH_VARARGS, "Sets the velocity of a static plane." },
  { "deleteStaticPlane", deleteStaticPlane, METH_VARARGS, "Deletes a static plane." },
  { "mu", mu, METH_NOARGS, "Returns the coefficients of friction." },
  { "cor", cor, METH_NOARGS, "Returns the coefficients of restitution." },
  { "numCollisions", numCollisions, METH_NOARGS, "Returns the number of collisions." },
  { "collisionType", collisionType, METH_VARARGS, "Returns the type of a collision." },
  { "collisionIndices", collisionIndices, METH_VARARGS, "Returns the indices of bodies involved in a given collision." },
  { nullptr, nullptr, 0, nullptr }
};

void PythonScripting::initializeCallbacks()
{
  if( _import_array() < 0 )
  {
    std::cerr << "Bad import array!" << std::endl;
    std::exit( EXIT_FAILURE );
  }
  Py_InitModule( "balls2d", Balls2DFunctions );
}
#endif
