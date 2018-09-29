#include "PythonScripting.h"

#ifdef USE_PYTHON
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include "scisim/PythonTools.h"
#include "scisim/Math/Rational.h"
#include "RigidBody2DState.h"
#endif

#include "scisim/Utilities.h"
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
  
  Utilities::ignoreUnusedVariable(s_active_set);
  Utilities::ignoreUnusedVariable(s_cor);
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

  Utilities::ignoreUnusedVariable(s_mu);
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
void PythonScripting::setState( RigidBody2DState& state )
#else
void PythonScripting::setState( RigidBody2DState& /*state*/ )
#endif
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

static PyObject* numStaticPlanes( PyObject* /*self*/, PyObject* /*args*/ )
{
  assert( s_state != nullptr );
  return Py_BuildValue( "I", s_state->planes().size() );
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
  assert( s_state != nullptr );
  if( plane_idx > s_state->planes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in setStaticPlanePosition, plane_idx must be less than " << s_state->planes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_state->planes()[ plane_idx ].setX( Vector2s{ x_position, y_position } );
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
  assert( s_state != nullptr );
  if( plane_idx > s_state->planes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in setStaticPlaneVelocity, plane_idx must be less than " << s_state->planes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_state->planes()[ plane_idx ].setV( Vector2s{ x_velocity, y_velocity } );
  return Py_BuildValue( "" );
}

static PyObject* setStaticPlaneNormal( PyObject* /*self*/, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned plane_idx;
  scalar nx;
  scalar ny;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Idd" : "Iff", &plane_idx, &nx, &ny ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setStaticPlaneNormal, parameters are: unsigned plane_idx, double nx, double ny. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_state != nullptr );
  if( plane_idx > s_state->planes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in setStaticPlaneNormal, plane_idx must be less than " << s_state->planes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_state->planes()[ plane_idx ].setN( Vector2s{ nx, ny } );
  return Py_BuildValue( "" );
}

static PyObject* setStaticPlaneAngularVelocity( PyObject* /*self*/, PyObject* args )
{
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  unsigned plane_idx;
  scalar omega;
  assert( args != nullptr );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "Id" : "If", &plane_idx, &omega ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for setStaticPlaneAngularVelocity, parameters are: unsigned plane_idx, double omega. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_state != nullptr );
  if( plane_idx > s_state->planes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in setStaticPlaneAngularVelocity, plane_idx must be less than " << s_state->planes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_state->planes()[ plane_idx ].setOmega( omega );
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
  assert( s_state != nullptr );
  if( plane_idx > s_state->planes().size() )
  {
    std::cerr << "Invalid plane_idx parameter of " << plane_idx << " in deleteStaticPlane, plane_idx must be less than " << s_state->planes().size() << ". Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_state->planes().erase( s_state->planes().begin() + plane_idx );
  return Py_BuildValue( "" );
}

static PyObject* numGeometryInstances( PyObject* /*self*/, PyObject* /*args*/ )
{
  assert( s_state != nullptr );
  return Py_BuildValue( "I", s_state->geometry().size() );
}

static PyObject* addCircleGeometry( PyObject* /*self*/, PyObject* args )
{
  scalar radius;
  assert( args != nullptr );
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "d" : "f", &radius ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for addCircleGeometry, parameters are: double radius. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( radius <= 0.0 )
  {
    std::cerr << "Error in addCircleGeometry, radius must be positive. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_state != nullptr );
  s_state->addCircleGeometry( radius );
  return Py_BuildValue( "" );
}

static PyObject* addBody( PyObject* /*self*/, PyObject* args )
{
  Vector2s q;
  scalar theta;
  Vector2s v;
  scalar omega;
  scalar rho;
  int geo_idx;
  int fixed;
  assert( args != nullptr );
  using std::is_same;
  static_assert( is_same<scalar,double>::value || is_same<scalar,float>::value, "Error, scalar type must be double or float for Python interface." );
  if( !PyArg_ParseTuple( args, is_same<scalar,double>::value ? "dddddddii" : "fffffffii", &q.x(), &q.y(), &theta, &v.x(), &v.y(), &omega, &rho, &geo_idx, &fixed ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for addBody, parameters are: double x, double y, double theta, double vx, double vy, double omega, double rho, int geo_idx, bool fixed. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( s_state != nullptr );
  if( rho <= 0.0 )
  {
    std::cerr << "Error in addBody, rho must be positive. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( geo_idx < 0 || geo_idx >= int( s_state->geometry().size() ) )
  {
    std::cerr << "Error in addBody, geo_idx must be a non-negative integer less than the number of geometry instances. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( fixed != 0 && fixed != 1 )
  {
    std::cerr << "Error in addBody, fixed value must be 0 or 1. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  s_state->addBody( q, theta, v, omega, rho, geo_idx, fixed == 1 );
  return Py_BuildValue( "" );
}

static PyObject* deleteBodies( PyObject* /*self*/, PyObject* args )
{
  PyArrayObject* body_list;
  if( !PyArg_ParseTuple( args, "O", &body_list ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for delete_bodies, parameters are: 32 bit unsigned NumPy array. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( body_list != nullptr );
  if( PyArray_NDIM( body_list ) != 1 )
  {
    std::cerr << "Error, bodies to delete list must be a one dimensional array." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( PyArray_DESCR( body_list )->kind != 'u' || PyArray_DESCR( body_list )->elsize != 4 )
  {
    std::cerr << "Error, bodies to delete list must contain 32 bit unsigned integers." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  const Eigen::Map<const VectorXu> bodies_to_delete( static_cast<unsigned*>( PyArray_DATA( body_list ) ), unsigned( PyArray_DIM( body_list, 0 ) ) );

  const unsigned num_total_bodies{ s_state->nbodies() };
  for( unsigned bdy_idx = 0; bdy_idx < bodies_to_delete.size(); ++bdy_idx )
  {
    if( bodies_to_delete[bdy_idx] >= num_total_bodies )
    {
      std::cerr << "Error, bodies to delete indices must be less than the total number of bodies." << std::endl;
      std::exit( EXIT_FAILURE );
    }
  }

  s_state->removeBodies( bodies_to_delete );

  return Py_BuildValue( "" );
}

static PyObject* deleteGeometry( PyObject* /*self*/, PyObject* args )
{
  PyArrayObject* geo_list;
  if( !PyArg_ParseTuple( args, "O", &geo_list ) )
  {
    PyErr_Print();
    std::cerr << "Failed to read parameters for delete_geometry, parameters are: 32 bit unsigned NumPy array. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  assert( geo_list != nullptr );
  if( PyArray_NDIM( geo_list ) != 1 )
  {
    std::cerr << "Error, geometry to delete list must be a one dimensional array." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( PyArray_DESCR( geo_list )->kind != 'u' || PyArray_DESCR( geo_list )->elsize != 4 )
  {
    std::cerr << "Error, geometry to delete list must contain 32 bit unsigned integers." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  const Eigen::Map<const VectorXu> geometry_to_delete( static_cast<unsigned*>( PyArray_DATA( geo_list ) ), unsigned( PyArray_DIM( geo_list, 0 ) ) );

  const unsigned num_geo{ static_cast<unsigned>( s_state->geometry().size() ) };
  for( unsigned geo_idx = 0; geo_idx < geometry_to_delete.size(); ++geo_idx )
  {
    if( geometry_to_delete[geo_idx] >= num_geo )
    {
      std::cerr << "Error, geometry to delete indices must be less than the total number of geometry instances." << std::endl;
      std::exit( EXIT_FAILURE );
    }
  }

  s_state->removeGeometry( geometry_to_delete );

  return Py_BuildValue( "" );
}

static PyObject* numBodies( PyObject* /*self*/, PyObject* /*args*/ )
{
  return Py_BuildValue( "I", s_state->nbodies() );
}

static PyObject* numGeometry( PyObject* /*self*/, PyObject* /*args*/ )
{
  return Py_BuildValue( "I", s_state->geometry().size() );
}

static PyMethodDef RigidBody2DFunctions[] = {
  { "timestep", timestep, METH_NOARGS, "Returns the timestep." },
  { "nextIteration", nextIteration, METH_NOARGS, "Returns the end of step iteration." },
  { "numStaticPlanes", numStaticPlanes, METH_NOARGS, "Returns the number of static planes." },
  { "setStaticPlanePosition", setStaticPlanePosition, METH_VARARGS, "Sets the position of a static plane." },
  { "setStaticPlaneVelocity", setStaticPlaneVelocity, METH_VARARGS, "Sets the velocity of a static plane." },
  { "setStaticPlaneNormal", setStaticPlaneNormal, METH_VARARGS, "Sets the normal of a static plane." },
  { "setStaticPlaneAngularVelocity", setStaticPlaneAngularVelocity, METH_VARARGS, "Sets the angular velocity of a static plane." },
  { "deleteStaticPlane", deleteStaticPlane, METH_VARARGS, "Deletes a static plane." },
  { "numGeometryInstances", numGeometryInstances, METH_NOARGS, "Returns the number of geometry instances in the simulation." },
  { "addCircleGeometry", addCircleGeometry, METH_VARARGS, "Adds a new circle geometry instance to the system." },
  { "addBody", addBody, METH_VARARGS, "Adds a new rigid body to the system." },
  { "delete_bodies", deleteBodies, METH_VARARGS, "Deletes the given bodies from the system." },
  { "delete_geometry", deleteGeometry, METH_VARARGS, "Deletes the given geometry instances from the system." },
  { "num_bodies", numBodies, METH_NOARGS, "Returns the number of bodies in the system." },
  { "num_geometry", numGeometry, METH_NOARGS, "Returns the number of geometry instances in the system." },
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
