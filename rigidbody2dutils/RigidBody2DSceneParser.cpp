#include "RigidBody2DSceneParser.h"

#include <iostream>
#include <fstream>

#include "rapidxml.hpp"

#include "rigidbody2d/RigidBody2DState.h"
#include "rigidbody2d/RigidBody2DGeometry.h"
#include "rigidbody2d/CircleGeometry.h"
#include "rigidbody2d/BoxGeometry.h"
#include "rigidbody2d/SymplecticEulerMap.h"
#include "rigidbody2d/VerletMap.h"
#include "rigidbody2d/RigidBody2DForce.h"
#include "rigidbody2d/NearEarthGravityForce.h"
#include "rigidbody2d/RigidBody2DStaticPlane.h"
#include "rigidbody2d/PlanarPortal.h"

#include "scisim/Math/Rational.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/GeometricImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/StabilizedImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GaussSeidelOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/JacobiOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GROperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GRROperator.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/StaggeredProjections.h"
#include "scisim/ConstrainedMaps/Sobogus.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"

#ifdef IPOPT_FOUND
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorIpopt.h"
#endif

#ifdef QL_FOUND
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorQL.h"
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorQLVP.h"
#endif

#include "CameraSettings2D.h"

static bool loadTextFileIntoVector( const std::string& filename, std::vector<char>& xmlchars )
{
  assert( xmlchars.empty() );

  // Attempt to open the text file for reading
  std::ifstream textfile( filename );
  if( !textfile.is_open() )
  {
    return false;
  }

  // Read the entire file into a single string
  std::string line;
  while( getline( textfile, line ) )
  {
    std::copy( line.cbegin(), line.cend(), back_inserter( xmlchars ) );
  }
  xmlchars.emplace_back( '\0' );

  return true;
}

static bool loadXMLFile( const std::string& filename, std::vector<char>& xmlchars, rapidxml::xml_document<>& doc )
{
  assert( xmlchars.empty() );

  // Attempt to read the text from the user-specified xml file
  if( !loadTextFileIntoVector( filename, xmlchars ) )
  {
    std::cerr << "Failed to read scene file: " << filename << std::endl;
    return false;
  }

  // Initialize the xml parser with the character vector
  try
  {
    doc.parse<0>( xmlchars.data() );
  }
  catch( const rapidxml::parse_error& e )
  {
    std::cerr << "Failed to parse scene file: " << filename << std::endl;
    std::cerr << "Error message: " << e.what() << std::endl;
    return false;
  }

  return true;
}

static bool loadCameraSettings( const rapidxml::xml_node<>& node, CameraSettings2D& camera_settings )
{
  // If the camera is not specified, we are done
  const rapidxml::xml_node<>* camera_node{ node.first_node( "camera" ) };
  if( camera_node == nullptr )
  {
    return true;
  }

  camera_settings.set = true;

  // Attempt to read the center setting
  {
    const rapidxml::xml_attribute<>* center_attrib{ camera_node->first_attribute( "center" ) };
    if( center_attrib == nullptr )
    {
      std::cerr << "Failed to locate center attribute for camera node." << std::endl;
      return false;
    }
    if( !StringUtilities::readScalarList( center_attrib->value(), 2, ' ', camera_settings.center ) )
    {
      std::cerr << "Failed to load center attribute for camera node, must provide 2 scalars." << std::endl;
      return false;
    }
  }

  // Attempt to read the scale setting
  {
    const rapidxml::xml_attribute<>* scale_attrib{ camera_node->first_attribute( "scale" ) };
    if( scale_attrib == nullptr )
    {
      std::cerr << "Failed to locate scale attribute for camera node." << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( scale_attrib->value(), camera_settings.scale ) || camera_settings.scale <= 0.0 )
    {
      std::cerr << "Failed to load scale attribute for camera node, must provide a single positive scalar." << std::endl;
      return false;
    }
  }

  // Attempt to read the fps setting
  {
    const rapidxml::xml_attribute<>* fps_attrib{ camera_node->first_attribute( "fps" ) };
    if( fps_attrib == nullptr )
    {
      std::cerr << "Failed to locate fps attribute for camera node." << std::endl;
      return false;
    }
    int fps;
    if( !StringUtilities::extractFromString( fps_attrib->value(), fps ) || fps <= 0 )
    {
      std::cerr << "Failed to parse fps attribute for camera node, must provide a non-negative integer." << std::endl;
      return false;
    }
    camera_settings.fps = unsigned( fps );
  }

  // Attempt to read the render_at_fps setting
  {
    const rapidxml::xml_attribute<>* render_at_fps_attrib{ camera_node->first_attribute( "render_at_fps" ) };
    if( render_at_fps_attrib == nullptr )
    {
      std::cerr << "Failed to locate render_at_fps attribute for camera node." << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( render_at_fps_attrib->value(), camera_settings.render_at_fps ) )
    {
      std::cerr << "Failed to parse render_at_fps attribute for camera node, must provide a boolean." << std::endl;
      return false;
    }
  }

  // Attempt to read the locked setting
  {
    const rapidxml::xml_attribute<>* locked_attrib{ camera_node->first_attribute( "locked" ) };
    if( locked_attrib == nullptr )
    {
      std::cerr << "Failed to locate locked attribute for camera node." << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( locked_attrib->value(), camera_settings.locked ) )
    {
      std::cerr << "Failed to parse locked attribute for camera node, must provide a boolean." << std::endl;
      return false;
    }
  }

  return true;
}

static bool loadEndTime( const rapidxml::xml_node<>& node, scalar& end_time )
{
  // If the end time is not specified, set it to infinity
  const rapidxml::xml_node<>* end_time_node{ node.first_node( "end_time" ) };
  if( end_time_node == nullptr )
  {
    end_time = SCALAR_INFINITY;
    return true;
  }

  // Attempt to parse the time setting
  const rapidxml::xml_attribute<>* t_attrib{ end_time_node->first_attribute( "t" ) };
  if( t_attrib == nullptr )
  {
    std::cerr << "Failed to locate t attribute for end_time node." << std::endl;
    return false;
  }
  if( !StringUtilities::extractFromString( t_attrib->value(), end_time ) || end_time <= 0.0 )
  {
    std::cerr << "Failed to parse t attribute for end_time. Value must be a positive scalar." << std::endl;
    return false;
  }

  return true;
}

static bool loadScriptingSetup( const rapidxml::xml_node<>& node, std::string& scripting_callback )
{
  assert( scripting_callback.empty() );

  const rapidxml::xml_node<>* scripting_node{ node.first_node( "scripting" ) };
  if( !scripting_node )
  {
    return true;
  }

  const rapidxml::xml_attribute<>* name_node{ scripting_node->first_attribute( "callback" ) };
  if( name_node )
  {
    scripting_callback = name_node->value();
  }
  else
  {
    return false;
  }

  return true;
}

static bool loadStaticPlanes( const rapidxml::xml_node<>& node, std::vector<RigidBody2DStaticPlane>& planes )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "static_plane" ); nd; nd = nd->next_sibling( "static_plane" ) )
  {
    // Attempt to read the point on the plane
    Vector2s x;
    {
      const rapidxml::xml_attribute<>* x_attrib{ nd->first_attribute( "x" ) };
      if( x_attrib == nullptr )
      {
        std::cerr << "Failed to locate x attribute for static_plane node." << std::endl;
        return false;
      }
      if( !StringUtilities::readScalarList( x_attrib->value(), 2, ' ', x ) )
      {
        std::cerr << "Failed to load x attribute for static_plane node, must provide 2 scalars." << std::endl;
        return false;
      }
    }

    // Attempt to read the plane's normal
    Vector2s n;
    {
      const rapidxml::xml_attribute<>* n_attrib{ nd->first_attribute( "n" ) };
      if( n_attrib == nullptr )
      {
        std::cerr << "Failed to locate n attribute for static_plane node." << std::endl;
        return false;
      }
      if( !StringUtilities::readScalarList( n_attrib->value(), 2, ' ', n ) )
      {
        std::cerr << "Failed to load n attribute for static_plane node, must provide 2 scalars." << std::endl;
        return false;
      }
      if( n.norm() == 0.0 )
      {
        std::cerr << "Failed to load n attribute for static_plane node, must provide a nonzero vector." << std::endl;
        return false;
      }
    }
    planes.emplace_back( x, n.normalized() );
  }

  return true;
}

// TODO: minus ones here can underflow
static bool loadPlanarPortals( const rapidxml::xml_node<>& node, std::vector<RigidBody2DStaticPlane>& planes, std::vector<PlanarPortal>& planar_portals )
{
  if( !( node.first_node( "planar_portal" ) || node.first_node( "lees_edwards_portal" ) )  )
  {
    return true;
  }

  // If we have a portal we must have at least one plane
  if( planes.size() < 2 )
  {
    std::cerr << "Error, must provide at least two planes before instantiating a planar portal." << std::endl;
    return false;
  }

  // Pairs of planes to turn into portals
  std::vector<std::pair<unsigned,unsigned>> plane_pairs;
  std::vector<scalar> plane_tangent_velocities;
  std::vector<scalar> plane_bounds;

  // Load planes without kinematic velocities
  for( rapidxml::xml_node<>* nd = node.first_node( "planar_portal" ); nd; nd = nd->next_sibling( "planar_portal" ) )
  {
    // Read the first plane index
    const rapidxml::xml_attribute<>* const attrib_a{ nd->first_attribute( "planeA" ) };
    if( !attrib_a )
    {
      std::cerr << "Failed to locate planeA attribute for planar_portal node." << std::endl;
      return false;
    }
    unsigned index_a;
    if( !StringUtilities::extractFromString( attrib_a->value(), index_a ) )
    {
      std::cerr << "Failed to parse planeA attribute for planar_portal node with value " << attrib_a->value() << ". Attribute must be an unsigned integer." << std::endl;
      return false;
    }
    if( index_a >= planes.size() )
    {
      std::cerr << "Failed to parse planeA attribute for planar_portal node with value " << attrib_a->value() << ". Attribute must be an index of a plane between " << 0 << " and " << planes.size() - 1 << std::endl;
      return false;
    }

    // Read the second plane index
    const rapidxml::xml_attribute<>* const attrib_b{ nd->first_attribute( "planeB" ) };
    if( !attrib_b )
    {
      std::cerr << "Failed to locate planeB attribute for planar_portal node." << std::endl;
      return false;
    }
    unsigned index_b;
    if( !StringUtilities::extractFromString( attrib_b->value(), index_b ) )
    {
      std::cerr << "Failed to parse planeB attribute for planar_portal node with value " << attrib_b->value() << ". Attribute must be an unsigned integer." << std::endl;
      return false;
    }
    if( index_b >= planes.size() )
    {
      std::cerr << "Failed to parse planeB attribute for planar_portal node with value " << attrib_b->value() << ". Attribute must be an index of a plane between " << 0 << " and " << planes.size() - 1 << std::endl;
      return false;
    }

    // Indices for this portal can't repeat
    if( index_a == index_b )
    {
      std::cerr << "Failed to parse planeB attribute for planar_portal node with value " << attrib_b->value() << ". Value is a repeat of attribute planeA." << std::endl;
      return false;
    }

    // Indices can not repeat previous portals
    for( std::vector<std::pair<unsigned,unsigned>>::size_type i = 0; i < plane_pairs.size(); ++i )
    {
      if( index_a == plane_pairs[i].first || index_a == plane_pairs[i].second )
      {
        std::cerr << "Failed to parse planeA attribute for planar_portal node with value " << attrib_a->value() << ". Plane index is used by an existing portal." << std::endl;
        return false;
      }
      if( index_b == plane_pairs[i].first || index_b == plane_pairs[i].second )
      {
        std::cerr << "Failed to parse planeB attribute for planar_portal node with value " << attrib_b->value() << ". Plane index is used by an existing portal." << std::endl;
        return false;
      }
    }

    plane_pairs.emplace_back( index_a, index_b );
    plane_tangent_velocities.emplace_back( 0.0 );
    plane_bounds.emplace_back( 0.0 );
  }

  // Load planes with kinematic velocities
  for( rapidxml::xml_node<>* nd = node.first_node( "lees_edwards_portal" ); nd; nd = nd->next_sibling( "lees_edwards_portal" ) )
  {
    // Read the first plane index
    const rapidxml::xml_attribute<>* const attrib_a = nd->first_attribute( "planeA" );
    if( !attrib_a )
    {
      std::cerr << "Failed to locate planeA attribute for lees_edwards_portal node." << std::endl;
      return false;
    }
    unsigned index_a;
    if( !StringUtilities::extractFromString( attrib_a->value(), index_a ) )
    {
      std::cerr << "Failed to parse planeA attribute for lees_edwards_portal node with value " << attrib_a->value() << ". Attribute must be an unsigned integer." << std::endl;
      return false;
    }
    if( index_a >= planes.size() )
    {
      std::cerr << "Failed to parse planeA attribute for lees_edwards_portal node with value " << attrib_a->value() << ". Attribute must be an index of a plane between " << 0 << " and " << planes.size() - 1 << std::endl;
      return false;
    }

    // Read the second plane index
    const rapidxml::xml_attribute<>* const attrib_b{ nd->first_attribute( "planeB" ) };
    if( !attrib_b )
    {
      std::cerr << "Failed to locate planeB attribute for lees_edwards_portal node." << std::endl;
      return false;
    }
    unsigned index_b;
    if( !StringUtilities::extractFromString( attrib_b->value(), index_b ) )
    {
      std::cerr << "Failed to parse planeB attribute for lees_edwards_portal node with value " << attrib_b->value() << ". Attribute must be an unsigned integer." << std::endl;
      return false;
    }
    if( index_b >= planes.size() )
    {
      std::cerr << "Failed to parse planeB attribute for lees_edwards_portal node with value " << attrib_b->value() << ". Attribute must be an index of a plane between " << 0 << " and " << planes.size() - 1 << std::endl;
      return false;
    }

    // Indices for this portal can't repeat
    if( index_a == index_b )
    {
      std::cerr << "Failed to parse planeB attribute for lees_edwards_portal node with value " << attrib_b->value() << ". Value is a repeat of attribute planeA." << std::endl;
      return false;
    }

    // Indices can not repeat previous portals
    for( std::vector<std::pair<unsigned,unsigned>>::size_type i = 0; i < plane_pairs.size(); ++i )
    {
      if( index_a == plane_pairs[i].first || index_a == plane_pairs[i].second )
      {
        std::cerr << "Failed to parse planeA attribute for lees_edwards_portal node with value " << attrib_a->value() << ". Plane index is used by an existing portal." << std::endl;
        return false;
      }
      if( index_b == plane_pairs[i].first || index_b == plane_pairs[i].second )
      {
        std::cerr << "Failed to parse planeB attribute for lees_edwards_portal node with value " << attrib_b->value() << ". Plane index is used by an existing portal." << std::endl;
        return false;
      }
    }

    plane_pairs.emplace_back( index_a, index_b );

    // Load the velocity of the portal
    scalar v;
    {
      if( nd->first_attribute( "v" ) == nullptr )
      {
        std::cerr << "Could not locate v attribue for lees_edwards_portal" << std::endl;
        return false;
      }
      const rapidxml::xml_attribute<>& v_attrib{ *nd->first_attribute( "v" ) };
      if( !StringUtilities::extractFromString( std::string{ v_attrib.value() }, v ) )
      {
        std::cerr << "Could not load v attribue for lees_edwards_portal, value must be a scalar" << std::endl;
        return false;
      }
    }

    // Load the bounds on portal a's translation
    scalar bounds;
    // TODO: Pull into an extract positive scalar function
    {
      const rapidxml::xml_attribute<>* const bound_attr{ nd->first_attribute( "bounds" ) };
      if( bound_attr != nullptr )
      {
        if( !StringUtilities::extractFromString( std::string{ bound_attr->value() }, bounds ) )
        {
          std::cerr << "Could not load bounds attribue for lees_edwards_portal, value must be a scalar" << std::endl;
          return false;
        }
        if( bounds < 0 )
        {
          std::cerr << "Failed to load bounds attribute for lees_edwards_portal, value must be positive" << std::endl;
          return false;
        }
      }
      else
      {
        std::cerr << "Could not locate bounds attribue for lees_edwards_portal" << std::endl;
        return false;
      }
    }

    plane_tangent_velocities.emplace_back( v );
    plane_bounds.emplace_back( bounds );
  }

  assert( plane_pairs.size() == plane_tangent_velocities.size() );
  assert( plane_pairs.size() == plane_bounds.size() );
  for( std::vector<std::pair<unsigned,unsigned>>::size_type i = 0; i < plane_pairs.size(); ++i )
  {
    planar_portals.emplace_back( planes[plane_pairs[i].first], planes[plane_pairs[i].second], plane_tangent_velocities[i], plane_bounds[i] );
  }

  // TODO: This could get slow if there are a ton of portals, but probably not too big of a deal for now
  {
    std::vector<unsigned> indices;
    for( std::vector<std::pair<unsigned,unsigned>>::size_type i = 0; i < plane_pairs.size(); ++i )
    {
      indices.emplace_back( plane_pairs[i].first );
      indices.emplace_back( plane_pairs[i].second );
    }
    std::sort( indices.begin(), indices.end() );
    for( std::vector<unsigned>::size_type i = indices.size(); i-- > 0; )
    {
      planes.erase( planes.begin() + indices[i] );
    }
  }

  return true;
}

static bool loadIntegrator( const rapidxml::xml_node<>& node, std::unique_ptr<UnconstrainedMap>& integrator, std::string& dt_string, Rational<std::intmax_t>& dt )
{
  // Attempt to locate the integrator node
  const rapidxml::xml_node<>* const nd{ node.first_node( "integrator" ) };
  // If not present, set the timestep to a default of 0
  if( nd == nullptr )
  {
    integrator.reset( nullptr );
    dt_string = "0";
    dt.set( 0, 1 );
    return true;
  }

  // Attempt to load the timestep
  {
    const rapidxml::xml_attribute<>* const dtnd{ nd->first_attribute( "dt" ) };
    if( dtnd == nullptr )
    {
      std::cerr << "Failed to locate dt attribute for integrator node." << std::endl;
      return false;
    }
    if( !extractFromString( std::string( dtnd->value() ), dt ) || !dt.positive() )
    {
      std::cerr << "Failed to load dt attribute for integrator. Must provide a positive number." << std::endl;
      return false;
    }
    dt_string = dtnd->value();
  }

  // Attempt to load the integrator type
  {
    const rapidxml::xml_attribute<>* const typend{ nd->first_attribute( "type" ) };
    if( typend == nullptr )
    {
      std::cerr << "Failed to locate type attribute for integrator node." << std::endl;
      return false;
    }
    const std::string integrator_type{ typend->value() };
    if( integrator_type == "symplectic_euler" )
    {
      integrator.reset( new SymplecticEulerMap );
    }
    else if( integrator_type == "verlet" )
    {
      integrator.reset( new VerletMap );
    }
    else
    {
      std::cerr << "Invalid integrator 'type' attribute specified for integrator node. Options are: symplectic_euler, verlet" << std::endl;
      return false;
    }
  }

  return true;
}

#if defined(QL_FOUND) || defined(IPOPT_FOUND)
static bool loadLCPSolver( const rapidxml::xml_node<>& node, std::unique_ptr<ImpactOperator>& impact_operator )
#else
static bool loadLCPSolver( const rapidxml::xml_node<>& node, std::unique_ptr<ImpactOperator>& /*impact_operator*/ )
#endif
{
  const rapidxml::xml_attribute<>* const nd{ node.first_attribute( "name" ) };

  if( nd == nullptr )
  {
    return false;
  }

  const std::string solver_name = std::string{ nd->value() };

  #ifdef QL_FOUND
  if( solver_name == "ql_vp" )
  {
    // Attempt to parse the solver tolerance
    const rapidxml::xml_attribute<>* const tol_nd{ node.first_attribute( "tol" ) };
    if( tol_nd == nullptr )
    {
      std::cerr << "Could not locate tol for ql_vp solver" << std::endl;
      return false;
    }
    scalar tol;
    if( !StringUtilities::extractFromString( std::string{ tol_nd->value() }, tol ) )
    {
      std::cerr << "Could not load tol for ql_vp solver" << std::endl;
      return false;
    }
    impact_operator.reset( new LCPOperatorQLVP{ tol } );
  }
  else if( solver_name == "ql" )
  {
    // Attempt to parse the solver tolerance
    const rapidxml::xml_attribute<>* const tol_nd{ node.first_attribute( "tol" ) };
    if( tol_nd == nullptr )
    {
      std::cerr << "Could not locate tol for ql solver" << std::endl;
      return false;
    }
    scalar tol;
    if( !StringUtilities::extractFromString( std::string{ tol_nd->value() }, tol ) )
    {
      std::cerr << "Could not load tol for ql solver" << std::endl;
      return false;
    }
    impact_operator.reset( new LCPOperatorQL{ tol } );
  }
  else
  #endif
  #ifdef IPOPT_FOUND
  if( solver_name == "ipopt" )
  {
    // Attempt to read the desired linear solvers
    std::vector<std::string> linear_solvers;
    {
      const rapidxml::xml_attribute<>* const attrib{ node.first_attribute( "linear_solvers" ) };
      if( attrib == nullptr )
      {
        std::cerr << "Could not locate linear solvers for ipopt solver" << std::endl;
        return false;
      }

      std::stringstream ss;
      ss << attrib->value();
      std::string input_string;
      while( ss >> input_string )
      {
        linear_solvers.emplace_back( input_string );
      }
      if( linear_solvers.empty() )
      {
        std::cerr << "Could not locate linear solvers for ipopt solver" << std::endl;
        return false;
      }
    }

    // Attempt to read the convergence tolerance
    scalar con_tol = std::numeric_limits<scalar>::signaling_NaN();
    {
      const rapidxml::xml_attribute<>* const attrib{ node.first_attribute( "tol" ) };
      if( attrib == nullptr )
      {
        std::cerr << "Could not locate tol attribute for ipopt solver" << std::endl;
        return false;
      }

      if( !StringUtilities::extractFromString( attrib->value(), con_tol ) )
      {
        std::cerr << "Could not load tol for ipopt solver" << std::endl;
        return false;
      }

      if( con_tol <= 0.0 )
      {
        std::cerr << "Could not load tol for ipopt solver, value must be positive scalar." << std::endl;
        return false;
      }
    }
    impact_operator.reset( new LCPOperatorIpopt{ linear_solvers, con_tol } );
  }
  else
  #endif
  #if defined(QL_FOUND) || defined(IPOPT_FOUND)
  {
    std::cerr << "Invalid lcp solver name: " << solver_name << std::endl;
    return false;
  }

  return true;
  #else
  std::cerr << "Invalid lcp solver name: " << solver_name << std::endl;
  return false;
  #endif
}

// TODO: Clean this function up, pull into SCISim
static bool loadImpactOperatorNoCoR( const rapidxml::xml_node<>& node, std::unique_ptr<ImpactOperator>& impact_operator )
{
  // Attempt to load the impact operator type
  std::string type;
  {
    const rapidxml::xml_attribute<>* const typend{ node.first_attribute( "type" ) };
    if( typend == nullptr )
    {
      std::cerr << "Could not locate type" << std::endl;
      return false;
    }
    type = typend->value();
  }

  scalar v_tol = std::numeric_limits<scalar>::signaling_NaN();
  if( type == "gauss_seidel" || type == "jacobi" || type == "gr" )
  {
    // Attempt to load the termination tolerance
    const rapidxml::xml_attribute<>* const v_tol_nd{ node.first_attribute( "v_tol" ) };
    if( v_tol_nd == nullptr )
    {
      std::cerr << "Could not locate v_tol" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( std::string( v_tol_nd->value() ), v_tol ) || v_tol < 0.0 )
    {
      std::cerr << "Could not load v_tol, value must be a positive scalar" << std::endl;
      return false;
    }
  }

  if( type == "gauss_seidel" )
  {
    impact_operator.reset( new GaussSeidelOperator( v_tol ) );
  }
  else if( type == "jacobi" )
  {
    impact_operator.reset( new JacobiOperator( v_tol ) );
  }
  else if( type == "lcp" )
  {
    if( node.first_node( "solver" ) != nullptr )
    {
      if( !loadLCPSolver( *node.first_node( "solver" ), impact_operator ) )
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  else if( type == "gr" )
  {
    if( node.first_node( "solver" ) != nullptr )
    {
      std::unique_ptr<ImpactOperator> lcp_solver{ nullptr };
      if( !loadLCPSolver( *node.first_node( "solver" ), lcp_solver ) )
      {
        return false;
      }
      impact_operator.reset( new GROperator( v_tol, *lcp_solver ) );
    }
    else
    {
      return false;
    }
  }
  else if( type == "grr" )
  {
    // Generalized restitution requires an elastic operator
    std::unique_ptr<ImpactOperator> elastic_operator{ nullptr };
    {
      const rapidxml::xml_node<>* const elastic_node{ node.first_node( "elastic_operator" ) };
      if( elastic_node == nullptr )
      {
        std::cerr << "Failed to locate elastic_operator for grr impact_operator" << std::endl;
        return false;
      }
      if( !loadImpactOperatorNoCoR( *elastic_node, elastic_operator ) )
      {
        std::cerr << "Failed to load elastic_operator for grr impact_operator" << std::endl;
        return false;
      }
    }

    // Generalized restitution requires an inelastic operator
    std::unique_ptr<ImpactOperator> inelastic_operator{ nullptr };
    {
      const rapidxml::xml_node<>* const inelastic_node{ node.first_node( "inelastic_operator" ) };
      if( inelastic_node == nullptr )
      {
        std::cerr << "Failed to locate inelastic_operator for grr impact_operator" << std::endl;
        return false;
      }
      if( !loadImpactOperatorNoCoR( *inelastic_node, inelastic_operator ) )
      {
        std::cerr << "Failed to load inelastic_operator for grr impact_operator" << std::endl;
        return false;
      }
    }

    impact_operator.reset( new GRROperator{ *elastic_operator, *inelastic_operator } );
  }
  else
  {
    return false;
  }

  return true;
}

static bool loadImpactOperator( const rapidxml::xml_node<>& node, std::unique_ptr<ImpactOperator>& impact_operator, scalar& CoR, bool& cache_impulses )
{
  // Attempt to load the CoR
  {
    const rapidxml::xml_attribute<>* const cor_nd{ node.first_attribute( "CoR" ) };
    if( cor_nd == nullptr )
    {
      std::cerr << "Could not locate CoR" << std::endl;
      return false;
    }

    CoR = std::numeric_limits<scalar>::signaling_NaN();
    if( !StringUtilities::extractFromString( std::string{ cor_nd->value() }, CoR ) )
    {
      std::cerr << "Could not load CoR value" << std::endl;
      return false;
    }
  }

  // Attempt to load the impulse cache option
  {
    const rapidxml::xml_attribute<>* const cache_nd{ node.first_attribute( "cache_impulses" ) };
    if( cache_nd == nullptr )
    {
      std::cerr << "Could not locate cache_impulses" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( cache_nd->value(), cache_impulses ) )
    {
      std::cerr << "Could not load cache_impulses" << std::endl;
      return false;
    }
  }
  
  return loadImpactOperatorNoCoR( node, impact_operator );
}

//static bool loadQLMDPOperator( const rapidxml::xml_node<>& node, std::unique_ptr<FrictionOperator>& friction_operator )
//{
//  // Attempt to parse the solver tolerance
//  scalar tol;
//  {
//    const rapidxml::xml_attribute<>* const tol_nd = node.first_attribute( "tol" );
//    if( tol_nd == nullptr )
//    {
//      std::cerr << "Could not locate tol for QL MDP solver" << std::endl;
//      return false;
//    }
//    if( !StringUtilities::extractFromString( std::string{ tol_nd->value() }, tol ) || tol < 0.0 )
//    {
//      std::cerr << "Could not load tol for QL MDP solver, value must be a positive scalar" << std::endl;
//      return false;
//    }
//  }
//  friction_operator.reset( new BoundConstrainedMDPOperatorQL{ tol } );
//  return true;
//}

static bool loadMDPOperator( const rapidxml::xml_node<>& /*node*/, std::unique_ptr<FrictionOperator>& /*friction_operator*/ )
{
  std::cerr << "2D rigid body loadMDPOperator not coded up." << std::endl;
  std::exit( EXIT_FAILURE );
//  // Attempt to load the impact operator type
//  std::string name;
//  {
//    const rapidxml::xml_attribute<>* const typend = node.first_attribute( "name" );
//    if( typend == nullptr )
//    {
//      std::cerr << "Could not locate name" << std::endl;
//      return false;
//    }
//    name = typend->value();
//  }
//
//  //if( name == "ql" )
//  //{
//  //  return loadQLMDPOperator( node, friction_operator );
//  //}
//  return false;
}

// Example:
//  <staggered_projections_friction_solver mu="2.0" CoR="0.8" max_iters="50" tol="1.0e-8" staggering="geometric" internal_warm_start_alpha="1" internal_warm_start_beta="1">
//    <lcp_impact_solver name="ipopt" tol="1.0e-12" linear_solvers="ma97"/>
//    <mdp_friction_solver name="ipopt" tol="1.0e-12" linear_solvers="ma97"/>
//  </staggered_projections_friction_solver>
static bool loadStaggeredProjectionsFrictionSolver( const rapidxml::xml_node<>& node, scalar& mu, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, std::unique_ptr<ImpactFrictionMap>& if_map )
{
  // Friction solver setup
  {
    // Attempt to load the coefficient of friction
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "mu" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate mu for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      mu = std::numeric_limits<scalar>::signaling_NaN();
      if( !StringUtilities::extractFromString( attrib_nd->value(), mu ) )
      {
        std::cerr << "Could not load mu value for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      if( mu < 0.0 )
      {
        std::cerr << "Could not load mu value for staggered_projections_friction_solver, value of mu must be a nonnegative scalar" << std::endl;
        return false;
      }
    }

    // Attempt to load the coefficient of restitution
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "CoR" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate CoR for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      CoR = std::numeric_limits<scalar>::signaling_NaN();
      if( !StringUtilities::extractFromString( attrib_nd->value(), CoR ) )
      {
        std::cerr << "Could not load CoR value for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      if( CoR < 0.0 || CoR > 1.0 )
      {
        std::cerr << "Could not load CoR value for staggered_projections_friction_solver, value of CoR must be a nonnegative scalar" << std::endl;
        return false;
      }
    }

    // Attempt to load a warm start alpha setting
    bool internal_warm_start_alpha;
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "internal_warm_start_alpha" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate internal_warm_start_alpha for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      if( !StringUtilities::extractFromString( attrib_nd->value(), internal_warm_start_alpha ) )
      {
        std::cerr << "Could not load internal_warm_start_alpha value for staggered_projections_friction_solver, value of internal_warm_start_alpha must be a boolean" << std::endl;
        return false;
      }
    }

    // Attempt to load a warm start beta setting
    bool internal_warm_start_beta;
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "internal_warm_start_beta" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate internal_warm_start_beta for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      if( !StringUtilities::extractFromString( attrib_nd->value(), internal_warm_start_beta ) )
      {
        std::cerr << "Could not load internal_warm_start_beta value for staggered_projections_friction_solver, value of internal_warm_start_alpha must be a boolean" << std::endl;
        return false;
      }
    }

    // Attempt to load the impact operator
    std::unique_ptr<ImpactOperator> impact_operator;
    {
      const rapidxml::xml_node<>* const impact_operator_node{ node.first_node( "lcp_impact_solver" ) };
      if( impact_operator_node == nullptr )
      {
        std::cerr << "Could not locate lcp_impact_solver node for staggered_projections_friction_solver" << std::endl;
        return false;
      }
      if( !loadLCPSolver( *impact_operator_node, impact_operator ) )
      {
        return false;
      }
    }

    // Attempt to load the friction operator
    std::unique_ptr<FrictionOperator> friction_operator;
    {
      const rapidxml::xml_node<>* const friction_operator_node{ node.first_node( "mdp_friction_solver" ) };
      if( friction_operator_node == nullptr )
      {
        std::cerr << "Could not locate mdp_friction_solver node for staggered_projections_friction_solver" << std::endl;
        return false;
      }
      if( !loadMDPOperator( *friction_operator_node, friction_operator ) )
      {
        return false;
      }
    }

    friction_solver.reset( new StaggeredProjections{ internal_warm_start_alpha, internal_warm_start_beta, *impact_operator, *friction_operator } );
  }

  // Impact-friction map setup
  // TODO: setting up if_map can be done in a separate function
  {
    // Attempt to load the staggering type
    std::string staggering_type;
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "staggering" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate staggering attribute for staggered_projections_friction_solver" << std::endl;
        return false;
      }
      staggering_type = attrib_nd->value();
    }

    // Attempt to load the termination tolerance
    scalar tol;
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "tol" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate tol for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      if( !StringUtilities::extractFromString( attrib_nd->value(), tol ) )
      {
        std::cerr << "Could not load tol value for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      if( tol < 0.0 )
      {
        std::cerr << "Could not load tol value for staggered_projections_friction_solver, value of tol must be a nonnegative scalar" << std::endl;
        return false;
      }
    }

    // Attempt to load the maximum number of iterations
    int max_iters;
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "max_iters" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate max_iters for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      if( !StringUtilities::extractFromString( attrib_nd->value(), max_iters ) )
      {
        std::cerr << "Could not load max_iters value for staggered_projections_friction_solver" << std::endl;
        return false;
      }

      if( max_iters <= 0 )
      {
        std::cerr << "Could not load max_iters value for staggered_projections_friction_solver, value of max_iters must be positive integer." << std::endl;
        return false;
      }
    }

    if( staggering_type == "geometric" )
    {
      if_map.reset( new GeometricImpactFrictionMap{ tol, static_cast<unsigned>( max_iters ), ImpulsesToCache::NONE } );
    }
    else if( staggering_type == "stabilized" )
    {
      if_map.reset( new StabilizedImpactFrictionMap{ tol, static_cast<unsigned>( max_iters ), false, false } );
    }
    else
    {
      std::cerr << "Invalid staggering attribute specified for staggered_projections_friction_solver, options are: ";
      std::cerr << "geometric, stabilized" << std::endl;
      return false;
    }
  }

  return true;
}

static bool loadSobogusFrictionSolver( const rapidxml::xml_node<>& node, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, scalar& CoR, std::unique_ptr<ImpactFrictionMap>& if_map )
{
  // Attempt to load the coefficient of friction
  {
    const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "mu" ) };
    if( attrib_nd == nullptr )
    {
      std::cerr << "Could not locate mu for sobogus_friction_solver" << std::endl;
      return false;
    }

    mu = std::numeric_limits<scalar>::signaling_NaN();
    if( !StringUtilities::extractFromString( attrib_nd->value(), mu ) )
    {
      std::cerr << "Could not load mu value for sobogus_friction_solver" << std::endl;
      return false;
    }

    if( mu < 0.0 )
    {
      std::cerr << "Could not load mu value for sobogus_friction_solver, value of mu must be a nonnegative scalar" << std::endl;
      return false;
    }
  }

  // Attempt to load the coefficient of restitution
  {
    const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "CoR" ) };
    if( attrib_nd == nullptr )
    {
      std::cerr << "Could not locate CoR for sobogus_friction_solver" << std::endl;
      return false;
    }

    CoR = std::numeric_limits<scalar>::signaling_NaN();
    if( !StringUtilities::extractFromString( attrib_nd->value(), CoR ) )
    {
      std::cerr << "Could not load CoR value for sobogus_friction_solver" << std::endl;
      return false;
    }

    if( CoR < 0.0 || CoR > 1.0 )
    {
      std::cerr << "Could not load CoR value for sobogus_friction_solver, value of CoR must be a nonnegative scalar" << std::endl;
      return false;
    }
  }

  // Attempt to load the maximum number of iterations
  int max_iters;
  {
    const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "max_iters" ) };
    if( attrib_nd == nullptr )
    {
      std::cerr << "Could not locate max_iters for sobogus_friction_solver" << std::endl;
      return false;
    }

    if( !StringUtilities::extractFromString( attrib_nd->value(), max_iters ) )
    {
      std::cerr << "Could not load max_iters value for sobogus_friction_solver" << std::endl;
      return false;
    }

    if( max_iters <= 0 )
    {
      std::cerr << "Could not load max_iters value for sobogus_friction_solver, value of max_iters must be positive integer." << std::endl;
      return false;
    }
  }

  // Attempt to load the spacing between error evaluation
  int eval_every;
  {
    const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "eval_every" ) };
    if( attrib_nd == nullptr )
    {
      std::cerr << "Could not locate eval_every for sobogus_friction_solver" << std::endl;
      return false;
    }

    if( !StringUtilities::extractFromString( attrib_nd->value(), eval_every ) )
    {
      std::cerr << "Could not load eval_every value for sobogus_friction_solver" << std::endl;
      return false;
    }

    if( eval_every <= 0 )
    {
      std::cerr << "Could not load eval_every value for sobogus_friction_solver, value of max_iters must be positive integer." << std::endl;
      return false;
    }

    if( eval_every > max_iters )
    {
      std::cerr << "Could not load eval_every value for sobogus_friction_solver, value of eval_every must must be less than or equal to max_iters." << std::endl;
      return false;
    }
  }

  // Attempt to load the termination tolerance
  scalar tol;
  {
    const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "tol" ) };
    if( attrib_nd == nullptr )
    {
      std::cerr << "Could not locate tol for sobogus_friction_solver" << std::endl;
      return false;
    }

    if( !StringUtilities::extractFromString( attrib_nd->value(), tol ) )
    {
      std::cerr << "Could not load tol value for sobogus_friction_solver" << std::endl;
      return false;
    }

    if( tol < 0.0 )
    {
      std::cerr << "Could not load tol value for sobogus_friction_solver, value of tol must be a nonnegative scalar" << std::endl;
      return false;
    }
  }

  // Attempt to load the cache_impulses option
  ImpulsesToCache cache_impulses;
  {
    const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "cache_impulses" ) };
    if( attrib_nd == nullptr )
    {
      std::cerr << "Could not locate cache_impulses attribute for sobogus_friction_solver" << std::endl;
      return false;
    }
    const std::string impulses_to_cache{ attrib_nd->value() };
    if( "none" == impulses_to_cache )
    {
      cache_impulses = ImpulsesToCache::NONE;
    }
    else if( "normal" == impulses_to_cache )
    {
      cache_impulses = ImpulsesToCache::NORMAL;
    }
    else if( "normal_and_friction" == impulses_to_cache )
    {
      cache_impulses = ImpulsesToCache::NORMAL_AND_FRICTION;
    }
    else
    {
      std::cerr << "Invalid option specified for cache_impulses. Valid options are: none, normal, normal_and_friction" << std::endl;
      std::exit( EXIT_FAILURE );
    }
  }

  // Attempt to load the staggering type
  std::string staggering_type;
  {
    const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "staggering" ) };
    if( attrib_nd == nullptr )
    {
      std::cerr << "Could not locate staggering attribute for sobogus_friction_solver" << std::endl;
      return false;
    }
    staggering_type = attrib_nd->value();
  }

  if( staggering_type == "geometric" )
  {
    if_map.reset( new GeometricImpactFrictionMap{ tol, static_cast<unsigned>( max_iters ), cache_impulses } );
  }
  else if( staggering_type == "stabilized" )
  {
    if_map.reset( new StabilizedImpactFrictionMap{ tol, static_cast<unsigned>( max_iters ), false, false } );
  }
  else
  {
    std::cerr << "Invalid staggering attribute specified for sobogus_friction_solver" << std::endl;
    return false;
  }

  friction_solver.reset( new Sobogus{ SobogusSolverType::RigidBody2D, unsigned( eval_every ) } );

  return true;
}

static bool loadGravityForce( const rapidxml::xml_node<>& node, std::vector<std::unique_ptr<RigidBody2DForce>>& forces )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "near_earth_gravity" ); nd; nd = nd->next_sibling( "near_earth_gravity" ) )
  {
    const rapidxml::xml_attribute<>* const f_attrib{ nd->first_attribute( "f" ) };
    if( f_attrib == nullptr )
    {
      std::cerr << "Failed to locate f attribute for near_earth_gravity node." << std::endl;
      return false;
    }
    Vector2s f;
    if( !StringUtilities::readScalarList( f_attrib->value(), 2, ' ', f ) )
    {
      std::cerr << "Failed to load f attribute for near_earth_gravity node, must provide two scalars." << std::endl;
      return false;
    }
    forces.emplace_back( new NearEarthGravityForce{ f } );
  }

  return true;
}

static bool loadGeometry( const rapidxml::xml_node<>& node, std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry )
{
  geometry.clear();

  for( rapidxml::xml_node<>* nd = node.first_node( "geometry" ); nd; nd = nd->next_sibling( "geometry" ) )
  {
    // Load the type of the geometry
    std::string geom_type;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "type" ) };
      if( attrib == nullptr )
      {
        std::cerr << "Failed to locate type attribute for geometry node." << std::endl;
        return false;
      }
      geom_type = attrib->value();
      // Parse the remaining arguments based on the geometry type
      if( geom_type == "circle" )
      {
        // Read the radius
        const rapidxml::xml_attribute<>* const r_attrib{ nd->first_attribute( "r" ) };
        if( r_attrib == nullptr )
        {
          std::cerr << "Failed to locate r attribute for circle geometry node." << std::endl;
          return false;
        }
        scalar r;
        if( !StringUtilities::extractFromString( r_attrib->value(), r ) || r <= 0.0 )
        {
          std::cerr << "Failed to read r attribute for circle geometry, must provide a positive scalar." << std::endl;
          return false;
        }
        geometry.emplace_back( new CircleGeometry{ r } );
      }
      else if( geom_type == "box" )
      {
        // Read the half-widths
        const rapidxml::xml_attribute<>* const r_attrib{ nd->first_attribute( "r" ) };
        if( r_attrib == nullptr )
        {
          std::cerr << "Failed to locate r attribute for box geometry node." << std::endl;
          return false;
        }
        Vector2s r;
        if( !StringUtilities::readScalarList( r_attrib->value(), 2, ' ', r ) || ( r.array() <= 0.0 ).any() )
        {
          std::cerr << "Failed to read r attribute for box geometry, must provide two positive scalars." << std::endl;
          return false;
        }
        geometry.emplace_back( new BoxGeometry{ r } );
      }
      else
      {
        std::cerr << "Invalid type specified for geometry node of " << geom_type << "." << std::endl;
        return false;
      }
    }
  }

  return true;
}

static bool loadBodies( const rapidxml::xml_node<>& node, const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry, VectorXs& q, VectorXs& v, VectorXs& m, VectorXu& indices, std::vector<bool>& fixed )
{
  std::vector<Vector2s> xs;
  std::vector<scalar> thetas;
  std::vector<Vector2s> vs;
  std::vector<scalar> omegas;
  std::vector<scalar> densities;
  std::vector<unsigned> geometry_indices;
  assert( fixed.empty() );

  for( rapidxml::xml_node<>* nd = node.first_node( "rigid_body" ); nd; nd = nd->next_sibling( "rigid_body" ) )
  {
    // Load the center of mass' position
    {
      const rapidxml::xml_attribute<>* const x_attrib{ nd->first_attribute( "x" ) };
      if( x_attrib == nullptr )
      {
        std::cerr << "Failed to locate x attribute for rigid_body node." << std::endl;
        return false;
      }
      Vector2s x;
      if( !StringUtilities::readScalarList( x_attrib->value(), 2, ' ', x ) )
      {
        std::cerr << "Failed to load x attribute for rigid_body node, must provide two scalars." << std::endl;
        return false;
      }
      xs.emplace_back( x );
    }

    // Load the rotation about the center of mass
    {
      const rapidxml::xml_attribute<>* const theta_attrib{ nd->first_attribute( "theta" ) };
      if( theta_attrib == nullptr )
      {
        std::cerr << "Failed to locate theta attribute for rigid_body node." << std::endl;
        return false;
      }
      scalar theta;
      if( !StringUtilities::extractFromString( theta_attrib->value(), theta ) )
      {
        std::cerr << "Failed to load theta attribute for rigid_body node, must provide a single scalar." << std::endl;
        return false;
      }
      thetas.emplace_back( theta );
    }

    // Load the center of mass' velocity
    {
      const rapidxml::xml_attribute<>* const v_attrib{ nd->first_attribute( "v" ) };
      if( v_attrib == nullptr )
      {
        std::cerr << "Failed to locate v attribute for rigid_body node." << std::endl;
        return false;
      }
      Vector2s v_body;
      if( !StringUtilities::readScalarList( v_attrib->value(), 2, ' ', v_body ) )
      {
        std::cerr << "Failed to load v attribute for rigid_body node, must provide two scalars." << std::endl;
        return false;
      }
      vs.emplace_back( v_body );
    }

    // Load the angular velocity
    {
      const rapidxml::xml_attribute<>* const omega_attrib{ nd->first_attribute( "omega" ) };
      if( omega_attrib == nullptr )
      {
        std::cerr << "Failed to locate omega attribute for rigid_body node." << std::endl;
        return false;
      }
      scalar omega;
      if( !StringUtilities::extractFromString( omega_attrib->value(), omega ) )
      {
        std::cerr << "Failed to load omega attribute for rigid_body node, must provide a single scalar." << std::endl;
        return false;
      }
      omegas.emplace_back( omega );
    }

    // Load the density
    {
      const rapidxml::xml_attribute<>* const rho_attrib{ nd->first_attribute( "rho" ) };
      if( rho_attrib == nullptr )
      {
        std::cerr << "Failed to locate rho attribute for rigid_body node." << std::endl;
        return false;
      }
      scalar rho;
      if( !StringUtilities::extractFromString( rho_attrib->value(), rho ) || rho <= 0.0 )
      {
        std::cerr << "Failed to load rho attribute for rigid_body node, must provide a single positive scalar." << std::endl;
        return false;
      }
      densities.emplace_back( rho );
    }

    // Load the index of this body's geometry
    {
      const rapidxml::xml_attribute<>* const geo_idx_attrib{ nd->first_attribute( "geo_idx" ) };
      if( geo_idx_attrib == nullptr )
      {
        std::cerr << "Failed to locate geo_idx attribute for rigid_body node." << std::endl;
        return false;
      }
      int geometry_index;
      if( !StringUtilities::extractFromString( geo_idx_attrib->value(), geometry_index ) || geometry_index < 0 || geometry_index >= int( geometry.size() ) )
      {
        std::cerr << "Failed to load geo_idx attribute for rigid_body node, must provide an unsigned integer less than the number of geometry instances." << std::endl;
        return false;
      }
      geometry_indices.emplace_back( unsigned( geometry_index ) );
    }

    // Load the optional fixed attribute
    {
      const rapidxml::xml_attribute<>* const fixed_attrib{ nd->first_attribute( "fixed" ) };
      if( fixed_attrib == nullptr )
      {
        fixed.push_back( false );
      }
      else
      {
        bool fixed_val;
        if( !StringUtilities::extractFromString( fixed_attrib->value(), fixed_val ) )
        {
          std::cerr << "Failed to load fixed attribute for rigid_body node, fixed must be a boolean." << std::endl;
          return false;
        }
        fixed.push_back( fixed_val );
      }
    }
  }

  const unsigned nbodies{ static_cast<unsigned>( xs.size() ) };

  // Build the generalized configuration
  q.resize( 3 * nbodies );
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    q.segment<2>( 3 * bdy_idx ) = xs[bdy_idx];
    q( 3 * bdy_idx + 2 ) = thetas[ bdy_idx ];
  }

  // Build the generalized velocity
  v.resize( 3 * nbodies );
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    v.segment<2>( 3 * bdy_idx ) = vs[bdy_idx];
    v( 3 * bdy_idx + 2 ) = omegas[ bdy_idx ];
  }

  // Copy the geometry indices to the output
  indices = Eigen::Map<VectorXu>( geometry_indices.data(), nbodies );

  // Build the diagonal of the mass matrix
  m.resize( 3 * nbodies );
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    scalar mass;
    scalar inertia;
    geometry[ indices(bdy_idx) ]->computeMassAndInertia( densities[bdy_idx], mass, inertia );
    m.segment<2>( 3 * bdy_idx ).setConstant( mass );
    m( 3 * bdy_idx + 2 ) = inertia;
  }

  return true;
}

bool RigidBody2DSceneParser::parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback, RigidBody2DState& sim_state, std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator, std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map, CameraSettings2D& camera_settings )
{
  // Attempt to load the xml document
  std::vector<char> xmlchars;
  rapidxml::xml_document<> doc;
  if( !loadXMLFile( file_name, xmlchars, doc ) )
  {
    return false;
  }

  // Attempt to locate the root node
  if( doc.first_node( "rigidbody2d_scene" ) == nullptr )
  {
    std::cerr << "Failed to locate root node rigidbody2d_scene in xml scene file: " << file_name << std::endl;
    return false;
  }
  const rapidxml::xml_node<>& root_node{ *doc.first_node( "rigidbody2d_scene" ) };

  // Attempt to load an optional scripting callback
  if( !loadScriptingSetup( root_node, scripting_callback ) )
  {
    return false;
  }

  // Attempt to load the optional end time, if present
  if( !loadEndTime( root_node, end_time ) )
  {
    return false;
  }

  // Attempt to load the optional camera settings, if present
  if( !loadCameraSettings( root_node, camera_settings ) )
  {
    return false;
  }

  // Attempt to load the unconstrained integrator
  if( !loadIntegrator( root_node, unconstrained_map, dt_string, dt ) )
  {
    return false;
  }

  // Attempt to load an impact operator
  if( root_node.first_node( "impact_operator" ) != nullptr )
  {
    bool cache_impulses;
    if( !loadImpactOperator( *root_node.first_node( "impact_operator" ), impact_operator, CoR, cache_impulses ) )
    {
      std::cerr << "Failed to load impact_operator in xml scene file: " << file_name << std::endl;
      return false;
    }
    impact_map.reset( new ImpactMap{ cache_impulses } );
  }
  else
  {
    impact_operator.reset( nullptr );
    impact_map.reset( nullptr );
    CoR = SCALAR_NAN;
  }

  friction_solver.reset( nullptr );
  mu = SCALAR_NAN;
  if_map.reset( nullptr );

  // Load a staggered projections friction solver, if present
  if( root_node.first_node( "staggered_projections_friction_solver" ) != nullptr )
  {
    if( impact_operator != nullptr || impact_map != nullptr )
    {
      std::cerr << "Error loading staggered_projections_friction_solver, solver of type " << impact_operator->name() << " already specified" << std::endl;
      return false;
    }
    if( friction_solver != nullptr )
    {
      std::cerr << "Error loading staggered_projections_friction_solver, solver of type " << friction_solver->name() << " already specified" << std::endl;
      return false;
    }
    if( !loadStaggeredProjectionsFrictionSolver( *root_node.first_node( "staggered_projections_friction_solver" ), mu, CoR, friction_solver, if_map ) )
    {
      std::cerr << "Failed to load staggered_projections_friction_solver in xml scene file: " << file_name << std::endl;
      return false;
    }
  }

  // Load a Sobogus friction solver, if present
  if( root_node.first_node( "sobogus_friction_solver" ) != nullptr )
  {
    if( impact_operator != nullptr || impact_map != nullptr )
    {
      std::cerr << "Error loading sobogus_friction_solver, solver of type " << impact_operator->name() << " already specified" << std::endl;
      return false;
    }
    if( friction_solver != nullptr )
    {
      std::cerr << "Error loading sobogus_friction_solver, solver of type " << friction_solver->name() << " already specified" << std::endl;
      return false;
    }
    if( !loadSobogusFrictionSolver( *root_node.first_node( "sobogus_friction_solver" ), friction_solver, mu, CoR, if_map ) )
    {
      return false;
    }
  }

  // Load forces
  std::vector<std::unique_ptr<RigidBody2DForce>> forces;
  // Attempt to load a gravity force
  if( !loadGravityForce( root_node, forces ) )
  {
    return false;
  }

  // Attempt to load any user-provided static drums
  std::vector<RigidBody2DStaticPlane> planes;
  if( !loadStaticPlanes( root_node, planes ) )
  {
    return false;
  }

  // Attempt to load planar portals
  std::vector<PlanarPortal> planar_portals;
  if( !loadPlanarPortals( root_node, planes, planar_portals ) )
  {
    return false;
  }

  // Load geometry to attatch to bodies
  std::vector<std::unique_ptr<RigidBody2DGeometry>> geometry;
  if( !loadGeometry( root_node, geometry ) )
  {
    return false;
  }

  // TODO: Load state directly into RigidBody2DState
  // Load the state of the bodies
  VectorXs q;
  VectorXs v;
  VectorXs m;
  VectorXu indices;
  std::vector<bool> fixed;
  if( !loadBodies( root_node, geometry, q, v, m, indices, fixed ) )
  {
    return false;
  }

  sim_state = RigidBody2DState{ q, v, m, fixed, indices, geometry, forces, planes, planar_portals };

  return true;
}

bool RigidBody2DSceneParser::parseXMLSceneFile( const std::string& /*file_name*/, SimSettings& /*sim_settings*/, RenderSettings& /*render_settings*/ )
{
  return true;
}
