#include "Ball2DSceneParser.h"

#include <iostream>
#include <fstream>

#include "Ball2D.h"

#include "ball2d/Ball2DState.h"
#include "ball2d/Forces/Ball2DForce.h"
#include "ball2d/Forces/Ball2DGravityForce.h"
#include "ball2d/Forces/PenaltyForce.h"
#include "ball2d/StaticGeometry/StaticDrum.h"
#include "ball2d/StaticGeometry/StaticPlane.h"
#include "ball2d/Portals/PlanarPortal.h"
#include "ball2d/VerletMap.h"
#include "ball2d/SymplecticEulerMap.h"

#include "scisim/StringUtilities.h"
#include "scisim/Math/Rational.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/GeometricImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/StabilizedImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/SymplecticEulerImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GaussSeidelOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/JacobiOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GROperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GRROperator.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorAPGD.h"
#include "scisim/ConstrainedMaps/StaggeredProjections.h"
#include "scisim/ConstrainedMaps/Sobogus.h"

#include "rapidxml.hpp"

#ifdef IPOPT_FOUND
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorIpopt.h"
#endif

#ifdef QL_FOUND
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorQL.h"
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorQLVP.h"
#include "scisim/ConstrainedMaps/FrictionMaps/BoundConstrainedMDPOperatorQL.h"
#endif

static bool loadTextFileIntoVector( const std::string& filename, std::vector<char>& xmlchars )
{
  assert( xmlchars.empty() );

  // Attempt to open the text file for reading
  std::ifstream textfile{ filename };
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

static bool loadCameraSettings( const rapidxml::xml_node<>& node, Eigen::Vector2d& camera_center, double& camera_scale_factor, unsigned& fps, bool& render_at_fps, bool& lock_camera )
{
  // Attempt to parse the camera's center
  {
    const rapidxml::xml_attribute<>* cx_attrib{ node.first_attribute( "cx" ) };
    if( !cx_attrib )
    {
      std::cerr << "Failed to locate cx attribute for camera" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( cx_attrib->value(), camera_center.x() ) )
    {
      std::cerr << "Failed to parse cx attribute for camera. Value must be a scalar." << std::endl;
      return false;
    }
  }
  {
    const rapidxml::xml_attribute<>* cy_attrib{ node.first_attribute( "cy" ) };
    if( !cy_attrib )
    {
      std::cerr << "Failed to locate cy attribute for camera" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( cy_attrib->value(), camera_center.y() ) )
    {
      std::cerr << "Failed to parse cy attribute for camera. Value must be a scalar." << std::endl;
      return false;
    }
  }

  // Attempt to parse the scale setting
  {
    const rapidxml::xml_attribute<>* scale_factor_attrib{ node.first_attribute( "scale_factor" ) };
    if( !scale_factor_attrib )
    {
      std::cerr << "Failed to locate scale_factor attribute for camera" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( scale_factor_attrib->value(), camera_scale_factor ) )
    {
      std::cerr << "Failed to parse scale_factor attribute for camera. Value must be a scalar." << std::endl;
      return false;
    }
  }

  // Attempt to parse the fps setting
  {
    const rapidxml::xml_attribute<>* fps_attrib{ node.first_attribute( "fps" ) };
    if( fps_attrib == nullptr )
    {
      std::cerr << "Failed to locate fps attribute for camera" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( fps_attrib->value(), fps ) )
    {
      std::cerr << "Failed to parse fps attribute for camera. Value must be a non-negative integer." << std::endl;
      return false;
    }
  }

  // Attempt to parse the render_at_fps setting
  {
    const rapidxml::xml_attribute<>* render_at_fps_attrib{ node.first_attribute( "render_at_fps" ) };
    if( render_at_fps_attrib == nullptr )
    {
      std::cerr << "Failed to locate render_at_fps attribute for camera" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( render_at_fps_attrib->value(), render_at_fps ) )
    {
      std::cerr << "Failed to parse render_at_fps attribute for camera. Value must be a boolean." << std::endl;
      return false;
    }
  }

  // Attempt to parse the locked setting
  {
    const rapidxml::xml_attribute<>* locked_attrib{ node.first_attribute( "locked" ) };
    if( locked_attrib == nullptr )
    {
      std::cerr << "Failed to locate locked attribute for camera" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( locked_attrib->value(), lock_camera ) )
    {
      std::cerr << "Failed to parse locked attribute for camera. Value must be a boolean." << std::endl;
      return false;
    }
  }

  return true;
}

static bool loadEndTime( const rapidxml::xml_node<>& node, scalar& end_time )
{
  // Attempt to parse the time setting
  const rapidxml::xml_attribute<>* t_attrib{ node.first_attribute( "t" ) };
  if( !t_attrib )
  {
    std::cerr << "Failed to locate t attribute for end_time node." << std::endl;
    return false;
  }
  if( !StringUtilities::extractFromString( t_attrib->value(), end_time ) )
  {
    std::cerr << "Failed to parse t attribute for end_time. Value must be a positive scalar." << std::endl;
    return false;
  }
  if( end_time <= 0.0 )
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

static bool loadBalls( const rapidxml::xml_node<>& node, std::vector<Ball2D>& balls )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "ball" ); nd; nd = nd->next_sibling( "ball" ) )
  {
    // Attempt to parse the ball's position
    Vector2s x;

    const rapidxml::xml_attribute<>* const x_attrib{ nd->first_attribute( "x" ) };
    if( !x_attrib ) { return false; }
    StringUtilities::extractFromString( x_attrib->value(), x.x() );

    const rapidxml::xml_attribute<>* const y_attrib{ nd->first_attribute( "y" ) };
    if( !y_attrib ) { return false; }
    StringUtilities::extractFromString( y_attrib->value(), x.y() );

    // Attempt to parse the ball's velocity
    Vector2s v;

    const rapidxml::xml_attribute<>* const vx_attrib{ nd->first_attribute( "vx" ) };
    if( !vx_attrib ) { return false; }
    StringUtilities::extractFromString( vx_attrib->value(), v.x() );

    const rapidxml::xml_attribute<>* const vy_attrib{ nd->first_attribute( "vy" ) };
    if( !vy_attrib ) { return false; }
    StringUtilities::extractFromString( vy_attrib->value(), v.y() );

    // Attempt to parse the ball's mass
    scalar m;
    const rapidxml::xml_attribute<>* const m_attrib{ nd->first_attribute( "m" ) };
    if( !m_attrib ) { return false; }
    StringUtilities::extractFromString( m_attrib->value(), m );

    // Attempt to parse the ball's radius
    scalar r;
    const rapidxml::xml_attribute<>* const r_attrib{ nd->first_attribute( "r" ) };
    if( !r_attrib ) { return false; }
    StringUtilities::extractFromString( r_attrib->value(), r );

    // Attempt to parse whether the ball is fixed
    bool fixed;
    const rapidxml::xml_attribute<>* const fixed_attrib{ nd->first_attribute( "fixed" ) };
    if( !fixed_attrib ) { return false; }
    StringUtilities::extractFromString( fixed_attrib->value(), fixed );

    balls.emplace_back( x, v, m, r, fixed );
  }

  return true;
}

static bool loadStaticDrums( const rapidxml::xml_node<>& node, std::vector<StaticDrum>& drums )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "static_drum" ); nd; nd = nd->next_sibling( "static_drum" ) )
  {
    // Attempt to parse the drums's position
    Vector2s x;

    const rapidxml::xml_attribute<>* const x_attrib{ nd->first_attribute( "x" ) };
    if( !x_attrib ) { return false; }
    StringUtilities::extractFromString( x_attrib->value(), x.x() );

    const rapidxml::xml_attribute<>* const y_attrib{ nd->first_attribute( "y" ) };
    if( !y_attrib ) { return false; }
    StringUtilities::extractFromString( y_attrib->value(), x.y() );

    // Attempt to parse the drums's radius
    scalar r;
    const rapidxml::xml_attribute<>* const r_attrib{ nd->first_attribute( "r" ) };
    if( !r_attrib ) { return false; }
    StringUtilities::extractFromString( r_attrib->value(), r );

    drums.emplace_back( x, r );
  }

  return true;
}

static bool loadStaticPlanes( const rapidxml::xml_node<>& node, std::vector<StaticPlane>& planes )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "static_plane" ); nd; nd = nd->next_sibling( "static_plane" ) )
  {
    // Read a point on the plane
    Vector2s x;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "x" ) };
      if( !attrib ) { return false; }

      std::stringstream ss;
      ss << attrib->value();
      scalar component;
      std::vector<scalar> components;
      while( ss >> component ) { components.emplace_back( component ); }
      if( components.size() != 2 )
      {
        std::cerr << "Invalid number of components for static_plane x. Two required." << std::endl;
        return false;
      }

      x << components[0], components[1];
    }

    // Read the normal
    Vector2s n;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "n" ) };
      if( !attrib ) { return false; }

      std::stringstream ss;
      ss << attrib->value();
      scalar component;
      std::vector<scalar> components;
      while( ss >> component ) { components.emplace_back( component ); }
      if( components.size() != 2 )
      {
        std::cerr << "Invalid number of components for static_plane n. Two required." << std::endl;
        return false;
      }

      n << components[0], components[1];
    }

    planes.emplace_back( x, n );
  }

  return true;
}

// TODO: minus ones here can underflow
static bool loadPlanarPortals( const rapidxml::xml_node<>& node, std::vector<StaticPlane>& planes, std::vector<PlanarPortal>& planar_portals )
{
  if( !( node.first_node( "planar_portal" ) || node.first_node( "lees_edwards_portal" ) )  )
  {
    return true;
  }

  // If we have a portal we must have at least one plane
  if( planes.size() < 2 )
  {
    std::cerr << "Must provide at least two planes before instantiating a planar portal." << std::endl;
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
    const rapidxml::xml_attribute<>* const attrib_a{ nd->first_attribute( "planeA" ) };
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

    // Load the velocity of portal a
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

    // Load the bounds on the portal translation
    scalar bounds;
    {
      if( nd->first_attribute( "bounds" ) == nullptr )
      {
        std::cerr << "Could not locate bounds attribue for lees_edwards_portal" << std::endl;
        return false;
      }
      const rapidxml::xml_attribute<>& bounds_attrib{ *nd->first_attribute( "bounds" ) };
      if( !StringUtilities::extractFromString( std::string{ bounds_attrib.value() }, bounds ) )
      {
        std::cerr << "Could not load bounds attribue for lees_edwards_portal, value must be a scalar" << std::endl;
        return false;
      }
      if( bounds <= 0.0 )
      {
        std::cerr << "Could not load bounds attribue for lees_edwards_portal, value must be a positive scalar" << std::endl;
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
  assert( dt_string.empty() );

  // Attempt to locate the integrator node
  const rapidxml::xml_node<>* nd{ node.first_node( "integrator" ) };
  if( nd == nullptr )
  {
    std::cerr << "Failed to locate integrator node." << std::endl;
    return false;
  }

  // Attempt to load the timestep
  {
    const rapidxml::xml_attribute<>* dtnd{ nd->first_attribute( "dt" ) };
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
    const rapidxml::xml_attribute<>* typend{ nd->first_attribute( "type" ) };
    if( typend == nullptr )
    {
      std::cerr << "Failed to locate type attribute for integrator node." << std::endl;
      return false;
    }
    const std::string integrator_type{ typend->value() };
    if( integrator_type == "verlet" )
    {
      integrator.reset( new VerletMap );
    }
    else if( integrator_type == "symplectic_euler" )
    {
      integrator.reset( new SymplecticEulerMap );
    }
    else
    {
      std::cerr << "Invalid integrator 'type' attribute specified for integrator node. Options are: verlet, symplectic_euler." << std::endl;
      return false;
    }
  }

  return true;
}

static bool loadLCPSolver( const rapidxml::xml_node<>& node, std::unique_ptr<ImpactOperator>& impact_operator )
{
  const rapidxml::xml_attribute<>* const nd{ node.first_attribute( "name" ) };

  if( nd == nullptr )
  {
    return false;
  }

  const std::string solver_name = std::string{ nd->value() };

  if( solver_name == "apgd" )
  {
    // Attempt to parse the solver tolerance
    scalar tol;
    {
      const rapidxml::xml_attribute<>* const tol_nd{ node.first_attribute( "tol" ) };
      if( tol_nd == nullptr )
      {
        std::cerr << "Could not locate tol for apgd solver" << std::endl;
        return false;
      }
      if( !StringUtilities::extractFromString( std::string{ tol_nd->value() }, tol ) || tol <= 0.0 )
      {
        std::cerr << "Could not load tol for apgd solver, value must be a positive scalar" << std::endl;
        return false;
      }
    }
    // Attempt to parse the max number of iterations
    unsigned max_iters;
    {
      const rapidxml::xml_attribute<>* const itr_nd{ node.first_attribute( "max_iters" ) };
      if( itr_nd == nullptr )
      {
        std::cerr << "Could not locate max_iters for apgd solver" << std::endl;
        return false;
      }
      if( !StringUtilities::extractFromString( std::string{ itr_nd->value() }, max_iters ) )
      {
        std::cerr << "Could not load max_iters for apgd solver, value must be an unsigned integer" << std::endl;
        return false;
      }
    }
    impact_operator.reset( new LCPOperatorAPGD{ tol, max_iters } );
  }
  #ifdef QL_FOUND
  else if( solver_name == "ql_vp" )
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
  #endif
  #ifdef IPOPT_FOUND
  else if( solver_name == "ipopt" )
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
      while( ss >> input_string ) { linear_solvers.emplace_back( input_string ); }
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
  #endif
  else
  {
    std::cerr << "Invalid lcp solver name: " << solver_name << std::endl;
    return false;
  }

  return true;
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
    if( !StringUtilities::extractFromString( std::string{ v_tol_nd->value() }, v_tol ) || v_tol < 0.0 )
    {
      std::cerr << "Could not load v_tol, value must be a positive scalar" << std::endl;
      return false;
    }
  }

  if( type == "gauss_seidel" )
  {
    impact_operator.reset( new GaussSeidelOperator{ v_tol } );
  }
  else if( type == "jacobi" )
  {
    impact_operator.reset( new JacobiOperator{ v_tol } );
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
      impact_operator.reset( new GROperator{ v_tol, *lcp_solver } );
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
      std::cerr << "Could not locate cache_impulses attribue." << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( cache_nd->value(), cache_impulses ) )
    {
      std::cerr << "Could not load cache_impulses attribue. Value must be a boolean." << std::endl;
      return false;
    }
  }

  return loadImpactOperatorNoCoR( node, impact_operator );
}

#ifdef QL_FOUND
static bool loadQLMDPOperator( const rapidxml::xml_node<>& node, std::unique_ptr<FrictionOperator>& friction_operator )
{
  // Attempt to parse the solver tolerance
  scalar tol;
  {
    const rapidxml::xml_attribute<>* const tol_nd{ node.first_attribute( "tol" ) };
    if( tol_nd == nullptr )
    {
      std::cerr << "Could not locate tol for QL MDP solver" << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( std::string{ tol_nd->value() }, tol ) || tol < 0.0 )
    {
      std::cerr << "Could not load tol for QL MDP solver, value must be a positive scalar" << std::endl;
      return false;
    }
  }
  friction_operator.reset( new BoundConstrainedMDPOperatorQL{ tol } );
  return true;
}
#endif

static bool loadMDPOperator( const rapidxml::xml_node<>& node, std::unique_ptr<FrictionOperator>& friction_operator )
{
  // Attempt to load the impact operator type
  std::string name;
  {
    const rapidxml::xml_attribute<>* const typend{ node.first_attribute( "name" ) };
    if( typend == nullptr )
    {
      std::cerr << "Could not locate name" << std::endl;
      return false;
    }
    name = typend->value();
  }

  #ifdef QL_FOUND
  if( name == "ql" )
  {
    return loadQLMDPOperator( node, friction_operator );
  }
  else
  {
    std::cerr << "Error, invalid MDP operator name provided: " << name << std::endl;
    std::cerr << "Valid names are: ql" << std::endl;
    return false;
  }
  #else
  std::cerr << "Error, invalid MDP operator name provided: " << name << std::endl;
  std::cerr << "MDP operator is not currently supported." << std::endl;
  return false;
  #endif
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
    // else if( staggering_type == "symplectic_euler" )
    // {
    //   if_map.reset( new StewartAndTrinkleImpactFrictionMap{ tol, static_cast<unsigned>( max_iters ), STImpulsesToCache::NONE } );
    // }
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
  else if( staggering_type == "symplectic_euler" )
  {
    if( CoR != 0.0 )
    {
      std::cerr << "Error, CoR must be set to 0 for the symplectic_euler staggering." << std::endl;
      return false;
    }

    bool enable_pose_stab;
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "stabilization" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate stabilization for sobogus_friction_solver with symplectic_euler" << std::endl;
        return false;
      }
      if( !StringUtilities::extractFromString( attrib_nd->value(), enable_pose_stab ) )
      {
        std::cerr << "Could not load stabilization value for sobogus_friction_solver with symplectic_euler, value of stabilization must be a boolean" << std::endl;
        return false;
      }
    }

    scalar penetration_threshold;
    {
      const rapidxml::xml_attribute<>* const attrib_nd{ node.first_attribute( "penetration_threshold" ) };
      if( attrib_nd == nullptr )
      {
        std::cerr << "Could not locate penetration_threshold for sobogus_friction_solver with symplectic_euler" << std::endl;
        return false;
      }
      if( !StringUtilities::extractFromString( attrib_nd->value(), penetration_threshold ) )
      {
        std::cerr << "Could not load penetration_threshold value for sobogus_friction_solver with symplectic_euler, value of stabilization must be a non-negative scalar" << std::endl;
        return false;
      }
      if( penetration_threshold < 0.0 )
      {
        std::cerr << "Could not load penetration_threshold value for sobogus_friction_solver with symplectic_euler, value of stabilization must be a non-negative scalar" << std::endl;
        return false;
      }
    }

    if_map.reset( new SymplecticEulerImpactFrictionMap{ tol, static_cast<unsigned>( max_iters ), cache_impulses, enable_pose_stab, penetration_threshold } );
  }
  else if( staggering_type == "stabilized" )
  {
    if( cache_impulses != ImpulsesToCache::NONE )
    {
      std::cerr << "Error, constraint cache not currently supported for the stabilized impact friction map." << std::endl;
      std::exit( EXIT_FAILURE );
    }
    if_map.reset( new StabilizedImpactFrictionMap{ tol, static_cast<unsigned>( max_iters ), false, false } );
  }
  else
  {
    std::cerr << "Invalid staggering attribute specified for sobogus_friction_solver" << std::endl;
    return false;
  }

  friction_solver.reset( new Sobogus{ SobogusSolverType::Balls2D, static_cast<unsigned>( eval_every ) } );

  return true;
}

static bool loadGravityForce( const rapidxml::xml_node<>& node, std::vector<std::unique_ptr<Ball2DForce>>& forces )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "gravity" ); nd; nd = nd->next_sibling( "gravity" ) )
  {
    Vector2s f;

    const rapidxml::xml_attribute<>* x_attrib{ nd->first_attribute( "fx" ) };
    if( !x_attrib ) { return false; }
    StringUtilities::extractFromString( x_attrib->value(), f.x() );

    const rapidxml::xml_attribute<>* y_attrib{ nd->first_attribute( "fy" ) };
    if( !y_attrib ) { return false; }
    StringUtilities::extractFromString( y_attrib->value(), f.y() );

    forces.emplace_back( new Ball2DGravityForce{ f } );
  }

  return true;
}

static bool loadPenaltyForce( const rapidxml::xml_node<>& node, std::vector<std::unique_ptr<Ball2DForce>>& forces )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "penalty" ); nd; nd = nd->next_sibling( "penalty" ) )
  {
    scalar stiffness;
    {
      const rapidxml::xml_attribute<>* stiffness_attrib{ nd->first_attribute( "stiffness" ) };
      if( !stiffness_attrib )
      {
        std::cerr << "Failed to locate stiffness attribute for penalty." << std::endl;
        return false;
      }
      if( !StringUtilities::extractFromString( stiffness_attrib->value(), stiffness ) || stiffness < 0.0 )
      {
        std::cerr << "Failed to load stiffness attribute for penalty. Value must be a positive scalar." << std::endl;
        return false;
      }
    }

    scalar potential_power;
    {
      const rapidxml::xml_attribute<>* power_attrib{ nd->first_attribute( "potential_power" ) };
      if( !power_attrib )
      {
        std::cerr << "Failed to locate potential_power attribute for penalty." << std::endl;
        return false;
      }
      if( !StringUtilities::extractFromString( power_attrib->value(), potential_power ) )
      {
        std::cerr << "Failed to load potential_power attribute for penalty. Value must be a scalar." << std::endl;
        return false;
      }
    }

    forces.emplace_back( new PenaltyForce{ stiffness, potential_power } );
  }
  
  return true;
}

static bool loadSimulationState( const rapidxml::xml_node<>& root_node, const std::string& file_name, std::string& scripting_callback_name, Ball2DState& state, std::unique_ptr<UnconstrainedMap>& integrator,
                                 std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator,
                                 std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map )
{
  std::vector<Ball2D> balls;
  std::vector<StaticDrum> drums;
  std::vector<StaticPlane> planes;
  std::vector<PlanarPortal> planar_portals;
  std::vector<std::unique_ptr<Ball2DForce>> forces;

  // Attempt to determine if scirpting is enabled and if so, the coresponding callback
  if( !loadScriptingSetup( root_node, scripting_callback_name ) )
  {
    std::cerr << "Failed to parse scripting node in xml scene file: " << file_name << std::endl;
    return false;
  }

  // Attempt to load the end time, if present
  end_time = SCALAR_INFINITY;
  if( root_node.first_node( "end_time" ) != nullptr )
  {
    if( !loadEndTime( *root_node.first_node( "end_time" ), end_time ) )
    {
      std::cerr << "Failed to parse end_time node: " << file_name << std::endl;
      return false;
    }
  }

  // Attempt to load a gravity force
  if( !loadGravityForce( root_node, forces ) )
  {
    std::cerr << "Failed to load gravity force: " << file_name << std::endl;
    return false;
  }

  // Attempt to load a hertzian penalty force
  if( !loadPenaltyForce( root_node, forces ) )
  {
    std::cerr << "Failed to load penalty force: " << file_name << std::endl;
    return false;
  }

  // Attempt to load the unconstrained integrator
  if( !loadIntegrator( root_node, integrator, dt_string, dt ) )
  {
    std::cerr << "Failed to load integrator: " << file_name << std::endl;
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
      std::cerr << "Failed to load sobogus_friction_solver in xml scene file: " << file_name << std::endl;
      return false;
    }
  }

  // TODO: GRR friction solver goes here

  // Attempt to load any user-provided static drums
  if( !loadStaticDrums( root_node, drums ) )
  {
    std::cerr << "Failed to load static drums: " << file_name << std::endl;
    return false;
  }

  // Attempt to load any user-provided static drums
  if( !loadStaticPlanes( root_node, planes ) )
  {
    std::cerr << "Failed to load static planes: " << file_name << std::endl;
    return false;
  }

  // Attempt to load planar portals
  if( !loadPlanarPortals( root_node, planes, planar_portals ) )
  {
    std::cerr << "Failed to load planar and lees edwards portals: " << file_name << std::endl;
    return false;
  }

  // Attempt to load any user-provided balls
  if( !loadBalls( root_node, balls ) )
  {
    std::cerr << "Failed to load balls: " << file_name << std::endl;
    return false;
  }

  VectorXs q{ static_cast<VectorXs::Index>( 2 * balls.size() ) };
  VectorXs v{ static_cast<VectorXs::Index>( 2 * balls.size() ) };
  VectorXs m{ static_cast<VectorXs::Index>( 2 * balls.size() ) };
  VectorXs r{ static_cast<VectorXs::Index>(  balls.size() ) };
  std::vector<bool> fixed( balls.size() );
  for( std::vector<Ball2D>::size_type ball_idx = 0; ball_idx < balls.size(); ++ball_idx )
  {
    q.segment<2>( 2 * ball_idx ) = balls[ball_idx].x();
    v.segment<2>( 2 * ball_idx ) = balls[ball_idx].v();
    m.segment<2>( 2 * ball_idx ).setConstant( balls[ball_idx].m() );
    r( ball_idx ) = balls[ball_idx].r();
    fixed[ ball_idx ] = balls[ball_idx].fixed();
  }

  using std::swap;
  swap( q, state.q() );
  swap( v, state.v() );
  swap( r, state.r() );
  swap( fixed, state.fixed() );
  state.setMass( m );
  swap( drums, state.staticDrums() );
  swap( planes, state.staticPlanes() );
  swap( planar_portals, state.planarPortals() );
  swap( forces, state.forces() );

  return true;
}

bool Ball2DSceneParser::parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback_name, Ball2DState& state, std::unique_ptr<UnconstrainedMap>& integrator,
                                           std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator,
                                           std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map )
{
  // Attempt to load the xml document
  std::vector<char> xmlchars;
  rapidxml::xml_document<> doc;
  if( !loadXMLFile( file_name, xmlchars, doc ) )
  {
    return false;
  }

  // Attempt to locate the root node
  if( doc.first_node( "ball2d_scene" ) == nullptr )
  {
    std::cerr << "Failed to locate root node in xml scene file: " << file_name << std::endl;
    return false;
  }
  const rapidxml::xml_node<>& root_node{ *doc.first_node( "ball2d_scene" ) };

  // Attempt to load the state
  const bool loaded{ loadSimulationState( root_node, file_name, scripting_callback_name, state, integrator, dt_string, dt, end_time, impact_operator, impact_map, CoR, friction_solver, mu, if_map ) };
  if( !loaded )
  {
    return false;
  }

  return true;
}

bool Ball2DSceneParser::parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback_name, Ball2DState& state, std::unique_ptr<UnconstrainedMap>& integrator,
                                           std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator,
                                           std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map,
                                           RenderSettings& render_settings )
{
  // Attempt to load the xml document
  std::vector<char> xmlchars;
  rapidxml::xml_document<> doc;
  if( !loadXMLFile( file_name, xmlchars, doc ) )
  {
    return false;
  }

  // Attempt to locate the root node
  if( doc.first_node( "ball2d_scene" ) == nullptr )
  {
    std::cerr << "Failed to locate root node in xml scene file: " << file_name << std::endl;
    return false;
  }
  const rapidxml::xml_node<>& root_node{ *doc.first_node( "ball2d_scene" ) };

  // Attempt to load the optional camera settings
  render_settings.camera_set = false;
  if( root_node.first_node( "camera" ) != nullptr )
  {
    render_settings.camera_set = true;
    if( !loadCameraSettings( *root_node.first_node( "camera" ), render_settings.camera_center, render_settings.camera_scale_factor, render_settings.fps, render_settings.render_at_fps, render_settings.lock_camera ) )
    {
      std::cerr << "Failed to parse camera node: " << file_name << std::endl;
      return false;
    }
  }

  // Attempt to load the state
  const bool loaded{ loadSimulationState( root_node, file_name, scripting_callback_name, state, integrator, dt_string, dt, end_time, impact_operator, impact_map, CoR, friction_solver, mu, if_map ) };
  if( !loaded )
  {
    return false;
  }

  return true;
}
