// RigidBody3DSceneParser.cpp
//
// Breannan Smith
// Last updated: 10/23/2015

#include "RigidBody3DSceneParser.h"

#include <fstream>
#include <iostream>
#include <algorithm>

#include "scisim/StringUtilities.h"
#include "scisim/Math/Rational.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GaussSeidelOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/JacobiOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GROperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GRROperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorQL.h"
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorQLVP.h"
#include "scisim/ConstrainedMaps/GeometricImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/StabilizedImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/StaggeredProjections.h"
#include "scisim/ConstrainedMaps/Sobogus.h"
#include "scisim/ConstrainedMaps/GRRFriction.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"

#ifdef IPOPT_FOUND
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorIpopt.h"
#include "scisim/ConstrainedMaps/FrictionMaps/SmoothMDPOperatorIpopt.h"
#endif

#include "rigidbody3d/Geometry/RigidBodyGeometry.h"
#include "rigidbody3d/Geometry/RigidBodyBox.h"
#include "rigidbody3d/Geometry/RigidBodySphere.h"
#include "rigidbody3d/Geometry/RigidBodyStaple.h"
#include "rigidbody3d/Geometry/RigidBodyTriangleMesh.h"
#include "rigidbody3d/RigidBody3DState.h"
#include "rigidbody3d/Forces/NearEarthGravityForce.h"
#include "rigidbody3d/UnconstrainedMaps/SplitHamMap.h"
#include "rigidbody3d/UnconstrainedMaps/DMVMap.h"
#include "rigidbody3d/UnconstrainedMaps/ExponentialEulerMap.h"
#include "rigidbody3d/StaticGeometry/StaticPlane.h"
#include "rigidbody3d/StaticGeometry/StaticCylinder.h"
#include "rigidbody3d/Portals/PlanarPortal.h"

#include "RenderingState.h"
#include "rapidxml.hpp"

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
    std::copy( line.begin(), line.end(), back_inserter( xmlchars ) );
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
    doc.parse<0>( &xmlchars.front() );
  }
  catch( const rapidxml::parse_error& e )
  {
    std::cerr << "Failed to parse scene file: " << filename << std::endl;
    std::cerr << "Error message: " << e.what() << std::endl;
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

static bool loadSimState( const rapidxml::xml_node<>& node, RigidBody3DState& sim_state )
{
  std::vector<std::unique_ptr<RigidBodyGeometry>> geometry;
  for( rapidxml::xml_node<>* nd = node.first_node( "geometry" ); nd; nd = nd->next_sibling( "geometry" ) )
  {
    // Load the type of the geometry
    std::string geom_type;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "type" ) };
      if( !attrib )
      {
        std::cerr << "Invalid type" << std::endl;
        return false;
      }
      geom_type = attrib->value();
    }

    // Parse the remaining arguments based on the geometry type
    if( geom_type == "sphere" )
    {
      // Parse the radius
      scalar r;
      {
        const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "r" ) };
        if( !attrib )
        {
          std::cerr << "Failed to locate r attribute for sphere geometry." << std::endl;
          return false;
        }
        const bool parsed{ StringUtilities::extractFromString( attrib->value(), r ) };
        if( !parsed || r <= 0.0 )
        {
          std::cerr << "Failed to parse r attribute for sphere geometry, must provide a positive scalar." << std::endl;
          return false;
        }
      }
      geometry.emplace_back( new RigidBodySphere{ r } );
    }
    else if( geom_type == "box" )
    {
      // Read in the half-width along each dimension
      VectorXs r;
      {
        const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "r" ) };
        if( !attrib )
        {
          std::cerr << "Failed to locate r attribute for box geometry" << std::endl;
          return false;
        }
        if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', r ) )
        {
          std::cerr << "Failed to load r attribute for box geometry, must provide 3 positive scalars" << std::endl;
          return false;
        }
        assert( r.size() == 3 );
        if( ( r.array() <= 0.0 ).any() )
        {
          std::cerr << "Failed to load r attribute for box geometry, must provide 3 positive scalars" << std::endl;
          return false;
        }
        geometry.emplace_back( new RigidBodyBox{ r } );
      }
    }
    else if( geom_type == "staple" )
    {
      // Parse width w along x axis
      scalar w;
      {
        const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "w" ) };
        if( !attrib )
        {
          std::cerr << "Failed to locate the w attribute for staple geometry" << std::endl;
          return false;
        }
        const bool parsed{ StringUtilities::extractFromString( attrib->value(), w ) };
        if( !parsed || w <= 0.0 )
        {
          std::cerr << "Failed to load the w attribute for staple geometry, must provide a positive scalar" << std::endl;
          return false;
        }
      }
      // Parse the length l along y axis
      scalar l;
      {
        const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "l" ) };
        if( !attrib )
        {
          std::cerr << "Failed to locate the l attribute for staple geometry" << std::endl;
          return false;
        }
        const bool parsed{ StringUtilities::extractFromString( attrib->value(), l ) };
        if( !parsed || l <= 0.0 )
        {
          std::cerr << "Failed to load the l attribute for staple geometry, must provide a positive scalar" << std::endl;
          return false;
        }
      }
      // Parse the diameter D of staple
      scalar D;
      {
        const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "D" ) };
        if( !attrib )
        {
          std::cerr << "Failed to locate the D attribute for staple geometry" << std::endl;
          return false;
        }
        const bool parsed{ StringUtilities::extractFromString( attrib->value(), D ) };
        if( !parsed || D <= 0.0 )
        {
          std::cerr << "Failed to load the D attribute for staple geometry, must provide a positive scalar" << std::endl;
          return false;
        }
        if( D > w || D > l )
        {
          std::cerr << "Failed to load the D attribute for staple geometry, value must be less than w and l" << std::endl;
          return false;
        }
      }
      geometry.emplace_back( std::unique_ptr<RigidBodyGeometry>{ new RigidBodyStaple{ w, l, D } } );
    }
    else if( geom_type == "mesh" )
    {
      // Read the name of the obj file with the mesh
      std::string mesh_file_name;
      {
        const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "filename" ) };
        if( !attrib )
        {
          return false;
        }
        mesh_file_name = attrib->value();
      }
      try
      {
        geometry.emplace_back( new RigidBodyTriangleMesh{ mesh_file_name } );
      }
      catch( const std::string& error )
      {
        std::cerr << "Failed to load triangle mesh " << mesh_file_name << ": " << error << std::endl;
        return false;
      }
    }
    else
    {
      std::cerr << "Invalid geometry type: " << geom_type << std::endl;
      return false;
    }
  }

  std::vector<Vector3s> xs;
  std::vector<Vector3s> vs;
  std::vector<scalar> Ms;
  std::vector<VectorXs> Rs;
  std::vector<Vector3s> omegas;
  std::vector<Vector3s> I0s;
  std::vector<bool> fixeds;
  std::vector<unsigned> geometry_indices;

  for( rapidxml::xml_node<>* nd = node.first_node( "rigid_body_with_density" ); nd; nd = nd->next_sibling( "rigid_body_with_density" ) )
  {
    // Load the center of mass' position
    Vector3s x;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "x" ) };
      if( !attrib )
      {
        std::cerr << "Failed to load x" << std::endl;
        return false;
      }

      std::stringstream ss;
      ss << attrib->value();
      std::vector<scalar> vals;
      scalar input_scalar;
      while( ss >> input_scalar )
      {
        vals.emplace_back( input_scalar );
      }
      if( vals.size() != 3 )
      {
        std::cerr << "Failed to load x" << std::endl;
        return false;
      }

      x << vals[0], vals[1], vals[2];
    }
    // Load the center of mass' velocity
    Vector3s v;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "v" ) };
      if( !attrib )
      {
        std::cerr << "Failed to load v" << std::endl;
        return false;
      }

      std::stringstream ss;
      ss << attrib->value();
      std::vector<scalar> vals;
      scalar input_scalar;
      while( ss >> input_scalar )
      {
        vals.emplace_back( input_scalar );
      }
      if( vals.size() != 3 )
      {
        std::cerr << "Failed to load v" << std::endl;
        return false;
      }

      v << vals[0], vals[1], vals[2];
    }
    vs.emplace_back( v );
    // Load the angular velocity about the center of mass
    Vector3s omega;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "omega" ) };
      if( !attrib )
      {
        std::cerr << "Failed to load omega" << std::endl;
        return false;
      }

      std::stringstream ss;
      ss << attrib->value();
      std::vector<scalar> vals;
      scalar input_scalar;
      while( ss >> input_scalar )
      {
        vals.emplace_back( input_scalar );
      }
      if( vals.size() != 3 )
      {
        std::cerr << "Failed to load omega" << std::endl;
        return false;
      }

      omega << vals[0], vals[1], vals[2];
    }
    omegas.emplace_back( omega );
    // Load the density of the body
    scalar rho;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "rho" ) };
      if( !attrib )
      {
        std::cerr << "Failed to load rho" << std::endl;
        return false;
      }
      const bool parsed{ StringUtilities::extractFromString( attrib->value(), rho ) };
      if( !parsed )
      {
        std::cerr << "Failed to load rho" << std::endl;
        return false;
      }
    }
    // Load whether or not the body is fixed
    bool fixed;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "fixed" ) };
      if( !attrib )
      {
        std::cerr << "Failed to load fixed" << std::endl;
        return false;
      }
      const bool parsed{ StringUtilities::extractFromString( attrib->value(), fixed ) };
      if( !parsed )
      {
        std::cerr << "Failed to load fixed" << std::endl;
        return false;
      }
    }
    fixeds.push_back( fixed );
    // Load the index of this body's geometry
    int geometry_index;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "geo_idx" ) };
      if( !attrib )
      {
        std::cerr << "Failed to load geo_idx" << std::endl;
        return false;
      }
      const bool parsed{ StringUtilities::extractFromString( attrib->value(), geometry_index ) };
      if( !parsed )
      {
        std::cerr << "Failed to load geo_idx" << std::endl;
        return false;
      }
    }
    geometry_indices.emplace_back( geometry_index );

    if( geometry_indices.back() >= geometry.size() )
    {
      std::cerr << "Invalid geometry index specified: " << geometry_indices.back() << std::endl;
      return false;
    }
    // Load an optional orientation
    Matrix33sr R0;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "R" ) };
      if( !attrib )
      {
        R0.setIdentity();
      }
      else
      {
        VectorXs rotation_vector;
        if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', rotation_vector ) )
        {
          std::cerr << "Failed to load R attribute for rigid_body_with_density, must provide 3 positive scalars" << std::endl;
          return false;
        }
        assert( rotation_vector.size() == 3 );
        if( rotation_vector.norm() != 0.0 )
        {
          R0 = Eigen::AngleAxis<scalar>( rotation_vector.norm(), rotation_vector.normalized() ).matrix();
        }
        else
        {
          R0.setIdentity();
        }
      }
    }

    scalar M;
    Vector3s CM;
    Vector3s I;
    Matrix33sr R;
    geometry[geometry_indices.back()]->computeMassAndInertia( rho, M, CM, I, R );
    Ms.emplace_back( M );
    xs.emplace_back( x + CM );
    I0s.emplace_back( I );
    R = R0 * R;
    VectorXs Rvec{ 9 };
    Rvec = Eigen::Map<VectorXs>{ R.data(), 9, 1 };
    Rs.emplace_back( Rvec );
  }

  // TODO: Replace with a swap?
  sim_state.setState( xs, vs, Ms, Rs, omegas, I0s, fixeds, geometry_indices, geometry );

  return true;
}

static bool loadNearEarthGravityForce( const rapidxml::xml_node<>& node, RigidBody3DState& sim )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "near_earth_gravity" ); nd; nd = nd->next_sibling( "near_earth_gravity" ) )
  {
    const rapidxml::xml_attribute<>* attrib{ nd->first_attribute( "f" ) };
    if( !attrib )
    {
      return false;
    }
    VectorXs f;
    if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', f ) )
    {
      std::cerr << "Failed to load f attribute for near_earth_gravity, must provide 3 scalars." << std::endl;
      return false;
    }
    assert( f.size() == 3 );
    sim.addForce( NearEarthGravityForce{ f } );
  }

  return true;
}

static bool loadStaticPlanes( const rapidxml::xml_node<>& node, RigidBody3DState& sim )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "static_plane" ); nd; nd = nd->next_sibling( "static_plane" ) )
  {
    // Read a point on the plane
    VectorXs x;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "x" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate x attribute for static_plane." << std::endl;
        return false;
      }
      if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', x ) )
      {
        std::cerr << "Failed to load x attribute for static_plane, must provide 3 scalars." << std::endl;
        return false;
      }
      assert( x.size() == 3 );
    }
    // Read the normal
    VectorXs n;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "n" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate n attribute for static_plane." << std::endl;
        return false;
      }
      if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', n ) )
      {
        std::cerr << "Failed to load n attribute for static_plane, must provide 3 scalars." << std::endl;
        return false;
      }
      assert( x.size() == 3 );
    }
    // Create the plane
    sim.addStaticPlane( StaticPlane{ x, n } );
  }

  return true;
}

static bool loadPlanarPortals( const rapidxml::xml_node<>& node, RigidBody3DState& sim, RenderingState& rendering_state )
{
  if( !node.first_node( "planar_portal" ) )
  {
    return true;
  }

  // If we have a portal we must have at least one plane
  if( sim.numStaticPlanes() < 2 )
  {
    std::cerr << "Must provide at least two planes before instantiating a planar portal." << std::endl;
    return false;
  }

  // Pairs of planes to turn into portals
  std::vector< std::pair< unsigned, unsigned > > plane_pairs;
  // Multipliers to control where in the other plan to teleport to
  std::vector<Array3i> portal_multipliers;

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
    if( index_a >= sim.numStaticPlanes() )
    {
      std::cerr << "Failed to parse planeA attribute for planar_portal node with value " << attrib_a->value() << ". Attribute must be an index of a plane between " << 0 << " and " << sim.numStaticPlanes() - 1 << std::endl;
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
    if( index_b >= sim.numStaticPlanes() )
    {
      std::cerr << "Failed to parse planeB attribute for planar_portal node with value " << attrib_b->value() << ". Attribute must be an index of a plane between " << 0 << " and " << sim.numStaticPlanes() - 1 << std::endl;
      return false;
    }

    // Indices for this portal can't repeat
    if( index_a == index_b )
    {
      std::cerr << "Failed to parse planeB attribute for planar_portal node with value " << attrib_b->value() << ". Value is a repeat of attribute planeA." << std::endl;
      return false;
    }

    // Indices can not repeat previous portals
    for( std::vector< std::pair< unsigned, unsigned > >::size_type i = 0; i < plane_pairs.size(); ++i )
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

    // Load the portal multipliers
    const rapidxml::xml_attribute<>* const attrib_multiplier{ nd->first_attribute( "multiplier" ) };
    if( !attrib_multiplier )
    {
      std::cerr << "Failed to locate multiplier attribute for planar_portal node." << std::endl;
      return false;
    }
    Array3i current_multiplier{ Array3i::Ones() };
    if( !StringUtilities::readScalarList( attrib_multiplier->value(), 3, ' ', current_multiplier ) )
    {
      std::cerr << "Failed to load multiplier attribute for planar_portal, must provide 3 numbers equal to +/- 1." << std::endl;
      return false;
    }
    if( ( current_multiplier < -1 ).any() || ( current_multiplier == 0 ).any() || ( current_multiplier > 1 ).any() )
    {
      std::cerr << "Failed to load multiplier attribute for planar_portal, must provide 3 numbers equal to +/- 1." << std::endl;
      return false;
    }
    portal_multipliers.emplace_back( current_multiplier );
  }

  for( std::vector< std::pair< unsigned, unsigned > >::size_type i = 0; i < plane_pairs.size(); ++i )
  {
    sim.addPlanarPortal( PlanarPortal( sim.staticPlane( plane_pairs[i].first ), sim.staticPlane( plane_pairs[i].second ), portal_multipliers[i] ) );
    Array2s half_width0{ Array2s::Zero() };
    Array2s half_width1{ Array2s::Zero() };
    for( unsigned j = 0; j <  rendering_state.numPlaneRenderers(); ++j )
    {
      if( rendering_state.planeRenderer( j ).index() == plane_pairs[i].first )
      {
        half_width0 = rendering_state.planeRenderer( j ).r();
      }
      else if( rendering_state.planeRenderer( j ).index() == plane_pairs[i].second )
      {
        half_width1 = rendering_state.planeRenderer( j ).r();
      }
    }
    if( ( half_width0 != 0.0 ).any() || ( half_width1 != 0.0 ).any() )
    {
      rendering_state.addPortalRenderer( unsigned( i ), half_width0, half_width1 );
    }
  }

  // TODO: This could get slow if there are a ton of portals, but probably not too big of a deal for now
  {
    std::vector<unsigned> indices;
    for( std::vector< std::pair< unsigned, unsigned > >::size_type i = 0; i < plane_pairs.size(); ++i )
    {
      indices.emplace_back( plane_pairs[i].first );
      indices.emplace_back( plane_pairs[i].second );
    }
    std::sort( indices.begin(), indices.end() );
    // Remove planes that we added to planar portals
    for( std::vector<unsigned>::size_type i = indices.size(); i-- > 0; )
    {
      sim.deleteStaticPlane( indices[i] );
      for( std::vector<PlaneRendererState>::size_type j = rendering_state.numPlaneRenderers(); j-- > 0; )
      {
        if( rendering_state.planeRenderer( j ).index() == indices[i] )
        {
          rendering_state.deleteStaticPlaneRenderer( j );
        }
      }
    }
    // Renumber the planes in plane renderers
    // Copy render indices into a temporary vector
    std::vector<unsigned> renderer_indices( rendering_state.numPlaneRenderers() );
    for( unsigned i = 0; i <  rendering_state.numPlaneRenderers(); ++i )
    {
      renderer_indices[i] = rendering_state.planeRenderer( i ).index();
    }
    assert( std::is_sorted( std::begin( renderer_indices ), std::end( renderer_indices ) ) );
    std::vector<unsigned> new_renderer_indices{ renderer_indices };
    // For each plane that we removed
    for( std::vector<unsigned>::size_type i = indices.size(); i-- > 0; )
    {
      // Find the first index larger than the removed plane
      std::vector<unsigned>::size_type j;
      for( j = 0; j < renderer_indices.size(); ++j )
      {
        if( renderer_indices[j] > indices[i] )
        {
          break;
        }
      }
      for( std::vector<unsigned>::size_type k = j; k < renderer_indices.size(); ++k )
      {
        new_renderer_indices[k]--;
      }
    }
    for( std::vector<unsigned>::size_type i = 0; i < new_renderer_indices.size(); ++i )
    {
      rendering_state.planeRenderer( i ).setIndex( new_renderer_indices[i] );
    }
  }

  #ifndef NDEBUG
  // Verify that the renderers have valid plane indices
  for( unsigned i = 0; i < rendering_state.numPlaneRenderers(); ++i )
  {
    assert( rendering_state.planeRenderer( i ).index() < sim.numStaticPlanes() );
  }
  #endif

  return true;
}

static bool loadStaticCylinders( const rapidxml::xml_node<>& node, RigidBody3DState& sim )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "static_cylinder" ); nd; nd = nd->next_sibling( "static_cylinder" ) )
  {
    // Read the center of the cylinder
    VectorXs x;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "x" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate x attribute for static_cylinder." << std::endl;
        return false;
      }
      if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', x ) )
      {
        std::cerr << "Failed to load x attribute for static_cylinder, must provide 3 scalars." << std::endl;
        return false;
      }
      assert( x.size() == 3 );
    }
    // Read the axis of the cylinder
    VectorXs axis;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "axis" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate axis attribute for static_cylinder." << std::endl;
        return false;
      }
      if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', axis ) )
      {
        std::cerr << "Failed to load axis attribute for static_cylinder. Must provide 3 scalars." << std::endl;
        return false;
      }
      assert( axis.size() == 3 );
    }
    // Read the radius of the cylinder
    scalar R;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "R" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate R attribute for static_cylinder." << std::endl;
        return false;
      }
      if( !StringUtilities::extractFromString( attrib->value(), R ) || R <= 0.0 )
      {
        std::cerr << "Failed to load attribute R for static_cylinder. Must provide a positive scalar." << std::endl;
        return false;
      }
    }
    // Create the cylinder
    sim.addStaticCylinder( StaticCylinder{ x, axis, R } );
  }

  return true;
}

static bool loadStaticPlaneRenderers( const rapidxml::xml_node<>& node, const std::vector<StaticPlane>& planes, RenderingState& rendering_state )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "static_plane_renderer" ); nd; nd = nd->next_sibling( "static_plane_renderer" ) )
  {
    // Read the index of plane to render
    std::vector<StaticPlane>::size_type idx;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "plane" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate plane attribute for static_plane_renderer." << std::endl;
        return false;
      }
      std::stringstream ss;
      ss << attrib->value();
      if( !( ss >> idx ) )
      {
        std::cerr << "Failed to load plane attribute for static_plane_renderer. Must provide a non-negative integer." << std::endl;
        return false;
      }
      if( idx > planes.size() )
      {
        std::cerr << "Failed to load plane attribute for static_plane_renderer. Invalid plane index specified." << std::endl;
        return false;
      }
    }
    // Read the half-width of the plane when rendered
    VectorXs r;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "r" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate r attribute for static_plane_renderer." << std::endl;
        return false;
      }
      if( !StringUtilities::readScalarList( attrib->value(), 2, ' ', r ) || !( r.array() >= 0.0 ).all() )
      {
        std::cerr << "Failed to load r attribute for static_plane_renderer. Must provide 2 positive scalars." << std::endl;
        return false;
      }
    }
    // Create the renderer
    rendering_state.addPlaneRenderer( unsigned( idx ), r );
  }

  rendering_state.sortPlaneRenderers();

  return true;
}

static bool loadStaticCylinderRenderers( const rapidxml::xml_node<>& node, const std::vector<StaticCylinder>& cylinders, RenderingState& rendering_state )
{
  for( rapidxml::xml_node<>* nd = node.first_node( "static_cylinder_renderer" ); nd; nd = nd->next_sibling( "static_cylinder_renderer" ) )
  {
    // Read the index of cylinder to render
    std::vector<StaticCylinder>::size_type idx;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "cylinder_idx" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate cylinder_idx attribute for static_cylinder_renderer." << std::endl;
        return false;
      }
      std::stringstream ss;
      ss << attrib->value();
      if( !( ss >> idx ) )
      {
        std::cerr << "Failed to load cylinder_idx attribute for static_cylinder_renderer. Must provide a non-negative integer." << std::endl;
        return false;
      }
      if( idx > cylinders.size() )
      {
        std::cerr << "Failed to load cylinder_idx attribute for static_cylinder_renderer. Invalid cylinder index specified." << std::endl;
        return false;
      }
    }
    // Read Length of the cylinder when rendered
    scalar L;
    {
      const rapidxml::xml_attribute<>* const attrib{ nd->first_attribute( "L" ) };
      if( !attrib )
      {
        std::cerr << "Failed to locate L attribute for static_cylinder_renderer." << std::endl;
        return false;
      }
      if( !StringUtilities::extractFromString( attrib->value(), L ) || L < 0.0 )
      {
        std::cerr << "Failed to load L attribute for static_cylinder_renderer. Must provide a positive scalar." << std::endl;
        return false;
      }
    }
    // Create the renderer
    rendering_state.addCylinderRenderer( unsigned( idx ), L );
  }
  return true;
}

static bool loadPerspectiveCameraSettings( const rapidxml::xml_node<>& root_node, RenderingState& rendering_state )
{
  const rapidxml::xml_node<>* node{ root_node.first_node( "camera_perspective" ) };
  if( node == nullptr )
  {
    return false;
  }

  // Attempt to load position on unit sphere
  scalar theta;
  {
    const rapidxml::xml_attribute<>* attrib{ node->first_attribute( "theta" ) };
    if( attrib == nullptr )
    {
      std::cerr << "Failed to locate theta attribute for camera_perspective node." << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( attrib->value(), theta ) )
    {
      std::cerr << "Failed to load theta attribute for camera_perspective node. Must provide a scalar." << std::endl;
      return false;
    }
  }
  scalar phi;
  {
    const rapidxml::xml_attribute<>* attrib{ node->first_attribute( "phi" ) };
    if( attrib == nullptr )
    {
      std::cerr << "Failed to locate phi attribute for camera_perspective node." << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( attrib->value(), phi ) )
    {
      std::cerr << "Failed to load phie attribute for camera_perspective node. Must provide a scalar." << std::endl;
      return false;
    }
  }

  // Attempt to load the radius of the sphere
  scalar rho;
  {
    const rapidxml::xml_attribute<>* attrib{ node->first_attribute( "rho" ) };
    if( attrib == nullptr )
    {
      std::cerr << "Failed to locate rho attribute for camera_perspective node." << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( attrib->value(), rho ) || rho <= 0.0 )
    {
      std::cerr << "Failed to load rho attribute for camera_perspective node. Must provide a positive scalar." << std::endl;
      return false;
    }
  }

  // Attempt to load the center of the sphere
  VectorXs lookat;
  {
    const rapidxml::xml_attribute<>* attrib{ node->first_attribute( "lookat" ) };
    if( attrib == nullptr )
    {
      std::cerr << "Failed to locate lookat attribute for camera_perspective node." << std::endl;
      return false;
    }
    if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', lookat ) )
    {
      std::cerr << "Failed to load lookat attribute for camera_perspective. Must provide 3 scalars." << std::endl;
      return false;
    }
    assert( lookat.size() == 3 );
  }

  // Load the up vector
  VectorXs up;
  {
    const rapidxml::xml_attribute<>* attrib{ node->first_attribute( "up" ) };
    if( attrib == nullptr )
    {
      std::cerr << "Failed to locate up attribute for camera_perspective node." << std::endl;
      return false;
    }
    if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', up ) )
    {
      std::cerr << "Failed to load up attribute for camera_perspective. Must provide 3 scalars." << std::endl;
      return false;
    }
    assert( up.size() == 3 );
  }

  // Attempt to parse the fps setting
  {
    const rapidxml::xml_attribute<>* fps_attrib{ node->first_attribute( "fps" ) };
    if( fps_attrib == nullptr )
    {
      std::cerr << "Failed to locate fps attribute for camera" << std::endl;
      return false;
    }
    unsigned fps;
    if( !StringUtilities::extractFromString( fps_attrib->value(), fps ) )
    {
      std::cerr << "Failed to parse fps attribute for camera. Value must be a non-negative integer." << std::endl;
      return false;
    }
    rendering_state.setFPS( fps );
  }

  // Attempt to parse the render_at_fps setting
  {
    const rapidxml::xml_attribute<>* render_at_fps_attrib{ node->first_attribute( "render_at_fps" ) };
    if( render_at_fps_attrib == nullptr )
    {
      std::cerr << "Failed to locate render_at_fps attribute for camera" << std::endl;
      return false;
    }
    bool render_at_fps;
    if( !StringUtilities::extractFromString( render_at_fps_attrib->value(), render_at_fps ) )
    {
      std::cerr << "Failed to parse render_at_fps attribute for camera. Value must be a boolean." << std::endl;
      return false;
    }
    rendering_state.setRenderAtFPS( render_at_fps );
  }

  // Attempt to parse the locked setting
  {
    const rapidxml::xml_attribute<>* locked_attrib{ node->first_attribute( "locked" ) };
    if( locked_attrib == nullptr )
    {
      std::cerr << "Failed to locate locked attribute for camera" << std::endl;
      return false;
    }
    bool lock_camera;
    if( !StringUtilities::extractFromString( locked_attrib->value(), lock_camera ) )
    {
      std::cerr << "Failed to parse locked attribute for camera. Value must be a boolean." << std::endl;
      return false;
    }
    rendering_state.setLocked( lock_camera );
  }

  rendering_state.setPerspectvieCameraSettings( theta, phi, rho, lookat, up );

  return true;
}

static bool loadOrthographicCameraSettings( const rapidxml::xml_node<>& root_node, RenderingState& rendering_state )
{
  const rapidxml::xml_node<>* node{ root_node.first_node( "camera_orthographic" ) };
  if( node == nullptr )
  {
    return false;
  }

  // Attempt to load the desired projection plane
  std::string projection_plane;
  {
    const rapidxml::xml_attribute<>* attrib{ node->first_attribute( "projection_plane" ) };
    if( attrib == nullptr )
    {
      std::cerr << "Failed to locate projection_plane attribute for camera_orthographic node." << std::endl;
      return false;
    }
    projection_plane = attrib->value();
    if( projection_plane != "xy" && projection_plane != "zy" && projection_plane != "zx" )
    {
      std::cerr << "Failed to load projection_plane attribute for camera_orthographic node. Value must by xy, zy, or zx." << std::endl;
      return false;
    }
  }

  // Attempt to load position in the projection plane
  VectorXs center;
  {
    const rapidxml::xml_attribute<>* attrib{ node->first_attribute( "x" ) };
    if( attrib == nullptr )
    {
      std::cerr << "Failed to locate x attribute for camera_orthographic node." << std::endl;
      return false;
    }
    if( !StringUtilities::readScalarList( attrib->value(), 3, ' ', center ) )
    {
      std::cerr << "Failed to load x attribute for camera_orthographic node. Must provide 3 scalars." << std::endl;
      return false;
    }
    assert( center.size() == 3 );
  }

  // Attempt to load the scale factor
  scalar scale;
  {
    const rapidxml::xml_attribute<>* attrib{ node->first_attribute( "scale" ) };
    if( attrib == nullptr )
    {
      std::cerr << "Failed to locate scale attribute for camera_orthographic node." << std::endl;
      return false;
    }
    if( !StringUtilities::extractFromString( attrib->value(), scale ) || scale <= 0.0 )
    {
      std::cerr << "Failed to load scale attribute for camera_orthographic node. Must provide a positive scalar." << std::endl;
      return false;
    }
  }

  // Attempt to parse the fps setting
  {
    const rapidxml::xml_attribute<>* fps_attrib{ node->first_attribute( "fps" ) };
    if( fps_attrib == nullptr )
    {
      std::cerr << "Failed to locate fps attribute for camera" << std::endl;
      return false;
    }
    unsigned fps;
    if( !StringUtilities::extractFromString( fps_attrib->value(), fps ) )
    {
      std::cerr << "Failed to parse fps attribute for camera. Value must be a non-negative integer." << std::endl;
      return false;
    }
    rendering_state.setFPS( fps );
  }

  // Attempt to parse the render_at_fps setting
  {
    const rapidxml::xml_attribute<>* render_at_fps_attrib{ node->first_attribute( "render_at_fps" ) };
    if( render_at_fps_attrib == nullptr )
    {
      std::cerr << "Failed to locate render_at_fps attribute for camera" << std::endl;
      return false;
    }
    bool render_at_fps;
    if( !StringUtilities::extractFromString( render_at_fps_attrib->value(), render_at_fps ) )
    {
      std::cerr << "Failed to parse render_at_fps attribute for camera. Value must be a boolean." << std::endl;
      return false;
    }
    rendering_state.setRenderAtFPS( render_at_fps );
  }

  // Attempt to parse the locked setting
  {
    const rapidxml::xml_attribute<>* locked_attrib{ node->first_attribute( "locked" ) };
    if( locked_attrib == nullptr )
    {
      std::cerr << "Failed to locate locked attribute for camera" << std::endl;
      return false;
    }
    bool lock_camera;
    if( !StringUtilities::extractFromString( locked_attrib->value(), lock_camera ) )
    {
      std::cerr << "Failed to parse locked attribute for camera. Value must be a boolean." << std::endl;
      return false;
    }
    rendering_state.setLocked( lock_camera );
  }

  rendering_state.setOrthographicCameraSettings( projection_plane, center, scale );

  return true;
}

static bool loadUnconstrainedMap( const rapidxml::xml_node<>& node, std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::string& dt_string, Rational<std::intmax_t>& dt )
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
    if( !extractFromString( std::string{ dtnd->value() }, dt ) || !dt.positive() )
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
    if( integrator_type == "split_ham" )
    {
      unconstrained_map.reset( new SplitHamMap );
    }
    else if( integrator_type == "dmv" )
    {
      unconstrained_map.reset( new DMVMap );
    }
    else if( integrator_type == "exponential_euler" )
    {
      unconstrained_map.reset( new ExponentialEulerMap );
    }
    else
    {
      std::cerr << "Invalid integrator 'type' attribute specified for integrator node. Options are: split_ham, dmv, exact, exponential_euler." << std::endl;
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

  const std::string solver_name{ nd->value() };

  if( solver_name == "ql" )
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
  #endif
  else
  {
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

  scalar v_tol{ std::numeric_limits<scalar>::signaling_NaN() };
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

static bool loadImpactOperator( const rapidxml::xml_node<>& node, std::unique_ptr<ImpactOperator>& impact_operator, scalar& CoR )
{
  // Attempt to load the CoR
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

  return loadImpactOperatorNoCoR( node, impact_operator );
}

static bool loadSmoothFrictionOperatorNoMu( const rapidxml::xml_node<>& node, std::unique_ptr<FrictionOperator>& friction_operator )
{
  // Attempt to load the solver name
  std::string solver_name;
  {
    rapidxml::xml_attribute<>* attrib_nd{ node.first_attribute( "name" ) };
    if( attrib_nd == nullptr )
    {
      std::cerr << "Could not locate name for solver of friction_operator" << std::endl;
      return false;
    }
    solver_name = attrib_nd->value();
  }

  #ifdef IPOPT_FOUND
  if( solver_name == "ipopt" )
  {
    // Attempt to read the desired linear solvers
    std::vector<std::string> linear_solvers;
    {
      rapidxml::xml_attribute<>* attrib{ node.first_attribute( "linear_solvers" ) };
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
    scalar con_tol{ std::numeric_limits<scalar>::signaling_NaN() };
    {
      rapidxml::xml_attribute<>* attrib = node.first_attribute( "tol" );
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

      if( con_tol < 0.0 )
      {
        std::cerr << "Could not load tol for ipopt solver, value must be positive scalar." << std::endl;
        return false;
      }
    }

    friction_operator.reset( new SmoothMDPOperatorIpopt{ linear_solvers, con_tol } );
  }
  else
  {
    std::cerr << "Error, invalid smooth friction solver: " << solver_name << std::endl;
    return false;
  }

  return true;
  #else
  std::cerr << "Error, invalid smooth friction solver: " << solver_name << std::endl;
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
      rapidxml::xml_attribute<>* attrib_nd{ node.first_attribute( "CoR" ) };
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
        std::cerr << "Failed to load LCP solver for staggered_projections_friction_solver" << std::endl;
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
      if( !loadSmoothFrictionOperatorNoMu( *friction_operator_node, friction_operator ) )
      {
        std::cerr << "Failed to load friction solver for staggered_projections_friction_solver" << std::endl;
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
      const rapidxml::xml_attribute<>* const attrib_nd = node.first_attribute( "staggering" );
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
      const rapidxml::xml_attribute<>* const attrib_nd = node.first_attribute( "tol" );
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
      const rapidxml::xml_attribute<>* const attrib_nd = node.first_attribute( "max_iters" );
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
    if_map.reset( new GeometricImpactFrictionMap{ tol, static_cast<unsigned>( max_iters ), ImpulsesToCache::NONE } );
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

  friction_solver.reset( new Sobogus{ SobogusSolverType::RigidBodies3D, static_cast<unsigned>( eval_every ) } );
  
  return true;
}

// Example:
//  <grr_friction_solver mu="2.0" CoR="0.8" staggering="geometric">
//    <impact_operator type="grr">
//      <elastic_operator type="gr" v_tol="1.0e-6">
//        <solver name="ipopt" tol="1.0e-12" linear_solvers="ma97"/>
//      </elastic_operator>
//      <inelastic_operator type="lcp">
//        <solver name="ipopt" tol="1.0e-12" linear_solvers="ma97"/>
//      </inelastic_operator>
//    </impact_operator>
//    <friction_operator type="smooth">
//      <solver name="ipopt" tol="1.0e-12" linear_solvers="ma97"/>
//    </friction_operator>
//  </grr_friction_solver>
static bool loadGRRFrictionSolver( const rapidxml::xml_node<>& node, scalar& mu, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, std::unique_ptr<ImpactFrictionMap>& if_map )
{
  // Friction solver setup
  {
    // Attempt to load the coefficient of friction
    {
      if( node.first_attribute( "mu" ) == nullptr )
      {
        std::cerr << "Could not locate mu for grr_friction_solver" << std::endl;
        return false;
      }
      const rapidxml::xml_attribute<>& attrib_nd{ *node.first_attribute( "mu" ) };

      if( !StringUtilities::extractFromString( attrib_nd.value(), mu ) )
      {
        std::cerr << "Could not load mu value for grr_friction_solver" << std::endl;
        return false;
      }

      if( mu < 0.0 )
      {
        std::cerr << "Could not load mu value for grr_friction_solver, value of mu must be a nonnegative scalar" << std::endl;
        return false;
      }
    }

    // Attempt to load the coefficient of restitution
    {
      if( node.first_attribute( "CoR" ) == nullptr )
      {
        std::cerr << "Could not locate CoR for grr_friction_solver" << std::endl;
        return false;
      }
      const rapidxml::xml_attribute<>& attrib_nd{ *node.first_attribute( "CoR" ) };

      if( !StringUtilities::extractFromString( attrib_nd.value(), CoR ) )
      {
        std::cerr << "Could not load CoR value for grr_friction_solver" << std::endl;
        return false;
      }

      if( CoR < 0.0 || CoR > 1.0 )
      {
        std::cerr << "Could not load CoR value for grr_friction_solver, value of CoR must be a nonnegative scalar" << std::endl;
        return false;
      }
    }

    // Attempt to load the impact operator
    std::unique_ptr<ImpactOperator> impact_operator;
    {
      if( node.first_node( "impact_operator" ) == nullptr )
      {
        std::cerr << "Could not locate impact_operator node for grr_friction_solver" << std::endl;
        return false;
      }
      const rapidxml::xml_node<>& impact_operator_node{ *node.first_node( "impact_operator" ) };
      if( !loadImpactOperatorNoCoR( impact_operator_node, impact_operator ) )
      {
        return false;
      }
    }

    // Attempt to load the friction operator
    std::unique_ptr<FrictionOperator> friction_operator;
    {
      if( node.first_node( "friction_operator" ) == nullptr )
      {
        std::cerr << "Could not locate friction_operator node for grr_friction_solver" << std::endl;
        return false;
      }
      const rapidxml::xml_node<>& friction_operator_node{ *node.first_node( "friction_operator" ) };
      if( !loadSmoothFrictionOperatorNoMu( friction_operator_node, friction_operator ) )
      {
        return false;
      }
    }

    friction_solver.reset( new GRRFriction{ *impact_operator, *friction_operator } );
  }

  // Impact-friction map setup
  // TODO: setting up if_map can be done in a separate function
  {
    // Attempt to load the staggering type
    std::string staggering_type;
    {
      if( node.first_attribute( "staggering" ) == nullptr )
      {
        std::cerr << "Could not locate staggering attribute for staggered_projections_friction_solver" << std::endl;
        return false;
      }
      const rapidxml::xml_attribute<>& attrib_nd{ *node.first_attribute( "staggering" ) };
      staggering_type = attrib_nd.value();
    }

    // TODO: Tolerance and max iters not used by this solver... pass tol and max_iters to the friction solver, not to the impact-friction-map
    if( staggering_type == "geometric" )
    {
      // 0.0, 0 are dummy parameters, for now
      if_map.reset( new GeometricImpactFrictionMap{ 0.0, 0, ImpulsesToCache::NONE } );
    }
    else if( staggering_type == "stabilized" )
    {
      // 0.0, 0 are dummy parameters, for now
      if_map.reset( new StabilizedImpactFrictionMap{ 0.0, 0, false, false } );
    }
    else
    {
      std::cerr << "Invalid staggering attribute specified for grr_friction_solver, options are: ";
      std::cerr << "geometric, stabilized" << std::endl;
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

// TODO: Do some kind of boost-optional thing to grab const refs to nodes, but not have to do first_node twice

bool RigidBody3DSceneParser::parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback, RigidBody3DState& sim_state, std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map, RenderingState& rendering_state )
{
  // Attempt to load the xml document
  std::vector<char> xmlchars;
  rapidxml::xml_document<> doc;
  if( !loadXMLFile( file_name, xmlchars, doc ) )
  {
    return false;
  }

  // Attempt to locate the root node
  if( doc.first_node( "rigidbody3d_scene" ) == nullptr )
  {
    std::cerr << "Failed to locate root node in xml scene file: " << file_name << std::endl;
    return false;
  }
  const rapidxml::xml_node<>& root_node{ *doc.first_node( "rigidbody3d_scene" ) };

  // Attempt to determine if scirpting is enabled and if so, the coresponding callback
  if( !loadScriptingSetup( root_node, scripting_callback ) )
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

  // Attempt to load forces
  if( !loadNearEarthGravityForce( root_node, sim_state ) )
  {
    std::cerr << "Failed to load near_earth_gravity_force in xml scene file: " << file_name << std::endl;
    return false;
  }

  // Attempt to load perspective camera settings
  if( root_node.first_node( "camera_perspective" ) != nullptr )
  {
    if( !loadPerspectiveCameraSettings( root_node, rendering_state ) )
    {
      std::cerr << "Failed to load persepctive camera settings in xml scene file: " << file_name << std::endl;
      return false;
    }
  }

  // Attempt to load orthographic camera settings
  if( root_node.first_node( "camera_orthographic" ) != nullptr )
  {
    if( !loadOrthographicCameraSettings( root_node, rendering_state ) )
    {
      std::cerr << "Failed to load orthographic camera settings in xml scene file: " << file_name << std::endl;
      return false;
    }
  }

  // Attempt to load the unconstrained integrator
  if( !loadUnconstrainedMap( root_node, unconstrained_map, dt_string, dt ) )
  {
    std::cerr << "Failed to load unconstrained map in xml scene file: " << file_name << std::endl;
    return false;
  }

  // Attempt to load an impact operator
  if( root_node.first_node( "impact_operator" ) != nullptr )
  {
    if( !loadImpactOperator( *root_node.first_node( "impact_operator" ), impact_operator, CoR ) )
    {
      std::cerr << "Failed to load impact_operator in xml scene file: " << file_name << std::endl;
      return false;
    }
  }
  else
  {
    impact_operator.reset( nullptr );
    CoR = SCALAR_NAN;
  }

  friction_solver.reset( nullptr );
  mu = SCALAR_NAN;
  if_map.reset( nullptr );

  // Load a staggered projections friction solver, if present
  if( root_node.first_node( "staggered_projections_friction_solver" ) != nullptr )
  {
    if( impact_operator != nullptr )
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
    if( impact_operator != nullptr )
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

  // Load a GRR friction solver, if present
  if( root_node.first_node( "grr_friction_solver" ) != nullptr )
  {
    if( impact_operator != nullptr )
    {
      std::cerr << "Error loading grr_friction_solver, solver of type " << impact_operator->name() << " already specified" << std::endl;
      return false;
    }
    if( friction_solver != nullptr )
    {
      std::cerr << "Error loading grr_friction_solver, solver of type " << friction_solver->name() << " already specified" << std::endl;
      return false;
    }
    if( !loadGRRFrictionSolver( *root_node.first_node( "grr_friction_solver" ), mu, CoR, friction_solver, if_map ) )
    {
      std::cerr << "Failed to load grr_friction_solver in xml scene file: " << file_name << std::endl;
      return false;
    }
  }

  // Attempt to load static planes
  if( !loadStaticPlanes( root_node, sim_state ) )
  {
    std::cerr << "Failed to load static_plane in xml scene file: " << file_name << std::endl;
    return false;
  }

  // Attempt to load static cylinders
  if( !loadStaticCylinders( root_node, sim_state ) )
  {
    std::cerr << "Failed to load static_cylinder in xml scene file: " << file_name << std::endl;
    return false;
  }

  // Attempt to load static plane renderers
  if( !loadStaticPlaneRenderers( root_node, sim_state.staticPlanes(), rendering_state ) )
  {
    std::cerr << "Failed to load static_plane_renderer in xml scene file: " << file_name << std::endl;
    return false;
  }

  // Attempt to load static cylinder renderers
  if( !loadStaticCylinderRenderers( root_node, sim_state.staticCylinders(), rendering_state ) )
  {
    std::cerr << "Failed to load static_cylinder_renderer in xml scene file: " << file_name << std::endl;
    return false;
  }

  // Attempt to load planar portals
  if( !loadPlanarPortals( root_node, sim_state, rendering_state ) )
  {
    std::cerr << "Failed to load planar portals: " << file_name << std::endl;
    return false;
  }

  // Attempt to load all rigid bodies
  if( !loadSimState( root_node, sim_state ) )
  {
    std::cerr << "Failed to load rigid bodies in xml scene file: " << file_name << std::endl;
    return false;
  }

  return true;
}
