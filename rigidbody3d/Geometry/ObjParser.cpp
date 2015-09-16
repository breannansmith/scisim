// ObjParser.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "ObjParser.h"

#include <fstream>
#include <iostream>

#include "SCISim/StringUtilities.h"

// TODO: Move error handling to exceptions
void parsevCommand( std::istringstream& commandstream, std::vector<Vector3s>& verts )
{
  Vector3s x;
  if( !( commandstream >> x.x() >> x.y() >> x.z() ) )
  {
    std::cerr << " Invalid 'v' definition in obj file. Vertices must have three real coordinates." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  verts.emplace_back( x );
}

// TODO: Move error handling to exceptions
void parsefCommand( std::istringstream& commandstream, std::vector<Vector3u>& face_verts, std::vector<Vector3u>& face_normals )
{
  Vector3u vertex_indices;
  std::vector<unsigned> normal_indices;

  std::string xstr;
  std::string ystr;
  std::string zstr;
  {
    std::vector<std::string> vertstrngs;
    std::string vertcmmnd;
    while( commandstream >> vertcmmnd )
    {
      vertstrngs.emplace_back( vertcmmnd );
    }

    if( vertstrngs.size() != 3 )
    {
      std::cerr << " Invalid face command in obj file." << std::endl;
      std::cerr << vertcmmnd << std::endl;
      std::exit( EXIT_FAILURE );
    }

    xstr = vertstrngs[0];
    ystr = vertstrngs[1];
    zstr = vertstrngs[2];
  }

  // Read the first vertex of this face
  std::vector<std::string> vertex_spec;
  StringUtilities::tokenize( xstr, '/', vertex_spec );
  if( vertex_spec.size() < 1 || vertex_spec.size() > 3 )
  {
    std::cerr << " Invalid face command in obj file." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // Attempt to extract the vertex index
  std::istringstream xstream{ vertex_spec[0] };
  if( !( xstream >> vertex_indices.x() ) )
  {
    std::cerr << " Invalid face command in obj file.";
    std::exit( EXIT_FAILURE );
  }
  // Attempt to extract the normal index
  if( vertex_spec.size() == 3 )
  {
    std::istringstream idxstm{ vertex_spec[2] };
    unsigned index;
    if( !( idxstm >> index ) )
    {
      std::cerr << " Invalid face command in obj file.";
      std::exit( EXIT_FAILURE );
    }
    normal_indices.emplace_back( index );
  }

  // Read the second vertex of this face
  vertex_spec.clear();
  StringUtilities::tokenize( ystr, '/', vertex_spec );
  if( vertex_spec.size() < 1 || vertex_spec.size() > 3 )
  {
    std::cerr << " Invalid face command in obj file.";
    std::exit( EXIT_FAILURE );
  }
  // Attempt to extract the vertex index
  std::istringstream ystream{ vertex_spec[0] };
  if( !( ystream >> vertex_indices.y() ) )
  {
    std::cerr << " Invalid face command in obj file.";
    std::exit( EXIT_FAILURE );
  }
  // Attempt to extract the normal index
  if( vertex_spec.size() == 3 )
  {
    std::istringstream idxstm{ vertex_spec[2] };
    unsigned index;
    if( !( idxstm >> index ) )
    {
      std::cerr << " Invalid face command in obj file.";
      std::exit( EXIT_FAILURE );
    }
    normal_indices.emplace_back( index );
  }

  // Read the third vertex of this face
  vertex_spec.clear();
  StringUtilities::tokenize( zstr, '/', vertex_spec );
  if( vertex_spec.size() < 1 || vertex_spec.size() > 3 )
  {
    std::cerr << " Invalid face command in obj file.";
    std::exit( EXIT_FAILURE );
  }
  // Attempt to extract the vertex index
  std::istringstream zstream{ vertex_spec[0] };
  if( !( zstream >> vertex_indices.z() ) )
  {
    std::cerr << " Invalid face command in obj file.";
    std::exit( EXIT_FAILURE );
  }
  // Attempt to extract the normal index
  if( vertex_spec.size() == 3 )
  {
    std::istringstream idxstm{ vertex_spec[2] };
    unsigned index = -1;
    if( !( idxstm >> index ) )
    {
      std::cerr << " Invalid face command in obj file.";
      std::exit( EXIT_FAILURE );
    }
    normal_indices.emplace_back( index );
  }

  face_verts.emplace_back( vertex_indices - Vector3u::Ones() );

  // If per-vertex normals are specified, they must be specified for EACH vertex of the face
  assert( normal_indices.size() == 0 || normal_indices.size() == 3 );
  if( !normal_indices.empty() )
  {
    face_normals.emplace_back( Vector3u{ normal_indices[0], normal_indices[1], normal_indices[2] } - Vector3u::Ones() );
  }
}

bool ObjParser::parseObjFile( const std::string& filename, Matrix3Xsc& vertices, Matrix3Xuc& faces )
{
  // Attempt to open the user specified file
  std::ifstream obj_file;
  obj_file.open( filename );
  if( !obj_file.is_open() )
  {
    std::cerr << "Failed to open obj file: " << filename << std::endl;
    return false;
  }

  // Don't know how many faces or vertices there are, so store in a vector
  std::vector<Vector3s> verts;
  std::vector<Vector3u> face_vertices;
  std::vector<Vector3u> face_normals;

  std::string obj_command;
  while( !obj_file.eof() )
  {
    // Read a single line at a time
    getline( obj_file, obj_command );

    // Use a string stream for easy tokenizing
    std::istringstream commandstream{ obj_command };

    // First element of a command is the command's name
    std::string command;
    commandstream >> command;

    // Vertex command
    if( command == "v" )
    {
      parsevCommand( commandstream, verts );
    }
    // Face command
    else if( command == "f" )
    {
      parsefCommand( commandstream, face_vertices, face_normals );
    }
  }
  obj_file.close();

  // Copy the temporary storage into final data structures
  vertices.resize( 3, verts.size() );
  for( std::vector<Vector3s>::size_type i = 0; i < verts.size(); ++i )
  {
    vertices.col( i ) = verts[i];
  }

  faces.resize( 3, face_vertices.size() );
  for( std::vector<Vector3u>::size_type i = 0; i < face_vertices.size(); ++i )
  {
    faces.col( i ) = face_vertices[i];
  }

  return true;
}
