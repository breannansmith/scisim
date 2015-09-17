// ObjParser.h
//
// Breannan Smith
// Last updated: 09/15/2015

// TODO:
// A. Load in with a half-edge data structure so I can:
//    1. Ensure that triangles are consistently oriented
//    2. Remove unused vertices
//    3. Reduce the bandwidth of the mesh
// B. More in depth error checking and reporting

#ifndef OBJ_PARSER_H
#define OBJ_PARSER_H

#include "scisim/Math/MathDefines.h"
#include <string>

namespace ObjParser
{

  bool parseObjFile( const std::string& filename, Matrix3Xsc& vertices, Matrix3Xuc& faces );

}

#endif
