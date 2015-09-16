// MomentTools.h
//
// Breannan Smith
// Last updated: 09/14/2015

#ifndef MOMENT_TOOLS_H
#define MOMENT_TOOLS_H

#include "scisim/Math/MathDefines.h"

namespace MomentTools
{

  // Implementation of Polyhedral Mass Properties (Revisited) by David Eberly
  // *** Caller must rescale mass and I by density ***
  void computeMoments( const Matrix3Xsc& vertices, const Matrix3Xuc& indices, scalar& mass, Vector3s& I, Vector3s& center, Matrix3s& R );

}

#endif
