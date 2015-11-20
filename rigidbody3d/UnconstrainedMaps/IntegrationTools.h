// IntegrationTools.h
//
// Breannan Smith
// Last updated: 11/15/2015

#ifndef INTEGRATION_TOOLS_H
#define INTEGRATION_TOOLS_H

#include "scisim/Math/MathDefines.h"

namespace IntegrationTools
{

  void exponentialEuler( const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 );

}

#endif
