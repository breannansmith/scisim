// IntegrationUtils.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef INTEGRATION_UTILS_H
#define INTEGRATION_UTILS_H

#include "SCISim/Math/MathDefines.h"

namespace IntegrationUtils
{

  void exponentialEuler( const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 );

}

#endif
