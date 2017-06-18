#ifndef INTEGRATION_TOOLS_H
#define INTEGRATION_TOOLS_H

#include "scisim/Math/MathDefines.h"

namespace IntegrationTools
{

  void exponentialEuler( const VectorXs& q0, const VectorXs& v0, const std::vector<bool>& fixed, const scalar& dt, VectorXs& q1 );

}

#endif
