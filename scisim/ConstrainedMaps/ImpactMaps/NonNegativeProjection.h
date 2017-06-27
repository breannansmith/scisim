#ifndef NON_NEGATIVE_PROJECTION_H
#define NON_NEGATIVE_PROJECTION_H

#include "scisim/Math/MathDefines.h"

struct NonNegativeProjection final
{
  void operator()( VectorXs& x ) const;
};

#endif
