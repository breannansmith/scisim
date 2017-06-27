#ifndef MIN_MAP_IMPACT_H
#define MIN_MAP_IMPACT_H

#include "scisim/Math/MathDefines.h"

struct MinMapImpact final
{

  scalar operator()( const VectorXs& alpha, const VectorXs& grad_objective ) const;

};

#endif
