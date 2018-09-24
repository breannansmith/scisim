#ifndef IMPACT_OPERATOR_UTILITIES_H
#define IMPACT_OPERATOR_UTILITIES_H

#include "scisim/Math/MathDefines.h"

#include <memory>

class FlowableSystem;
class Constraint;

namespace ImpactOperatorUtilities
{

  void computeN( const FlowableSystem& fsys, const std::vector<std::unique_ptr<Constraint>>& V, const VectorXs& q, SparseMatrixsc& N );

  void computeLCPQPLinearTerm( const SparseMatrixsc& N, const VectorXs& nrel, const VectorXs& CoR, const VectorXs& v0, const VectorXs& v0F, VectorXs& linear_term );

  void evalKinematicRelativeVelocityN( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& gdotN );

}

#endif
