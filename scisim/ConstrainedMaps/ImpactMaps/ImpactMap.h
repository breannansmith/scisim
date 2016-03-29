// ImpactMap.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef IMPACT_MAP_H
#define IMPACT_MAP_H

#include "scisim/Math/MathDefines.h"

class ScriptingCallback;
class FlowableSystem;
class ConstrainedSystem;
class UnconstrainedMap;
class ImpactOperator;
class ImpactSolution;

class ImpactMap final
{

public:

  explicit ImpactMap( const bool warm_start = false );
  explicit ImpactMap( std::istream& input_stream );

  void flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, ImpactOperator& imap, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 );

  void serialize( std::ostream& output_stream ) const;

  void exportForcesNextStep( ImpactSolution& impact_solution );

private:

  bool m_warm_start;

  // Temporary state for writing constraint forces
  bool m_write_constraint_forces;
  ImpactSolution* m_impact_solution;

};

#endif
