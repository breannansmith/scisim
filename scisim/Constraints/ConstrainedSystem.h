// ConstrainedSystem.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef CONSTRAINED_SYSTEM_H
#define CONSTRAINED_SYSTEM_H

#include <memory>

#include "scisim/Math/MathDefines.h"

class Constraint;

class ConstrainedSystem
{

public:

  virtual ~ConstrainedSystem() = 0;

  virtual void computeActiveSet( const VectorXs& q0, const VectorXs& qp, std::vector<std::unique_ptr<Constraint>>& active_set ) = 0;
  virtual void computeImpactBases( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& impact_bases ) const = 0;
  virtual void computeContactBases( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& contact_bases ) const = 0;

  virtual void clearConstraintCache() = 0;
  virtual void cacheConstraint( const Constraint& constraint, const VectorXs& r ) = 0;
  virtual void getCachedConstraintImpulse( const Constraint& constraint, VectorXs& r  ) const = 0;

};

#endif
