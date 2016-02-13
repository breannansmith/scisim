// ConstrainedSystem.h
//
// Breannan Smith
// Last updated: 12/07/2015

#ifndef CONSTRAINED_SYSTEM_H
#define CONSTRAINED_SYSTEM_H

#include <memory>

#include "scisim/Math/MathDefines.h"

class Constraint;

class ConstrainedSystem
{

public:

  virtual void computeActiveSet( const VectorXs& q0, const VectorXs& qp, const VectorXs& v, std::vector<std::unique_ptr<Constraint>>& active_set ) = 0;
  virtual void computeImpactBases( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& impact_bases ) const = 0;
  virtual void computeContactBases( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& contact_bases ) const = 0;

  virtual void clearConstraintCache() = 0;
  virtual void cacheConstraint( const Constraint& constraint, const VectorXs& r ) = 0;
  virtual void getCachedConstraintImpulse( const Constraint& constraint, VectorXs& r  ) const = 0;
  virtual bool constraintCacheEmpty() const = 0;

protected:

  ConstrainedSystem() = default;
  ConstrainedSystem( const ConstrainedSystem& ) = default;
  ConstrainedSystem( ConstrainedSystem&& ) = default;
  ConstrainedSystem& operator=( const ConstrainedSystem& other ) = default;
  ConstrainedSystem& operator=( ConstrainedSystem&& other ) = default;
  virtual ~ConstrainedSystem() = 0;

};

#endif
