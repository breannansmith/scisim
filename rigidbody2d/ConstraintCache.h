// ConstraintCache.h
//
// Breannan Smith
// Last updated: 01/18/2016

#ifndef CONSTRAINT_CACHE_H
#define CONSTRAINT_CACHE_H

#include "scisim/Math/MathDefines.h"

class Constraint;

class ConstraintCache final
{

public:

  void cacheConstraint( const Constraint& constraint, const VectorXs& r );
  void getCachedConstraint( const Constraint& constraint, VectorXs& r ) const;
  void clear();
  bool empty() const;

  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

private:

  std::map<std::pair<unsigned,unsigned>,VectorXs> m_circle_circle_constraints;
  // TODO: This cacheing strategy will not work for box-box collisions
  std::map<std::pair<unsigned,unsigned>,VectorXs> m_body_body_constraints;
  std::map<std::pair<unsigned,unsigned>,VectorXs> m_kinematic_object_circle_constraints;
  std::map<std::pair<unsigned,unsigned>,VectorXs> m_plane_circle_constraints;

};

#endif
