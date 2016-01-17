// ConstraintCache.h
//
// Breannan Smith
// Last updated: 09/03/2015

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

  std::map<std::pair<unsigned,unsigned>,VectorXs> m_ball_ball_constraints;
  std::map<std::pair<unsigned,unsigned>,VectorXs> m_plane_ball_constraints;
  std::map<std::pair<unsigned,unsigned>,VectorXs> m_drum_ball_constraints;

};

#endif
