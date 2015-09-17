// StapleStapleUtilities.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef STAPLE_STAPLE_UTILITIES_H
#define STAPLE_STAPLE_UTILITIES_H

#include "scisim/Math/MathDefines.h"

class RigidBodyStaple;

class StapleStapleCollision final
{

public:

  StapleStapleCollision( const int e0_in, const int e1_in, const scalar& s_in, const scalar& t_in )
  : e0( e0_in )
  , e1( e1_in )
  , s( s_in )
  , t( t_in )
  {
    // Two possible encodings for collisions at joints, so force one
    if( this->e0 == 0 && this->s == 1.0 )
    {
      this->e0 = 1;
      this->s = 0.0;
    }
    if( this->e1 == 0 && this->t == 1.0 )
    {
      this->e1 = 1;
      this->t = 0.0;
    }

    if( this->e0 == 2 && this->s == 0.0 )
    {
      this->e0 = 1;
      this->s = 1.0;
    }
    if( this->e1 == 2 && this->t == 0.0 )
    {
      this->e1 = 1;
      this->t = 1.0;
    }

    assert( this->e0 >= 0 ); assert( this->e0 <= 3 );
    assert( this->e1 >= 0 ); assert( this->e1 <= 3 );
    assert( this->s >= 0.0 ); assert( this->s <= 1.0 );
    assert( this->t >= 0.0 ); assert( this->t <= 1.0 );
  }

  // Edge on first staple
  int e0;

  // Edge on second staple
  int e1;

  // Barycentric coordinate of collision on first staple's edge
  scalar s;

  // Barycentric coordinate of collision on second staple's edge
  scalar t;

  inline bool operator<( const StapleStapleCollision& rhs ) const
  {
    if( e0 == rhs.e0 )
    {
      if( e1 == rhs.e1 )
      {
        if( s == rhs.s )
        {
          if( t == rhs.t )
          {
            return false;
          }
          return t < rhs.t;
        }
        return s < rhs.s;
      }
      return e1 < rhs.e1;
    }
    return e0 < rhs.e0;
  }
};

namespace StapleStapleUtilities
{

  // Staple-Staple
  bool isActive( const Vector3s& cm0, const Matrix33sr& R0, const RigidBodyStaple& staple0,
                 const Vector3s& cm1, const Matrix33sr& R1, const RigidBodyStaple& staple1 );

  // Staple-Staple
  [[noreturn]] void computeConstraints( const Vector3s& cm0, const Matrix33sr& R0, const RigidBodyStaple& staple0,
                                        const Vector3s& cm1, const Matrix33sr& R1, const RigidBodyStaple& staple1,
                                        std::vector<Vector3s>& p, std::vector<Vector3s>& n );

  void computeStapleHalfPlaneActiveSet( const Vector3s& cm, const Matrix33sr& R, const RigidBodyStaple& staple,
                                        const Vector3s& x0, const Vector3s& n, std::vector<int>& points );

}

#endif
