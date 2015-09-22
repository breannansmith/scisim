// CollisionUtilities.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef COLLISION_UTILITIES_H
#define COLLISION_UTILITIES_H

#include "scisim/Math/MathDefines.h"

namespace CollisionUtilities
{

  // Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
  // S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
  // distance between between S1(s) and S2(t)
  // Adapted from real time collision detection
  scalar closestPointSegmentSegment( const Vector3s& p1, const Vector3s& q1, const Vector3s& p2, const Vector3s& q2, scalar& s, scalar& t, Vector3s& c1, Vector3s& c2 );

  // xb: Center of mass of box
  // Rb: Orientation of box
  // wb: Half-widths of box
  // xs: Center of mass of sphere
  // rs: radius of sphere
  // p: collision points
  // n: collision normals
  void computeBoxSphereActiveSet( const Vector3s& xb, const Matrix33sc& Rb, const Vector3s& wb, const Vector3s& xs, const scalar& rs, std::vector<Vector3s>& p, std::vector<Vector3s>& n );

}

#endif
