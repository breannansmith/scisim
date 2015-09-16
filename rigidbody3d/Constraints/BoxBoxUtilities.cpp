// BoxBoxUtilities.cpp
//
// Adapted from ODE
// Last updated: 09/15/2015

#include "BoxBoxUtilities.h"

#include <limits>
#include <cassert>
#include <cmath>
#include <iostream>

typedef scalar dMatrix3[4*3];

static void computeCentroid( const int n, const scalar p[], scalar& cx, scalar& cy )
{
  if( n == 1 )
  {
    cx = p[0];
    cy = p[1];
  }
  else if( n == 2 )
  {
    cx = 0.5 * ( p[0] + p[2] );
    cy = 0.5 * ( p[1] + p[3] );
  }
  else
  {
    scalar a{ 0.0 };
    cx = 0.0;
    cy = 0.0;
    for( int i = 0; i < n - 1; ++i )
    {
      // BS: This looks like a cross product
      const scalar q{ p[ 2 * i + 0 ] * p[ 2 * i + 3 ] - p[ 2 * i + 2 ] * p[ 2 * i + 1 ] };
      a += q;
      cx += q * ( p[ 2 * i + 0 ] + p[ 2 * i + 2 ] );
      cy += q * ( p[ 2 * i + 1 ] + p[ 2 * i + 3 ] );
    }
    const scalar q{ p[ 2 * n - 2 ] * p[1] - p[0] * p[ 2 * n - 1 ] };
    a = 1.0 / ( 3.0 * ( a + q ) );
    cx = a * ( cx + q * ( p[ 2 * n - 2 ] + p[0] ) );
    cy = a * ( cy + q * ( p[ 2 * n - 1 ] + p[1] ) );
  }
}

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].
// BS: n should be in [1..8), I think, no 8!

// n: number of points in the polygon
// p: points in the polygon, size should be 2 * n
// m: ?
// i0: ?
// iret: ?
static void cullPoints( const int n, const scalar p[], const int m, const int i0, int iret[] )
{
  assert( n < 8 );

  // Compute the centroid of the polygon in cx,cy
  scalar cx;
  scalar cy;
  computeCentroid( n, p, cx, cy );

  // Compute the angle of each point w.r.t. the centroid
  scalar A[8];
  for( int i = 0; i < n; ++i )
  {
    A[i] = atan2( p[ 2 * i + 1 ] - cy, p[ 2 * i ] - cx );
  }

  // Search for points that have angles closest to A[i0] + i * ( 2 * pi / m ).
  int avail[8];
  for( int i = 0; i < n; ++i )
  {
    avail[i] = 1;
  }
  avail[i0] = 0;
  iret[0] = i0;
  iret++;
  for( int j = 1; j < m; ++j )
  {
    scalar a{ scalar( scalar(j) * ( 2 * M_PI / m ) + A[i0] ) };
    if( a > M_PI )
    {
      a -= scalar( 2 * M_PI );
    }
    scalar maxdiff{ 1e9 };
    scalar diff;
    #ifndef NDEBUG
    *iret = i0; // iret is not allowed to keep this value
    #endif
    for( int i = 0; i < n; ++i )
    {
      if( avail[i] )
      {
        diff = fabs( A[i] - a );
        if( diff > M_PI )
        {
          diff = scalar( 2 * M_PI - diff );
        }
        if( diff < maxdiff )
        {
          maxdiff = diff;
          *iret = i;
        }
      }
    }
    // ensure iret got set
    assert (*iret != i0);
    avail[*iret] = 0;
    iret++;
  }
}

static scalar dot( const scalar* a, const scalar* b )
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static scalar dot14( const scalar* a, const scalar* b )
{
  return a[0] * b[0] + a[1] * b[4] + a[2] * b[8];
}

static scalar dot41( const scalar* a, const scalar* b )
{
  return a[0] * b[0] + a[4] * b[1] + a[8] * b[2];
}

static scalar dot44( const scalar* a, const scalar* b )
{
  return a[0] * b[0] + a[4] * b[4] + a[8] * b[8];
}

static void dMULTIPLY0_331( Vector3s& A, const scalar* B, const Vector3s& C )
{
  A.x() = dot( B + 0, C.data() );
  A.y() = dot( B + 4, C.data() );
  A.z() = dot( B + 8, C.data() );
}

static void dMULTIPLY1_331( Vector3s& A, const scalar* B, const Vector3s& C )
{
  A.x() = dot41( B + 0, C.data() );
  A.y() = dot41( B + 1, C.data() );
  A.z() = dot41( B + 2, C.data() );
}

// BS: I think this is getting the closest points between two edges, compare to other codes that do that
// Get closest points between edges (one at each)
static void lineClosestApproach( const Vector3s& pa, const Vector3s& ua, const Vector3s& pb, const Vector3s& ub, scalar& alpha, scalar& beta)
{
  const Vector3s p{ pb - pa };
  const scalar uaub{ ua.dot( ub ) };
  const scalar q1{ ua.dot( p ) };
  const scalar q2{ -ub.dot( p ) };
  scalar d{ 1.0 - uaub * uaub };
  if( d <= 0.0001 )
  {
    // This needs to be made more robust
    alpha = 0.0;
    beta  = 0.0;
  }
  else
  {
    d = 1.0 / d;
    alpha = ( q1 + uaub * q2 ) * d;
    beta = ( uaub * q1 + q2 ) * d;
  }
}

// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).
static int intersectRectQuad( scalar h[2], scalar p[8], scalar ret[16] )
{
  // q (and r) contain nq (and nr) coordinate points for the current (and
  // chopped) polygons
  int nq{ 4 };
  int nr{ 0 };
  scalar buffer[16];
  scalar* q{ p };
  scalar* r{ ret };
  for( int dir = 0; dir <= 1; ++dir )
  {
    // direction notation: xy[0] = x axis, xy[1] = y axis
    for( int sign = -1; sign <= 1; sign += 2 )
    {
      // chop q along the line xy[dir] = sign*h[dir]
      scalar* pq{ q };
      scalar* pr{ r };
      nr = 0; // move the = 0 out of here
      for( int i = nq; i > 0; --i )
      {
        // go through all points in q and all lines between adjacent points
        if( sign * pq[dir] < h[dir] )
        {
          // this point is inside the chopping line
          pr[0] = pq[0];
          pr[1] = pq[1];
          pr += 2;
          nr++;
          if( nr & 8 )
          {
            q = r;
            goto done;
          }
        }
        scalar* nextq{ (i > 1) ? pq + 2 : q };
        if( ( sign * pq[dir] < h[dir] ) ^ ( sign * nextq[dir] < h[dir] ) )
        {
          // this line crosses the chopping line
          pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) / (nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
          pr[dir] = sign*h[dir];
          pr += 2;
          nr++;
          if( nr & 8 )
          {
            q = r;
            goto done;
          }
        }
        pq += 2;
      }
      q = r;
      r = (q==ret) ? buffer : ret;
      nq = nr;
    }
  }
done:
  if( q != ret )
  {
    memcpy( ret, q, nr * 2 * sizeof(scalar) );
  }
  return nr;
}

// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.


static bool test0( const scalar& expr1, const scalar& expr2, const scalar* norm, const int cc, scalar& s, const scalar** normalR, int& invert_normal, int& code )
{
  const scalar s2{ fabs(expr1) - expr2 };
  // Separating axis?
  if( s2 > 0 )
  {
    return true;
  }
  if( s2 > s )
  {
    s = s2;
    *normalR = norm;
    invert_normal = expr1 < 0;
    code = cc;
  }
  return false;
}

bool test1( const scalar& expr1, const scalar& expr2, const scalar& n1, const scalar& n2, const scalar& n3, const int cc, scalar& s, const scalar** normalR, scalar* normalC, int& invert_normal, int& code )
{
  const scalar fudge_factor{ 1.05 };
  scalar s2{ fabs(expr1) - expr2 };
  if( s2 > 0 )
  {
    return true;
  }
  const scalar l{ sqrt( n1 * n1 + n2 * n2 + n3 * n3 ) };
  if( l > 0 )
  {
    s2 /= l;
    if( s2 * fudge_factor > s )
    {
      s = s2;
      *normalR = nullptr;
      normalC[0] = n1 / l;
      normalC[1] = n2 / l;
      normalC[2] = n3 / l;
      invert_normal = expr1 < 0;
      code = cc;
    }
  }
  return false;
}

static bool isEdgeEdge( const int code )
{
  return code > 6;
}

static void dBoxBox( const Vector3s& p1, const dMatrix3 R1, const Vector3s& side1, const Vector3s& p2, const dMatrix3 R2, const Vector3s& side2, Vector3s& normal, scalar& depth, int& return_code, const int max_num_contacts, std::vector<Vector3s>& contact_points, std::vector<scalar>& depths )
{
  // Relative displacement vector from center of box 1 to box 2, relative to box 1
  const Vector3s p{ p2 - p1 };
  // get pp = p relative to body 1
  Vector3s pp;
  dMULTIPLY1_331( pp, R1, p );

  // get side lengths / 2
  const Vector3s A{ 0.5 * side1 };
  const Vector3s B{ 0.5 * side2 };

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
  const scalar R11{ dot44( R1 + 0, R2 + 0 ) };
  const scalar R12{ dot44( R1 + 0, R2 + 1 ) };
  const scalar R13{ dot44( R1 + 0, R2 + 2 ) };
  const scalar R21{ dot44( R1 + 1, R2 + 0 ) };
  const scalar R22{ dot44( R1 + 1, R2 + 1 ) };
  const scalar R23{ dot44( R1 + 1, R2 + 2 ) };
  const scalar R31{ dot44( R1 + 2, R2 + 0 ) };
  const scalar R32{ dot44( R1 + 2, R2 + 1 ) };
  const scalar R33{ dot44( R1 + 2, R2 + 2 ) };
//  std::cout << "Next: " << std::endl;
//  std::cout << R11 << " " << R12 << " " << R13 << std::endl;
//  std::cout << R21 << " " << R22 << " " << R23 << std::endl;
//  std::cout << R31 << " " << R32 << " " << R33 << std::endl;

  const scalar Q11{ fabs( R11 ) };
  const scalar Q12{ fabs( R12 ) };
  const scalar Q13{ fabs( R13 ) };
  const scalar Q21{ fabs( R21 ) };
  const scalar Q22{ fabs( R22 ) };
  const scalar Q23{ fabs( R23 ) };
  const scalar Q31{ fabs( R31 ) };
  const scalar Q32{ fabs( R32 ) };
  const scalar Q33{ fabs( R33 ) };

  // for all 15 possible separating axes:
  //   * see if the axis separates the boxes. if so, return 0.
  //   * find the depth of the penetration along the separating axis (s2)
  //   * if this is the largest depth so far, record it.
  // the normal vector will be set to the separating axis with the smallest
  // depth. note: normalR is set to point to a column of R1 or R2 if that is
  // the smallest depth normal so far. otherwise normalR is 0 and normalC is
  // set to a vector relative to body 1. invert_normal is 1 if the sign of
  // the normal should be flipped.

  Vector3s normalC{ 0.0, 0.0, 0.0 };
  scalar s{ -std::numeric_limits<scalar>::infinity() };
  const scalar* normalR{ nullptr };
  int invert_normal{ 0 };
  int code{ 0 };

  // separating axis = u1,u2,u3
  if( test0( pp[0], A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13, R1 + 0, 1, s, &normalR, invert_normal, code ) )
  {
    return;
  }
  if( test0( pp[1], A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23, R1 + 1, 2, s, &normalR, invert_normal, code ) )
  {
    return;
  }
  if( test0( pp[2], A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33, R1 + 2, 3, s, &normalR, invert_normal, code ) )
  {
    return;
  }

  // separating axis = v1,v2,v3
  if( test0( dot41( R2 + 0, p.data() ), A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0], R2 + 0, 4, s, &normalR, invert_normal, code ) )
  {
    return;
  }
  if( test0( dot41( R2 + 1, p.data() ), A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1], R2 + 1, 5, s, &normalR, invert_normal, code ) )
  {
    return;
  }
  if( test0( dot41( R2 + 2, p.data() ), A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2], R2 + 2, 6, s, &normalR, invert_normal, code ) )
  {
    return;
  }

  // note: cross product axes need to be scaled when s is computed.
  // normal (n1,n2,n3) is relative to box 1.

  // We only need to check 3 edges per box
  // since parallel edges are equivalent.

  // separating axis = u1 x (v1,v2,v3)
  if( test1( pp[2] * R21 - pp[1] * R31, A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12, 0, -R31, R21, 7, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }
  if( test1( pp[2] * R22 - pp[1] * R32, A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11, 0, -R32, R22, 8, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }
  if( test1( pp[2] * R23 - pp[1] * R33, A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11, 0, -R33, R23, 9, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }

  // separating axis = u2 x (v1,v2,v3)
  if( test1( pp[0] * R31 - pp[2] * R11, A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22, R31, 0, -R11, 10, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }
  if( test1( pp[0] * R32 - pp[2] * R12, A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21, R32, 0, -R12, 11, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }
  if( test1( pp[0] * R33 - pp[2] * R13, A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21, R33, 0, -R13, 12, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }

  // separating axis = u3 x (v1,v2,v3)
  if( test1( pp[1] * R11 - pp[0] * R21, A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32, -R21, R11, 0, 13, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }
  if( test1( pp[1] * R12 - pp[0] * R22, A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31, -R22, R12, 0, 14, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }
  if( test1( pp[1] * R13 - pp[0] * R23, A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31, -R23, R13, 0, 15, s, &normalR, normalC.data(), invert_normal, code ) )
  {
    return;
  }

  // TODO: Why can code be 0 here? No separating axis but not touching, maybe?
  if( code == 0 )
  {
    return;
  }

  // If we get to this point, the boxes interpenetrate. Compute the normal in global coordinates.
  if( normalR != nullptr )
  {
    normal.x() = normalR[0];
    normal.y() = normalR[4];
    normal.z() = normalR[8];
  }
  else
  {
    dMULTIPLY0_331( normal, R1, normalC );
  }
  if( invert_normal )
  {
    normal *= -1.0;
  }
  depth = -s;

  // compute contact point(s)

  // TODO: Abstract this into an edge-edge function
  if( isEdgeEdge( code ) )
  {
    // An edge from box 1 touches an edge from box 2.

    // Find a point pa on the intersecting edge of box 1
    Vector3s pa;
    scalar sign;
    // Copy p1 into pa
    pa = p1;
    // Get world position of p2 into pa
    for( int j = 0; j < 3; ++j )
    {
      sign = dot14( normal.data(), R1 + j ) > 0 ? 1.0 : -1.0;
      for( int i = 0; i < 3; ++i )
      {
        pa[i] += sign * A[j] * R1[ 4 * i + j ];
      }
    }

    // Find a point pb on the intersecting edge of box 2
    Vector3s pb;
    // Copy p2 into pb
    pb = p2;
    // Get world position of p2 into pb
    for( int j = 0; j < 3; ++j )
    {
      sign = dot14( normal.data(), R2 + j ) > 0 ? -1.0 : 1.0;
      for( int i = 0; i < 3; ++i )
      {
        pb[i] += sign * B[j] * R2[ 4 * i + j ];
      }
    }

    Vector3s ua;
    Vector3s ub;
    // Get direction of first edge
    for( int i = 0; i < 3; ++i )
    {
      ua[i] = R1[ ( code - 7 ) / 3 + i * 4 ];
    }
    // Get direction of second edge
    for( int i = 0; i < 3; ++i )
    {
      ub[i] = R2[ ( code - 7 ) % 3 + i * 4 ];
    }
    // Get closest points between edges (one at each)
    scalar alpha;
    scalar beta;
    lineClosestApproach( pa, ua, pb, ub, alpha, beta );
    pa += alpha * ua;
    pb += beta * ub;
    // Set the contact point as halfway between the 2 closest points
    contact_points.emplace_back( 0.5 * ( pa + pb ) );
    depths.emplace_back( depth );
    return_code = code;

    // We are done!
    return;
  }

  // TODO: Factor the following code into a separate function, instead of these pointers just swap argument order

  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). define face 'a' to be the reference
  // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
  // the incident face (the closest face of the other box).
  // Note: Unmodified parameter values are being used here
  const scalar* Ra;
  const scalar* Rb;
  const scalar* pa;
  const scalar* pb;
  const scalar* Sa;
  const scalar* Sb;
  // One of the faces of box 1 is the reference face
  if( code <= 3 )
  {
    Ra = R1; // Rotation of 'a'
    Rb = R2; // Rotation of 'b'
    pa = p1.data(); // Center (location) of 'a'
    pb = p2.data(); // Center (location) of 'b'
    Sa = A.data();  // Side Lenght of 'a'
    Sb = B.data();  // Side Lenght of 'b'
  }
  // One of the faces of box 2 is the reference face
  else
  {
    Ra = R2; // Rotation of 'a'
    Rb = R1; // Rotation of 'b'
    pa = p2.data(); // Center (location) of 'a'
    pb = p1.data(); // Center (location) of 'b'
    Sa = B.data();  // Side Lenght of 'a'
    Sb = A.data();  // Side Lenght of 'b'
  }

  // nr = normal vector of reference face dotted with axes of incident box.
  // anr = absolute values of nr.
  // The normal is flipped if necessary so it always points outward from box 'a',
  // box 'b' is thus always the incident box
  Vector3s normal2;
  if( code <= 3 )
  {
    normal2 = normal;
  }
  else
  {
    normal2 = -normal;
  }
  // Rotate normal2 in incident box opposite direction
  Vector3s nr;
  dMULTIPLY1_331( nr, Rb, normal2 );
  const Vector3s anr{ fabs( nr[0] ), fabs( nr[1] ), fabs( nr[2] ) };

  // find the largest compontent of anr: this corresponds to the normal
  // for the incident face. the other axis numbers of the incident face
  // are stored in a1,a2.
  int lanr,a1,a2;
  if( anr[1] > anr[0] )
  {
    if( anr[1] > anr[2] )
    {
      a1 = 0;
      lanr = 1;
      a2 = 2;
    }
    else
    {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }
  else
  {
    if( anr[0] > anr[2] )
    {
      lanr = 0;
      a1 = 1;
      a2 = 2;
    }
    else
    {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }

  // compute center point of incident face, in reference-face coordinates
  Vector3s center;
  if( nr[lanr] < 0 )
  {
    for( int i = 0; i < 3; ++i )
    {
      center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i*4+lanr];
    }
  }
  else
  {
    for( int i = 0; i < 3; ++i )
    {
      center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i*4+lanr];
    }
  }

  // find the normal and non-normal axis numbers of the reference box
  int codeN;
  if( code <= 3 )
  {
    codeN = code-1;
  }
  else
  {
    codeN = code-4;
  }
  int code1;
  int code2;
  if( codeN == 0 )
  {
    code1 = 1;
    code2 = 2;
  }
  else if( codeN == 1 )
  {
    code1 = 0;
    code2 = 2;
  }
  else
  {
    code1 = 0;
    code2 = 1;
  }

  // find the four corners of the incident face, in reference-face coordinates
  // 2D coordinate of incident face (x,y pairs)
  const scalar c1{ dot14( center.data(), Ra + code1 ) };
  const scalar c2{ dot14( center.data(), Ra + code2 ) };
  // optimize this? - we have already computed this data above, but it is not
  // stored in an easy-to-index format. for now it's quicker just to recompute
  // the four dot products.
  scalar m11{ dot44( Ra + code1, Rb + a1 ) };
  scalar m12{ dot44( Ra + code1, Rb + a2 ) };
  scalar m21{ dot44( Ra + code2, Rb + a1 ) };
  scalar m22{ dot44( Ra + code2, Rb + a2 ) };

  scalar quad[8];
  {
    const scalar k1{ m11 * Sb[a1] };
    const scalar k2{ m21 * Sb[a1] };
    const scalar k3{ m12 * Sb[a2] };
    const scalar k4{ m22 * Sb[a2] };
    quad[0] = c1 - k1 - k3;
    quad[1] = c2 - k2 - k4;
    quad[2] = c1 - k1 + k3;
    quad[3] = c2 - k2 + k4;
    quad[4] = c1 + k1 + k3;
    quad[5] = c2 + k2 + k4;
    quad[6] = c1 + k1 - k3;
    quad[7] = c2 + k2 - k4;
  }

  // find the size of the reference face
  scalar rect[2];
  rect[0] = Sa[code1];
  rect[1] = Sa[code2];

  // intersect the incident and reference faces
  scalar ret[16];
  const int n{ intersectRectQuad( rect, quad, ret ) };
  if( n < 1 )
  {
    std::cerr << "Hit first corner case that shouldn't happen! This is probably a bug." << std::endl;
    // This should never happen
    return;
  }

  // convert the intersection points into reference-face coordinates,
  // and compute the contact position and depth for each point. only keep
  // those points that have a positive (penetrating) depth. delete points in
  // the 'ret' array as necessary so that 'point' and 'ret' correspond.

  // penetrating contact points
  scalar point[3*8];
  // depths for those points
  scalar dep[8];
  const scalar det1{ 1.0 / ( m11 * m22 - m12 * m21 ) };
  m11 *= det1;
  m12 *= det1;
  m21 *= det1;
  m22 *= det1;
  // number of penetrating contact points found
  int cnum = 0;
  for( int j = 0; j < n; ++j )
  {
    const scalar k1{  m22 * ( ret[j*2] - c1 ) - m12 * ( ret[j*2+1] - c2 ) };
    const scalar k2{ -m21 * ( ret[j*2] - c1 ) + m11 * ( ret[j*2+1] - c2 ) };
    for( int i = 0; i < 3; ++i )
    {
      point[ 3 * cnum + i ] = center[i] + k1 * Rb[ 4 * i + a1 ] + k2 * Rb[ 4 * i + a2 ];
    }
    dep[cnum] = Sa[codeN] - dot( normal2.data(), point + 3 * cnum );
    if( dep[cnum] >= 0 )
    {
      ret[ 2 * cnum ] = ret[ 2 * j ];
      ret[ 2 * cnum + 1 ] = ret[ 2 * j + 1 ];
      cnum++;
      if( cnum == max_num_contacts )
      {
        std::cerr << "Hit max number of contacts in box-box! This is probably a bug." << std::endl;
        break;
      }
    }
  }
  if( cnum < 1 )
  {
    std::cerr << "Hit second corner case that shouldn't happen! This is probably a bug." << std::endl;
    // this should not happen, yet does at times (demo_plane2d single precision).
    return;
  }

  // We can't generate more contacts than we actually have
  int maxc{ max_num_contacts };
  if( maxc > cnum )
  {
    maxc = cnum;
  }
  if( maxc < 1 )
  {
    // Even though max count must not be zero this check is kept for backward compatibility as this is a public function
    maxc = 1;
  }

  if( cnum <= maxc )
  {
    // We have fewer contacts than we need, so we use them all
    for( int j = 0; j < cnum; ++j )
    {
      Vector3s new_point;
      for( int i = 0; i < 3; ++i )
      {
        new_point[i] = point[ 3 * j + i ] + pa[i];
      }
      contact_points.emplace_back( new_point );
      depths.emplace_back( dep[j] );
    }
  }
  else
  {
    std::cerr << "Third box-box corner case hit! This is probably a bug." << std::endl;
    // cnum should be generated not greater than maxc so that "then" clause is executed
    // we have more contacts than are wanted, some of them must be culled.
    // find the deepest point, it is always the first contact.
    int i1{ 0 };
    scalar maxdepth{ dep[0] };
    for( int i = 1; i < cnum; ++i )
    {
      if( dep[i] > maxdepth )
      {
        maxdepth = dep[i];
        i1 = i;
      }
    }

    int iret[8];
    cullPoints( cnum, ret, maxc, i1, iret );

    for( int j = 0; j < maxc; ++j )
    {
      Vector3s new_point;
      for( int i = 0; i < 3; ++i )
      {
        new_point[i] = point[iret[j]*3+i] + pa[i];
      }
      contact_points.emplace_back( new_point );
      depths.emplace_back( dep[iret[j]] );
    }
    cnum = maxc;
  }

  return_code = code;
  return;
}

static void load_dMatrix3( const Matrix33sr& eig_mat, dMatrix3 dMat )
{
  dMat[0] = eig_mat(0,0); dMat[1] = eig_mat(0,1); dMat[2 ] = eig_mat(0,2); // dMat[3 ]
  dMat[4] = eig_mat(1,0); dMat[5] = eig_mat(1,1); dMat[6 ] = eig_mat(1,2); // dMat[7 ]
  dMat[8] = eig_mat(2,0); dMat[9] = eig_mat(2,1); dMat[10] = eig_mat(2,2); // dMat[11]
}

static void dBoxBoxWrapper( const Vector3s& p1, const Matrix33sr& R1_in, const Vector3s& side1, const Vector3s& p2, const Matrix33sr& R2_in, const Vector3s& side2, Vector3s& normal, scalar& depth, int& return_code, std::vector<Vector3s>& contact_points, std::vector<scalar>& depths )
{
  dMatrix3 R1;
  load_dMatrix3( R1_in, R1 );

  dMatrix3 R2;
  load_dMatrix3( R2_in, R2 );

  const int max_num_contacts{ 16 }; // I think 8 should be max but let's see

  //std::cout << "Orig: " << std::endl;
  //std::cout << R1_in.transpose() * R2_in << std::endl;

  dBoxBox( p1, R1, side1, p2, R2, side2, normal, depth, return_code, max_num_contacts, contact_points, depths );
  assert( contact_points.size() == depths.size() );
}

void BoxBoxUtilities::isActive( const Vector3s& cm0, const Matrix33sr& R0, const Vector3s& side0, const Vector3s& cm1, const Matrix33sr& R1, const Vector3s& side1, Vector3s& n, std::vector<Vector3s>& points )
{
  scalar max_depth;
  std::vector<scalar> depths;
  int return_code;
  dBoxBoxWrapper( cm0, R0, 2.0 * side0, cm1, R1, 2.0 * side1, n, max_depth, return_code, points, depths );
  assert( points.size() == depths.size() );
  // Invert the normal so it points from the second body to the first body
  n *= -1.0;
  // TODO: check collision_type, check max_depth
}
