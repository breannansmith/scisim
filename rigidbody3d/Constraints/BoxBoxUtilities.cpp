// BoxBoxUtilities.cpp
//
// Adapted from ODE
// Last updated: 01/05/2016

#include "BoxBoxUtilities.h"

#include <limits>
#include <cassert>
#include <cmath>

static scalar dot( const scalar* a, const scalar* b )
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// BS: I think this is getting the closest points between two edges, compare to other codes that do that
// Get closest points between edges (one at each)
static void lineClosestApproach( const Vector3s& pa, const Vector3s& ua, const Vector3s& pb, const Vector3s& ub, scalar& alpha, scalar& beta )
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

// TODO: Clean this code up
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

static bool axisAlignedSeperatingTest( const scalar& projected_center_dist, const scalar& projected_aabb_widths, const int crnt_code, scalar& smallest_pen_depth, bool& invert_normal, int& code )
{
  const scalar pen_depth{ fabs(projected_center_dist) - projected_aabb_widths };
  if( pen_depth > 0 )
  {
    return true;
  }
  assert( smallest_pen_depth <= 0.0 );
  if( pen_depth > smallest_pen_depth )
  {
    smallest_pen_depth = pen_depth;
    invert_normal = projected_center_dist < 0.0;
    code = crnt_code;
  }
  return false;
}

static bool edgeEdgeSeparatingTest( const scalar& projected_center_dist, const scalar& projected_aabb_widths, const scalar& n1, const scalar& n2, const scalar& n3, const int crnt_code, scalar& smallest_pen_depth, Vector3s& normalC, bool& invert_normal, int& code )
{
  scalar s2{ fabs(projected_center_dist) - projected_aabb_widths };
  if( s2 > 0 )
  {
    return true;
  }
  const scalar l{ sqrt( n1 * n1 + n2 * n2 + n3 * n3 ) };
  if( l > 0 )
  {
    s2 /= l;
    assert( smallest_pen_depth <= 0.0 );
    const scalar fudge_factor{ 1.05 }; // <- TODO: Biasing towards faces, I think?
    if( s2 * fudge_factor > smallest_pen_depth )
    {
      smallest_pen_depth = s2;
      normalC << n1, n2, n3;
      normalC /= l;
      invert_normal = projected_center_dist < 0;
      code = crnt_code;
    }
  }
  return false;
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
static void boxBox( const Vector3s& p1, const Matrix33sr& R1, const Vector3s& side1, const Vector3s& p2, const Matrix33sr& R2, const Vector3s& side2, Vector3s& normal, scalar& depth, int& code, std::vector<Vector3s>& contact_points, std::vector<scalar>& depths )
{
  // Relative displacement vector from center of box 1 to box 2, relative to box 1
  const Vector3s p{ p2 - p1 };
  // pp = p relative to body 1
  const Vector3s pp{ R1.transpose() * p };

  // R = R1^T * R2, i.e. the relative rotation between R1 and R2
  const Matrix33sr R{ R1.transpose() * R2 };
  const Matrix33sr Q{ R.array().abs().matrix() };

  // For all 15 possible separating axes:
  //   * See if the axis separates the boxes. If so, return 0.
  //   * Find the depth of the penetration along the separating axis (s2).
  //   * If this is the largest depth so far, record it.
  // The normal vector will be set to the separating axis with the smallest
  // depth.

  // Tracks the smallest penetrating depth in the simulation
  depth = -SCALAR_INFINITY;
  // Each test can apply two faces or edges, track which one was active
  bool invert_normal{ false };
  // Tracks which piece of geometry has teh smallest penetrating depth
  code = 0;

  // Check first box's principle axes
  // TODO: Replace with for loop
  {
    // Width of the axis-aligned bounding box around the second body in R1's frame
    const Vector3s QbA{ Q * side2 + side1 };

    // Separating axis = u1
    if( axisAlignedSeperatingTest( pp.x(), QbA.x(), 1, depth, invert_normal, code ) )
    {
      return;
    }
    // Separating axis = u2
    if( axisAlignedSeperatingTest( pp.y(), QbA.y(), 2, depth, invert_normal, code ) )
    {
      return;
    }
    // Separating axis = u3
    if( axisAlignedSeperatingTest( pp.z(), QbA.z(), 3, depth, invert_normal, code ) )
    {
      return;
    }
  }

  // Check the second box's principle axes
  // TODO: Replace with for loop
  {
    // Separation vector projected onto R2's axes
    const Vector3s p_on_R2{ R2.transpose() * p };

    // Width of the axis-aligned bounding box around the first body in R2's frame
    const Vector3s QTaB{ Q.transpose() * side1 + side2 };

    // separating axis = v1
    if( axisAlignedSeperatingTest( p_on_R2.x(), QTaB.x(), 4, depth, invert_normal, code ) )
    {
      return;
    }
    // separating axis = v2
    if( axisAlignedSeperatingTest( p_on_R2.y(), QTaB.y(), 5, depth, invert_normal, code ) )
    {
      return;
    }
    // separating axis = v3
    if( axisAlignedSeperatingTest( p_on_R2.z(), QTaB.z(), 6, depth, invert_normal, code ) )
    {
      return;
    }
  }

  // Note: cross product axes need to be scaled when s is computed.
  // Normal (n1,n2,n3) is relative to box 1.
  // We only need to check 3 edges per box since parallel edges are equivalent.

  Vector3s normalC;

  // separating axis = u1 x v1
  if( edgeEdgeSeparatingTest( pp.z() * R(1,0) - pp.y() * R(2,0),
             side1[1] * Q(2,0) + side1[2] * Q(1,0) +
             side2[1] * Q(0,2) + side2[2] * Q(0,1),
             0, -R(2,0), R(1,0),
             7, depth, normalC, invert_normal, code ) )
  {
    return;
  }
  // separating axis = u1 x v2
  if( edgeEdgeSeparatingTest( pp.z() * R(1,1) - pp.y() * R(2,1),
             side1[1] * Q(2,1) + side1[2] * Q(1,1) +
             side2[0] * Q(0,2) + side2[2] * Q(0,0),
             0, -R(2,1), R(1,1),
             8, depth, normalC, invert_normal, code ) )
  {
    return;
  }
  // separating axis = u1 x v3
  if( edgeEdgeSeparatingTest( pp.z() * R(1,2) - pp.y() * R(2,2),
             side1[1] * Q(2,2) + side1[2] * Q(1,2) +
             side2[0] * Q(0,1) + side2[1] * Q(0,0),
             0, -R(2,2), R(1,2),
             9, depth, normalC, invert_normal, code ) )
  {
    return;
  }

  // separating axis = u2 x v1
  if( edgeEdgeSeparatingTest( pp.x() * R(2,0) - pp.z() * R(0,0),
             side1[0] * Q(2,0) + side1[2] * Q(0,0) +
             side2[1] * Q(1,2) + side2[2] * Q(1,1),
             R(2,0), 0, -R(0,0),
             10, depth, normalC, invert_normal, code ) )
  {
    return;
  }
  // separating axis = u2 x v2
  if( edgeEdgeSeparatingTest( pp.x() * R(2,1) - pp.z() * R(0,1),
             side1[0] * Q(2,1) + side1[2] * Q(0,1) +
             side2[0] * Q(1,2) + side2[2] * Q(1,0),
             R(2,1), 0, -R(0,1),
             11, depth, normalC, invert_normal, code ) )
  {
    return;
  }
  // separating axis = u2 x v3
  if( edgeEdgeSeparatingTest( pp.x() * R(2,2) - pp.z() * R(0,2),
             side1[0] * Q(2,2) + side1[2] * Q(0,2) +
             side2[0] * Q(1,1) + side2[1] * Q(1,0),
             R(2,2), 0, -R(0,2),
             12, depth, normalC, invert_normal, code ) )
  {
    return;
  }

  // separating axis = u3 x v1
  if( edgeEdgeSeparatingTest( pp.y() * R(0,0) - pp.x() * R(1,0),
             side1[0] * Q(1,0) + side1[1] * Q(0,0) +
             side2[1] * Q(2,2) + side2[2] * Q(2,1),
             -R(1,0), R(0,0), 0,
             13, depth, normalC, invert_normal, code ) )
  {
    return;
  }
  // separating axis = u3 x v2
  if( edgeEdgeSeparatingTest( pp.y() * R(0,1) - pp.x() * R(1,1),
             side1[0] * Q(1,1) + side1[1] * Q(0,1) +
             side2[0] * Q(2,2) + side2[2] * Q(2,0),
             -R(1,1), R(0,1), 0,
             14, depth, normalC, invert_normal, code ) )
  {
    return;
  }
  // separating axis = u3 x v3
  if( edgeEdgeSeparatingTest( pp.y() * R(0,2) - pp.x() * R(1,2),
             side1[0] * Q(1,2) + side1[1] * Q(0,2) +
             side2[0] * Q(2,1) + side2[1] * Q(2,0),
             -R(1,2), R(0,2), 0,
             15, depth, normalC, invert_normal, code ) )
  {
    return;
  }

  assert( code != 0 );

  // If we get to this point, the boxes interpenetrate. Compute the normal in global coordinates.
  if( code <= 6 )
  {
    assert( code >= 1 );
    // Codes 1...3 are box 0 faces, codes 4...6 are box 1 faces
    normal = code <= 3 ? R1.col( code - 1 ) : R2.col( code - 4 );
  }
  else
  {
    assert( code > 6 && code <= 15 );
    normal = R1 * normalC;
  }
  if( invert_normal )
  {
    normal *= -1.0;
  }
  depth *= -1.0;

  // compute contact point(s)

  // TODO: Abstract this into an edge-edge function
  if( code > 6 && code <= 15 )
  {
    // An edge from box 1 touches an edge from box 2.

    // Find a point pa on the intersecting edge of box 1
    Vector3s pa{ p1 };
    // Get world position of p2 into pa
    for( int j = 0; j < 3; ++j )
    {
      const scalar sign{ normal.dot( R1.col(j) ) > 0 ? 1.0 : -1.0 };
      for( int i = 0; i < 3; ++i )
      {
        pa[i] += sign * side1[j] * R1( i, j );
      }
    }

    // Find a point pb on the intersecting edge of box 2
    Vector3s pb{ p2 };
    // Get world position of p2 into pb
    for( int j = 0; j < 3; ++j )
    {
      const scalar sign{ normal.dot( R2.col(j) ) > 0 ? -1.0 : 1.0 };
      for( int i = 0; i < 3; ++i )
      {
        pb[i] += sign * side2[j] * R2( i, j );
      }
    }

    // Get direction of first edge
    Vector3s ua;
    for( int i = 0; i < 3; ++i )
    {
      ua[i] = R1( i, ( code - 7 ) / 3 );
    }
    // Get direction of second edge
    Vector3s ub;
    for( int i = 0; i < 3; ++i )
    {
      ub[i] = R2( i, ( code - 7 ) % 3 );
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

    // We are done!
    return;
  }

  assert( code >= 1 ); assert( code <= 6 );
  // TODO: Factor the following code into a separate function, instead of these pointers just swap argument order
  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). define face 'a' to be the reference
  // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
  // the incident face (the closest face of the other box).
  // Note: Unmodified parameter values are being used here
  Matrix33sr Ra;
  Matrix33sr Rb;
  Vector3s pa;
  Vector3s pb;
  Vector3s Sa;
  Vector3s Sb;
  // One of the faces of box 1 is the reference face
  if( code <= 3 )
  {
    Ra = R1; // Rotation of 'a'
    Rb = R2; // Rotation of 'b'
    pa = p1; // Center (location) of 'a'
    pb = p2; // Center (location) of 'b'
    Sa = side1;  // Side Lenght of 'a'
    Sb = side2;  // Side Lenght of 'b'
  }
  // One of the faces of box 2 is the reference face
  else
  {
    Ra = R2; // Rotation of 'a'
    Rb = R1; // Rotation of 'b'
    pa = p2; // Center (location) of 'a'
    pb = p1; // Center (location) of 'b'
    Sa = side2;  // Side Lenght of 'a'
    Sb = side1;  // Side Lenght of 'b'
  }

  // nr = normal vector of reference face dotted with axes of incident box.
  // anr = absolute values of nr.
  // The normal is flipped if necessary so it always points outward from box 'a',
  // box 'b' is thus always the incident box
  const Vector3s normal2{ code <= 3 ? normal : (-normal).matrix() };
  // Rotate normal2 in incident box opposite direction
  const Vector3s nr{ Rb.transpose() * normal2 };

  // find the largest compontent of anr: this corresponds to the normal
  // for the incident face. The other axis numbers of the incident face
  // are stored in a1, a2.
  int lanr;
  int a1;
  int a2;
  {
    const Vector3s anr{ nr.array().abs().matrix() };

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
  }

  // Compute center point of incident face, in reference-face coordinates
  Vector3s center;
  if( nr[lanr] < 0 )
  {
    center = pb - pa + Sb(lanr) * Rb.col(lanr);
  }
  else
  {
    center = pb - pa - Sb(lanr) * Rb.col(lanr);
  }

  // find the normal and non-normal axis numbers of the reference box
  int codeN;
  if( code <= 3 )
  {
    codeN = code - 1;
  }
  else
  {
    codeN = code - 4;
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

  // Find the four corners of the incident face, in reference-face coordinates
  // 2D coordinate of incident face (x,y pairs)
  const scalar c1{ center.dot( Ra.col( code1 ) ) };
  const scalar c2{ center.dot( Ra.col( code2 ) ) };
  // optimize this? - we have already computed this data above, but it is not
  // stored in an easy-to-index format. for now it's quicker just to recompute
  // the four dot products.
  scalar m11{ Ra.col( code1 ).dot( Rb.col( a1 ) ) };
  scalar m12{ Ra.col( code1 ).dot( Rb.col( a2 ) ) };
  scalar m21{ Ra.col( code2 ).dot( Rb.col( a1 ) ) };
  scalar m22{ Ra.col( code2 ).dot( Rb.col( a2 ) ) };

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
  assert( n >= 1 ); assert( n <= 8 );

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
      point[ 3 * cnum + i ] = center[i] + k1 * Rb( i, a1 ) + k2 * Rb( i, a2 );
    }
    dep[cnum] = Sa[codeN] - dot( normal2.data(), point + 3 * cnum );
    if( dep[cnum] >= 0 )
    {
      ret[ 2 * cnum ] = ret[ 2 * j ];
      ret[ 2 * cnum + 1 ] = ret[ 2 * j + 1 ];
      cnum++;
    }
  }
  assert( cnum >= 1 );

  // TODO: Emplace directly in code above
  assert( contact_points.empty() );
  assert( depths.empty() );
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

void BoxBoxUtilities::isActive( const Vector3s& cm0, const Matrix33sr& R0, const Vector3s& side0, const Vector3s& cm1, const Matrix33sr& R1, const Vector3s& side1, Vector3s& n, std::vector<Vector3s>& points )
{
  scalar max_depth;
  std::vector<scalar> depths;
  int return_code;
  boxBox( cm0, R0, side0, cm1, R1, side1, n, max_depth, return_code, points, depths );
  assert( points.size() == depths.size() );
  // Invert the normal so it points from the second body to the first body
  n *= -1.0;
  // TODO: check collision_type, check max_depth
}
