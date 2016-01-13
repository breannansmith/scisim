// BoxBoxTools.cpp
//
// Breannan Smith
// Last updated: 01/09/2016

#include "BoxBoxTools.h"

// TODO: Don't use an enum, instead have a couple of bools and indices
enum class CollidingFeature : std::uint8_t
{
  BOX_0_AXIS_0,
  BOX_0_AXIS_1,
  BOX_1_AXIS_0,
  BOX_1_AXIS_1
};

static bool axisAlignedSeperatingTest( const scalar& projected_center_dist, const scalar& projected_aabb_widths, const CollidingFeature crnt_feature, scalar& smallest_pen_depth, bool& invert_normal, CollidingFeature& feature )
{
  assert( projected_aabb_widths > 0.0 );
  assert( smallest_pen_depth <= 0.0 );
  const scalar pen_depth{ fabs(projected_center_dist) - projected_aabb_widths };
  if( pen_depth > 0 )
  {
    return true;
  }
  if( pen_depth > smallest_pen_depth )
  {
    smallest_pen_depth = pen_depth;
    invert_normal = projected_center_dist < 0.0;
    feature = crnt_feature;
  }
  return false;
}

#ifndef NDEBUG
static bool pointInBox( const Vector2s& x, const Matrix22sc& R, const Vector2s& r, const Vector2s& p )
{
  // Try to find a separating axis (up to a small numerical tolerance)
  if( ( ( R.transpose() * ( p - x ) ).array().abs() > r.array() + 1.0e-12 ).any() )
  {
    return false;
  }
  return true;
}
#endif

void BoxBoxTools::isActive( const Vector2s& x0, const scalar& theta0, const Vector2s& r0, const Vector2s& x1, const scalar& theta1, const Vector2s& r1, Vector2s& n, std::vector<Vector2s>& points )
{
  //using enum_type = std::underlying_type<CollidingFeature>::type;
  using enum_type = std::uint8_t;

  // Principle axes of the bodies
  const Matrix22sc R0{ Eigen::Rotation2D<scalar>{ theta0 } };
  const Matrix22sc R1{ Eigen::Rotation2D<scalar>{ theta1 } };

  CollidingFeature colliding_feature;
  // False if a front face, true if a back face
  bool invert_normal{ false };
  //bool box0_is_reference;
  //int collision_axis;

  // TODO: Pull this into a support function
  // Run a full separating axis test
  {
    // Track the feature with the smallest magnitude penetrating velocity
    scalar min_pen_depth{ -SCALAR_INFINITY };

    // Component-wise absolute value of relative rotation between body 0 and body 1
    // (Trick to easily compute AABB for a body in the other body's frame)
    const Matrix22sc Q{ ( R0.transpose() * R1 ).cwiseAbs() };

    // Relative displacement vector from first box to the second box
    const Vector2s p{ x1 - x0 };

    // Separating axis tests using the first box's principle axes
    {
      // Separation vector projected on the first body's frame
      const Vector2s p_in_R0{ R0.transpose() * p };

      // Width of the axis-aligned bounding box around the second body in the first body's frame, plus first body's width
      const Vector2s widths_in_R0{ Q * r1 + r0 };

      if( axisAlignedSeperatingTest( p_in_R0.x(), widths_in_R0.x(), CollidingFeature::BOX_0_AXIS_0, min_pen_depth, invert_normal, colliding_feature ) )
      {
        return;
      }
      if( axisAlignedSeperatingTest( p_in_R0.y(), widths_in_R0.y(), CollidingFeature::BOX_0_AXIS_1, min_pen_depth, invert_normal, colliding_feature ) )
      {
        return;
      }
    }

    // Separating axis tests using the second box's principle axes
    {
      // Separation vector projected on the second body's frame
      const Vector2s p_in_R1{ R1.transpose() * p };

      // Width of the axis-aligned bounding box around the first body in the second body's frame, plus second body's width
      const Vector2s widths_in_R1{ Q.transpose() * r0 + r1 };

      if( axisAlignedSeperatingTest( p_in_R1.x(), widths_in_R1.x(), CollidingFeature::BOX_1_AXIS_0, min_pen_depth, invert_normal, colliding_feature ) )
      {
        return;
      }
      if( axisAlignedSeperatingTest( p_in_R1.y(), widths_in_R1.y(), CollidingFeature::BOX_1_AXIS_1, min_pen_depth, invert_normal, colliding_feature ) )
      {
        return;
      }
    }
  }

  assert( enum_type(colliding_feature) <= 3 );
  const bool first_is_reference{ enum_type(colliding_feature) <= 1 };

  // The contact normal will lie along one of the box's principle axes
  n = first_is_reference ? R0.col( enum_type(colliding_feature) ) : R1.col( enum_type(colliding_feature) - 2 );
  if( invert_normal )
  {
    n *= -1.0;
  }
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-9 );

  // TODO: Can avoid copies here by reworking to use swaps
  // Let face 'a' be the reference face (the face the normal is perpendicular to)
  // Let face 'b' be the incident face (the closest face on the other box)
  const Matrix22sc Ra{ first_is_reference ? R0 : R1 };
  const Matrix22sc Rb{ first_is_reference ? R1 : R0 };
  const Vector2s xa{ first_is_reference ? x0 : x1 };
  const Vector2s xb{ first_is_reference ? x1 : x0 };
  const Vector2s ra{ first_is_reference ? r0 : r1 };
  const Vector2s rb{ first_is_reference ? r1 : r0 };
  const Vector2s normal2{ first_is_reference ? n : (-n).matrix() };

  // Normal of the reference face projected on the incident body's frame
  const Vector2s n_in_b{ Rb.transpose() * normal2 };

  // The largest magnitude compontent of n_in_b corresponds to the normal for the incident face.
  const int b_nrml_idx{ fabs( n_in_b.y() ) > fabs( n_in_b.x() ) ? 1 : 0 };
  const int b_tngt_idx{ 1 - b_nrml_idx };
  assert( b_nrml_idx + b_tngt_idx == 1 );

  // Compute the center point of the incident face relative to the reference body's center
  const Vector2s b_face_center{ xb - xa + ( n_in_b( b_nrml_idx ) < 0.0 ? 1.0 : -1.0 ) * rb( b_nrml_idx ) * Rb.col( b_nrml_idx ) };

  // Find the normal and tangent axis numbers of the reference box
  const int a_nrml_idx{ first_is_reference ? int(colliding_feature) : int(colliding_feature) - 2 };
  const int a_tngt_idx{ 1 - a_nrml_idx };
  assert( a_nrml_idx + a_tngt_idx == 1 );

  // Project the center of the incident face on the reference face
  const scalar c_on_a{ b_face_center.dot( Ra.col( a_tngt_idx ) ) };

  // Compute cos of angle between the incident face and the reference face
  const scalar costheta{ Ra.col( a_tngt_idx ).dot( Rb.col( b_tngt_idx ) ) };
  // The incident face projected on the refernce face
  Vector2s projected_extents{ c_on_a - costheta * rb( b_tngt_idx ), c_on_a + costheta * rb( b_tngt_idx ) };
  // If the incident face's projection isn't in order, swap
  if( projected_extents.x() > projected_extents.y() )
  {
    using std::swap;
    swap( projected_extents.x(), projected_extents.y() );
  }

  // Intersect the reference face (in its frame) with the projected extents of the incident face
  assert( projected_extents(0) <= projected_extents(1) );
  assert( projected_extents(1) >= -ra( a_tngt_idx ) - 1.0e-15 && projected_extents(0) <= ra( a_tngt_idx ) + 1.0e-15 );
  projected_extents(0) = std::min( projected_extents(0), ra( a_tngt_idx ) );
  projected_extents(1) = std::max( projected_extents(1), -ra( a_tngt_idx ) );
  const Vector2s intersection{ std::max( -ra( a_tngt_idx ), projected_extents(0) ), std::min( ra( a_tngt_idx ), projected_extents(1) ) };
  assert( intersection(1) >= intersection(0) );

  const int num_contacts{ intersection(0) != intersection(1) ? 2 : 1 };

  for( int cntct_idx = 0; cntct_idx < num_contacts; ++cntct_idx )
  {
    const Vector2s point{ b_face_center + ( ( intersection( cntct_idx ) - c_on_a ) / costheta ) * Rb.col( b_tngt_idx ) };
    const scalar depth{ ra( a_nrml_idx ) - normal2.dot( point ) };
    if( depth >= 0.0 )
    {
      // Offset by 1/2 depth to avoid biasing the response towards one body over another
      points.emplace_back( xa + point + 0.5 * depth * normal2 );
      assert( pointInBox( x0, R0, r0, points.back() ) || pointInBox( x1, R1, r1, points.back() ) );
    }
  }

  // Code expects the normal to point from body 1 to body 0
  n *= -1.0;
}
