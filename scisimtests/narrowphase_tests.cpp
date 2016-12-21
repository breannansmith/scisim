#include <iostream>

#include "scisim/Math/MathDefines.h"
#include "scisim/CollisionDetection/CollisionDetectionUtilities.h"

// No roots
static int executeCCDTest00()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 1.0, 1.0 };
  constexpr scalar ra{ 0.7 };
  const Vector2s q0b{ 3.0, 0.0 };
  const Vector2s q1b{ 2.1, -1.0 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ 7.56, -11.4, 7.61 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( collision_happens.first )
  {
    std::cerr << "Collision incorrectly identified." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Roots in the past
static int executeCCDTest01()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 0.5, 0.5 };
  constexpr scalar ra{ 0.7 };
  const Vector2s q0b{ 1.4, -0.4 };
  const Vector2s q1b{ 4.0, -1.0 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ 0.68, 6.76, 5.62 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( collision_happens.first )
  {
    std::cerr << "Collision incorrectly identified." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Roots in the future
static int executeCCDTest02()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 0.5, 0.5 };
  constexpr scalar ra{ 0.7 };
  const Vector2s q0b{ 4.0, -1.0 };
  const Vector2s q1b{ 1.4, -0.4 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ 15.56, -25.0, 9.62 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( collision_happens.first )
  {
    std::cerr << "Collision incorrectly identified." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Starts colliding and is separating case
static int executeCCDTest03()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 0.5, 0.5 };
  constexpr scalar ra{ 0.7 };
  const Vector2s q0b{ 1.0, 0.0 };
  const Vector2s q1b{ 4.0, -1.0 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ -0.44, 5.0, 8.5 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( !collision_happens.first )
  {
    std::cerr << "Collision incorrectly missed." << std::endl;
    return EXIT_FAILURE;
  }

  if( collision_happens.second != 0.0 )
  {
    std::cerr << "Collision time computed incorrectly." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Starts separated but collides case
static int executeCCDTest04()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 0.5, 0.5 };
  constexpr scalar ra{ 0.7 };
  const Vector2s q0b{ 4.0, 0.0 };
  const Vector2s q1b{ 1.4, 1.2 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ 14.56, -24.8, 10.1 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( !collision_happens.first )
  {
    std::cerr << "Collision incorrectly missed." << std::endl;
    return EXIT_FAILURE;
  }

  using std::sqrt;
  if( fabs( collision_happens.second - ( 2.0 / 505.0 ) * ( 310.0 - sqrt(4190.0) ) ) > 1.0e-9 )
  {
    std::cerr << "Collision time computed incorrectly." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// The tunneling case
static int executeCCDTest05()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 0.5, 0.5 };
  constexpr scalar ra{ 0.7 };
  const Vector2s q0b{ 4.0, 0.0 };
  const Vector2s q1b{ -1.0, 1.2 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ 14.56, -44.0, 30.74 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( !collision_happens.first )
  {
    std::cerr << "Collision incorrectly missed." << std::endl;
    return EXIT_FAILURE;
  }

  using std::sqrt;
  if( fabs( collision_happens.second - ( ( 2.0 * ( 550.0 - sqrt( 22766.0 ) ) ) / 1537.0 ) ) > 1.0e-9 )
  {
    std::cerr << "Collision time computed incorrectly." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Glancing contact
static int executeCCDTest06()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 0.0, 0.0 };
  constexpr scalar ra{ 0.5 };
  const Vector2s q0b{ 2.0, 1.0 };
  const Vector2s q1b{ -2.0, 1.0 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ 4.0, -16.0, 16.0 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( !collision_happens.first )
  {
    std::cerr << "Collision incorrectly missed." << std::endl;
    return EXIT_FAILURE;
  }

  if( fabs( collision_happens.second - 0.5 ) > 1.0e-9 )
  {
    std::cerr << "Collision time computed incorrectly." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Not contact, no movement
static int executeCCDTest07()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 0.0, 0.0 };
  constexpr scalar ra{ 0.5 };
  const Vector2s q0b{ 2.0, 1.0 };
  const Vector2s q1b{ 2.0, 1.0 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ 4.0, 0.0, 0.0 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( collision_happens.first )
  {
    std::cerr << "Collision incorrectly identified." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Contact, no relative movement
static int executeCCDTest08()
{
  const Vector2s q0a{ 0.0, 0.0 };
  const Vector2s q1a{ 1.0, 0.0 };
  constexpr scalar ra{ 0.5 };
  const Vector2s q0b{ 0.5, 0.5 };
  const Vector2s q1b{ 1.5, 0.5 };
  constexpr scalar rb{ 0.5 };
  const Vector3s cexpected{ -0.5, 0.0, 0.0 };

  const Vector3s c{ CollisionDetectionUtilities::computeCCDQuadraticCoeffs( q0a, q1a, ra, q0b, q1b, rb ) };

  using std::fabs;
  if( fabs( c(0) - cexpected(0) ) > 1.0e-9 )
  {
    std::cerr << "Constant coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(1) - cexpected(1) ) > 1.0e-9 )
  {
    std::cerr << "Linear coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }
  if( fabs( c(2) - cexpected(2) ) > 1.0e-9 )
  {
    std::cerr << "Quadratic coefficient is incorrect." << std::endl;
    return EXIT_FAILURE;
  }

  const std::pair<bool,scalar> collision_happens{ CollisionDetectionUtilities::ballBallCCDCollisionHappens( c ) };

  if( !collision_happens.first )
  {
    std::cerr << "Collision incorrectly missed." << std::endl;
    return EXIT_FAILURE;
  }

  if( fabs( collision_happens.second ) > 1.0e-9 )
  {
    std::cerr << "Collision time computed incorrectly." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

int main( int argc, char** argv )
{
  if( argc != 2 )
  {
    std::cerr << "Usage: " << argv[0] << " test_name" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string test_name{ argv[1] };

  if( test_name == "ball_ball_ccd_00" )
  {
    return executeCCDTest00();
  }
  else if( test_name == "ball_ball_ccd_01" )
  {
    return executeCCDTest01();
  }
  else if( test_name == "ball_ball_ccd_02" )
  {
    return executeCCDTest02();
  }
  else if( test_name == "ball_ball_ccd_03" )
  {
    return executeCCDTest03();
  }
  else if( test_name == "ball_ball_ccd_04" )
  {
    return executeCCDTest04();
  }
  else if( test_name == "ball_ball_ccd_05" )
  {
    return executeCCDTest05();
  }
  else if( test_name == "ball_ball_ccd_06" )
  {
    return executeCCDTest06();
  }
  else if( test_name == "ball_ball_ccd_07" )
  {
    return executeCCDTest07();
  }
  else if( test_name == "ball_ball_ccd_08" )
  {
    return executeCCDTest08();
  }

  std::cerr << "Invalid test specified: " << argv[1] << std::endl;
  return EXIT_FAILURE;
}
