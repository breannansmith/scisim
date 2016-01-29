// rigidbody3d_inertia_tests.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include <iostream>
#include <cstdlib>

#include "rigidbody3d/Geometry/RigidBodySphere.h"
#include "rigidbody3d/Geometry/RigidBodyBox.h"
//#include "rigidbody3d/Geometry/RigidBodyTriangleMesh.h"

// TODO: For meshes, check that vertices are in correct transformed positions

// SPHERE TESTS

static int testSpherePoolBreak()
{
  // Radius of 2.8575 cm
  const RigidBodySphere sphere{ 2.8575 };

  scalar M;
  Vector3s CM;
  Vector3s I;
  Matrix33sr R;
  
  // Density of 1.74040 g / cm^3
  sphere.computeMassAndInertia( 1.74040, M, CM, I, R );

  // Check the solution against hand-computed values
  if( fabs( M - 170.096900946 ) > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  if( ( I - Vector3s::Constant( 555.557315361 ) ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  // By default spheres are centered at the origin
  if( CM.lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  // By default orientation is set to the identity
  if( ( R - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  
  return EXIT_SUCCESS;
}



// BOX TESTS

// A cube with uniform inertia tensor
static int testBoxCube()
{
  // Half-width of 3
  const RigidBodyBox cube{ Vector3s::Constant( 3.0 ) };

  scalar M;
  Vector3s CM;
  Vector3s I;
  Matrix33sr R;

  // Density of 7
  cube.computeMassAndInertia( 7, M, CM, I, R );

  // Check the solution against hand-computed values
  if( fabs( M - 1512.0 ) > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  if( ( I - Vector3s::Constant( 9072.0 ) ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  // By default boxes are centered at the origin
  if( CM.lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  // By default orientation is set to the identity
  if( ( R - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// A box with non-uniform inertia tensor
static int testBoxElongated()
{
  // Half-widths of 3.5, 1.2, 8.9
  const RigidBodyBox cube{ Vector3s{ 3.5, 1.2, 8.9 } };

  scalar M;
  Vector3s CM;
  Vector3s I;
  Matrix33sr R;

  // Density of 6.65
  cube.computeMassAndInertia( 6.65, M, CM, I, R );

  // Check the solution against hand-computed values
  if( fabs( M - 1988.616 ) > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  if( ( I - Vector3s( 53460.6268, 60626.27312, 9074.71768 ) ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  // By default boxes are centered at the origin
  if( CM.lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  // By default orientation is set to the identity
  if( ( R - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Another box with non-uniform inertia tensor
static int testBoxElongatedTwo()
{
  // Half-widths of 3.5, 1.2, 8.9
  const RigidBodyBox cube{ Vector3s{ 21.0, 87.0, 1.5 } };
  
  scalar M;
  Vector3s CM;
  Vector3s I;
  Matrix33sr R;

  // Density of 3.21
  cube.computeMassAndInertia( 3.21, M, CM, I, R );

  // Check the solution against hand-computed values
  if( fabs( M - 70376.04 ) > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  if( ( I - Vector3s{ 177611530.95, 10398059.91, 187904026.80 } ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  // By default boxes are centered at the origin
  if( CM.lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }
  // By default orientation is set to the identity
  if( ( R - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}



// MESH TESTS

// Compute the inertia of a box represented as a triangle mesh
//int testMeshBox00()
//{
//  Matrix3Xsc vertices( 3, 8 );
//  vertices.col(0) = Vector3s( -3, -3, -3 );
//  vertices.col(1) = Vector3s( 3, -3, -3 );
//  vertices.col(2) = Vector3s( -3, -3, 3 );
//  vertices.col(3) = Vector3s( 3, -3, 3 );
//  vertices.col(4) = Vector3s( -3, 3, 3 );
//  vertices.col(5) = Vector3s( 3, 3, 3 );
//  vertices.col(6) = Vector3s( -3, 3, -3 );
//  vertices.col(7) = Vector3s( 3, 3, -3 );
//
//  Matrix3Xuc faces( 3, 12 );
//  faces.col(0) = Vector3u( 1, 2, 3 );
//  faces.col(1) = Vector3u( 3, 2, 4 );
//  faces.col(2) = Vector3u( 3, 4, 5 );
//  faces.col(3) = Vector3u( 5, 4, 6 );
//  faces.col(4) = Vector3u( 5, 6, 7 );
//  faces.col(5) = Vector3u( 7, 6, 8 );
//  faces.col(6) = Vector3u( 7, 8, 1 );
//  faces.col(7) = Vector3u( 1, 8, 2 );
//  faces.col(8) = Vector3u( 2, 8, 4 );
//  faces.col(9) = Vector3u( 4, 8, 6 );
//  faces.col(10) = Vector3u( 7, 1, 5 );
//  faces.col(11) = Vector3u( 5, 1, 3 );
//  faces.array() -= 1;
//
//  const RigidBodyTriangleMesh test_mesh( vertices, faces );
//
//  scalar M;
//  Vector3s CM;
//  Vector3s I;
//  Matrix33sr R;
//
//  // Density of 7
//  test_mesh.computeMassAndInertia( 7, M, CM, I, R );
//
//  // Check the solution against hand-computed values
//  if( fabs( M - 1512.0 ) > 1.0e-6 ) return EXIT_FAILURE;
//  if( ( I - Vector3s::Constant( 9072.0 ) ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  if( CM.lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  if( ( R - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//
//  return EXIT_SUCCESS;
//}

// Compute the inertia of a cube represented as a triangle mesh
//int testMeshBox01()
//{
//  Matrix3Xsc vertices( 3, 8 );
//  vertices.col(0) = Vector3s( -3.5, -1.2, -8.9 );
//  vertices.col(1) = Vector3s( 3.5, -1.2, -8.9 );
//  vertices.col(2) = Vector3s( -3.5, -1.2, 8.9 );
//  vertices.col(3) = Vector3s( 3.5, -1.2, 8.9 );
//  vertices.col(4) = Vector3s( -3.5, 1.2, 8.9 );
//  vertices.col(5) = Vector3s( 3.5, 1.2, 8.9 );
//  vertices.col(6) = Vector3s( -3.5, 1.2, -8.9 );
//  vertices.col(7) = Vector3s( 3.5, 1.2, -8.9 );
//  
//  Matrix3Xuc faces( 3, 12 );
//  faces.col(0) = Vector3u( 1, 2, 3 );
//  faces.col(1) = Vector3u( 3, 2, 4 );
//  faces.col(2) = Vector3u( 3, 4, 5 );
//  faces.col(3) = Vector3u( 5, 4, 6 );
//  faces.col(4) = Vector3u( 5, 6, 7 );
//  faces.col(5) = Vector3u( 7, 6, 8 );
//  faces.col(6) = Vector3u( 7, 8, 1 );
//  faces.col(7) = Vector3u( 1, 8, 2 );
//  faces.col(8) = Vector3u( 2, 8, 4 );
//  faces.col(9) = Vector3u( 4, 8, 6 );
//  faces.col(10) = Vector3u( 7, 1, 5 );
//  faces.col(11) = Vector3u( 5, 1, 3 );
//  faces.array() -= 1;
//  
//  const RigidBodyTriangleMesh test_mesh( vertices, faces );
//  
//  scalar M;
//  Vector3s CM;
//  Vector3s I;
//  Matrix33sr R;
//
//  // Density of 6.65
//  test_mesh.computeMassAndInertia( 6.65, M, CM, I, R );
//
//  // Check the solution against hand-computed values
//  if( fabs( M - 1988.616 ) > 1.0e-6 ) return EXIT_FAILURE;
//  if( ( I - Vector3s( 9074.71768, 53460.6268, 60626.27312 ) ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  if( CM.lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  if( ( R * R.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  if( fabs( R.determinant() - 1.0 ) > 1.0e-6 ) return EXIT_FAILURE;
//  // TODO: Work out by hand this rotation to verify
//  //if( ( R - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//
//  return EXIT_SUCCESS;
//}

// Compute the inertia of a cube represented as a triangle mesh
//int testMeshBox02()
//{
//  RigidBodyTriangleMesh test_mesh;
//  const bool loaded = test_mesh.loadMesh( "assets/Meshes/box02/box02.obj" );
//  if( !loaded ) return EXIT_FAILURE;
//
//  scalar M;
//  Vector3s CM;
//  Vector3s I;
//  Matrix33sr R;
//  
//  // Density of 3.21
//  test_mesh.computeMassAndInertia( 3.21, M, CM, I, R );
//
//  // Check the solution against hand-computed values
//  if( fabs( M - 70376.04 ) > 1.0e-6 ) return EXIT_FAILURE;
//  if( ( I - Vector3s( 10398059.91, 177611530.95, 187904026.80 ) ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  if( ( CM - Vector3s( -5, 12, 2.9 ) ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  if( ( R * R.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  if( fabs( R.determinant() - 1.0 ) > 1.0e-6 ) return EXIT_FAILURE;
//  // TODO: Work out the additional rotations by hand so I can compute this
//  {
//    Matrix33sr expected_R;
//    expected_R << -0.4068210695593360522186582266554141203149340599072966542997045421861826909158628581746318955428833392239840145755165169620461644017492134069900405538017400927950984934687795927698161676352099080568140283897255117434353578683011785882988375252577675946665400,0.9036968026385374602783894169561041295069821456748330203455683203076147843584747379074000884088336992149645033161985623324046172127292625598656096079486170485711151683408665532766027528289694911514303021853802514957670744682665433508576293863226154398159305,0.1335241785725796227793735357522647128996767438141231287971440328096822926003622058800560937582613530686483306856268692342543566745697705709137262206984980014490439477376511712611297793409096914153488586603449970826995970227439693711388070824624877616551070,-0.4708287812356648288264945779852074771023870503187417420995053842503278025382092430844364282417926717614309603698857878825442589352679661269455971298557739430957002472735497555012747012489049040570259857577724017285561951241738730159964486093202254106877644,-0.08217005350718157862973709742724163101148773839022819561515734014321745455066373705740915041760256863383385736578193612465089569365324108230003119523210776368853730266829199443832012895016146773601079106901962441802719836023167583715295194250597507282041540,-0.8783896294166759953619895909465635797084541756997327111099400215122542372131789057330849096923326030096995583671498867106153501075248160971818865067733298234909249508700445814593049863830759465096825226319372168548701972815457415634341174981107251854428651,-0.7828262106768885699572157942086096915065693868482600461662384368956127653307604481145015840088228942489486451050960309090448940907617152202937449378377626596621663293386263679241218567110065720569553333017567717335159160388829748734305782479660728053473562,-0.4202144347919418989936949259661263775053311103685411236282487866596067082476175787358060708087904793509010704717884366389657247248590732015781509275051994929353198123319058118666791683571178147735304266508863324467624407507322694411494248328964449019416334,0.4589149732464092106851314512863791844942561308048859021924213299283912727246681314712954247911987156830830713171090319376745521531733794588499844023839461181557313486658540027808399355249192661319946044654901877909864008198841620814235240287470124635897923;
//    //if( ( R - expected_R ).lpNorm<Eigen::Infinity>() > 1.0e-6 ) return EXIT_FAILURE;
//  }
//  
//  return EXIT_SUCCESS;
//}

int main( int argc, char** argv )
{
  if( argc != 3 )
  {
    std::cerr << "Usage: " << argv[0] << " body_type test_name" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string body_type{ argv[1] };
  const std::string test_name{ argv[2] };

  if( body_type == "sphere" )
  {
    if( test_name == "pool_ball" )
    {
      return testSpherePoolBreak();
    }
  }
  else if( body_type == "box" )
  {
    if( test_name == "cube" )
    {
      return testBoxCube();
    }
    else if( test_name == "elongated" )
    {
      return testBoxElongated();
    }
    else if( test_name == "elongated_two" )
    {
      return testBoxElongatedTwo();
    }
  }
//  else if( body_type == "mesh" )
//  {
//    if( test_name == "box00" )
//    {
//      return testMeshBox00();
//    }
//    else if( test_name == "box01" )
//    {
//      return testMeshBox01();
//    }
//    else if( test_name == "box02" )
//    {
//      return testMeshBox02();
//    }
//  }

  std::cerr << "Invalid object type or test name specified" << std::endl;
  return EXIT_FAILURE;
}
