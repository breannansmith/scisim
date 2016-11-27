// MathDefines.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef MATH_DEFINES_H
#define MATH_DEFINES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/StdVector>

#include <climits>

using scalar = double;

template<typename T>
constexpr T PI = T(3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811174502841027019385211055596446229489549303819644288109756659334461284756482337867831652712019091456485669234603486104543266482133936072602491412737245870066063155881748815209209628292540917153643678925903600113305305488204665213841469519415116094330572703657595919530921861173819326117931051185480744623799627495673518857527248912279381830119491298336733624);

// Special scalar values (infinity, not-a-number, etc)
static_assert( std::numeric_limits<scalar>::has_infinity, "Error, scalar type does not have an infinity value." );
constexpr scalar SCALAR_INFINITY = std::numeric_limits<scalar>::infinity();

static_assert( std::numeric_limits<scalar>::has_signaling_NaN, "Error, scalar type does not have a signaling NaN value." );
constexpr scalar SCALAR_NAN = std::numeric_limits<scalar>::signaling_NaN();

// TODO: Some of these are incosistent with others (e.g. MatrixXs vs Matrix3Xic), fix that

// Define integer-valued version of Eigen types
using Vector3i = Eigen::Vector3i;
using Vector1u = Eigen::Matrix<unsigned,1,1>;
using Vector2u = Eigen::Matrix<unsigned,2,1>;
using Vector3u = Eigen::Matrix<unsigned,3,1>;
using Vector4u = Eigen::Matrix<unsigned,4,1>;
using VectorXi = Eigen::VectorXi;
using VectorXu = Eigen::Matrix<unsigned,Eigen::Dynamic,1>;
using MatrixXXic = Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>;
using MatrixX3ir = Eigen::Matrix<int,Eigen::Dynamic,3,Eigen::RowMajor>;
using Matrix2Xic = Eigen::Matrix<int,2,Eigen::Dynamic,Eigen::ColMajor>;
using Matrix3Xic = Eigen::Matrix<int,3,Eigen::Dynamic,Eigen::ColMajor>;
using Matrix3Xuc = Eigen::Matrix<unsigned,3,Eigen::Dynamic,Eigen::ColMajor>;

using Array2u = Eigen::Array<unsigned,2,1>;
using Array3u = Eigen::Array<unsigned,3,1>;
using Array3i = Eigen::Array<int,3,1>;

using Matrix2s = Eigen::Matrix<scalar,2,2>;
using Matrix3s = Eigen::Matrix<scalar,3,3>;
using Matrix4s = Eigen::Matrix<scalar,4,4>;
using MatrixXs = Eigen::Matrix<scalar,Eigen::Dynamic,Eigen::Dynamic>;
using MatrixXXfc = Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>;

// Define scalar-valued versions of the Eigen vector types
using Vector2s = Eigen::Matrix<scalar,2,1>;
using Vector3s = Eigen::Matrix<scalar,3,1>;
using Vector4s = Eigen::Matrix<scalar,4,1>;
using Vector6s = Eigen::Matrix<scalar,6,1>;
using VectorXs = Eigen::Matrix<scalar,Eigen::Dynamic,1>;

using Array2s = Eigen::Array<scalar,2,1>;
using Array3s = Eigen::Array<scalar,3,1>;
using Array4s = Eigen::Array<scalar,4,1>;
using ArrayXs = Eigen::Array<scalar,Eigen::Dynamic,1>;


using Quaternions = Eigen::Quaternion<scalar>;

// TODO: Clean this up :)
// Define scalar-valued versions of the Eigen matrix types. r postfix denotes row-major, c postfix denotes column-major.
using Matrix22sr = Eigen::Matrix<scalar,2,2,Eigen::RowMajor>;
using Matrix22sc = Eigen::Matrix<scalar,2,2,Eigen::ColMajor>;
using Matrix33sr = Eigen::Matrix<scalar,3,3,Eigen::RowMajor>;
using Matrix33sc = Eigen::Matrix<scalar,3,3,Eigen::ColMajor>;
using Matrix66sc = Eigen::Matrix<scalar,6,6,Eigen::ColMajor>;
using Matrix2Xsc = Eigen::Matrix<scalar,2,Eigen::Dynamic,Eigen::ColMajor>;
using Matrix3Xsc = Eigen::Matrix<scalar,3,Eigen::Dynamic,Eigen::ColMajor>;
using MatrixX2sr = Eigen::Matrix<scalar,Eigen::Dynamic,2,Eigen::RowMajor>;
using MatrixXXsr = Eigen::Matrix<scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
using MatrixXXsc = Eigen::Matrix<scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>;

// Define scalar-valued versions of the Eigen sparse matrix types
using SparseMatrixsc = Eigen::SparseMatrix<scalar,Eigen::ColMajor>;
using SparseMatrixsr = Eigen::SparseMatrix<scalar,Eigen::RowMajor>;

// Define some transformation geometry
using AnglesAxis3s = Eigen::AngleAxis<scalar>;

// Permutation matrix
using PermutationMatrix = Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic,int>;

#endif
