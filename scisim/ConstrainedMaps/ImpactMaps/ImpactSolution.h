// ImpactSolution.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef IMPACT_SOLUTION_H
#define IMPACT_SOLUTION_H

#include "scisim/Math/MathDefines.h"

#include <memory>

class Constraint;
class HDF5File;

class ImpactSolution final
{

public:

  ImpactSolution();

  void setSolution( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& impact_bases, const VectorXs& alpha, const scalar& dt );

  void writeSolution( HDF5File& output_file );

private:

  Matrix2Xic m_indices;
  MatrixXXsc m_points;
  MatrixXXsc m_normals;
  MatrixXXsc m_forces;
  scalar m_dt;

};

#endif
