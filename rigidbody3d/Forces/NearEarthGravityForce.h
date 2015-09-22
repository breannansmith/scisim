// NearEarthGravityForce.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef NEAR_EARTH_GRAVITY_FORCE
#define NEAR_EARTH_GRAVITY_FORCE

#include "Force.h"

class NearEarthGravityForce final : public Force
{

public:

  explicit NearEarthGravityForce( const Vector3s& g );
  explicit NearEarthGravityForce( std::istream& input_stream );

  virtual ~NearEarthGravityForce() override;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M ) const override;
  
  // result += Force
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, VectorXs& result ) const override;
  
  virtual std::string name() const override;

  virtual std::unique_ptr<Force> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  void setForce( const Vector3s& g );

private:

  Vector3s m_g;

};

#endif
