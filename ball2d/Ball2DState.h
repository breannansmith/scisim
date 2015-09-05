// Ball2DState.h
//
// Breannan Smith
// Last updated: 09/05/2015

#ifndef BALL_2D_STATE_H
#define BALL_2D_STATE_H

#include <memory>

// TODO: Forward declare these, if possible
#include "SCISim/Math/MathDefines.h"
#include "Forces/Ball2DForce.h"

class StaticDrum;
class StaticPlane;
class PlanarPortal;
class Ball2DForce;

class Ball2DState final
{

public:

  friend void swap( Ball2DState& first, Ball2DState& second );

  Ball2DState& operator=( const Ball2DState& other );

  ~Ball2DState();

  void setState( const VectorXs& q, const VectorXs& v, const VectorXs& m, const VectorXs& r, const std::vector<bool>& fixed, const std::vector<StaticDrum>& drums, const std::vector<StaticPlane>& planes, const std::vector<PlanarPortal>& planar_portals, const std::vector<std::unique_ptr<Ball2DForce>>& forces );

  // Configuration, velocity, mass, and geometry of the system
  unsigned nballs() const;
  VectorXs& q();
  VectorXs& v();
  const VectorXs& q() const;
  const VectorXs& v() const;
  const VectorXs& r() const;
  const SparseMatrixsc& M() const;
  const SparseMatrixsc& Minv() const;

  // Kinematic boundary conditions
  std::vector<StaticPlane>& staticPlanes();
  const std::vector<StaticDrum>& staticDrums() const;
  const std::vector<StaticPlane>& staticPlanes() const;
  std::vector<PlanarPortal>& planarPortals();
  const std::vector<PlanarPortal>& planarPortals() const;

  // Energy, momentum, etc computations
  scalar computeKineticEnergy() const;
  scalar computePotentialEnergy() const;
  scalar computeTotalEnergy() const;
  Vector2s computeMomentum() const;
  scalar computeAngularMomentum() const;

  // Adds force into the given vector
  void accumulateForce( const VectorXs& q, const VectorXs& v, VectorXs& force_accm ) const;

  // Computes a bounding box around the system
  Vector4s computeBoundingBox() const;

  // Serialization and deserialization
  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

  // Inserts a new ball after all current balls
  // NOTE: This is currently quite slow...
  void pushBallBack( const Vector2s& q, const Vector2s& v, const scalar& r, const scalar& m, const bool fixed );

private:

  VectorXs m_q;
  VectorXs m_v;
  VectorXs m_r;
  std::vector<bool> m_fixed;
  SparseMatrixsc m_M;
  SparseMatrixsc m_Minv;

  std::vector<StaticDrum> m_static_drums;
  std::vector<StaticPlane> m_static_planes;

  std::vector<PlanarPortal> m_planar_portals;

  std::vector<std::unique_ptr<Ball2DForce>> m_forces;

};

#endif
