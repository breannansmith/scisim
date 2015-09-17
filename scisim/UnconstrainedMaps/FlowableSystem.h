// FlowableSystem.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef FLOWABLE_SYSTEM_H
#define FLOWABLE_SYSTEM_H

#include "scisim/Math/MathDefines.h"

class FlowableSystem
{

public:

  virtual ~FlowableSystem() = 0;

  // TODO: Change to unsigned
  // Returns the number of degrees of freedom in this system
  virtual int nqdofs() const = 0;
  virtual int nvdofs() const = 0;
  virtual unsigned numVelDoFsPerBody() const = 0;
  virtual unsigned ambientSpaceDimensions() const = 0;
  unsigned numBodies() const;

  // True if the ith object in the simulation is 'kinematically scripted'
  // TODO: Change parameter to unsigned
  virtual bool isKinematicallyScripted( const int i ) const = 0;

  // Given positions and velocities, computes the force acting on the DoFs, overwriting the contents of F
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const scalar& t, VectorXs& F ) = 0;

  // Returns the mass matrix as a sparse matrix
  virtual const SparseMatrixsc& M() const = 0;
  // Returns the inverse mass matrix as a sparse matrix
  virtual const SparseMatrixsc& Minv() const = 0;

  // Returns the reference configuration mass matrix as a sparse matrix
  virtual const SparseMatrixsc& M0() const = 0;
  // Returns the reference configuration inverse mass matrix as a sparse matrix
  virtual const SparseMatrixsc& Minv0() const = 0;

  // For the given velocity and the system's current configuration and mass, computes the momentum
  virtual void computeMomentum( const VectorXs& v, VectorXs& p ) const = 0;
  // For the given velocity and the system's current configuration and mass, computes the angular momentum
  virtual void computeAngularMomentum( const VectorXs& v, VectorXs& L ) const = 0;

};

#endif
