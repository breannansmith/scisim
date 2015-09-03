// TODO: Rework so this builds DualFrictionProblem directly
// TODO: Convert to use our internal types
// TODO: Pass sparse mass matrix directly in
// TODO: Can eliminate temporary storage like ObjA and ObjB by passing constraints directly in

#ifndef RIGID_BODY_2D_SOBOGUS_INTERFACE_H
#define RIGID_BODY_2D_SOBOGUS_INTERFACE_H

#include <memory>
#include <Eigen/Core>

namespace bogus
{

template<unsigned Dimension> struct PrimalFrictionProblem;
template<unsigned Dimension> struct DualFrictionProblem;

class RigidBody2DSobogusInterface final
{

public:

  RigidBody2DSobogusInterface();
  ~RigidBody2DSobogusInterface();

  void fromPrimal( const unsigned num_bodies, const Eigen::VectorXd& masses, const Eigen::VectorXd& f_in, const unsigned num_contacts, const Eigen::VectorXd& mu, const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>& contact_bases, const Eigen::VectorXd& w_in, const Eigen::VectorXi& obj_a, const Eigen::VectorXi& obj_b, const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& HA, const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& HB );

	double solve( Eigen::VectorXd& r, Eigen::VectorXd& v, unsigned& num_iterations, const unsigned max_threads, const double& tol, const unsigned max_iters, const unsigned eval_every, const bool use_infinity_norm );

  double evalInfNormError( const Eigen::VectorXd& r );

private:

  void computeDual();
  void reset();

  std::unique_ptr<PrimalFrictionProblem<2u>> m_primal;
  std::unique_ptr<DualFrictionProblem<2u>> m_dual;

};

}

#endif
