// LCPOperatorIpopt.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef LCP_OPERATOR_IPOPT_H
#define LCP_OPERATOR_IPOPT_H

#include "ImpactOperator.h"

#ifdef IPOPT_FOUND
#include "IpIpoptApplication.hpp"
#endif

#include "scisim/ConstrainedMaps/QPTerminationOperator.h"

class LCPOperatorIpopt final : public ImpactOperator
{

public:

  LCPOperatorIpopt( const std::vector<std::string>& linear_solvers, const scalar& tol );
  explicit LCPOperatorIpopt( std::istream& input_stream );
  LCPOperatorIpopt( const LCPOperatorIpopt& other );
  virtual ~LCPOperatorIpopt() override = default;

  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<ImpactOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  #ifndef IPOPT_FOUND
  [[noreturn]]
  #endif
  void solveQP( const QPTerminationOperator& termination_operator, const SparseMatrixsc& Minv, const SparseMatrixsc& N, const VectorXs& b, VectorXs& alpha, scalar& achieved_tol ) const;

private:

  const std::vector<std::string> m_linear_solver_order;
  const scalar m_tol;

};

#ifdef IPOPT_FOUND
class QPNLP final : public Ipopt::TNLP
{

public:

  QPNLP( const SparseMatrixsc& Q, const bool use_custom_termination, const QPTerminationOperator& termination_operator );

  virtual ~QPNLP() override;

  // Method to return some info about the nlp
  virtual bool get_nlp_info( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style ) override;

  // Method to return the bounds for the nlp 
  virtual bool get_bounds_info( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u ) override;

  // Method to return the starting point for the algorithm 
  virtual bool get_starting_point( Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda ) override;

  // Method to return the objective value
  virtual bool eval_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value ) override;

  // Method to return the gradient of the objective
  virtual bool eval_grad_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f ) override;

  // Method to return the constraint residuals
  virtual bool eval_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g ) override;

  // Method to return the jacobian of the constraint residuals
  virtual bool eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) override;

  virtual bool eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) override;

  // This method is called when the algorithm is complete so the TNLP can store/write the solution
  virtual void finalize_solution( Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq ) override;

  virtual bool intermediate_callback( Ipopt::AlgorithmMode mode, Ipopt::Index iter, Ipopt::Number obj_value, Ipopt::Number inf_pr, Ipopt::Number inf_du, Ipopt::Number mu, Ipopt::Number d_norm, Ipopt::Number regularization_size, Ipopt::Number alpha_du, Ipopt::Number alpha_pr, Ipopt::Index ls_trials, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq ) override;

  inline const SparseMatrixsc& Q() const
  {
    return m_Q;
  }

  inline const VectorXs& A() const
  {
    return m_A;
  }

  inline VectorXs& A()
  {
    return m_A;
  }

  void setAlpha( VectorXs* alpha );

  inline const VectorXs& alpha() const
  {
    return *m_alpha;
  }

  Ipopt::SolverReturn getReturnStatus() const;

  inline const scalar& achievedTolerance() const
  {
    return m_achieved_tolerance;
  }

private:

  const SparseMatrixsc& m_Q;
  VectorXs m_A;
  VectorXs* m_alpha;

  // Return status from the last solve
  Ipopt::SolverReturn m_solve_return_status;

  const bool m_use_custom_termination;
  const QPTerminationOperator& m_termination_operator;
  scalar m_achieved_tolerance;

  QPNLP( const QPNLP& );
  QPNLP& operator=( const QPNLP& );

};
#endif

#endif
