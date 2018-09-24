// N.B. this solver can give quite large residuals in the friction disk constraint.
// For the BallPlane01.xml exmaple, the residual when two contacts are active is 1e-4.

#ifndef SMOOTH_MDP_OPERATOR_IPOPT
#define SMOOTH_MDP_OPERATOR_IPOPT

#include "FrictionOperator.h"

#include "IpIpoptApplication.hpp"

class QPTerminationOperator;

class SmoothMDPOperatorIpopt final : public FrictionOperator
{

public:

  SmoothMDPOperatorIpopt( const std::vector<std::string>& linear_solvers, const scalar& tol );
  explicit SmoothMDPOperatorIpopt( std::istream& input_stream );

  virtual ~SmoothMDPOperatorIpopt() override = default;

  virtual void flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda ) override;

  virtual int numFrictionImpulsesPerNormal() const override;

  virtual std::string name() const override;

  virtual std::unique_ptr<FrictionOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual bool isLinearized() const override;

  void solveQP( const QPTerminationOperator& termination_operator, const SparseMatrixsc& Minv, const SparseMatrixsc& D, const VectorXs& b, const VectorXs& c, VectorXs& beta, VectorXs& lambda, scalar& achieved_tol ) const;

private:

  const std::vector<std::string> m_linear_solver_order;
  const scalar m_tol;

};

class SmoothMDPNLP final : public Ipopt::TNLP
{
public:

  SmoothMDPNLP( const SparseMatrixsc& Q, VectorXs& beta, const bool use_custom_termination, const QPTerminationOperator& termination_operator );

  virtual ~SmoothMDPNLP() override;

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

  inline VectorXs& A()
  {
    return m_A;
  }

  inline VectorXs& C()
  {
    return m_C;
  }
  
  inline VectorXs& beta()
  {
    return m_beta;
  }
  
  Ipopt::SolverReturn getReturnStatus() const;

  inline const scalar& achievedTolerance() const
  {
    return m_achieved_tolerance;
  }

private:

  const SparseMatrixsc& m_Q;
  VectorXs m_A;
  VectorXs m_C;
  VectorXs& m_beta;

  // Indices of the diagonals in the sparse rep
  std::vector<int> m_diagonal_indices;

  // Return status from the last solve
  Ipopt::SolverReturn m_solve_return_status;

  const bool m_use_custom_termination;
  const QPTerminationOperator& m_termination_operator;
  scalar m_achieved_tolerance;

  SmoothMDPNLP( const SmoothMDPNLP& );
  SmoothMDPNLP& operator=( const SmoothMDPNLP& );

};

#endif
