#pragma once

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <vector>
#include <osqp/osqp.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace sco
{

#ifdef TRAJOPT_OSQP_V1
using c_int = OSQPInt;
using c_float = OSQPFloat;
using csc = OSQPCscMatrix;
using OSQPSolverHandle = OSQPSolver*;
inline csc* make_csc(c_int nr, c_int nc, c_int nz, c_float* x, c_int* i, c_int* p)
{
  return OSQPCscMatrix_new(nr, nc, nz, x, i, p);
}
inline void free_csc(csc* M)
{
  if (M)
    OSQPCscMatrix_free(M);
}
#else
using c_int = ::c_int;
using c_float = ::c_float;
using csc = ::csc;
using OSQPSolverHandle = ::OSQPWorkspace*;
inline csc* make_csc(c_int nr, c_int nc, c_int nz, c_float* x, c_int* i, c_int* p)
{
  return csc_matrix(nr, nc, nz, x, i, p);
}
inline void free_csc(csc* M)
{
  if (M)
    free(M);
}
#endif

}  // namespace sco

#include <trajopt_sco/solver_interface.hpp>

namespace sco
{

/** @brief The OSQP configuration settings */
struct OSQPModelConfig : public ModelConfig
{
  using Ptr = std::shared_ptr<OSQPModelConfig>;
  using ConstPtr = std::shared_ptr<const OSQPModelConfig>;

  OSQPModelConfig();

  OSQPSettings settings{};

  /**
   * @brief Update the OSQP workspace for subsequent optimizations, instead of recreating it each time.
   */
  bool update_workspace{ false };

  /**
   * @brief Set the default OSQP Settings
   * @param settings The object to apply default settings to
   */
  static void setDefaultOSQPSettings(OSQPSettings& settings);
};

/**
 * OSQPModel uses the BSD solver OSQP to solve a linearly constrained QP.
 * OSQP solves a problem in the form:
 * ```
 * min   1/2*x'Px + q'x
 * s.t.  l <= Ax <= u
 * ```
 *
 * More informations about the solver are available at:
 * https://osqp.org/docs/
 */
class OSQPModel : public Model
{
  OSQPData osqp_data_{};
  std::unique_ptr<csc, decltype(&free_csc)> P_{ nullptr, &free_csc };
  std::unique_ptr<csc, decltype(&free_csc)> A_{ nullptr, &free_csc };
  std::vector<c_int> P_row_indices_;
  std::vector<c_int> P_column_pointers_;
  std::vector<c_int> A_row_indices_;
  std::vector<c_int> A_column_pointers_;
  OSQPSolverHandle osqp_workspace_{ nullptr };

  DblVec P_csc_data_;
  Eigen::VectorXd q_;

  DblVec A_csc_data_;
  DblVec l_, u_;

  DblVec solution_;
  std::vector<Var> vars_;
  std::vector<Cnt> cnts_;
  DblVec lbs_, ubs_;
  std::vector<AffExpr> cnt_exprs_;
  ConstraintTypeVector cnt_types_;

  QuadExpr objective_;

  OSQPModelConfig config_;

  std::mutex mutex_;

  bool updateObjective(bool check_sparsity);
  bool updateConstraints(bool check_sparsity);
  void createOrUpdateSolver();

public:
  OSQPModel(const ModelConfig::ConstPtr& config = nullptr);
  ~OSQPModel() override;
  OSQPModel(const OSQPModel& model) = delete;
  OSQPModel& operator=(const OSQPModel& model) = delete;
  OSQPModel(OSQPModel&&) = delete;
  OSQPModel& operator=(OSQPModel&&) = delete;

  Var addVar(const std::string& name) override;
  Cnt addEqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const QuadExpr&, const std::string& name) override;
  void removeVars(const VarVector& vars) override;
  void removeCnts(const CntVector& cnts) override;

  void update() override;
  CvxOptStatus optimize() override;
  void setObjective(const AffExpr&) override;
  void setObjective(const QuadExpr&) override;
  void setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper) override;
  DblVec getVarValues(const VarVector& vars) const override;
  void writeToFile(const std::string& fname) const override;
  VarVector getVars() const override;
};

}  // namespace sco

