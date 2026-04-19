#include "cobot_force_estimation/wrench_estimator.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/SVD>

namespace cobot_force_estimation
{

WrenchEstimator::WrenchEstimator(
  const std::string & urdf_content,
  const std::string & base_link,
  const std::string & ee_link)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_content, tree)) {
    throw std::runtime_error("Failed to construct KDL tree");
  }
  if (!tree.getChain(base_link, ee_link, chain_)) {
    throw std::runtime_error("Failed to construct KDL chain");
  }

  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  id_solver_ = std::make_unique<KDL::ChainIdSolver_RNE>(chain_, gravity_);
}

Eigen::Vector6d WrenchEstimator::estimate_wrench(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qd,
  const Eigen::VectorXd & tau_meas)
{
  size_t nj = chain_.getNrOfJoints();
  KDL::JntArray jnt_pos(nj);
  KDL::JntArray jnt_vel(nj);
  KDL::JntArray jnt_acc(nj);
  KDL::Wrenches external_wrenches(chain_.getNrOfSegments(), KDL::Wrench::Zero());

  for (size_t i = 0; i < nj; ++i) {
    jnt_pos(i) = q(i);
    jnt_vel(i) = qd(i);
    jnt_acc(i) = 0.0; // Static/Quasi-static assumption
  }

  // 1. Calculate gravity/dynamic torques (Predicted)
  KDL::JntArray jnt_torques_model(nj);
  id_solver_->CartToJnt(jnt_pos, jnt_vel, jnt_acc, external_wrenches, jnt_torques_model);

  // 2. Compute External Torque Discrepancy
  Eigen::VectorXd tau_ext = tau_meas - jnt_torques_model.data;

  // 3. Compute Jacobian
  KDL::Jacobian jac(nj);
  jac_solver_->JntToJac(jnt_pos, jac);
  Eigen::MatrixXd J = jac.data;

  // 4. Map to Cartesian Space: F = (J^T)^+ * tau_ext
  // Using SVD for robust pseudo-inverse
  Eigen::MatrixXd JT = J.transpose();
  Eigen::Matrix<double, 6, 1> wrench = JT.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(tau_ext);

  return wrench;
}

}  // namespace cobot_force_estimation
