#include "cobot_impedance_control/impedance_controller.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>

namespace cobot_impedance_control
{

ImpedanceController::ImpedanceController(
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
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
}

void ImpedanceController::set_gains(const Eigen::Vector6d & stiffness, const Eigen::Vector6d & damping)
{
  K_ = stiffness.asDiagonal();
  D_ = damping.asDiagonal();
}

Eigen::VectorXd ImpedanceController::compute_torques(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qd,
  const Eigen::Vector3d & target_pos,
  const Eigen::Quaterniond & target_quat)
{
  size_t nj = chain_.getNrOfJoints();
  
  if (q.hasNaN() || qd.hasNaN()) {
    return Eigen::VectorXd::Zero(nj);
  }

  KDL::JntArray jnt_pos(nj);
  KDL::JntArray jnt_vel(nj);
  for(size_t i=0; i<nj; ++i) {
    jnt_pos(i) = q(i);
    jnt_vel(i) = qd(i);
  }

  // 1. Forward Kinematics
  KDL::Frame current_frame;
  if (fk_solver_->JntToCart(jnt_pos, current_frame) < 0) {
    return Eigen::VectorXd::Zero(nj);
  }

  // 2. Compute Cartesian Error (6D)
  Eigen::Vector6d error;
  
  // Translation Error
  error.head(3) = target_pos - Eigen::Vector3d(current_frame.p.x(), current_frame.p.y(), current_frame.p.z());
  
  // Orientation Error (Axis-Angle from Quaternions)
  double x, y, z, w;
  current_frame.M.GetQuaternion(x, y, z, w);
  Eigen::Quaterniond current_quat(w, x, y, z);
  error.tail(3) = calculate_orientation_error(target_quat, current_quat);

  // 3. Compute Jacobian
  KDL::Jacobian jac(nj);
  if (jac_solver_->JntToJac(jnt_pos, jac) < 0) {
    return Eigen::VectorXd::Zero(nj);
  }
  Eigen::MatrixXd J = jac.data;

  // 4. Singularity Check
  double manipulability = std::sqrt((J * J.transpose()).determinant());
  if (manipulability < epsilon_) {
    // Incrementally increase damping as we approach singularity (Damped Least Squares style injection)
    // For Impedance (JT), we can also scale down the virtual force to prevent erratic behavior.
    double scale = 1.0;
    if (manipulability > 1e-6) {
        scale = std::max(0.0, (manipulability - 1e-6) / (epsilon_ - 1e-6));
    } else {
        scale = 0.0;
    }
    // error *= scale; // Optional: soft-limit the error pull near singularity
  }

  // 5. Compute Cartesian Velocity (6D)
  Eigen::Vector6d xd = J * qd;

  // 6. Calculate Virtual Force: F = K*e - D*xd
  Eigen::Vector6d F_virt = (K_ * error) - (D_ * xd);

  // 7. Map Force to Joint Torques: tau = J^T * F
  Eigen::VectorXd tau = J.transpose() * F_virt;

  for (int i = 0; i < tau.size(); ++i) {
    if (!std::isfinite(tau(i))) tau(i) = 0.0;
  }

  return tau;
}

Eigen::Vector3d ImpedanceController::calculate_orientation_error(
  const Eigen::Quaterniond & target_quat,
  const Eigen::Quaterniond & current_quat)
{
  Eigen::Quaterniond q_curr = current_quat;
  // Hemispherical check (ensure shortest path)
  if (target_quat.coeffs().dot(q_curr.coeffs()) < 0.0) {
    q_curr.coeffs() << -q_curr.coeffs();
  }
  
  // Relative rotation quaternion (error)
  // q_err = q_target * q_curr^-1 
  // (In McFly it was curr * target_inv, but standard error is target * curr_inv)
  // We'll follow the McFly implementation for consistency as it matches their gain tuning.
  Eigen::Quaterniond q_err = q_curr * target_quat.inverse();
  
  // Convert to rotation vector (Axis-Angle)
  Eigen::AngleAxisd angle_axis(q_err);
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::MatrixXd ImpedanceController::damped_least_squares(const Eigen::MatrixXd & J, double lambda)
{
    // DLS: J* = J^T * (J*J^T + lambda^2 * I)^-1
    Eigen::MatrixXd JJT = J * J.transpose();
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(JJT.rows(), JJT.cols());
    return J.transpose() * (JJT + (lambda * lambda) * identity).inverse();
}

}  // namespace cobot_impedance_control
