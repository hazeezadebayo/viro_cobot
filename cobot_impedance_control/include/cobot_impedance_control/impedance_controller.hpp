#ifndef COBOT_IMPEDANCE_CONTROL__IMPEDANCE_CONTROLLER_HPP_
#define COBOT_IMPEDANCE_CONTROL__IMPEDANCE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
}

namespace cobot_impedance_control
{

/**
 * @class ImpedanceController
 * @brief Pure math class for Cartesian Impedance Control using Jacobian Transpose.
 * 
 * Logic: tau = J^T * (K*error + D*vel_error)
 */
class ImpedanceController
{
public:
  ImpedanceController(
    const std::string & urdf_content,
    const std::string & base_link,
    const std::string & ee_link);

  /**
   * @brief Update the impedance control gains.
   */
  void set_gains(const Eigen::Vector6d & stiffness, const Eigen::Vector6d & damping);

  /**
   * @brief Compute joint torques for Cartesian impedance.
   * 
   * @param q Current joint positions.
   * @param qd Current joint velocities.
   * @param target_pos Target end-effector position [x, y, z].
   * @param target_quat Target end-effector orientation.
   * @return Eigen::VectorXd Commanded joint torques.
   */
  Eigen::VectorXd compute_torques(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qd,
    const Eigen::Vector3d & target_pos,
    const Eigen::Quaterniond & target_quat);

  /**
   * @brief Calculate the 3D rotation error vector using Quaternions (Axis-Angle).
   */
  Eigen::Vector3d calculate_orientation_error(
    const Eigen::Quaterniond & target_quat,
    const Eigen::Quaterniond & current_quat);

  /**
   * @brief Calculate singularity-robust pseudo-inverse using Damped Least Squares (DLS).
   */
  Eigen::MatrixXd damped_least_squares(const Eigen::MatrixXd & J, double lambda);

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  Eigen::DiagonalMatrix<double, 6> K_;
  Eigen::DiagonalMatrix<double, 6> D_;
  
  double lambda_max_{0.1};  // Maximum damping factor
  double epsilon_{0.01};    // Threshold for singularity proximity
};

}  // namespace cobot_impedance_control

#endif  // COBOT_IMPEDANCE_CONTROL__IMPEDANCE_CONTROLLER_HPP_
