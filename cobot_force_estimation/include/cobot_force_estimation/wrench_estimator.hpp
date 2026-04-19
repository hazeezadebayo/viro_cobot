#ifndef COBOT_FORCE_ESTIMATION__WRENCH_ESTIMATOR_HPP_
#define COBOT_FORCE_ESTIMATION__WRENCH_ESTIMATOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarray.hpp>

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
}

namespace cobot_force_estimation
{

class WrenchEstimator
{
public:
  WrenchEstimator(
    const std::string & urdf_content,
    const std::string & base_link,
    const std::string & ee_link);

  /**
   * @brief Estimate the external wrench at the end-effector.
   * @param q Current joint positions.
   * @param qd Current joint velocities.
   * @param tau_meas Measured joint torques (efforts).
   * @return Eigen::Matrix<double, 6, 1> Estimated wrench [Fx, Fy, Fz, Tx, Ty, Tz].
   */
  Eigen::Vector6d estimate_wrench(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qd,
    const Eigen::VectorXd & tau_meas);

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainIdSolver_RNE> id_solver_;

  KDL::Vector gravity_{0.0, 0.0, -9.81};
};

}  // namespace cobot_force_estimation

#endif  // COBOT_FORCE_ESTIMATION__WRENCH_ESTIMATOR_HPP_
