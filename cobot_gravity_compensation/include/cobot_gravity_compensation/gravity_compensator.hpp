#ifndef COBOT_GRAVITY_COMPENSATION__GRAVITY_COMPENSATOR_HPP_
#define COBOT_GRAVITY_COMPENSATION__GRAVITY_COMPENSATOR_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarray.hpp>

namespace cobot_gravity_compensation
{

/**
 * @struct FrictionParams
 * @brief Holds parameters for the Stribeck friction model.
 * 
 * Logic: Tf = f_v*qd + [f_c + (f_s - f_c) * exp(-|qd/v_s|^step)] * sgn(qd)
 * Simplified educational version: Tf = f_v*qd + f_s*sgn(qd)*exp(-f2*|qd|) + f_c*sgn(qd)*(1 - exp(-f1*|qd|))
 */
struct FrictionParams
{
  double f_v{0.0};  // Viscous friction coefficient
  double f_c{0.0};  // Coulomb friction coefficient
  double f_s{0.0};  // Static (Stribeck) friction coefficient
  double f_1{1.0};  // Decay rate for Coulomb transition
  double f_2{1.0};  // Decay rate for Stribeck effect
};

/**
 * @class GravityCompensator
 * @brief Pure mathematical logic for computing gravity and friction compensation torques.
 * 
 * This class is decoupled from ROS 2 to make it easily testable and readable for students.
 */
class GravityCompensator
{
public:
  /**
   * @brief Constructor
   * @param urdf_content String containing the robot URDF.
   * @param base_link Name of the base link.
   * @param ee_link Name of the end-effector link.
   * @param gravity_vector Vector representing gravity (usually [0, 0, -9.81]).
   */
  GravityCompensator(
    const std::string & urdf_content,
    const std::string & base_link,
    const std::string & ee_link,
    const Eigen::Vector3d & gravity_vector = Eigen::Vector3d(0, 0, -9.81));

  /**
   * @brief Set friction parameters for each joint.
   * @param params Map of joint names to their friction parameters.
   */
  void set_friction_params(const std::map<std::string, FrictionParams> & params);

  /**
   * @brief Compute the required torques to compensate for gravity and friction.
   * 
   * Mathematically: tau = G(q) + Tau_friction(qd)
   * 
   * @param q Current joint positions.
   * @param qd Current joint velocities.
   * @return Eigen::VectorXd Vector of compensation torques.
   */
  Eigen::VectorXd compute_compensation(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qd);

  /**
   * @brief Compute compensation torques including an external wrench at the end-effector.
   * 
   * @param q Current joint positions.
   * @param qd Current joint velocities.
   * @param wrench External wrench [Fx, Fy, Fz, Tx, Ty, Tz] at the end-effector.
   * @param gain Scaling factor for the assistance/interaction force.
   * @return Eigen::VectorXd Vector of compensation + interaction torques.
   */
  Eigen::VectorXd compute_compensation_with_wrench(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qd,
    const Eigen::VectorXd & wrench,
    double gain = 1.0);

  size_t get_num_joints() const { return chain_.getNrOfJoints(); }

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIdSolver_RNE> id_solver_;
  std::map<int, FrictionParams> joint_friction_map_;
  KDL::Vector gravity_vec_;

  /**
   * @brief Internal friction model implementation.
   * @param velocity Joint velocity.
   * @param p Friction parameters for the joint.
   * @return Computed friction torque.
   */
  double calculate_friction(double velocity, const FrictionParams & p);
};

}  // namespace cobot_gravity_compensation

#endif  // COBOT_GRAVITY_COMPENSATION__GRAVITY_COMPENSATOR_HPP_
