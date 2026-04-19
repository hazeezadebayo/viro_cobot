#include "cobot_gravity_compensation/gravity_compensator.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>
#include <cmath>

namespace cobot_gravity_compensation
{

GravityCompensator::GravityCompensator(
  const std::string & urdf_content,
  const std::string & base_link,
  const std::string & ee_link,
  const Eigen::Vector3d & gravity_vector)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_content, tree)) {
    throw std::runtime_error("Failed to construct KDL tree from URDF");
  }

  if (!tree.getChain(base_link, ee_link, chain_)) {
    throw std::runtime_error("Failed to construct KDL chain from tree");
  }

  // Define the gravity vector for the solver
  gravity_vec_ = KDL::Vector(gravity_vector.x(), gravity_vector.y(), gravity_vector.z());
  
  // Initialize the Inverse Dynamics solver using Recursive Newton-Euler
  id_solver_ = std::make_unique<KDL::ChainIdSolver_RNE>(chain_, gravity_vec_);
}

void GravityCompensator::set_friction_params(const std::map<std::string, FrictionParams> & params)
{
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i) {
    const auto & segment = chain_.getSegment(i);
    const auto & joint = segment.getJoint();
    
    // Check if the joint name exists in the provided parameters
    if (params.count(joint.getName())) {
      joint_friction_map_[chain_.getNrOfJoints() - (chain_.getNrOfSegments() - i)] = params.at(joint.getName());
    }
  }
  
  // Note: KDL handles joints by index within the chain. 
  // We map the joint names to their corresponding zero-based index for faster lookup during computation.
  int jnt_idx = 0;
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i) {
    if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None) {
      std::string name = chain_.getSegment(i).getJoint().getName();
      if (params.count(name)) {
        joint_friction_map_[jnt_idx] = params.at(name);
      }
      jnt_idx++;
    }
  }
}

Eigen::VectorXd GravityCompensator::compute_compensation(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qd)
{
  size_t nj = chain_.getNrOfJoints();
  KDL::JntArray jnt_pos(nj);
  KDL::JntArray jnt_vel(nj);
  KDL::JntArray jnt_acc(nj); // Acceleration is zero for gravity compensation
  KDL::Wrenches external_wrenches(chain_.getNrOfSegments(), KDL::Wrench::Zero());
  KDL::JntArray jnt_torques(nj);

  // Convert Eigen to KDL
  for (size_t i = 0; i < nj; ++i) {
    jnt_pos(i) = q(i);
    jnt_vel(i) = qd(i);
    jnt_acc(i) = 0.0; 
  }

  // Calculate Inverse Dynamics: tau = M(q)qdd + C(q,qd)qd + G(q)
  // Since qdd = 0 and qd is low in free-drive, this effectively returns G(q)
  if (id_solver_->CartToJnt(jnt_pos, jnt_vel, jnt_acc, external_wrenches, jnt_torques) < 0) {
    return Eigen::VectorXd::Zero(nj);
  }

  Eigen::VectorXd result(nj);
  for (size_t i = 0; i < nj; ++i) {
    double friction_torque = 0.0;
    if (joint_friction_map_.count(i)) {
      friction_torque = calculate_friction(qd(i), joint_friction_map_[i]);
    }
    result(i) = jnt_torques(i) + friction_torque;
  }

  return result;
}

double GravityCompensator::calculate_friction(double velocity, const FrictionParams & p)
{
  // Implementation of the Stribeck + Coulomb + Viscous friction model
  // Ref: "A Survey of Robot Friction Models", Bona and Indri.
  
  double sgn = (velocity > 0) ? 1.0 : ((velocity < 0) ? -1.0 : 0.0);
  double abs_vel = std::abs(velocity);

  // Viscous component: Linear with velocity
  double tau_v = p.f_v * velocity;

  // Stribeck component: Higher friction at low speeds that decays
  double tau_s = p.f_s * sgn * std::exp(-p.f_2 * abs_vel);

  // Coulomb component: Constant friction that switches sign with velocity
  // We use a smoothed transition (1 - exp) to avoid numerical instability at zero velocity
  double tau_c = p.f_c * sgn * (1.0 - std::exp(-p.f_1 * abs_vel));

  return tau_v + tau_s + tau_c;
}

Eigen::VectorXd GravityCompensator::compute_compensation_with_wrench(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qd,
  const Eigen::VectorXd & wrench,
  double gain)
{
  size_t nj = chain_.getNrOfJoints();
  size_t ns = chain_.getNrOfSegments();
  KDL::JntArray jnt_pos(nj);
  KDL::JntArray jnt_vel(nj);
  KDL::JntArray jnt_acc(nj);
  KDL::JntArray jnt_torques(nj);

  // 1. External Wrenches Vector (one per segment)
  KDL::Wrenches external_wrenches(ns, KDL::Wrench::Zero());
  
  // 2. Map Wrist Wrench to the Last Segment (End Effector)
  // We apply the NEGATIVE wrench because id_solver calculates the torque 
  // NEEDED to hold against it. For 'Free Mode' (Active Assistance), 
  // we want to APPLY torque in the direction of the force.
  external_wrenches[ns - 1] = KDL::Wrench(
    KDL::Vector(-wrench(0) * gain, -wrench(1) * gain, -wrench(2) * gain),
    KDL::Vector(-wrench(3) * gain, -wrench(4) * gain, -wrench(5) * gain)
  );

  for (size_t i = 0; i < nj; ++i) {
    jnt_pos(i) = q(i);
    jnt_vel(i) = qd(i);
    jnt_acc(i) = 0.0;
  }

  if (id_solver_->CartToJnt(jnt_pos, jnt_vel, jnt_acc, external_wrenches, jnt_torques) < 0) {
    return Eigen::VectorXd::Zero(nj);
  }

  Eigen::VectorXd result(nj);
  for (size_t i = 0; i < nj; ++i) {
    double friction_torque = calculate_friction(qd(i), joint_friction_map_[i]);
    result(i) = jnt_torques(i) + friction_torque;
  }

  return result;
}

}  // namespace cobot_gravity_compensation
