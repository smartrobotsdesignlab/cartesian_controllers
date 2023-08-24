////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_compliance_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <cartesian_compliance_controller/cartesian_compliance_controller.h>

namespace cartesian_compliance_controller
{

CartesianComplianceController::CartesianComplianceController()
// Base constructor won't be called in diamond inheritance, so call that
// explicitly
: Base::CartesianControllerBase(),
  MotionBase::CartesianMotionController(),
  ForceBase::CartesianForceController()
{
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianComplianceController::on_init()
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_init() != TYPE::SUCCESS || ForceBase::on_init() != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  auto_declare<std::string>("compliance_ref_link", "");

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;

  constexpr double default_lin_damping = 0.7;
  constexpr double default_rot_damping = 0.7;

  auto_declare<double>("stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("stiffness.rot_z", default_rot_stiff);

  auto_declare<double>("damping.trans_x", default_lin_damping);
  auto_declare<double>("damping.trans_y", default_lin_damping);
  auto_declare<double>("damping.trans_z", default_lin_damping);
  auto_declare<double>("damping.rot_x", default_rot_damping);
  auto_declare<double>("damping.rot_y", default_rot_damping);
  auto_declare<double>("damping.rot_z", default_rot_damping);

  m_last_error.resize(10);
  std::fill(m_last_error.begin(), m_last_error.end(), ctrl::Vector6D::Zero());

  return TYPE::SUCCESS;
}
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianComplianceController::init(const std::string & controller_name)
{
  using TYPE = controller_interface::return_type;
  if (MotionBase::init(controller_name) != TYPE::OK || ForceBase::init(controller_name) != TYPE::OK)
  {
    return TYPE::ERROR;
  }

  auto_declare<std::string>("compliance_ref_link", "");

  return TYPE::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianComplianceController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_configure(previous_state) != TYPE::SUCCESS || ForceBase::on_configure(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  // Make sure compliance link is part of the robot chain
  m_compliance_ref_link = get_node()->get_parameter("compliance_ref_link").as_string();
  if(!Base::robotChainContains(m_compliance_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_compliance_ref_link << " is not part of the kinematic chain from "
                                              << Base::m_robot_base_link << " to "
                                              << Base::m_end_effector_link);
    return TYPE::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianComplianceController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Base::on_activation(..) will get called twice,
  // but that's fine.
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_activate(previous_state) != TYPE::SUCCESS || ForceBase::on_activate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianComplianceController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_deactivate(previous_state) != TYPE::SUCCESS || ForceBase::on_deactivate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
controller_interface::return_type CartesianComplianceController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianComplianceController::update()
#endif
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.02);

    // Compute the net force
    ctrl::Vector6D error = computeComplianceError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error,internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianComplianceController::computeComplianceError()
{
  ctrl::Vector6D tmp;
  tmp[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

  m_stiffness = tmp.asDiagonal();

  // Get the damping
  tmp[0] = get_node()->get_parameter("damping.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("damping.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("damping.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("damping.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("damping.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("damping.rot_z").as_double();

  m_damping = tmp.asDiagonal();

  ctrl::Vector6D current_motion_error = MotionBase::computeMotionError();

  auto internal_period = rclcpp::Duration::from_seconds(0.02);
  ctrl::Vector6D filterd_error = ctrl::Vector6D::Zero();
  for (size_t i = 0; i < m_last_error.size(); i++)
  {
    filterd_error[0] += m_last_error[i][0];
    filterd_error[1] += m_last_error[i][1];
    filterd_error[2] += m_last_error[i][2];
    filterd_error[3] += m_last_error[i][3];
    filterd_error[4] += m_last_error[i][4];
    filterd_error[5] += m_last_error[i][5]; 
  }
  filterd_error = filterd_error / m_last_error.size();

  // print last_motion_error
  
  ctrl::Vector6D net_force =

    // Spring force in base orientation
    // Base::displayInBaseLink(m_stiffness,m_compliance_ref_link) * current_motion_error
    // + Base::displayInBaseLink(m_damping,m_compliance_ref_link) * ((current_motion_error - filterd_error)/ internal_period.seconds())
    m_stiffness * current_motion_error
    + m_damping * ((current_motion_error - filterd_error)/ internal_period.seconds())

    // Sensor and target force in base orientation
    + ForceBase::computeForceError();

  m_last_error.erase(m_last_error.begin());
  m_last_error.push_back(current_motion_error);

  return net_force;
}

} // namespace


// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_compliance_controller::CartesianComplianceController, controller_interface::ControllerInterface)
