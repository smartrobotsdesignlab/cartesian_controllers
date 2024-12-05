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
/*!\file    cartesian_force_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <cartesian_force_controller/cartesian_force_controller.h>
#include <cmath>

namespace cartesian_force_controller
{

CartesianForceController::CartesianForceController()
: Base::CartesianControllerBase(), m_hand_frame_control(true)
{
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianForceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", false);
  auto_declare<bool>("gravity_compensation", false);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;;
}
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianForceController::init(const std::string & controller_name)
{
  const auto ret = Base::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", false);
  auto_declare<bool>("gravity_compensation", false);

  return controller_interface::return_type::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianForceController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if(!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link << " is not part of the kinematic chain from "
                                             << Base::m_robot_base_link << " to "
                                             << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"),
    10,
    std::bind(&CartesianForceController::targetWrenchCallback, this, std::placeholders::_1));

  m_ft_sensor_wrench_subscriber =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_node()->get_name() + std::string("/ft_sensor_wrench"),
      10,
      std::bind(&CartesianForceController::ftSensorWrenchCallback, this, std::placeholders::_1));

  // Controller-internal state publishing
  m_feedback_force_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped> >(
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
        std::string(get_node()->get_name()) + "/current_wrench", 3));

  // Gravity compensation
  m_gravity_compensation = get_node()->get_parameter("gravity_compensation").as_bool();
  if(m_gravity_compensation)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Gravity compensation is enabled.");
    std::vector<double> force_gravity, torque_gravity;
    // Declare parameters that could be set on this node
    if (!get_node()->get_parameter("gravity_compensation_value.force", force_gravity))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get parameter gravity_compensation_value.force");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    m_force_gravity << force_gravity[0], force_gravity[1], force_gravity[2], force_gravity[3];

    if (!get_node()->get_parameter("gravity_compensation_value.torque", torque_gravity))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get parameter gravity_compensation_value.torque");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    m_torque_gravity << torque_gravity[0], torque_gravity[1], torque_gravity[2], torque_gravity[3];
  }

  // Force state publish default 
  auto_declare<bool>("force_state_pub", true);
  m_force_state_pub = get_node()->get_parameter("force_state_pub").as_bool();

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();
  m_ft_sensor_wrench_raw.setZero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianForceController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianForceController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
controller_interface::return_type CartesianForceController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianForceController::update()
#endif
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes.  The internal 'simulation time' is deliberately independent of
  // the outer control cycle.
  auto internal_period = rclcpp::Duration::from_seconds(0.02);

  // Compute the net force
  ctrl::Vector6D error = computeForceError();

  // Turn Cartesian error into joint motion
  Base::computeJointControlCmds(error,internal_period);

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianForceController::computeForceError()
{
  ctrl::Vector6D target_wrench;
  m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();

  if (m_hand_frame_control) // Assume end-effector frame by convention
  {
    target_wrench = Base::displayInBaseLink(m_target_wrench,Base::m_end_effector_link);
  }
  else // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  // Superimpose target wrench and sensor wrench in base frame
  return m_ft_sensor_wrench + target_wrench;
}

void CartesianForceController::setFtSensorReferenceFrame(const std::string& new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(Base::m_ik_solver->getPositions());

  KDL::Frame sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(
      jnts,
      sensor_ref,
      m_ft_sensor_ref_link);

  KDL::Frame new_sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(
      jnts,
      new_sensor_ref,
      m_new_ft_sensor_ref);

  m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
  m_ft_sensor_rotation_eigen = kdl2Eigen(m_ft_sensor_transform);
}

void CartesianForceController::targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto& clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(),
                                clock,
                                3000,
                                "NaN detected in target wrench. Ignoring input.");
    return;
  }

  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;
}

void CartesianForceController::ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto& clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(),
                                clock,
                                3000,
                                "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }


// #if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE

  ctrl::Vector6D force_tmp;
  force_tmp[0] = wrench->wrench.force.x;
  force_tmp[1] = wrench->wrench.force.y;
  force_tmp[2] = wrench->wrench.force.z;
  force_tmp[3] = wrench->wrench.torque.x;
  force_tmp[4] = wrench->wrench.torque.y;
  force_tmp[5] = wrench->wrench.torque.z;

  Eigen::Vector3d force = m_ft_sensor_rotation_eigen * force_tmp.head(3);
  Eigen::Vector3d torque = m_ft_sensor_rotation_eigen * force_tmp.tail(3);  

  force_tmp[0] = force[0];
  force_tmp[1] = force[1];
  force_tmp[2] = force[2];
  force_tmp[3] = torque[0];
  force_tmp[4] = torque[1];
  force_tmp[5] = torque[2];

  // Low pass fileter
  double alpha = 0.5;
  m_ft_sensor_wrench_raw[0] = alpha * force_tmp[0] + (1 - alpha) * m_ft_sensor_wrench_raw[0];
  m_ft_sensor_wrench_raw[1] = alpha * force_tmp[1] + (1 - alpha) * m_ft_sensor_wrench_raw[1];
  m_ft_sensor_wrench_raw[2] = alpha * force_tmp[2] + (1 - alpha) * m_ft_sensor_wrench_raw[2];
  m_ft_sensor_wrench_raw[3] = alpha * force_tmp[3] + (1 - alpha) * m_ft_sensor_wrench_raw[3];
  m_ft_sensor_wrench_raw[4] = alpha * force_tmp[4] + (1 - alpha) * m_ft_sensor_wrench_raw[4];
  m_ft_sensor_wrench_raw[5] = alpha * force_tmp[5] + (1 - alpha) * m_ft_sensor_wrench_raw[5];
}

void CartesianForceController::gravityCompensation(void)
{
  if(!Base::m_compute_initialized)
  {
    RCLCPP_WARN_STREAM(get_node()->get_logger(),
                        "Cannot compute ft sensor wrench. Controller is not initialized.");
    return;
  }
  
  // Display ft sensor to robot base frame
  ctrl::Vector6D ft_sensor_wrench = Base::displayInBaseLink(m_ft_sensor_wrench_raw,m_new_ft_sensor_ref);
  ctrl::Vector6D ft_sensor_wrench_tmp;

  if (m_gravity_compensation)
  {
    ctrl::Vector6D gravity_comp = ctrl::Vector6D::Zero();
    // std::cout << "gravity compensation is on" << gravity_comp << std::endl;

    // Force compenstation
    // std::cout << "force_gravity is: " << force_gravity << std::endl;

    KDL::Frame transform_kdl;
    m_forward_kinematics_solver->JntToCart(
      m_ik_solver->getPositions(),
      transform_kdl,
      m_new_ft_sensor_ref);
    Eigen::Matrix3d eigen_rot =  kdl2Eigen(transform_kdl);
    // std::cout << "eigen_rot is: \r\n" << eigen_rot << std::endl;

    Eigen::Matrix<double,3,4> A_comp = Eigen::Matrix<double,3,4>::Zero();
    A_comp(2,0) = 1;
    // std::cout << "top left corner is: \r\n" << A_comp.bottomRightCorner(3,3) << std::endl;
    A_comp.bottomRightCorner(3,3) = eigen_rot;
    // std::cout << "A_comp is: \r\n" << A_comp << std::endl;
    Eigen::Vector3d force_comp = A_comp * m_force_gravity;
    // std::cout << "force_comp is: \r\n" << force_comp << std::endl;

    // Torque compensation
    // std::cout << "torque_gravity is: \r\n" << torque_gravity << std::endl;
    Eigen::Matrix3d cross_prod;
    cross_prod << 0., 1., 0., -1., 0., 0., 0., 0., 0.;
    // std::cout << "cross_prod is: \r\n" << cross_prod << std::endl;
    Eigen::Matrix3d torque_rots = Eigen::Matrix3d::Zero();
    torque_rots.col(2) = eigen_rot.col(2);
    // std::cout << "torque_rots is: \r\n" << torque_rots << std::endl;
    Eigen::Matrix3d At_comp_part = m_force_gravity[0] * cross_prod * torque_rots;
    // std::cout << "At_comp_part is: \r\n" << At_comp_part << std::endl;
    Eigen::Matrix<double, 3, 4> At_comp = Eigen::Matrix<double, 3,4>::Zero();
    At_comp.col(0) = At_comp_part.col(2);
    // std::cout << "At_comp is: \r\n" << At_comp << std::endl;
    At_comp.bottomRightCorner(3,3) = eigen_rot;
    // std::cout << "At_comp is: \r\n" << At_comp << std::endl;
    Eigen::Vector3d torque_comp = At_comp * m_torque_gravity;
    // std::cout << "torque_comp is: \r\n" << torque_comp << std::endl;
    
    gravity_comp << force_comp , torque_comp;
    // std::cout << "gravity compensation is: \r\n" << gravity_comp << std::endl;

    ft_sensor_wrench_tmp = ft_sensor_wrench - gravity_comp;
  }
  else
  {
    ft_sensor_wrench_tmp = ft_sensor_wrench;
  }

  double started_duration = Base::m_clock.now().seconds() - Base::m_start_time.seconds();
  double smooth_duratiuon = 0.5;
  double start_time = 0.5;
  if (started_duration < start_time){
    m_ft_sensor_wrench_start = ft_sensor_wrench_tmp;
  }
  else if (started_duration >= start_time && started_duration < (start_time + smooth_duratiuon)){
    m_ft_sensor_wrench = m_ft_sensor_wrench_start * (started_duration - start_time)/smooth_duratiuon + (ft_sensor_wrench_tmp - m_ft_sensor_wrench_start);
  }
  else {
    m_ft_sensor_wrench = ft_sensor_wrench_tmp;
  }

  // Publish current wrench if force enable is true
  if (m_feedback_force_publisher->trylock() && !m_emergency_stop){
    m_feedback_force_publisher->msg_.header.stamp = Base::m_clock.now();
      m_feedback_force_publisher->msg_.header.frame_id = m_robot_base_link;
    if (m_force_state_pub){
      m_feedback_force_publisher->msg_.wrench.force.x = m_ft_sensor_wrench[0];
      m_feedback_force_publisher->msg_.wrench.force.y = m_ft_sensor_wrench[1];
      m_feedback_force_publisher->msg_.wrench.force.z = m_ft_sensor_wrench[2];
      m_feedback_force_publisher->msg_.wrench.torque.x = m_ft_sensor_wrench[3];
      m_feedback_force_publisher->msg_.wrench.torque.y = m_ft_sensor_wrench[4];
      m_feedback_force_publisher->msg_.wrench.torque.z = m_ft_sensor_wrench[5];
    }

    m_feedback_force_publisher->unlockAndPublish();
  }

  // Normalize the force part of the wrench to the mass of the end-effector
  double normalized_force = m_ft_sensor_wrench.head(3).norm();
  if (!m_emergency_stop && normalized_force > m_emergency_stop_threshold){
    m_emergency_stop = true;
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Emergency stop triggered. Please restart robot. Force norm: " << normalized_force);
  }
}

Eigen::Matrix3d CartesianForceController::kdl2Eigen(const KDL::Frame& kdl_frame)
{
  Eigen::Matrix3d eigen_rot;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
        eigen_rot(i, j) = kdl_frame.M(i, j);
    }
  }
  return eigen_rot;
}


}

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_force_controller::CartesianForceController, controller_interface::ControllerInterface)
