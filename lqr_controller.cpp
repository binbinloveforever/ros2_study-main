// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschr盲nkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lqr_controller/lqr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = lqr_controller::LqrController::ControllerReferenceMsg;

void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

namespace lqr_controller
{
LqrController::LqrController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn LqrController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<lqr_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LqrController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&LqrController::reference_callback, this, std::placeholders::_1));

  imu_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
    "imu", subscribers_qos, std::bind(&LqrController::imu_callback, this, std::placeholders::_1));

  joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", subscribers_qos,
    std::bind(&LqrController::joint_states_callback, this, std::placeholders::_1));

  desired_states_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    params_.cmd_vel_topic, subscribers_qos,
    std::bind(&LqrController::desired_states_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  last_command_time_ = get_node()->get_clock()->now();
  command_received_ = false;

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    control_mode_.writeFromNonRT(request->data ? control_mode_type::SLOW : control_mode_type::FAST);
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  state_publisher_->lock();
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void LqrController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu, but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(), params_.joints.size());
  }
}

void LqrController::imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
  tf2::Quaternion quat(
    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  robotstate_.orientation.roll = static_cast<float>(roll);
  robotstate_.orientation.pitch = static_cast<float>(pitch);
  robotstate_.orientation.yaw = static_cast<float>(yaw);

  robotstate_.orientation.velocity.roll = static_cast<float>(msg->angular_velocity.x);
  robotstate_.orientation.velocity.pitch = static_cast<float>(msg->angular_velocity.y);
  robotstate_.orientation.velocity.yaw = static_cast<float>(msg->angular_velocity.z);

  robotstate_.linear_acceleration[0] = static_cast<float>(msg->linear_acceleration.x);
  robotstate_.linear_acceleration[1] = static_cast<float>(msg->linear_acceleration.y);
  robotstate_.linear_acceleration[2] = static_cast<float>(msg->linear_acceleration.z);
}

void LqrController::joint_states_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
  if (params_.joints.size() < 2)
  {
    return;
  }

  auto update_joint_state = [&](const std::string & joint_name, WheelState & wheel_state)
  {
    const auto joint_it = std::find(msg->name.begin(), msg->name.end(), joint_name);
    if (joint_it == msg->name.end())
    {
      return;
    }

    const auto index = static_cast<size_t>(std::distance(msg->name.begin(), joint_it));
    if (index < msg->velocity.size())
    {
      wheel_state.velocity = static_cast<float>(msg->velocity[index]);
    }
    if (index < msg->effort.size())
    {
      wheel_state.effort = static_cast<float>(msg->effort[index]);
    }
  };

  update_joint_state(params_.joints[0], robotstate_.wheels.left);
  update_joint_state(params_.joints[1], robotstate_.wheels.right);
}

void LqrController::desired_states_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  robotstate_.command.velocity = static_cast<float>(std::clamp(
    msg->linear.x, -params_.max_reverse_speed, params_.max_linear_speed));
  robotstate_.command.yaw = static_cast<float>(std::clamp(
    msg->angular.z, -params_.max_angular_speed, params_.max_angular_speed));
  last_command_time_ = get_node()->get_clock()->now();
  command_received_ = true;
}

controller_interface::InterfaceConfiguration LqrController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration LqrController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn LqrController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);
  robotstate_.mean_displacement = 0.0f;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LqrController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LqrController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)input_ref_.readFromRT();

  const float euler_angle[3] = {
    robotstate_.orientation.roll, robotstate_.orientation.pitch, robotstate_.orientation.yaw};
  const float euler_angle_velocity[3] = {
    robotstate_.orientation.velocity.roll,
    robotstate_.orientation.velocity.pitch,
    robotstate_.orientation.velocity.yaw};
  const float wheel_velocity[2] = {
    robotstate_.wheels.left.velocity, robotstate_.wheels.right.velocity};

  if (
    !command_received_ ||
    (time - last_command_time_).seconds() > params_.command_timeout ||
    std::abs(euler_angle[1] * rad2angle) > params_.max_pitch_for_motion_deg)
  {
    robotstate_.command.velocity = 0.0f;
    robotstate_.command.yaw = 0.0f;
  }

  robotstate_.mean_velocity = static_cast<float>(
    (wheel_velocity[0] * params_.wheel_radius + wheel_velocity[1] * params_.wheel_radius) / 2.0 -
    robotstate_.command.velocity);
  robotstate_.mean_displacement += robotstate_.mean_velocity * static_cast<float>(period.seconds());

  const double yaw_error = euler_angle_velocity[2] - robotstate_.command.yaw;
  const float common_effort = static_cast<float>(
    K_[0] * euler_angle[1] +
    K_[1] * euler_angle_velocity[1] +
    K_[2] * robotstate_.mean_displacement +
    K_[3] * robotstate_.mean_velocity);

  const float left_wheel_set_effort =
    common_effort - static_cast<float>(params_.k_yaw * yaw_error);
  const float right_wheel_set_effort =
    common_effort + static_cast<float>(params_.k_yaw * yaw_error);
  const float max_effort = static_cast<float>(params_.max_effort);

  const float set_effort[2] = {
    std::clamp(-left_wheel_set_effort, -max_effort, max_effort),
    std::clamp(-right_wheel_set_effort, -max_effort, max_effort)};

  for (size_t i = 0; i < command_interfaces_.size() && i < 2; ++i)
  {
    command_interfaces_[i].set_value(set_effort[i]);
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.joint_names.clear();
    state_publisher_->msg_.angles.clear();
    state_publisher_->msg_.set_effort.clear();

    for (size_t i = 0; i < state_interfaces_.size(); ++i)
    {
      state_publisher_->msg_.joint_names.emplace_back(state_interfaces_[i].get_name());
    }
    for (int j = 0; j < 3; ++j)
    {
      state_publisher_->msg_.angles.emplace_back(euler_angle[j] * rad2angle);
    }
    state_publisher_->msg_.set_effort.emplace_back(robotstate_.wheels.left.effort);
    state_publisher_->msg_.set_effort.emplace_back(robotstate_.wheels.right.effort);
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace lqr_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  lqr_controller::LqrController, controller_interface::ControllerInterface)
