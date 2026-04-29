// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef LQR_CONTROLLER__LQR_CONTROLLER_HPP_
#define LQR_CONTROLLER__LQR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <algorithm>

#include "controller_interface/controller_interface.hpp"
#include "lqr_controller_parameters.hpp"
#include "lqr_controller/visibility_control.h"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "robot_state_pub_interface/msg/robot_state.hpp"


namespace lqr_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

static const float rad2angle = 180/3.14159265358979323846;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};


struct WheelState {
    float velocity = 0.0f;
    float effort = 0.0f;
};

struct OrientationState {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    
    // 角速度使用独立结构体保持关联性
    struct AngularVelocity {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
    } velocity;
};

struct RobotState
{
// 位姿相关数据
    OrientationState orientation;
    
    // 加速度使用连续内存
    std::array<float, 3> linear_acceleration = {0.0f, 0.0f, 0.0f};
    
    // 轮组状态
    struct {
        WheelState left;
        WheelState right;
    } wheels;
    
    // 车辆综合状态
    float mean_velocity = 0.0f;
    float mean_displacement = 0.0f;
    
    // 控制指令
    struct {
        float velocity = 0.0f;  
        float yaw = 0.0f;    
    } command;

};

class LqrController : public controller_interface::ControllerInterface
{
public:
  LQR_CONTROLLER__VISIBILITY_PUBLIC
  LqrController();

  LQR_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  LQR_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  LQR_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  LQR_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  LQR_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  LQR_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  LQR_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = robot_state_pub_interface::msg::RobotState;

protected:
  std::shared_ptr<lqr_controller::ParamListener> param_listener_;
  lqr_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_states_subscriber_ = nullptr;

  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // callback for topic interface
  LQR_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  void imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg);
  void joint_states_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
  void desired_states_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

  RobotState robotstate_;
  const float K_[4] = {-11.45837939,-2.63622853,-2.73861279,-2.741906355};//theta d_theta x d_x
  rclcpp::Time last_command_time_;
  bool command_received_ = false;
};

}  // namespace lqr_controller

#endif  // LQR_CONTROLLER__LQR_CONTROLLER_HPP_
