#include "joystick/joystick_handle_controller.hpp"

#include <atomic>
#include <bupt_interfaces/msg/joystick.hpp>
#include <bupt_interfaces/srv/action_control.hpp>
#include <cassert>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "joystick/action.hpp"
#include "utils/utils.hpp"

Joystick_Handle_Node::Joystick_Handle_Node(const std::string &name, double max_linear_speed, double max_angular_speed)
    : Node(name),
      joysub_(this->create_subscription<bupt_interfaces::msg::Joystick>(
          "/joystick", 10, std::bind(&Joystick_Handle_Node::joysub_callback, this, std::placeholders::_1))),
      velpub_(this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10)),
      odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>(
          "/lidar_odom", 10, std::bind(&Joystick_Handle_Node::odom_sub_callback, this, std::placeholders::_1))),
      joysrv_(this->create_service<bupt_interfaces::srv::ActionControl>(
          "joyvel_srv", std::bind(&Joystick_Handle_Node::joysrv_callback, this, std::placeholders::_1, std::placeholders::_2))),
      cur_pose_(),
      pid_angular_(std::make_unique<PID_Controller>(2.5, 0.0, 0.0, 2.0)),
      max_linear_speed_(max_linear_speed),
      max_angular_speed_(max_angular_speed),
      prev_linear_x_(0.0),
      prev_linear_y_(0.0),
      prev_angular_z_(0.0),
      joy_switch_(true),
      aim_mode_switch_(false),
      angle_offset_(0) {
}

void Joystick_Handle_Node::joysub_callback(const bupt_interfaces::msg::Joystick::SharedPtr msg) {
  if (joy_switch_.load() == false) {
    return;
  }

  /* 排除摇杆死区 */
  for (int i = 0; i < 4; i++) {
#ifdef SELF_HANDLE_ENABLE
    if (std::abs(msg->action[i]) < static_cast<int>(HANDLE_DEAD_ZONE::SELF_MADE_HANDLE)) {
      msg->action[i] = 0;
    }
#else
    if (std::abs(msg->action[i]) < static_cast<int>(HANDLE_DEAD_ZONE::AIRCRAFT_HANDLE)) {
      msg->action[i] = 0;
    }
#endif
  }

  /* 平滑平移速度 */
  auto twist = geometry_msgs::msg::Twist();
#ifdef SELF_HANDLE_ENABLE
  double dx = msg->action[0];
  double dy = -msg->action[1];
#else
  double dx = msg->action[1];
  double dy = msg->action[0];
#endif
  double magnitude = std::hypot(dx, dy);
  /* 将斜方向最大速度约束到与垂直方向一样 */
  if (magnitude > MAX_ACTION) {
    magnitude = MAX_ACTION;
  }
  double max_magnitude = std::hypot(MAX_ACTION, MAX_ACTION);

  double coe = (magnitude > 0) ? (magnitude / max_magnitude) : 0.0;
  double current_speed = max_linear_speed_ * coe;

  /* 单位向量 */
  double unit_dx = (magnitude > 0) ? (dx / magnitude) : 0.0;
  double unit_dy = (magnitude > 0) ? (dy / magnitude) : 0.0;

  double target_linear_x = current_speed * unit_dx;
  double target_linear_y = current_speed * unit_dy;

  joyhanle_filter(target_linear_x, prev_linear_x_, vel_coe, vel_threshold);
  joyhanle_filter(target_linear_y, prev_linear_y_, vel_coe, vel_threshold);

  double target_angular_z;
  /* 如果使用瞄准模式，就发布瞄准 PID 角速度，否则使用手柄给出的角速度 */
  if (aim_mode_switch_.load() == true) {
    const auto dx = rim_index.first - cur_pose_.position.x;
    const auto dy = rim_index.second - cur_pose_.position.y;
    const auto target_yaw = std::atan2(dy, dx) + degToRad(static_cast<double>(angle_offset_.load()));
    const auto yaw = cur_pose_.position.z;
    const auto err_angle = -normalizeRad(target_yaw - yaw);
    const auto pid_output = pid_angular_->update(err_angle);
    if (std::fabs(pid_output) < min_angular_velocity) {
      if (pid_output > 0) {
        target_angular_z = min_angular_velocity;
      } else {
        target_angular_z = -min_angular_velocity;
      }
      if (std::fabs(err_angle) < eps_angle_range) {
        target_angular_z = 0;
      }
    } else {
      target_angular_z = pid_output;
    }
    RCLCPP_INFO(this->get_logger(), "target_yaw: %.3lf(deg), yaw: %.3lf(deg), pid_output: %.3lf", radToDeg(target_yaw), radToDeg(yaw),
                target_angular_z);
  } else {
    /* 平滑角速度 */
    coe = std::hypot(msg->action[2], msg->action[3]) / max_magnitude;
    current_speed = max_angular_speed_ * coe;
#ifdef SELF_HANDLE_ENABLE
    if (msg->action[3] < 0) {
      current_speed = -current_speed;
    }
#else
    if (msg->action[2] > 0) {
      current_speed = -current_speed;
    }
#endif
    target_angular_z = current_speed;
    joyhanle_filter(target_angular_z, prev_angular_z_, angular_coe, angular_threshold);
  }
  twist.linear.x = target_linear_x;
  twist.linear.y = target_linear_y;
  twist.angular.z = target_angular_z;
  velpub_->publish(twist);
}

void Joystick_Handle_Node::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  cur_pose_ = msg->pose.pose;
}

void Joystick_Handle_Node::joysrv_callback(const bupt_interfaces::srv::ActionControl::Request::SharedPtr &req,
                                           const bupt_interfaces::srv::ActionControl::Response::SharedPtr &res) {
  if (req->action == static_cast<uint32_t>(ACTION::ACTION_JOY_SWITCH)) {
    joy_switch_ = !joy_switch_;
  } else if (req->action == static_cast<uint32_t>(ACTION::BUTTON_AIM_MODE)) {
    aim_mode_switch_ = !aim_mode_switch_;
  } else if (req->action == static_cast<uint32_t>(ACTION::BUTTON_YAW_CLOCK)) {
    fine_tune_angle(true);
  } else if (req->action == static_cast<uint32_t>(ACTION::BUTTON_YAW_ANTI_CLOCK)) {
    fine_tune_angle(false);
  }

  RCLCPP_INFO(this->get_logger(), "JOY_SWITCH: %s, AIM_SWITCH: %s", (joy_switch_.load() ? "ON" : "OFF"),
              (aim_mode_switch_.load() ? "ON" : "OFF"));
  res->finish = true;
}

void Joystick_Handle_Node::joyhanle_filter(double &cur_data, double &prev_data, const double alpha, const double threshold) {
  assert(alpha >= 0 && alpha <= 1);
#ifdef SELF_HANDLE_ENABLE
  cur_data = alpha * cur_data + (1 - alpha) * prev_data;
#else
  if (std::abs(cur_data - prev_data) < threshold) cur_data = alpha * cur_data + (1 - alpha) * prev_data;
#endif
  /* 过滤误差级别的波动值 */
  if (std::abs(cur_data) < eps) cur_data = 0;
  prev_data = cur_data;
}

void Joystick_Handle_Node::fine_tune_angle(bool is_clockwise) {
  angle_offset_ += YAW_UNIT * (is_clockwise ? -1 : 1);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
#ifdef SELF_HANDLE_ENABLE
  rclcpp::spin(std::make_shared<Joystick_Handle_Node>("Joystick_Handle_Node", 3, 5));
#else
  rclcpp::spin(std::make_shared<Joystick_Handle_Node>("Joystick_Handle_Node", 4, 6));
#endif
  rclcpp::shutdown();
  return 0;
}
