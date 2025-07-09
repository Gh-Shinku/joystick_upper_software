#ifndef JOYSTICK_HANDLE_CONTROLLER_H
#define JOYSTICK_HANDLE_CONTROLLER_H

#include <atomic>
#include <bupt_interfaces/msg/joystick.hpp>
#include <bupt_interfaces/srv/action_control.hpp>
#include <cassert>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "utils/utils.hpp"

#define SELF_HANDLE_ENABLE

enum class HANDLE_DEAD_ZONE { AIRCRAFT_HANDLE = 10, SELF_MADE_HANDLE = 100 };

class Joystick_Handle_Node : public rclcpp::Node {
private:
#ifdef SELF_HANDLE_ENABLE
  static constexpr double MAX_ACTION = 1670;
#else
  static constexpr double MAX_ACTION = 664;
#endif
  static constexpr double eps = 1e-2;
/* 指数滑动平均系数 (alpha 控制平滑度，0.0~1.0)，alpha 越大响应延迟越低 */
#ifdef SELF_HANDLE_ENABLE
  static constexpr double vel_coe = 0.75;
  static constexpr double angular_coe = 0.75;
#else
  static constexpr double vel_coe = 0.3;
  static constexpr double angular_coe = 0.2;
#endif
  static constexpr double vel_threshold = 0.8;
  static constexpr double angular_threshold = 0.8;
  static constexpr int BUTTON_AIM_MODE = 10;
  static constexpr std::pair<double, double> rim_index{5.0, 0};
  static constexpr double min_angular_velocity = 0.2;
  static constexpr double eps_angle_range = 0.025;

  rclcpp::Subscription<bupt_interfaces::msg::Joystick>::SharedPtr joysub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velpub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<bupt_interfaces::srv::ActionControl>::SharedPtr joysrv_;

  geometry_msgs::msg::Pose cur_pose_;
  std::unique_ptr<PID_Controller> pid_angular_;

  double max_linear_speed_;
  double max_angular_speed_;

  double prev_linear_x_;
  double prev_linear_y_;
  double prev_angular_z_;

  std::atomic<bool> joy_switch_;
  std::atomic<bool> aim_mode_switch_;

  void joysub_callback(const bupt_interfaces::msg::Joystick::SharedPtr msg);
  void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void joysrv_callback(const bupt_interfaces::srv::ActionControl::Request::SharedPtr &req,
                       const bupt_interfaces::srv::ActionControl::Response::SharedPtr &res);
  void joyhanle_filter(double &cur_data, double &prev_data, const double alpha, const double threshold);

public:
  Joystick_Handle_Node(const std::string &name, double max_linear_speed, double max_angular_speed);
};

#endif /* JOYSTICK_HANDLE_CONTROLLER_H */