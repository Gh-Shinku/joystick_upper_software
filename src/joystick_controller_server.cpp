#include <atomic>
#include <bupt_interfaces/msg/joystick.hpp>
#include <bupt_interfaces/srv/r2_top_control2025.hpp>
#include <cassert>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "utils/utils.hpp"

enum class HANDLE_DEAD_ZONE : int { AIRCRAFT_HANDLE = 10, SELF_MADE_HANDLE = 100 };
enum class JoystickSwitch : int { BIT_JOY_HANDLE, BIT_AIM_MODE };

class Joystick_Vel_Server : public rclcpp::Node {
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

  rclcpp::Subscription<bupt_interfaces::msg::Joystick>::SharedPtr joysub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velpub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<bupt_interfaces::srv::R2TopControl2025>::SharedPtr joysrv_;

  geometry_msgs::msg::Pose cur_pose_;
  std::unique_ptr<PID_Controller> pid_angular_;

  double max_linear_speed_;
  double max_angular_speed_;

  double prev_linear_x_;
  double prev_linear_y_;
  double prev_angular_z_;

  uint16_t cur_button_;
  uint16_t last_button_;

  std::atomic<bool> joy_start_;
  std::atomic<bool> aim_mode_;

  void joysub_callback(bupt_interfaces::msg::Joystick::SharedPtr msg);
  void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void joysrv_callback(const bupt_interfaces::srv::R2TopControl2025::Request::SharedPtr &req,
                       const bupt_interfaces::srv::R2TopControl2025::Response::SharedPtr &res);
  void joyhanle_filter(double &cur_data, double &prev_data, const double alpha, const double threshold);

  /**
   * @brief 边沿检测
   *
   * @param cur_button 当前按键编码
   * @param last_button 上次按键编码
   * @param bits 检测比特位号
   * @param trigger_mode true 上升沿，false 下降沿
   *
   * @return true 说明检测到，否则无
   */
  static bool edge_detect(uint16_t cur_button, uint16_t last_button, uint16_t bits, bool trigger_mode);

public:
  Joystick_Vel_Server(const std::string &name, double max_linear_speed, double max_angular_speed);
};

Joystick_Vel_Server::Joystick_Vel_Server(const std::string &name, double max_linear_speed, double max_angular_speed)
    : Node(name),
      joysub_(this->create_subscription<bupt_interfaces::msg::Joystick>(
          "/joystick", 10, std::bind(&Joystick_Vel_Server::joysub_callback, this, std::placeholders::_1))),
      velpub_(this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10)),
      odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>(
          "/lidar_odom", 10, std::bind(&Joystick_Vel_Server::odom_sub_callback, this, std::placeholders::_1))),
      joysrv_(this->create_service<bupt_interfaces::srv::R2TopControl2025>(
          "joyvel_srv", std::bind(&Joystick_Vel_Server::joysrv_callback, this, std::placeholders::_1, std::placeholders::_2))),
      cur_pose_(),
      pid_angular_(std::make_unique<PID_Controller>(1.2, 0.0, 2.2, 3.0)),
      max_linear_speed_(max_linear_speed),
      max_angular_speed_(max_angular_speed),
      prev_linear_x_(0.0),
      prev_linear_y_(0.0),
      prev_angular_z_(0.0),
      cur_button_(0),
      last_button_(0),
      joy_start_(true),
      aim_mode_(false) {
}

void Joystick_Vel_Server::joysub_callback(const bupt_interfaces::msg::Joystick::SharedPtr msg) {
  if (!joy_start_.load()) {
    return;
  }

  /* 检测瞄准模式 button 是否按下 */
  last_button_ = cur_button_;
  cur_button_ = msg->button;
  if (edge_detect(cur_button_, last_button_, BUTTON_AIM_MODE, true) == true) {
    aim_mode_ = !aim_mode_;
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
  if (aim_mode_.load() == true) {
    const auto dx = rim_index.first - cur_pose_.position.x;
    const auto dy = rim_index.second - cur_pose_.position.y;
    const auto target_yaw = std::atan2(dy, dx);
    const auto yaw = cur_pose_.position.z;
    const auto err_angle = normalizeRad(target_yaw - yaw);
    target_angular_z = pid_angular_->update(err_angle);
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

void Joystick_Vel_Server::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  cur_pose_ = msg->pose.pose;
}

void Joystick_Vel_Server::joysrv_callback(const bupt_interfaces::srv::R2TopControl2025::Request::SharedPtr &req,
                                          const bupt_interfaces::srv::R2TopControl2025::Response::SharedPtr &res) {
  joy_start_ = (req->action >> static_cast<int>(JoystickSwitch::BIT_JOY_HANDLE)) & 0x01 ? true : false;
  aim_mode_ = (req->action >> static_cast<int>(JoystickSwitch::BIT_AIM_MODE)) & 0x01 ? true : false;
  res->finish = true;
}

inline void Joystick_Vel_Server::joyhanle_filter(double &cur_data, double &prev_data, const double alpha, const double threshold) {
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
#ifdef SELF_HANDLE_ENABLE
  rclcpp::spin(std::make_shared<Joystick_Vel_Server>("joystick_vel_server", 1.5, 5));
#else
  rclcpp::spin(std::make_shared<Joystick_Vel_Server>("joystick_vel_server", 4, 6));
#endif
  rclcpp::shutdown();
  return 0;
}

bool Joystick_Vel_Server::edge_detect(uint16_t cur_button, uint16_t last_button, uint16_t bits, bool trigger_mode) {
  /* 0-->1 */
  if (trigger_mode == true) {
    return ((cur_button >> bits) & 0x01) && !((last_button >> bits) & 0x01);
  } else /* 1-->0 */ {
    return !((cur_button >> bits) & 0x01) && ((last_button >> bits) & 0x01);
  }
}
