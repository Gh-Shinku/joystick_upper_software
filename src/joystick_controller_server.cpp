#include <atomic>
#include <bupt_interfaces/msg/joystick.hpp>
#include <bupt_interfaces/srv/joy_stick_interaction.hpp>
#include <cassert>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

template <typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type clamp(const T &val, const T &min, const T &max) {
  if (val < min)
    return min;
  else if (val > max)
    return max;
  else
    return val;
}

enum class HANDLE_DEAD_ZONE : int { AIRCRAFT_HANDLE = 10, SELF_MADE_HANDLE = 100 };

class Joystick_Vel_Server : public rclcpp::Node {
private:
  rclcpp::Subscription<bupt_interfaces::msg::Joystick>::SharedPtr joysub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velpub_;
  rclcpp::Service<bupt_interfaces::srv::JoyStickInteraction>::SharedPtr joysrv_;

  double max_linear_speed;
  double max_angular_speed;

  double prev_linear_x = 0.0;
  double prev_linear_y = 0.0;
  double prev_angular_z = 0.0;

  static constexpr double MAX_ACTION = 664;
  static constexpr double eps = 1e-2;
  /* 指数滑动平均系数 (alpha 控制平滑度，0.0~1.0)，alpha 越大响应延迟越低 */
  static constexpr double vel_coe = 0.2;
  static constexpr double angular_coe = 0.2;
  static constexpr double vel_threshold = 0.8;
  static constexpr double angular_threshold = 0.8;

  std::atomic<bool> joy_start_;
  std::mutex joy_mutex_;
  std::condition_variable joy_cond_;

  void joysub_callback(bupt_interfaces::msg::Joystick::SharedPtr msg);
  void joysrv_callback(const bupt_interfaces::srv::JoyStickInteraction::Request::SharedPtr &req,
                       const bupt_interfaces::srv::JoyStickInteraction::Response::SharedPtr &res);
  inline void joyhanle_filter(double &cur_data, double &prev_data, const double alpha, const double threshold);

public:
  Joystick_Vel_Server(const std::string &name, double max_linear_speed, double max_angular_speed);
};

Joystick_Vel_Server::Joystick_Vel_Server(const std::string &name, double max_linear_speed, double max_angular_speed)
    : Node(name),
      joysub_(this->create_subscription<bupt_interfaces::msg::Joystick>(
          "joystick", 10, std::bind(&Joystick_Vel_Server::joysub_callback, this, std::placeholders::_1))),
      velpub_(this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10)),
      joysrv_(this->create_service<bupt_interfaces::srv::JoyStickInteraction>(
          "joyvel_srv", std::bind(&Joystick_Vel_Server::joysrv_callback, this, std::placeholders::_1, std::placeholders::_2))),
      max_linear_speed(max_linear_speed),
      max_angular_speed(max_angular_speed),
      joy_start_(true) {
}

void Joystick_Vel_Server::joysub_callback(const bupt_interfaces::msg::Joystick::SharedPtr msg) {
  {
    std::unique_lock<std::mutex> lock(joy_mutex_);
    joy_cond_.wait(lock, [this]() { return joy_start_.load(); });
  }

  /* 排除摇杆死区 */
  for (int i = 0; i < 4; i++) {
    if (std::abs(msg->action[i]) < static_cast<int>(HANDLE_DEAD_ZONE::AIRCRAFT_HANDLE)) {
      msg->action[i] = 0;
    }
  }

  /* 平滑平移速度 */
  auto twist = geometry_msgs::msg::Twist();
  double dx = msg->action[1];
  double dy = msg->action[0];
  double magnitude = std::hypot(dx, dy);
  /* 将斜方向最大速度约束到与垂直方向一样 */
  if (magnitude > MAX_ACTION) {
    magnitude = MAX_ACTION;
  }
  double max_magnitude = std::hypot(MAX_ACTION, MAX_ACTION);

  double coe = (magnitude > 0) ? (magnitude / max_magnitude) : 0.0;
  double current_speed = max_linear_speed * coe;

  /* 单位向量 */
  double unit_dx = (magnitude > 0) ? (dx / magnitude) : 0.0;
  double unit_dy = (magnitude > 0) ? (dy / magnitude) : 0.0;

  double target_linear_x = current_speed * unit_dx;
  double target_linear_y = current_speed * unit_dy;

  joyhanle_filter(target_linear_x, prev_linear_x, vel_coe, vel_threshold);
  joyhanle_filter(target_linear_y, prev_linear_y, vel_coe, vel_threshold);

  /* 平滑角速度 */
  coe = std::hypot(msg->action[2], msg->action[3]) / max_magnitude;
  current_speed = max_angular_speed * coe;
  if (msg->action[2] < 0) {
    current_speed = -current_speed;
  }
  double target_angular_z = current_speed;

  joyhanle_filter(target_angular_z, prev_angular_z, angular_coe, angular_threshold);

  velpub_->publish(twist);
}

void Joystick_Vel_Server::joysrv_callback(const bupt_interfaces::srv::JoyStickInteraction::Request::SharedPtr &req,
                                          const bupt_interfaces::srv::JoyStickInteraction::Response::SharedPtr &res) {
  joy_start_ = req->open;
  res->finish = true;
}

inline void Joystick_Vel_Server::joyhanle_filter(double &cur_data, double &prev_data, const double alpha, const double threshold) {
  assert(alpha >= 0 && alpha <= 1);
  if (std::abs(cur_data - prev_data) < threshold) cur_data = alpha * cur_data + (1 - alpha) * prev_data;
  /* 过滤误差级别的波动值 */
  if (std::abs(cur_data) < eps) cur_data = 0;
  prev_data = cur_data;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joystick_Vel_Server>("joystick_vel_server", 2, 4));
  rclcpp::shutdown();
  return 0;
}
