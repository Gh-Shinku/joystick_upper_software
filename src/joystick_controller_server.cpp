#include <bupt_interfaces/msg/joystick.hpp>
#include <bupt_interfaces/srv/joy_stick_interaction.hpp>
#include <bupt_interfaces/srv/shoot_control.hpp>
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

class Joystick_Vel_Server : public rclcpp::Node {
private:
  rclcpp::Subscription<bupt_interfaces::msg::Joystick>::SharedPtr joysub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velpub_;
  rclcpp::Service<bupt_interfaces::srv::JoyStickInteraction>::SharedPtr joysrv_;
  rclcpp::Client<bupt_interfaces::srv::ShootControl>::SharedPtr shootcli_;

  double max_linear_speed;
  double max_angular_speed;

  double prev_linear_x = 0.0;
  double prev_linear_y = 0.0;
  double prev_angular_z = 0.0;

  static constexpr double MAX_ACTION = 664;
  static constexpr double eps = 1e-2;
  /* 指数滑动平均 (acc_coe 控制平滑度，0.0~1.0) */
  static constexpr double acc_coe = 0.2;
  static constexpr double dec_coe = 0.2;

  bool cond_joy_;

  std::mutex mtx_;
  std::condition_variable cv_;

  void joysub_callback(bupt_interfaces::msg::Joystick::SharedPtr msg);
  void joysrv_callback(bupt_interfaces::srv::JoyStickInteraction::Request::SharedPtr req);
  void send_request(const rclcpp::Client<bupt_interfaces::srv::ShootControl>::SharedPtr &client, bool is_auto, uint8_t level);

public:
  Joystick_Vel_Server(const std::string &name, double max_linear_speed, double max_angular_speed);
};

Joystick_Vel_Server::Joystick_Vel_Server(const std::string &name, double max_linear_speed, double max_angular_speed)
    : Node(name),
      max_linear_speed(max_linear_speed),
      max_angular_speed(max_angular_speed),
      cond_joy_(true),
      mtx_(std::mutex()),
      cv_(std::condition_variable()) {
  joysub_ = this->create_subscription<bupt_interfaces::msg::Joystick>(
      "joystick", 10, std::bind(&Joystick_Vel_Server::joysub_callback, this, std::placeholders::_1));
  velpub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  joysrv_ = this->create_service<bupt_interfaces::srv::JoyStickInteraction>(
      "joyvel_srv", std::bind(&Joystick_Vel_Server::joysrv_callback, this, std::placeholders::_1));
}

void Joystick_Vel_Server::joysub_callback(bupt_interfaces::msg::Joystick::SharedPtr msg) {
  if (!cond_joy_) {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return cond_joy_; });
  }

  /* 排除死区 */
  for (int i = 0; i < 4; i++) {
    if (std::abs(msg->action[i]) < 10) {
      msg->action[i] = 0;
    }
  }

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

  // 计算方向向量的单位向量
  double unit_dx = (magnitude > 0) ? (dx / magnitude) : 0.0;
  double unit_dy = (magnitude > 0) ? (dy / magnitude) : 0.0;

  double target_linear_x = current_speed * unit_dx;
  double target_linear_y = current_speed * unit_dy;

  twist.linear.x = acc_coe * target_linear_x + (1 - acc_coe) * prev_linear_x;
  twist.linear.y = acc_coe * target_linear_y + (1 - acc_coe) * prev_linear_y;

  if (std::abs(twist.linear.x) < eps) twist.linear.x = 0;
  if (std::abs(twist.linear.y) < eps) twist.linear.y = 0;

  prev_linear_x = twist.linear.x;
  prev_linear_y = twist.linear.y;

  coe = std::hypot(msg->action[2], msg->action[3]) / max_magnitude;
  current_speed = max_angular_speed * coe;
  if (msg->action[2] < 0) {
    current_speed = -current_speed;
  }
  double target_angular_z = current_speed;
  twist.angular.z = acc_coe * target_angular_z + (1 - acc_coe) * prev_angular_z;
  if (std::abs(twist.angular.z) < eps) twist.angular.z = 0;
  prev_angular_z = twist.angular.z;

  velpub_->publish(twist);

  /* 功能键用于投篮 or 运球 */
  /* 临时的投篮功能键 */
  for (int i = 0; i <= 6; ++i) {
    if (msg->button & (1 << i)) {
      send_request(shootcli_, false, i);
      break;
    }
  }
}

void Joystick_Vel_Server::joysrv_callback(bupt_interfaces::srv::JoyStickInteraction::Request::SharedPtr req) {
  if (req->open == true) {
    if (!cond_joy_) {
      mtx_.lock();
      cond_joy_ = true;
      mtx_.unlock();
      cv_.notify_one();
    }
  } else {
    if (cond_joy_) {
      mtx_.lock();
      cond_joy_ = false;
      mtx_.unlock();
    }
  }
}

void Joystick_Vel_Server::send_request(const rclcpp::Client<bupt_interfaces::srv::ShootControl>::SharedPtr &client, bool is_auto,
                                       uint8_t level) {
  // 等待服务端上线
  int wait_time = 0;
  while (!client->wait_for_service(std::chrono::milliseconds(100))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "等待服务的过程中ROS2事件循环被中断");
      return;
    }
    if (wait_time > 300) {
      RCLCPP_ERROR(this->get_logger(), "等待服务超时");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "等待joystick_controller服务启动");
    ++wait_time;
  }

  auto request = std::make_shared<bupt_interfaces::srv::ShootControl::Request>();
  request->is_auto = true;
  request->level = level;

  RCLCPP_INFO(this->get_logger(), "发送请求给shoot_server服务");
  auto result = client->async_send_request(request);
  if (!(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)) {
    RCLCPP_ERROR(this->get_logger(), "请求失败");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joystick_Vel_Server>("joystick_vel_server", 2, 4));
  rclcpp::shutdown();
  return 0;
}
