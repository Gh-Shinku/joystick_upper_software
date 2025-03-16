#include <bupt_interfaces/msg/joystick.hpp>
#include <bupt_interfaces/srv/joy_stick_interaction.hpp>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

template <typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type clamp(const T &val,
                                                                      const T &min,
                                                                      const T &max)
{
  if (val < min)
    return min;
  else if (val > max)
    return max;
  else
    return val;
}

int get_sign(double x)
{
  if (x > 0)
    return 1;
  else if (x < 0)
    return -1;
  else
    return 0;
}

class Joystick_Vel_Server : public rclcpp::Node
{
private:
  rclcpp::Subscription<bupt_interfaces::msg::Joystick>::SharedPtr joysub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velpub_;
  rclcpp::Service<bupt_interfaces::srv::JoyStickInteraction>::SharedPtr joysrv_;

  double max_linear_speed;
  double max_angular_speed;
  // double max_linear_acc;
  // double max_angular_acc;

  double current_linear_x;
  double current_linear_y;
  double current_angular;

  static constexpr double MAX_ACTION[4] = {1616, 1665, 1681, 1617};
  static constexpr double eps = 1e-2;
  static constexpr double cofficient_acc = 0.2;

  bool cond_joy_;

  std::mutex mtx_;
  std::condition_variable cv_;

  void joysub_callback(bupt_interfaces::msg::Joystick::SharedPtr msg);
  void joysrv_callback(bupt_interfaces::srv::JoyStickInteraction::Request::SharedPtr req);

public:
  Joystick_Vel_Server(const std::string &name,
                      double max_linear_speed,
                      double max_angular_speed);
};

Joystick_Vel_Server::Joystick_Vel_Server(const std::string &name,
                                         double max_linear_speed,
                                         double max_angular_speed)
    : Node(name),
      max_linear_speed(max_linear_speed),
      max_angular_speed(max_angular_speed),
      cond_joy_(true),
      mtx_(std::mutex()),
      cv_(std::condition_variable())
{
  joysub_ = this->create_subscription<bupt_interfaces::msg::Joystick>(
      "joystick", 10,
      std::bind(&Joystick_Vel_Server::joysub_callback, this, std::placeholders::_1));
  velpub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  joysrv_ = this->create_service<bupt_interfaces::srv::JoyStickInteraction>(
      "joyvel_srv", std::bind(&Joystick_Vel_Server::joysrv_callback, this, std::placeholders::_1));
}

void Joystick_Vel_Server::joysub_callback(bupt_interfaces::msg::Joystick::SharedPtr msg)
{
  {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] {
      return cond_joy_;
    });
  }

  auto twist = geometry_msgs::msg::Twist();

  double current_speed_x = 0, current_speed_y = 0, current_angular_speed = 0;
  double current_acc_x = 0, current_acc_y = 0, current_angular_acc = 0;
  // 计算加速度
  current_acc_x = msg->action[1] / std::hypot(MAX_ACTION[0], MAX_ACTION[1]);
  current_acc_y = msg->action[0] / std::hypot(MAX_ACTION[0], MAX_ACTION[1]);
  current_angular_acc = get_sign(msg->action[2]) * std::hypot(msg->action[2], msg->action[3]) / std::hypot(MAX_ACTION[2], MAX_ACTION[3]);
  // 计算当前速度
  current_speed_x = current_linear_x + current_acc_x * cofficient_acc;
  if (current_speed_x < 0)
    current_speed_x = 0;
  current_speed_y = current_linear_y + current_acc_y * cofficient_acc;
  if (current_speed_y < 0)
    current_speed_y = 0;
  current_angular_speed = current_angular + current_angular_acc * cofficient_acc;
  // 速度限制
  double current_speed = std::hypot(current_speed_x, current_speed_y);
  if (current_speed > eps) {
    double current_speed_limit = clamp(current_speed, 0.0, max_linear_speed);
    current_linear_x = current_speed_x / current_speed * current_speed_limit;
    current_linear_y = current_speed_y / current_speed * current_speed_limit;
  } else {
    current_linear_x = 0;
    current_linear_y = 0;
  }
  current_angular = clamp(current_angular_speed, -max_angular_speed, max_angular_speed);
  if (current_angular < 0.5) {
    current_angular = 0;
  }
  // 设置发送速度
  twist.linear.x = current_linear_x;
  twist.linear.y = current_linear_y;
  twist.linear.z = 0;
  twist.angular.z = current_angular;
  // 发送速度消息
  velpub_->publish(twist);
}

void Joystick_Vel_Server::joysrv_callback(
    bupt_interfaces::srv::JoyStickInteraction::Request::SharedPtr req)
{
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

class ROS_EVENT_LOOP
{
public:
  ROS_EVENT_LOOP(int argc, char **argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joystick_Vel_Server>("joystick_vel_server", 10, 5));
  }
  ~ROS_EVENT_LOOP()
  {
    rclcpp::shutdown();
  }
};

int main(int argc, char **argv)
{
  auto node = ROS_EVENT_LOOP(argc, argv);
  return 0;
}
