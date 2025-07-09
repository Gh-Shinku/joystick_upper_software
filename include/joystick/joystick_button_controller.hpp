#ifndef JOYSTICK_BUTTON_CONTROLLER_H
#define JOYSTICK_BUTTON_CONTROLLER_H

#include <bupt_interfaces/msg/joystick.hpp>
#include <bupt_interfaces/srv/action_control.hpp>
#include <bupt_interfaces/srv/r1_top_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

std::string action_to_string(uint32_t action);

class Joystick_Button_Node : public rclcpp::Node {
private:
  using EdgeCallback = std::function<void()>;

  struct EdgeDetectConfig {
    uint32_t bit_position;
    bool trigger_on_rising_edge; /* true: 上升沿   false: 下降沿 */
    EdgeCallback callback;
  };

  static constexpr int max_wait_time = 30;

  std::string joystick_topic_;
  std::string R2_service_;
  std::string R1_service_;
  std::string go_service_;
  std::string joy_service_;

  rclcpp::Subscription<bupt_interfaces::msg::Joystick>::SharedPtr joystick_sub_;
  rclcpp::Client<bupt_interfaces::srv::ActionControl>::SharedPtr R1_client_;
  rclcpp::Client<bupt_interfaces::srv::ActionControl>::SharedPtr R2_client_;
  rclcpp::Client<bupt_interfaces::srv::ActionControl>::SharedPtr go_client_;
  rclcpp::Client<bupt_interfaces::srv::ActionControl>::SharedPtr joy_client_;

  std::vector<EdgeDetectConfig> edge_detect_configs_; /* 存储边沿注册配置 */

  /* TODO: 引入引用？移动语义？ */
  void register_edge_detect(uint32_t bit_position, bool trigger_on_rising_edge, EdgeCallback callback);

  void send_request_action_control(const rclcpp::Client<bupt_interfaces::srv::ActionControl>::SharedPtr& client, uint32_t action);

  void joystick_callback(const bupt_interfaces::msg::Joystick::SharedPtr msg);

  void setup_edge_detect_configs();

  /**
   * @brief 边沿检测
   *
   * @param cur_button 当前按键编码
   * @param bits 检测比特位号
   * @param trigger_mode true 上升沿，false 下降沿
   *
   * @return true 说明检测到，否则无
   */
  static bool edge_detect(uint16_t cur_button, uint16_t bits, bool trigger_mode);

public:
  Joystick_Button_Node(std::string name);
  ~Joystick_Button_Node();
};

#endif /* JOYSTICK_BUTTON_CONTROLLER_H */