#include "joystick/joystick_button_controller.hpp"

#include "joystick/action.hpp"

std::string action_to_string(uint32_t action) {
  std::string str;
  switch (action) {
    case static_cast<uint32_t>(ACTION::BUTTON_SHOOT): {
      str = "SHOOT";
      break;
    }
    case static_cast<uint32_t>(ACTION::BUTTON_AIM_MODE): {
      str = "AIM_MODE";
      break;
    }
    case static_cast<uint32_t>(ACTION::BUTTON_ACC_RPM): {
      str = "ACC_RPM";
      break;
    }
    case static_cast<uint32_t>(ACTION::BUTTON_DEC_RPM): {
      str = "DEC_RPM";
      break;
    }
    case static_cast<uint32_t>(ACTION::BUTTON_YAW_CLOCK): {
      str = "YAW_CLOCK";
      break;
    }
    case static_cast<uint32_t>(ACTION::BUTTON_YAW_ANTI_CLOCK): {
      str = "YAW_ANTI_CLOCK";
      break;
    }
  }
  return str;
}

Joystick_Button_Node::Joystick_Button_Node(std::string name) : Node(name) {
  this->declare_parameter("joystick_topic", "/joystick");
  this->declare_parameter("R1_service", "R1_srv");
  this->declare_parameter("R2_service", "R2_srv");
  this->declare_parameter("go_service", "go_srv");
  this->declare_parameter("joy_service", "joyvel_srv");
  this->get_parameter("joystick_topic", joystick_topic_);
  this->get_parameter("R1_service", R1_service_name_);
  this->get_parameter("R2_service", R2_service_);
  this->get_parameter("go_service", go_service_);
  this->get_parameter("joy_service", joy_service_);
  joystick_sub_ = this->create_subscription<bupt_interfaces::msg::Joystick>(
      joystick_topic_, 10, std::bind(&Joystick_Button_Node::joystick_callback, this, std::placeholders::_1));
  R1_client_ = this->create_client<bupt_interfaces::srv::ActionControl>(R1_service_name_);
  R2_client_ = this->create_client<bupt_interfaces::srv::ActionControl>(R2_service_);
  go_client_ = this->create_client<bupt_interfaces::srv::ActionControl>(go_service_);
  joy_client_ = this->create_client<bupt_interfaces::srv::ActionControl>(joy_service_);

  setup_edge_detect_configs();
}

Joystick_Button_Node::~Joystick_Button_Node() {
  RCLCPP_INFO(this->get_logger(), "上位机节点退出");
}

/* TODO: 引入引用？移动语义？ */
void Joystick_Button_Node::register_edge_detect(uint32_t bit_position, bool trigger_on_rising_edge, EdgeCallback callback) {
  edge_detect_configs_.emplace_back(EdgeDetectConfig{bit_position, trigger_on_rising_edge, callback});
}

void Joystick_Button_Node::send_request_action_control(const rclcpp::Client<bupt_interfaces::srv::ActionControl>::SharedPtr& client,
                                                       uint32_t action) {
  auto action_str = action_to_string(action);

  /* 等待服务启动 */
  int wait_time = 0;
  while (!client->wait_for_service(std::chrono::milliseconds(100))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "等待服务的过程中 ROS2 事件循环被中断");
      return;
    }
    if (wait_time > max_wait_time) {
      RCLCPP_ERROR(this->get_logger(), "等待 service: %s 超时", action_str.c_str());
      return;
    }
    ++wait_time;
  }

  auto request = std::make_shared<bupt_interfaces::srv::ActionControl::Request>();
  request->action = action;

  RCLCPP_INFO(this->get_logger(), "发送请求给 service: %s", action_str.c_str());
  auto result =
      client->async_send_request(request, [this, &action_str](rclcpp::Client<bupt_interfaces::srv::ActionControl>::SharedFuture future) {
        auto response = future.get();
        if (response) {
          RCLCPP_INFO(this->get_logger(), "执行结果: %s", response->finish ? "已完成" : "未完成");
        } else {
          RCLCPP_ERROR(this->get_logger(), "未能从 service: %s 获得有效响应", action_str.c_str());
        }
      });
}

void Joystick_Button_Node::joystick_callback(const bupt_interfaces::msg::Joystick::SharedPtr msg) {
  uint16_t curr_button = msg->button;

  /* 通过检测上升沿来避免粘滞问题 */
  for (const auto& config : edge_detect_configs_) {
    if (edge_detect(curr_button, config.bit_position, true)) {
      config.callback();
      /* 保留按键优先级机制，一次只相应一个功能服务请求 */
      break;
    }
  }
}

void Joystick_Button_Node::setup_edge_detect_configs() {
  /* SHOOT service 对接结束，server 端不对 request 做检查，直接开始发射 */
  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_SHOOT), true,
                       [this]() { send_request_action_control(R1_client_, static_cast<uint32_t>(ACTION::BUTTON_SHOOT)); });

  /* AIM_MODE service 对接结束，server 端检测 action 是否为对应的，是则将手柄开关取反 */
  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_AIM_MODE), true,
                       [this]() { send_request_action_control(joy_client_, static_cast<uint32_t>(ACTION::BUTTON_AIM_MODE)); });

  /* ACC_RPM service 转速微调对接结束 */
  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_ACC_RPM), true,
                       [this]() { send_request_action_control(R1_client_, static_cast<uint32_t>(ACTION::BUTTON_ACC_RPM)); });

  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_DEC_RPM), true,
                       [this]() { send_request_action_control(R1_client_, static_cast<uint32_t>(ACTION::BUTTON_DEC_RPM)); });

  /* YAW_CLOCK service 暂未对接 */
  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_YAW_CLOCK), true,
                       [this]() { send_request_action_control(joy_client_, static_cast<uint32_t>(ACTION::BUTTON_YAW_CLOCK)); });

  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_YAW_ANTI_CLOCK), true,
                       [this]() { send_request_action_control(joy_client_, static_cast<uint32_t>(ACTION::BUTTON_YAW_ANTI_CLOCK)); });

  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_DRIBBLE), true,
                       [this]() { send_request_action_control(R2_client_, static_cast<uint32_t>(ACTION::BUTTON_DRIBBLE)); });

  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_PAWL), true,
                       [this]() { send_request_action_control(R2_client_, static_cast<uint32_t>(ACTION::BUTTON_PAWL)); });

  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_SLIDEWAY), true,
                       [this]() { send_request_action_control(R2_client_, static_cast<uint32_t>(ACTION::BUTTON_SLIDEWAY)); });

  register_edge_detect(static_cast<uint32_t>(ACTION::BUTTON_GO_POINTS), true,
                       [this]() { send_request_action_control(go_client_, static_cast<uint32_t>(ACTION::BUTTON_GO_POINTS)); });
}

bool Joystick_Button_Node::edge_detect(uint16_t cur_button, uint16_t bits, bool trigger_mode) {
  static uint16_t last_button = 0;
  /* 0-->1 */
  if (trigger_mode == true) {
    return ((cur_button >> bits) & 0x01) && !((last_button >> bits) & 0x01);
  } else /* 1-->0 */ {
    return !((cur_button >> bits) & 0x01) && ((last_button >> bits) & 0x01);
  }
  last_button = cur_button;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Joystick_Button_Node>("Joystick_Button_Node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}