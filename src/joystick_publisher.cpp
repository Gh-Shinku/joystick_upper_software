#include <boost/asio/signal_set.hpp>
#include <csignal>
#include <rclcpp/rclcpp.hpp>

#include "bupt_interfaces/msg/joystick.hpp"
#include "logger/logger.h"
#include "message/message.h"
#include "serial_port/serial_port.h"
#include "tcp_socket/tcp_socket.h"

class JoystickPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<bupt_interfaces::msg::Joystick>::SharedPtr publisher_;

  boost::asio::io_context &context;

  std::unique_ptr<boost::asio::steady_timer> timer_for_count;
  std::unique_ptr<boost::asio::steady_timer> timer_for_publish;
  // std::unique_ptr<TCPSocketClient> tcp_client;
  std::unique_ptr<SerialPortClient> serial_client;
  LoggerImpl logger;

  bupt_interfaces::msg::Joystick message;
  uint32_t number = 0;
  uint32_t last_number = 0;

public:
  JoystickPublisher(boost::asio::io_context &context, const std::string &topic)
      : Node("joystick_node"),
        context(context),
        logger("joystick_node"),
        timer_for_count(
            std::make_unique<boost::asio::steady_timer>(context, std::chrono::seconds(5))),
        timer_for_publish(
            std::make_unique<boost::asio::steady_timer>(context, std::chrono::milliseconds(20)))
  {
    publisher_ = this->create_publisher<bupt_interfaces::msg::Joystick>(topic, 10);

    // tcp_client = std::make_unique<TCPSocketClient>(context, [this](const Message::MessagePacket
    // &msg)
    //                                                { update(msg); });

    // this->declare_parameter("ip", "192.168.4.1");
    // this->declare_parameter("port", 3456);
    this->declare_parameter("serial_port", "/dev/joystick-serial");

    // uint16_t port_int;
    // std::string ip, port;
    std::string serial_port;
    // this->get_parameter("ip", ip);
    // this->get_parameter("port", port_int);
    this->get_parameter("serial_port", serial_port);
    // port = std::to_string(port_int);

    // tcp_client->connect(ip, port);
    // 注意，这个函数仅仅会发起一个异步的连接请求，而并不确保连接成功

    try {
      serial_client = std::make_unique<SerialPortClient>(context, serial_port,
                                                         [this](const Message::MessagePacket &msg) {
                                                           update(msg);
                                                         });
    } catch (boost::system::system_error &e) {
      logger.error(e.what());
    }
    timer_for_count->async_wait([this](boost::system::error_code ec) {
      calculate_rate(ec);
    });
    timer_for_publish->async_wait([this](boost::system::error_code ec) {
      publish(ec);
    });
  }

  void calculate_rate(boost::system::error_code ec)
  {
    if (!ec) {
      logger.info("Receive {} messages per second", (number - last_number) / 5);
      last_number = number;
      timer_for_count->expires_after(std::chrono::seconds(5));
      timer_for_count->async_wait([this](boost::system::error_code ec) {
        calculate_rate(ec);
      });
    }
  }

  // 更新待发送的消息
  void update(const Message::MessagePacket &message_packet)
  {
    if (message_packet.number > number) {
      number = message_packet.number;
      message.action = message_packet.action;
      message.button = message_packet.button;
    }
  }

  // 按照设定的频率发布消息
  void publish(boost::system::error_code ec)
  {
    if (!ec) {
      publisher_->publish(message);
      timer_for_publish->expires_after(std::chrono::milliseconds(20));
      timer_for_publish->async_wait([this](boost::system::error_code ec) {
        publish(ec);
      });
    }
  }
};

class RAII
{
  boost::asio::io_context context;
  std::unique_ptr<boost::asio::signal_set> signals_;
  LoggerImpl logger;
  std::shared_ptr<rclcpp::Node> node;
  std::thread asio_thread;
  std::thread ros_thread;

public:
  RAII(int argc, char *argv[]) : logger("RAII")
  {
    rclcpp::init(argc, argv);
    try {
      signals_ = std::make_unique<boost::asio::signal_set>(context, SIGINT);
      signals_->async_wait([this](const boost::system::error_code &error, int signal_number) {
        if (!error) {
          logger.warn("Receive Signal: {}", signal_number);
        }
      });
      node = std::make_shared<JoystickPublisher>(context, "/joystick");
      asio_thread = std::thread([this]() {
        context.run();
      });

      ros_thread = std::thread([this]() {
        rclcpp::spin(node);
      });
    } catch (const std::exception &e) {
      logger.error("Exception: {}", e.what());
      rclcpp::shutdown();
      throw;
    }
  }
  ~RAII()
  {
    if (signals_) {
      signals_->cancel();
    }
    if (asio_thread.joinable()) {
      asio_thread.join();
    }
    context.stop();
    if (ros_thread.joinable()) {
      ros_thread.join();
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char **argv)
{
  auto raii = RAII(argc, argv);
  return 0;
}
