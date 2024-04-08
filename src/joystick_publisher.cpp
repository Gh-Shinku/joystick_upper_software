#include "tcp_socket/tcp_socket.h"
#include "serial_port/serial_port.h"
#include "message/message.h"
#include "bupt_interfaces/msg/joystick.hpp"

#include <rclcpp/rclcpp.hpp>
#include <boost/asio/signal_set.hpp>
#include <csignal>

class JoystickPublisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<bupt_interfaces::msg::Joystick>::SharedPtr publisher_;

    boost::asio::io_context &context;

    std::unique_ptr<boost::asio::steady_timer> timer_;
    std::unique_ptr<TCPSocketClient> tcp_client;
    std::unique_ptr<SerialPortClient> serial_client;

    std::function<void(boost::system::error_code ec)> timeout_callback;
    std::function<void(const MessagePacket &message_packet)> msg_callback;

    bupt_interfaces::msg::Joystick message;
    uint32_t number = 0;
    uint32_t last_number = 0;

public:
    JoystickPublisher(boost::asio::io_context &context, std::string topic) : Node("joystick_node"), context(context)
    {
        publisher_ = this->create_publisher<bupt_interfaces::msg::Joystick>(topic, 10);
        timer_ = std::make_unique<boost::asio::steady_timer>(context, std::chrono::seconds(5));

        timeout_callback = [this](boost::system::error_code ec)
        {
            if (!ec)
            {
                RCLCPP_INFO(this->get_logger(), "Receive %d messages per second", (number - last_number) / 5);
                last_number = number;
                timer_->expires_after(std::chrono::seconds(5));
                timer_->async_wait(timeout_callback);
            }
        };
        msg_callback = [this](const MessagePacket &message_packet)
        {
            // 更新时间戳，只有时间戳大于当前时间戳才更新
            if (message_packet.number > number)
            {
                number = message_packet.number;
                update(message_packet);
            }
        };

        tcp_client = std::make_unique<TCPSocketClient>(context, msg_callback);
        tcp_client->connect("192.168.4.1", "3456");
        // 注意，这个函数仅仅会发起一个异步的连接请求，而并不确保连接成功

        bool failed_to_open_serial = false;
        try
        {
            serial_client = std::make_unique<SerialPortClient>(context, "/dev/ttyUSB0", msg_callback);
        }
        catch (boost::system::system_error &e)
        {
            RCLCPP_WARN(this->get_logger(), e.what());
            failed_to_open_serial = true;
        }
        timer_->async_wait(timeout_callback);
    }

    // 更新待发送的消息
    void update(const MessagePacket &message_packet)
    {

        message.action = message_packet.action;
        message.button = message_packet.button;
    }

    // 按照设定的频率发布消息
    void publish()
    {
        publisher_->publish(message);
    }
};

class RAII
{
    boost::asio::io_context context;
    std::unique_ptr<boost::asio::signal_set> signals_;

    std::shared_ptr<rclcpp::Node> node;

public:
    RAII(int argc, char *argv[]) : context()
    {
        rclcpp::init(argc, argv);
        try
        {
            signals_ = std::make_unique<boost::asio::signal_set>(context, SIGINT);
            signals_->async_wait([this](const boost::system::error_code &error, int signal_number)
                                 {
                if (!error) {
                    context.stop();
                    RCLCPP_WARN(rclcpp::get_logger("KeyInterrupt"), "Receive Signal: %d", signal_number);
                } });
            node = std::make_shared<JoystickPublisher>(context, "joystick");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Exception"), e.what());
        }
        context.run();
    }
    ~RAII()
    {

        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    auto raii = RAII(argc, argv);
    return 0;
}
