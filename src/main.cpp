#include "tcp_socket/tcp_socket.h"
#include "serial_port/serial_port.h"
#include "message/message.h"
#include "bupt_interfaces/msg/new_joystick.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <boost/asio/signal_set.hpp>
#include <csignal>

class JoyStickNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<bupt_interfaces::msg::NewJoystick>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    boost::asio::io_context &context;
    std::unique_ptr<TCPSocketClient> tcp_client;
    std::unique_ptr<SerialPortClient> serial_client;
public:
    JoyStickNode(boost::asio::io_context &context, std::string topic): Node("joystick_node"), context(context)
    {
        publisher_ = this->create_publisher<bupt_interfaces::msg::NewJoystick>(topic, 10);
        tcp_client = std::make_unique<TCPSocketClient>(context,[this](const MessagePacket &message_packet)
        {
            publish(message_packet);
        });
        tcp_client -> connect("192.168.4.1","3456");

    }

    void publish(const MessagePacket &message_packet)
    {
        auto message = bupt_interfaces::msg::NewJoystick();
        message.action = message_packet.action;
        message.button = message_packet.button;
        publisher_->publish(message);
    }
};

class RAII
{
    boost::asio::io_context context;
    std::unique_ptr<boost::asio::signal_set> signals_;
public:
    RAII(int argc,char * argv[]): context()
    {
        rclcpp::init(argc,argv);
        try {
            signals_ = std::make_unique<boost::asio::signal_set>(context, SIGINT);
            signals_->async_wait([this](const boost::system::error_code& error, int signal_number) {
                if (!error) {
                    context.stop();
                    spdlog::warn("Received signal: SIGINT");
                }
            });
            JoyStickNode node(context, "joystick");
            context.run();
        } catch (const std::exception &e) {
            spdlog::error("Exception: {}", e.what());
        }
    }
    ~RAII()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv) 
{
    auto raii = RAII(argc,argv);
    return 0;
}
