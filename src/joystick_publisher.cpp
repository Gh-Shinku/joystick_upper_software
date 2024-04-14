#include "tcp_socket/tcp_socket.h"
//#include "serial_port/serial_port.h"
#include "message/message.h"
#include "bupt_interfaces/msg/joystick.hpp"

#include <rclcpp/rclcpp.hpp>
#include <boost/asio/signal_set.hpp>
#include <csignal>

class JoystickPublisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<bupt_interfaces::msg::Joystick>::SharedPtr publisher_;
    // rclcpp::TimerBase::SharedPtr timer_;
    boost::asio::io_context &context;
    std::unique_ptr<TCPSocketClient> tcp_client;
    //std::unique_ptr<SerialPortClient> serial_client;

public:
    JoystickPublisher(boost::asio::io_context &context, std::string topic) : Node("joystick_node"), context(context)
    {
        publisher_ = this->create_publisher<bupt_interfaces::msg::Joystick>(topic, 10);
        tcp_client = std::make_unique<TCPSocketClient>(context, [this](const MessagePacket &message_packet)
        { publish(message_packet); });
        tcp_client->connect("192.168.4.1", "3456");
        //serial_client = std::make_unique<SerialPortClient>(context, "/dev/ttyUSB0", [this](const MessagePacket &message_packet)
        //                                                   { publish(message_packet); });
    }

    void publish(const MessagePacket &message_packet)
    {
        auto message = bupt_interfaces::msg::Joystick();
        message.action = message_packet.action;
        message.button = message_packet.button;
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
            context.run();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Exception"), e.what());
        }
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
