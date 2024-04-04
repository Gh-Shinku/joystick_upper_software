#include "serial_port/serial_port.h"

#include "message/message.h"

#include <iostream>
#include <thread>

using namespace boost::asio;
using boost::asio::serial_port_base;
namespace asio = boost::asio;

SerialPortClient::SerialPortClient(boost::asio::io_context &context_arg, const std::string& port, std::function<void(const MessagePacket&)> on_receive)
    : context(context_arg), serial(context_arg, port), on_receive(on_receive) {
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    read_start();
}

void SerialPortClient::read_start() {
    asio::async_read(serial, asio::dynamic_buffer(read_buffer_), asio::transfer_at_least(1),
            [this](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    read_packet();
                    read_start(); 
                } else {
                    read_start();
                }
            });
}

void SerialPortClient::read_packet() {
    auto it = read_buffer_.begin() + read_buffer_.find("*+");
    if (it != read_buffer_.end()) {
        std::size_t packetStartIndex = std::distance(read_buffer_.begin(), it) + 1;
        if (read_buffer_.size() >= packetStartIndex + 22) {
            std::array<uint8_t, 22> buffer = {0};
            std::copy(read_buffer_.begin() + packetStartIndex, read_buffer_.begin() + packetStartIndex + 22, buffer.begin());
            MessagePacket message_packet;
            if (get_message_packet(message_packet, buffer))
            {
                if (on_receive)
                {
                    on_receive(message_packet);
                }
            }
            read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + packetStartIndex + 22);
        }
    }
}

void SerialPortClient::run() {
    context.run();
}
