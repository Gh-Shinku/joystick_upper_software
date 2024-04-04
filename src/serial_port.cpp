#include "serial_port/serial_port.h"
#include "message/message.h"

#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>
#include <boost/asio/read_until.hpp>


using namespace boost::asio;
using boost::asio::serial_port_base;
namespace asio = boost::asio;

SerialPortClient::SerialPortClient(boost::asio::io_context &context_arg, const std::string& port)
    : context(context_arg), serial(context_arg, port) {
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
                    read_start(); // 继续读取更多数据
                } else {
                    std::cerr << "Read error: " << ec.message() << std::endl;
                    read_start();
                }
            });
}

void SerialPortClient::read_packet() {
    // 在缓冲区中搜索数据包结尾的标志（0x2A）
    auto it = std::find(read_buffer_.begin(), read_buffer_.end(), 0x2A);
    if (it != read_buffer_.end()) {
        // 找到数据包结尾，检查是否有足够的数据读取下一个完整的数据包
        std::size_t packetStartIndex = std::distance(read_buffer_.begin(), it) + 1;
        if (read_buffer_.size() >= packetStartIndex + 22) {
            // 读取并处理22字节的数据包
            std::array<uint8_t, 22> buffer = {0};
            std::copy(read_buffer_.begin() + packetStartIndex, read_buffer_.begin() + packetStartIndex + 22, buffer.begin());
            spdlog::info("Received from serial port");
            MessagePacket message_packet;
            get_message_packet(message_packet, buffer);
            
            // 从缓冲区中移除已处理的数据
            read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + packetStartIndex + 22);
        }
    }
}

void SerialPortClient::run() {
    context.run();
}
