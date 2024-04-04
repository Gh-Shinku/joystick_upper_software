#include <boost/asio.hpp>
#include <iostream>
#include <deque>
#include <array>
#include <spdlog/spdlog.h>
#include "message/message.h"

using namespace boost::asio;
using boost::asio::serial_port_base;
namespace asio = boost::asio;

class SerialPortReader {
public:
    SerialPortReader(asio::io_context& io_context, const std::string& port_name)
        : io_context_(io_context), serial_port_(io_context, port_name) {
        // 配置串口参数
        serial_port_.set_option(serial_port_base::baud_rate(115200));
        serial_port_.set_option(serial_port_base::character_size(8));
        serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

        startRead();
    }

    void startRead() {
        asio::async_read(serial_port_, asio::dynamic_buffer(read_buffer_), asio::transfer_at_least(1),
            [this](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    processData();
                    startRead(); // 继续读取更多数据
                } else {
                    std::cerr << "Read error: " << ec.message() << std::endl;
                }
            });
    }

    void processData() {
        // 在缓冲区中搜索数据包结尾的标志（0x2A）
        auto it = std::find(read_buffer_.begin(), read_buffer_.end(), 0x2A);
        if (it != read_buffer_.end()) {
            // 找到数据包结尾，检查是否有足够的数据读取下一个完整的数据包
            std::size_t packetStartIndex = std::distance(read_buffer_.begin(), it) + 1;
            if (read_buffer_.size() >= packetStartIndex + 22) {
                // 读取并处理22字节的数据包
                std::array<uint8_t, 22> buffer = {0};
                std::copy(read_buffer_.begin() + packetStartIndex, read_buffer_.begin() + packetStartIndex + 22, buffer.begin());
                MessagePacket message_packet;
                get_message_packet(message_packet, buffer);
                
                // 从缓冲区中移除已处理的数据
                read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + packetStartIndex + 22);
            }
        }
    }

private:
    asio::io_context& io_context_;
    asio::serial_port serial_port_;
    std::string read_buffer_;
};

int main() {
    asio::io_context io_context;
    SerialPortReader reader(io_context, "COM6"); // 更换为你的串口名称
    io_context.run();
}
